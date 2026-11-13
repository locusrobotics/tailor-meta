"""Persisted build plans and individually runnable Tailor Actions stages."""
import argparse
from contextlib import contextmanager
import datetime
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

import yaml


MODES = ('validate', 'graph', 'packages', 'release')
STAGES = ('fetch', 'mirror', 'graph', 'packages', 'publish', 'images')
REQUIRES = {'fetch': None, 'mirror': 'fetch', 'graph': 'fetch', 'packages': 'graph',
            'publish': 'packages', 'images': 'publish'}


def release(ref):
    if ref.startswith('refs/tags/'):
        label = ref.removeprefix('refs/tags/')
        if not re.fullmatch(r'\d+\.\d+\.\d+(?:[.+-][a-z0-9.+-]+)?', label):
            raise ValueError('Release tags must have a numeric major.minor.patch')
        track, kind = '.'.join(label.split('.')[:2]), 'final'
    else:
        branch = ref.removeprefix('refs/heads/')
        if branch == 'master':
            kind, track, label = 'hotdog', 'hotdog', 'hotdog'
        elif branch.startswith('release/'):
            track = branch.removeprefix('release/')
            kind, label = 'candidate', track + '-rc'
        elif branch.startswith('feature/'):
            kind, track, label = 'feature', 'hotdog', branch.replace('feature/', 'feature-', 1).lower()
        else:
            return dict(kind='trivial', track='', label='', days=1)
    if not re.fullmatch(r'[a-z0-9][a-z0-9.+-]*', label):
        raise ValueError('Release label is not Debian-compatible')
    return dict(kind=kind, track=track, label=label, days=0 if kind in ('candidate', 'final') else 30)


def config_digest(root):
    digest = hashlib.sha256()
    for directory in ('config', 'rosdep', 'rosdistro'):
        for path in sorted((root / directory).rglob('*')):
            if path.is_file():
                digest.update(str(path.relative_to(root)).encode())
                digest.update(path.read_bytes())
    return digest.hexdigest()


def revision(root):
    return subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip()


def file_digest(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def plan(root, ref, mode='packages'):
    if mode not in MODES:
        raise ValueError(f'Unsupported mode: {mode}')
    recipes = yaml.safe_load((root / 'config/recipes.yaml').read_text())
    images = yaml.safe_load((root / 'config/images.yaml').read_text()).get('images') or {}
    if set(recipes['os']) != {'ubuntu'}:
        raise ValueError('The packaging environment currently supports Ubuntu only')
    targets = recipes['os']['ubuntu']
    common = recipes['common']
    result = release(ref)
    if result['kind'] == 'trivial' and mode != 'validate':
        raise ValueError('Builds require master, feature/*, release/*, or a release tag')
    result.update(schema=1, mode=mode, ref=ref, os=targets, common=common, images=[
        dict(name=name, os=version, flavour=config.get('bundle_flavour', common['testing_flavour']))
        for name, config in images.items() for version in targets
        if version in config.get('os_versions', targets)
    ])
    result.update(timestamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%d.%H%M%S'),
                  config_sha=revision(root), implementation_sha=revision(Path(__file__).resolve().parents[1]),
                  config_digest=config_digest(root),
                  run=os.environ.get('GITHUB_RUN_ID', 'local') + '-' + os.environ.get('GITHUB_RUN_ATTEMPT', '1'),
                  repository=os.environ.get('GITHUB_REPOSITORY', 'local'), sources={}, completed=[])
    return result


def save_plan(path, config):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(config, indent=2) + '\n')
    temporary.replace(path)


def claim_release(config, client):
    """Reserve a final label before its first remote write. Failed claims remain.

    Conditional PUT prevents competing runs from claiming the same final label.
    Existing legacy releases are also rejected. Never delete a claim on failure:
    a partially published final label requires operator recovery, not overwrite.
    """
    if config['kind'] != 'final':
        return
    bucket = config['common']['apt_repo'].removeprefix('s3://')
    prefix = config['label'] + '/'
    existing = client.list_objects_v2(Bucket=bucket, Prefix=prefix, MaxKeys=1)
    if existing.get('KeyCount', 0):
        raise RuntimeError(f'Final release {config["label"]} already exists; refusing to overwrite')
    client.put_object(Bucket=bucket, Key=prefix + 'release-claim.json', IfNoneMatch='*',
                      Body=json.dumps({key: config[key] for key in
                                       ('repository', 'run', 'config_sha', 'implementation_sha', 'timestamp')}).encode(),
                      ContentType='application/json')


def verify_release_claim(config, client):
    if config['kind'] != 'final':
        return
    bucket = config['common']['apt_repo'].removeprefix('s3://')
    response = client.get_object(Bucket=bucket, Key=config['label'] + '/release-claim.json')
    claim = json.loads(response['Body'].read())
    if any(claim[key] != config[key] for key in ('repository', 'run', 'config_sha', 'implementation_sha')):
        raise RuntimeError('This run does not own the final release claim')


class Build:
    def __init__(self, config, force_mirror=False, rebuild=False, no_cache=False):
        self.config = config
        self.force_mirror, self.rebuild, self.no_cache = force_mirror, rebuild, no_cache
        self.root = Path.cwd()
        self.label, self.timestamp = config['label'], config['timestamp']
        self.apt = config['common']['apt_repo']
        self.parent = f'tailor-parent:{config["run"]}'
        self.secret_dir = None

    def run(self, *args):
        subprocess.run([str(arg) for arg in args], check=True)

    @contextmanager
    def credentials(self):
        # Per-stage temporary files, never included in the Docker context.
        with tempfile.TemporaryDirectory(prefix='tailor-auth-', dir=os.environ.get('RUNNER_TEMP')) as directory:
            self.secret_dir = Path(directory)
            values = [('AccessKeyId', 'aws_access_key_id', 'AWS_ACCESS_KEY_ID'),
                      ('SecretAccessKey', 'aws_secret_access_key', 'AWS_SECRET_ACCESS_KEY'),
                      ('Token', 'aws_session_token', 'AWS_SESSION_TOKEN'),
                      ('Region', 'region', 'AWS_DEFAULT_REGION')]
            for name, text in (
                ('s3auth', '\n'.join(f'{apt} = {os.environ.get(env, "")}' for apt, _, env in values)),
                ('aws', '[default]\n' + '\n'.join(f'{aws} = {os.environ.get(env, "")}' for _, aws, env in values)),
                ('vault', os.environ.get('ANSIBLE_VAULT_PASS', '')),
                ('gitconfig', '[url "https://x-access-token:' + os.environ.get('GITHUB_TOKEN', '') +
                 '@github.com/"]\n\tinsteadOf = ssh://git@github.com/\n\tinsteadOf = git@github.com:\n'),
            ):
                (self.secret_dir / name).write_text(text)
                (self.secret_dir / name).chmod(0o600)
            try:
                yield
            finally:
                self.secret_dir = None

    def container(self, image, *command, apt=False, aws=False, github=False, git_auth=False,
                  signing=False, vault=False, privileged=False, workdir='/work'):
        args = ['docker', 'run', '--rm', '--user', '0', '-e', 'HOME=/home/tailor',
                '-v', f'{self.root}:/work', '-w', workdir]
        if apt:
            args += ['-v', f'{self.secret_dir}/s3auth:/etc/apt/s3auth.conf:ro']
        if aws:
            for name in ('AWS_ACCESS_KEY_ID', 'AWS_SECRET_ACCESS_KEY', 'AWS_SESSION_TOKEN', 'AWS_DEFAULT_REGION'):
                args += ['-e', name]
        if signing:
            args += ['-v', f'{os.environ["TAILOR_GPG_DIR"]}:/gpg:ro']
        if vault:
            args += ['-v', f'{self.secret_dir}/vault:/home/tailor/.vault_pass.txt:ro']
        if github:
            args += ['-e', 'GITHUB_TOKEN']
        if git_auth:
            args += ['-v', f'{self.secret_dir}/gitconfig:/root/.gitconfig:ro',
                     '-v', f'{self.secret_dir}/gitconfig:/home/tailor/.gitconfig:ro']
        if privileged:
            args += ['--privileged', '-v', '/var/run/docker.sock:/var/run/docker.sock',
                     '-v', '/dev:/dev', '-v', '/lib/modules:/lib/modules:ro', '-v', '/boot:/boot:ro']
        self.run(*args, image, *command)

    def docker_build(self, image, dockerfile, context='.', authenticated=False, **build_args):
        shutil.copyfile('tailor-meta/ci/dockerignore', Path(context) / '.dockerignore')
        args = ['docker', 'build', '-t', image, '-f', dockerfile]
        if self.no_cache:
            args += ['--no-cache']
        for key, value in build_args.items():
            args += ['--build-arg', f'{key}={value}']
        if authenticated:
            args += ['--secret', f'id=s3auth,src={self.secret_dir}/s3auth',
                     '--secret', f'id=aws,src={self.secret_dir}/aws']
        self.run(*args, context)

    def execute(self, stage):
        mode = self.config['mode']
        allowed = {'validate': (), 'graph': ('fetch', 'graph'),
                   'packages': ('fetch', 'graph', 'packages'), 'release': STAGES}
        if stage not in allowed[mode]:
            raise ValueError(f'Stage {stage} is not allowed in {mode} mode')
        required = REQUIRES[stage]
        if required and required not in self.config['completed']:
            raise ValueError(f'{stage} requires successful {required}')
        if mode == 'release' and stage == 'graph' and 'mirror' not in self.config['completed']:
            raise ValueError('Release graphs require the published mirror')
        if stage in self.config['completed']:
            raise ValueError(f'{stage} already completed; refusing accidental replay')
        if stage in ('mirror', 'publish', 'images') and self.config['kind'] == 'final':
            import boto3
            if stage == 'mirror':
                claim_release(self.config, boto3.client('s3'))
            else:
                verify_release_claim(self.config, boto3.client('s3'))
        with self.credentials():
            getattr(self, stage)()
        self.config['completed'].append(stage)

    def fetch(self):
        # Docker writes root-owned files, but later host steps must be able to
        # create the packaging context beside them.
        Path('workspace').mkdir(exist_ok=True)
        self.docker_build(self.parent, 'tailor-meta/environment/distro/Dockerfile',
                          APT_REFRESH_KEY=self.timestamp[:6])
        self.container(self.parent, 'create_recipes', '--recipes', 'rosdistro/config/recipes.yaml',
                       '--recipes-dir', 'workspace/recipes', '--release-track', self.config['track'],
                       '--release-label', self.label, '--debian-version', self.timestamp)
        self.container(self.parent, 'bash', '-ec',
                       'pull_distro_repositories --src-dir workspace/src --github-key "$GITHUB_TOKEN" '
                       '--recipes rosdistro/config/recipes.yaml --rosdistro-index rosdistro/rosdistro/index.yaml --clean',
                       github=True)
        self.config['sources'] = {
            str(path.relative_to('workspace/src')): [json.loads(line) for line in path.read_text().splitlines() if line]
            for path in Path('workspace/src').rglob('*_repositories_data.jsonl')
        }

    def mirror(self):
        for version in self.config['os']:
            self.container(self.parent, 'bash', '-ec', 'mirror_upstream "$@" --keys /gpg/*.key', 'mirror',
                           'rosdistro/config/upstream.yaml', '--version', self.timestamp, '--apt-repo', self.apt,
                           '--release-label', self.label, '--distribution', version, '--publish',
                           *(['--force-mirror'] if self.force_mirror else []), aws=True, signing=True)

    def graph(self):
        self.container(self.parent, 'generate_graphs', '--recipe', 'rosdistro/config/recipes.yaml',
                       '--release-label', self.label, '--package-release-label', self.label,
                       '--timestamp', self.timestamp, '--workspace', 'workspace',
                       '--apt-configs', '/etc/apt/s3auth.conf', apt=True, aws=True)

    def packages(self):
        shutil.copytree('tailor-meta', 'workspace/tailor-meta', ignore=shutil.ignore_patterns('.git', '__pycache__'))
        recipes = 'rosdistro/config/recipes.yaml'
        for version in self.config['os']:
            graph = f'workspace/graphs/ubuntu-{version}-graph.yaml'
            self.container(self.parent, 'get_dependency_list', '--graph', graph, '--recipe', recipes,
                           '--workspace', 'workspace', apt=True)
            # Build all graph packages, not just a hard-coded "dev" flavour.
            dependencies = Path(f'workspace/dependencies/all-{version}-{self.label}-dependencies.txt')
            image = f'tailor-package:{version}-{self.config["run"]}'
            self.docker_build(image, f'workspace/debian/Dockerfile-{version}', 'workspace', authenticated=True,
                              UNION_BUILD_DEPENDS=' '.join(sorted(set(dependencies.read_text().splitlines()))),
                              UNION_RUN_DEPENDS='', ORGANIZATION=self.config['common']['organization'],
                              APT_REFRESH_KEY=self.timestamp)
            root = Path('builds') / version
            (root / 'workspace').mkdir(parents=True, exist_ok=True)
            (root / 'workspace/src').symlink_to('/work/workspace/src', target_is_directory=True)
            self.container(image, 'bash', '/work/tailor-meta/ci/build-os.sh', '/work/' + graph,
                           '/work/' + recipes, version, str(self.rebuild).lower(),
                           apt=True, git_auth=True, workdir='/work/' + str(root))
            destination = Path('packages') / version
            destination.mkdir(parents=True, exist_ok=True)
            for deb in root.glob('*.deb'):
                deb.rename(destination / deb.name)
            self.config.setdefault('packages', {})[version] = {
                path.name: file_digest(path) for path in destination.glob('*.deb')
            }

    def publish(self):
        # Every OS completed before this stage. Validate the package snapshot
        # before mutating any APT repository.
        for version in self.config['os']:
            actual = {p.name: file_digest(p)
                      for p in (Path('packages') / version).glob('*.deb')}
            if actual != self.config['packages'][version]:
                raise ValueError(f'Package snapshot changed for {version}')
        for version in self.config['os']:
            self.container(self.parent, 'bash', '-ec',
                           'shopt -s nullglob; publish_packages *.deb "$@" --keys /gpg/*.key', 'publish',
                           '--release-label', self.label, '--apt-repo', self.apt, '--distribution', version,
                           '--organization', self.config['common']['organization'], *self.retention(),
                           aws=True, signing=True, workdir=f'/work/packages/{version}')
        for directory in ('rosdistro', 'rosdep', 'config'):
            self.run('aws', 's3', 'sync', f'rosdistro/{directory}', f'{self.apt}/{self.label}/{directory}')
        self.run('aws', 's3', 'cp', 'workspace/src', f'{self.apt}/{self.label}/changes/{self.timestamp}/',
                 '--recursive', '--exclude', '*', '--include', '*_repositories_data.jsonl')
        self.run('aws', 's3', 'sync', 'workspace/dependencies', f'{self.apt}/{self.label}/dependencies',
                 '--exclude', '*', '--include', '*.txt')
        if self.config['kind'] != 'final':
            self.container(self.parent, 's3_directory_cleanup', '--release-label', self.label,
                           '--apt-repo', self.apt.removeprefix('s3://'), *self.retention(), aws=True)
            for version in self.config['os']:
                self.container(self.parent, 's3_versions_cleanup', '--release-label', self.label,
                               '--apt-repo', self.apt.removeprefix('s3://'), '--distribution', version, aws=True)
        self.invalidate(f'/{self.label}/ubuntu/dists/*')

    def retention(self):
        if self.config['kind'] == 'final':
            return []
        return ['--num-to-keep', '30'] + (['--days-to-keep', str(self.config['days'])] if self.config['days'] else [])

    def invalidate(self, path):
        distribution = self.config['common'].get('cloudfront_distribution_id')
        if distribution:
            self.run('aws', 'cloudfront', 'create-invalidation', '--distribution-id', distribution, '--paths', path)

    def images(self):
        if not self.config['images']:
            return
        common = self.config['common']
        image = f'tailor-image:{self.config["run"]}'
        self.docker_build(image, 'tailor-meta/environment/image/Dockerfile', authenticated=True,
                          APT_REPO=self.apt, APT_REGION=common.get('apt_region', 'us-east-1'),
                          RELEASE_LABEL=self.label, ORGANIZATION=common['organization'],
                          FLAVOUR=common['build_flavour'], BUNDLE_ROOT=f'/opt/{common["organization"]}/{self.label}',
                          APT_REFRESH_KEY=self.timestamp)
        for target in self.config['images']:
            self.container(image, 'create_image', '--name', target['name'], '--distribution', target['os'],
                           '--apt-repo', self.apt.removeprefix('s3://'), '--release-label', self.label,
                           '--flavour', target['flavour'], '--organization', common['organization'],
                           '--docker-registry', common['docker_registry'], '--rosdistro-path', '/rosdistro',
                           '--timestamp', self.timestamp, '--publish', privileged=True, apt=True, aws=True, vault=True)
        if self.config['kind'] != 'final':
            self.container(image, 'cleanup_images', '--release-label', self.label,
                           '--apt-repo', self.apt.removeprefix('s3://'), '--organization', common['organization'],
                           *self.retention(), aws=True)
        self.invalidate(f'/{self.label}/images/index')

    def cleanup(self):
        images = [self.parent, f'tailor-image:{self.config["run"]}'] + [
            f'tailor-package:{version}-{self.config["run"]}' for version in self.config['os']]
        for image in images:
            subprocess.run(['docker', 'image', 'rm', image], check=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage', choices=('plan', 'cleanup') + STAGES)
    parser.add_argument('--plan', type=Path, default=Path('build-plan.json'))
    parser.add_argument('--mode', choices=MODES, default='packages')
    parser.add_argument('--ref', default=os.environ.get('GITHUB_REF', 'refs/heads/master'))
    parser.add_argument('--force-mirror', action='store_true')
    parser.add_argument('--rebuild', action='store_true')
    parser.add_argument('--no-cache', action='store_true')
    args = parser.parse_args()
    if args.stage == 'plan':
        if args.plan.exists():
            raise ValueError('A plan already exists; do not change identity mid-build')
        save_plan(args.plan, plan(Path('rosdistro'), args.ref, args.mode))
        return
    config = json.loads(args.plan.read_text())
    build = Build(config, args.force_mirror, args.rebuild, args.no_cache)
    if args.stage == 'cleanup':
        build.cleanup()
        return
    if config_digest(Path('rosdistro')) != config['config_digest']:
        raise ValueError('Configuration changed after planning')
    if revision(Path(__file__).resolve().parents[1]) != config['implementation_sha']:
        raise ValueError('Implementation changed after planning')
    build.execute(args.stage)
    save_plan(args.plan, config)
    if config['mode'] == 'release' and args.stage in ('publish', 'images'):
        build.run('aws', 's3', 'cp', args.plan, f'{build.apt}/{build.label}/build-plan.json')


if __name__ == '__main__':
    main()
