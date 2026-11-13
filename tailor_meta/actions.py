"""Persisted build plans and individually runnable Tailor Actions stages."""
import argparse
from contextlib import contextmanager
import datetime
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile

import yaml

from . import toolchains


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


def plan(root, ref, mode='packages', images=None, allow_local=False):
    if mode not in MODES:
        raise ValueError(f'Unsupported mode: {mode}')
    recipes = yaml.safe_load((root / 'config/recipes.yaml').read_text())
    definitions = yaml.safe_load((root / 'config/images.yaml').read_text()).get('images') or {}
    if set(recipes['os']) != {'ubuntu'}:
        raise ValueError('The packaging environment currently supports Ubuntu only')
    targets = recipes['os']['ubuntu']
    common = recipes['common']
    result = release(ref)
    if result['kind'] == 'trivial' and mode != 'validate':
        raise ValueError('Builds require master, feature/*, release/*, or a release tag')
    result.update(schema=1, mode=mode, ref=ref, os=targets, common=common, images=[
        dict(name=name, os=version, flavour=config.get('bundle_flavour', common['testing_flavour']))
        for name, config in definitions.items() for version in targets
        if version in config.get('os_versions', targets)
    ])
    result.update(timestamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%d.%H%M%S'),
                  config_sha=revision(root), implementation_sha=revision(Path(__file__).resolve().parents[1]),
                  config_digest=config_digest(root),
                  run=os.environ.get('GITHUB_RUN_ID', 'local') + '-' + os.environ.get('GITHUB_RUN_ATTEMPT', '1'),
                  repository=os.environ.get('GITHUB_REPOSITORY', 'local'), sources={}, completed=[])
    roles = ['tools']
    if mode in ('packages', 'release'):
        roles += ['builder-' + version for version in targets]
    if mode == 'release' and result['images']:
        roles.append('imager')
    if mode != 'validate':
        result['toolchains'] = images or toolchains.resolve(result['implementation_sha'], roles, result['repository'])
        for role in roles:
            toolchains.verify(result['toolchains'][role], result['implementation_sha'], allow_local, role)
    else:
        result['toolchains'] = images or {}
    result['local_toolchains'] = allow_local
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
                                       ('repository', 'run', 'config_sha', 'implementation_sha', 'timestamp')}
                                      | {'toolchains': config.get('toolchains', {})}).encode(),
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
    def __init__(self, config, force_mirror=False, rebuild=False):
        self.config = config
        self.force_mirror, self.rebuild = force_mirror, rebuild
        self.root = Path.cwd()
        self.label, self.timestamp = config['label'], config['timestamp']
        self.apt = config['common']['apt_repo']
        self.secret_dir = None

    def image(self, role):
        image = self.config['toolchains'][role]
        toolchains.verify(image, self.config['implementation_sha'], self.config.get('local_toolchains', False), role)
        return image

    def run(self, *args):
        subprocess.run([str(arg) for arg in args], check=True)

    @contextmanager
    def credentials(self):
        # Per-stage temporary files, never included in the Docker context.
        with tempfile.TemporaryDirectory(prefix='tailor-auth-', dir=os.environ.get('RUNNER_TEMP')) as directory:
            self.secret_dir = Path(directory)
            values = [('AccessKeyId', 'AWS_ACCESS_KEY_ID'), ('SecretAccessKey', 'AWS_SECRET_ACCESS_KEY'),
                      ('Token', 'AWS_SESSION_TOKEN')]
            region = self.config['common'].get('apt_region', os.environ.get('AWS_DEFAULT_REGION', 'us-east-1'))
            for name, text in (
                ('s3auth', '\n'.join(f'{apt} = {os.environ.get(env, "")}' for apt, env in values) + f'\nRegion = {region}\n'),
                ('vault', os.environ.get('ANSIBLE_VAULT_PASS', '')),
                ('gitconfig', '[url "https://x-access-token:' + os.environ.get('GITHUB_TOKEN', '') +
                 '@github.com/"]\n\tinsteadOf = ssh://git@github.com/\n\tinsteadOf = git@github.com:\n'),
                ('rosdep-list', 'yaml file:///etc/ros/rosdep/rosdep.yaml\n'),
            ):
                (self.secret_dir / name).write_text(text)
                (self.secret_dir / name).chmod(0o600)
            try:
                yield
            finally:
                self.secret_dir = None

    def container(self, image, *command, apt=False, aws=False, github=False, git_auth=False, rosdep=False,
                  signing=False, vault=False, privileged=False, workdir='/work'):
        args = ['docker', 'run', '--rm', '--user', '0', '-e', 'HOME=/home/tailor',
                '-v', f'{self.root}:/work', '-w', workdir]
        if apt:
            args += ['-v', f'{self.secret_dir}/s3auth:/etc/apt/s3auth.conf:ro']
        if aws:
            for name in ('AWS_ACCESS_KEY_ID', 'AWS_SECRET_ACCESS_KEY', 'AWS_SESSION_TOKEN'):
                args += ['-e', name]
            args += ['-e', f'AWS_DEFAULT_REGION={self.config["common"].get("apt_region", "us-east-1")}']
        if signing:
            args += ['-v', f'{os.environ["TAILOR_GPG_DIR"]}:/gpg:ro']
        if vault:
            args += ['-v', f'{self.secret_dir}/vault:/home/tailor/.vault_pass.txt:ro']
        if github:
            args += ['-e', 'GITHUB_TOKEN']
        if git_auth:
            args += ['-v', f'{self.secret_dir}/gitconfig:/root/.gitconfig:ro',
                     '-v', f'{self.secret_dir}/gitconfig:/home/tailor/.gitconfig:ro']
        if rosdep:
            args += ['-v', f'{self.root}/rosdistro/rosdep/rosdep.yaml:/etc/ros/rosdep/rosdep.yaml:ro',
                     '-v', f'{self.secret_dir}/rosdep-list:/etc/ros/rosdep/sources.list.d/10-tailor.list:ro']
        if privileged:
            args += ['--privileged', '-v', '/var/run/docker.sock:/var/run/docker.sock',
                     '-v', '/dev:/dev', '-v', '/lib/modules:/lib/modules:ro', '-v', '/boot:/boot:ro']
        self.run(*args, image, *command)

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
        image = self.image('tools')
        self.container(image, 'create_recipes', '--recipes', 'rosdistro/config/recipes.yaml',
                       '--recipes-dir', 'workspace/recipes', '--release-track', self.config['track'],
                       '--release-label', self.label, '--debian-version', self.timestamp)
        self.container(image, 'bash', '-ec',
                       'pull_distro_repositories --src-dir workspace/src --github-key "$GITHUB_TOKEN" '
                       '--recipes rosdistro/config/recipes.yaml --rosdistro-index rosdistro/rosdistro/index.yaml --clean',
                       github=True)
        self.config['sources'] = {
            str(path.relative_to('workspace/src')): [json.loads(line) for line in path.read_text().splitlines() if line]
            for path in Path('workspace/src').rglob('*_repositories_data.jsonl')
        }

    def mirror(self):
        for version in self.config['os']:
            self.container(self.image('tools'), 'bash', '-ec', 'mirror_upstream "$@" --keys /gpg/*.key', 'mirror',
                           'rosdistro/config/upstream.yaml', '--version', self.timestamp, '--apt-repo', self.apt,
                           '--release-label', self.label, '--distribution', version, '--publish',
                           *(['--force-mirror'] if self.force_mirror else []), aws=True, signing=True)

    def graph(self):
        self.container(self.image('tools'), 'bash', '-ec', 'rosdep update; exec generate_graphs "$@"', 'graphs',
                       '--recipe', 'rosdistro/config/recipes.yaml',
                       '--release-label', self.label, '--package-release-label', self.label,
                       '--timestamp', self.timestamp, '--workspace', 'workspace',
                       '--apt-configs', '/etc/apt/s3auth.conf', apt=True, aws=True, rosdep=True)

    def packages(self):
        recipes = 'rosdistro/config/recipes.yaml'
        for version in self.config['os']:
            graph = f'workspace/graphs/ubuntu-{version}-graph.yaml'
            self.container(self.image('tools'), 'get_dependency_list', '--graph', graph, '--recipe', recipes,
                           '--workspace', 'workspace', apt=True)
            self.build_os(version)

    def build_os(self, version):
        if self.config['mode'] not in ('packages', 'release') or 'graph' not in self.config['completed']:
            raise ValueError('build-os requires package/release mode and resolved graphs')
        if version not in self.config['os']:
            raise ValueError(f'{version} is not in the build plan')
        if version in self.config.get('packages', {}):
            raise ValueError(f'{version} packages already built; refusing accidental replay')
        self.container(self.image('builder-' + version), 'tailor-build-os', '--plan', '/work/build-plan.json',
                       '--os', version, *(['--rebuild'] if self.rebuild else []),
                       apt=bool(self.apt), aws=bool(self.apt), git_auth=bool(os.environ.get('GITHUB_TOKEN')))
        checksums = Path('builds') / version / 'package-checksums.json'
        self.config.setdefault('packages', {})[version] = json.loads(checksums.read_text())

    def publish(self):
        # Every OS completed before this stage. Validate the package snapshot
        # before mutating any APT repository.
        for version in self.config['os']:
            actual = {p.name: file_digest(p)
                      for p in (Path('packages') / version).glob('*.deb')}
            if actual != self.config['packages'][version]:
                raise ValueError(f'Package snapshot changed for {version}')
        for version in self.config['os']:
            self.container(self.image('tools'), 'bash', '-ec',
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
        self.run('aws', 's3', 'sync', 'workspace/graphs', f'{self.apt}/{self.label}/graphs/{self.timestamp}',
                 '--exclude', '*', '--include', '*.yaml')
        if self.config['kind'] != 'final':
            self.container(self.image('tools'), 's3_directory_cleanup', '--release-label', self.label,
                           '--apt-repo', self.apt.removeprefix('s3://'), *self.retention(), aws=True)
            for version in self.config['os']:
                self.container(self.image('tools'), 's3_versions_cleanup', '--release-label', self.label,
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
        self.container(self.image('imager'), 'python3', '-m', 'tailor_meta.build_images',
                       '--plan', '/work/build-plan.json', privileged=True, apt=True, aws=True, vault=True)
        # Source CI must not combine an old build plan with a newly moved test
        # image tag while a later feature build is publishing.
        import boto3
        registry = self.config['common']['docker_registry'].removeprefix('https://')
        endpoint, repository = registry.split('/', 1)
        client = boto3.client('ecr', region_name=self.config['common'].get('apt_region', 'us-east-1'))
        self.config['test_images'] = {}
        for target in self.config['images']:
            if target['name'] != 'test':
                continue
            image = client.describe_images(registryId=endpoint.split('.')[0], repositoryName=repository,
                                           imageIds=[{'imageTag': f'tailor-image-test-{target["os"]}-{self.label}'}])
            self.config['test_images'][target['os']] = registry + '@' + image['imageDetails'][0]['imageDigest']
        self.invalidate(f'/{self.label}/images/index')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage', choices=('plan', 'build-os') + STAGES)
    parser.add_argument('--plan', type=Path, default=Path('build-plan.json'))
    parser.add_argument('--mode', choices=MODES, default='packages')
    parser.add_argument('--ref', default=os.environ.get('GITHUB_REF', 'refs/heads/master'))
    parser.add_argument('--force-mirror', action='store_true')
    parser.add_argument('--rebuild', action='store_true')
    parser.add_argument('--os')
    parser.add_argument('--toolchains', type=Path, help='Pre-resolved digest map; normally resolved from GHCR')
    parser.add_argument('--allow-local-toolchains', action='store_true', help='CI fixtures only')
    args = parser.parse_args()
    if args.stage == 'plan':
        if args.plan.exists():
            raise ValueError('A plan already exists; do not change identity mid-build')
        images = json.loads(args.toolchains.read_text()) if args.toolchains else None
        save_plan(args.plan, plan(Path('rosdistro'), args.ref, args.mode, images, args.allow_local_toolchains))
        return
    config = json.loads(args.plan.read_text())
    build = Build(config, args.force_mirror, args.rebuild)
    if config_digest(Path('rosdistro')) != config['config_digest']:
        raise ValueError('Configuration changed after planning')
    if revision(Path(__file__).resolve().parents[1]) != config['implementation_sha']:
        raise ValueError('Implementation changed after planning')
    if args.stage == 'build-os':
        with build.credentials():
            build.build_os(args.os)
        if all(version in config['packages'] for version in config['os']):
            config['completed'].append('packages')
    else:
        build.execute(args.stage)
    save_plan(args.plan, config)
    if config['mode'] == 'release' and args.stage in ('publish', 'images'):
        build.run('aws', 's3', 'cp', args.plan, f'{build.apt}/{build.label}/build-plan.json')


if __name__ == '__main__':
    main()
