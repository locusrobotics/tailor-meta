"""GitHub Actions planning and orchestration for the Tailor buildfarm.

Builder libraries remain versioned independently; CI policy lives here.
"""
import argparse
import datetime
import json
import os
import pathlib
import re
import subprocess

import yaml


def release(ref):
    if ref.startswith('refs/tags/'):
        label = ref.removeprefix('refs/tags/')
        if not re.fullmatch(r'\d+\.\d+\.\d+(?:[.+-][a-z0-9.+-]+)?', label):
            raise ValueError('Release tags must have a numeric major.minor.patch')
        track = '.'.join(label.split('.')[:2])
        kind = 'final'
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


def plan(root, ref):
    recipes = yaml.safe_load((root / 'config/recipes.yaml').read_text())
    images = yaml.safe_load((root / 'config/images.yaml').read_text())['images']
    targets = [version for versions in recipes['os'].values() for version in versions]
    result = release(ref)
    result.update(os=targets, common=recipes['common'], images=[
        dict(name=name, os=version, flavour=config.get('bundle_flavour', recipes['common']['testing_flavour']))
        for name, config in images.items() for version in targets
        if version in config.get('os_versions', targets)
    ])
    result['timestamp'] = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%d.%H%M%S')
    return result


class Build:
    def __init__(self, config, deploy=False, force_mirror=False, rebuild=False, no_cache=False, graph_only=False):
        self.config = config
        self.deploy = deploy
        self.force_mirror = force_mirror
        self.rebuild = rebuild
        self.no_cache = no_cache
        self.graph_only = graph_only
        self.root = pathlib.Path.cwd()
        self.label = config['label']
        self.timestamp = config['timestamp']
        self.apt = config['common']['apt_repo']
        self.parent = 'tailor-parent:' + os.environ.get('GITHUB_RUN_ID', 'local')
        self.secret_dir = pathlib.Path(os.environ['RUNNER_TEMP']) / 'tailor-secrets'

    def run(self, *args):
        subprocess.run([str(arg) for arg in args], check=True)

    def container(self, image, *command, privileged=False):
        args = ['docker', 'run', '--rm', '--user', '0', '-v', f'{self.root}:/work', '-w', '/work',
                '-v', f'{self.secret_dir}/s3auth.conf:/etc/apt/s3auth.conf:ro',
                '-v', f'{self.secret_dir}/vault:/home/tailor/.vault_pass.txt:ro',
                '-v', f'{os.environ["TAILOR_GPG_DIR"]}:/gpg:ro']
        for name in ('AWS_ACCESS_KEY_ID', 'AWS_SECRET_ACCESS_KEY', 'AWS_SESSION_TOKEN',
                     'AWS_DEFAULT_REGION', 'GITHUB_TOKEN'):
            args += ['-e', name]
        if privileged:
            args += ['--privileged', '-v', '/var/run/docker.sock:/var/run/docker.sock',
                     '-v', '/dev:/dev', '-v', '/lib/modules:/lib/modules:ro', '-v', '/boot:/boot:ro']
        self.run(*args, image, *command)

    def docker_build(self, image, dockerfile, context='.', **build_args):
        source = pathlib.Path(dockerfile)
        if source.name != 'Dockerfile-secure':
            secured = source.with_name('Dockerfile-secure')
            secured.write_text(secure_dockerfile(source.read_text()))
            dockerfile = str(secured)
        args = ['docker', 'build', '-t', image, '-f', dockerfile]
        if self.no_cache:
            args += ['--no-cache']
        for key, value in build_args.items():
            args += ['--build-arg', f'{key}={value}']
        for name in ('AWS_ACCESS_KEY_ID', 'AWS_SECRET_ACCESS_KEY', 'AWS_SESSION_TOKEN'):
            if os.environ.get(name):
                args += ['--secret', f'id={name},env={name}']
        args += ['--secret', f'id=s3auth,src={self.secret_dir}/s3auth.conf', context]
        self.run(*args)

    def execute(self):
        if self.config['kind'] == 'trivial':
            return
        self.secret_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
        auth = '\n'.join(f'{key} = {value}' for key, value in (
            ('AccessKeyId', os.environ.get('AWS_ACCESS_KEY_ID', '')),
            ('SecretAccessKey', os.environ.get('AWS_SECRET_ACCESS_KEY', '')),
            ('Token', os.environ.get('AWS_SESSION_TOKEN', '')),
            ('Region', os.environ.get('AWS_DEFAULT_REGION', 'us-east-1'))))
        (self.secret_dir / 's3auth.conf').write_text(auth)
        (self.secret_dir / 'vault').write_text(os.environ.get('ANSIBLE_VAULT_PASS', ''))
        for file in self.secret_dir.iterdir():
            file.chmod(0o600)
        try:
            self.distribution()
            if self.deploy and not self.graph_only:
                self.images()
        finally:
            for file in self.secret_dir.iterdir():
                file.unlink()

    def distribution(self):
        recipes = 'rosdistro/config/recipes.yaml'
        self.docker_build(self.parent, 'tailor-distro/environment/Dockerfile', AWS_REGION='us-east-1',
                          APT_REFRESH_KEY=datetime.date.today().strftime('%G-W%V'))
        self.container(self.parent, 'create_recipes', '--recipes', recipes, '--recipes-dir', 'workspace/recipes',
                       '--release-track', self.config['track'], '--release-label', self.label,
                       '--debian-version', self.timestamp)
        # Token is expanded inside the container, never added to the host command line.
        self.container(self.parent, 'bash', '-ec',
                       'pull_distro_repositories --src-dir workspace/src --github-key "$GITHUB_TOKEN" '
                       '--recipes rosdistro/config/recipes.yaml --rosdistro-index rosdistro/rosdistro/index.yaml --clean')
        if not self.graph_only:
            for version in self.config['os']:
                self.container(self.parent, 'bash', '-ec',
                               'mirror_upstream "$@" --keys /gpg/*.key', 'mirror', 'rosdistro/config/upstream.yaml',
                               '--version', self.timestamp, '--apt-repo', self.apt, '--release-label', self.label,
                               '--distribution', version, *(['--force-mirror'] if self.force_mirror else []),
                               *(['--publish'] if self.deploy else []))
        self.container(self.parent, 'generate_graphs', '--recipe', recipes, '--release-label', self.label,
                       '--package-release-label', self.label, '--timestamp', self.timestamp,
                       '--workspace', 'workspace', '--apt-configs', '/etc/apt/s3auth.conf')
        if self.graph_only:
            return
        self.run('cp', '-a', 'tailor-distro', 'workspace/tailor-distro')
        for version in self.config['os']:
            graph = f'workspace/graphs/ubuntu-{version}-graph.yaml'
            self.container(self.parent, 'get_dependency_list', '--graph', graph, '--recipe', recipes,
                           '--workspace', 'workspace')
            dependencies = pathlib.Path(f'workspace/dependencies/dev-{version}-{self.label}-dependencies.txt')
            dockerfile = pathlib.Path(f'workspace/debian/Dockerfile-{version}')
            image = 'tailor-package:' + version + '-' + os.environ.get('GITHUB_RUN_ID', 'local')
            self.docker_build(image, str(dockerfile), 'workspace',
                              UNION_BUILD_DEPENDS=' '.join(sorted(set(dependencies.read_text().splitlines()))),
                              UNION_RUN_DEPENDS='', ORGANIZATION=self.config['common']['organization'],
                              APT_REFRESH_KEY=datetime.date.today().strftime('%G-W%V'))
            for distro in ('ros1', 'ros2'):
                self.container(image, 'build_packages', '--graph', graph, '--workspace', 'workspace',
                               '--recipe', recipes, '--ros-distro', distro,
                               '--build-report', f'build-report-{version}-{distro}.yaml',
                               *(['--rebuild-all'] if self.rebuild else []))
            self.container(image, 'build_bundles', '--graph', graph, '--recipe', recipes, '--workspace', 'workspace',
                           *(['--rebuild-all'] if self.rebuild else []))
            if self.deploy:
                self.container(self.parent, 'bash', '-ec',
                               'publish_packages *.deb "$@" --keys /gpg/*.key', 'publish',
                               '--release-label', self.label, '--apt-repo', self.apt, '--distribution', version,
                               *self.retention())
            # Isolate per-OS package output; never publish the previous OS's debs.
            destination = pathlib.Path('packages') / version
            destination.mkdir(parents=True, exist_ok=True)
            for deb in pathlib.Path('.').glob('*.deb'):
                deb.rename(destination / deb.name)
        if self.deploy:
            self.container(self.parent, 's3_directory_cleanup', '--release-label', self.label,
                           '--apt-repo', self.apt.removeprefix('s3://'), *self.retention())
            for version in self.config['os']:
                self.container(self.parent, 's3_versions_cleanup', '--release-label', self.label,
                               '--apt-repo', self.apt.removeprefix('s3://'), '--distribution', version)
            self.invalidate(f'/{self.label}/ubuntu/dists/*')

    def retention(self):
        return ['--num-to-keep', '30'] + (['--days-to-keep', str(self.config['days'])] if self.config['days'] else [])

    def invalidate(self, path):
        distribution = self.config['common'].get('cloudfront_distribution_id')
        if distribution:
            self.run('aws', 'cloudfront', 'create-invalidation', '--distribution-id', distribution, '--paths', path)

    def images(self):
        common = self.config['common']
        image = 'tailor-image:' + os.environ.get('GITHUB_RUN_ID', 'local')
        self.docker_build(image, 'tailor-image/environment/Dockerfile', APT_REPO=self.apt, APT_REGION='us-east-1',
                          RELEASE_LABEL=self.label, ORGANIZATION=common['organization'],
                          FLAVOUR=common['build_flavour'],
                          BUNDLE_ROOT=f'/opt/{common["organization"]}/{self.label}',
                          APT_REFRESH_KEY=self.timestamp)
        for target in self.config['images']:
            self.container(image, 'create_image', '--name', target['name'], '--distribution', target['os'],
                           '--apt-repo', self.apt.removeprefix('s3://'), '--release-label', self.label,
                           '--flavour', target['flavour'], '--organization', common['organization'],
                           '--docker-registry', common['docker_registry'], '--rosdistro-path', '/rosdistro',
                           '--timestamp', self.timestamp, '--publish', privileged=True)
        self.container(image, 'cleanup_images', '--release-label', self.label,
                       '--apt-repo', self.apt.removeprefix('s3://'), '--organization', common['organization'],
                       *self.retention())
        self.invalidate(f'/{self.label}/images/index')


def secure_dockerfile(text):
    """Adapt pinned legacy Dockerfiles without storing credentials in layers.

    All RUN instructions receive ephemeral auth mounts. Runtime auth is mounted
    independently by container(). No credential-bearing image is pushed.
    """
    text = re.sub(r'^ARG (?:AWS_ACCESS_KEY_ID|AWS_SECRET_ACCESS_KEY|GITHUB_TOKEN|ANSIBLE_VAULT_PASS)\n', '', text,
                  flags=re.MULTILINE)
    text = re.sub(r'^ENV AWS_ACCESS_KEY_ID=.*?\n\n', '', text, flags=re.MULTILINE | re.DOTALL)
    text = re.sub(r' \\\n\s*(?:AWS_ACCESS_KEY_ID|AWS_SECRET_ACCESS_KEY|GITHUB_TOKEN)=\$\{[^}]+\}', '', text)
    text = re.sub(r'^RUN echo "AccessKeyId = .*?(?=\n\n)', '', text, flags=re.MULTILINE | re.DOTALL)
    text = re.sub(r'^RUN git config --global url\..*\n?', '', text, flags=re.MULTILINE)
    text = re.sub(r'^RUN echo "\$\{ANSIBLE_VAULT_PASS\}".*', '', text, flags=re.MULTILINE | re.DOTALL)
    mount = ('RUN --mount=type=secret,id=s3auth,target=/etc/apt/s3auth.conf '
             '--mount=type=secret,id=AWS_ACCESS_KEY_ID '
             '--mount=type=secret,id=AWS_SECRET_ACCESS_KEY '
             '--mount=type=secret,id=AWS_SESSION_TOKEN ')
    setup = ('export AWS_ACCESS_KEY_ID="$(cat /run/secrets/AWS_ACCESS_KEY_ID 2>/dev/null || true)" '
             'AWS_SECRET_ACCESS_KEY="$(cat /run/secrets/AWS_SECRET_ACCESS_KEY 2>/dev/null || true)" '
             'AWS_SESSION_TOKEN="$(cat /run/secrets/AWS_SESSION_TOKEN 2>/dev/null || true)"; ')
    return '# syntax=docker/dockerfile:1\n' + re.sub(r'^RUN ', mount + setup, text, flags=re.MULTILINE)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['plan', 'build'])
    parser.add_argument('--ref', default=os.environ.get('GITHUB_REF', 'refs/heads/master'))
    parser.add_argument('--deploy', action='store_true')
    parser.add_argument('--force-mirror', action='store_true')
    parser.add_argument('--rebuild', action='store_true')
    parser.add_argument('--no-cache', action='store_true')
    parser.add_argument('--graph-only', action='store_true')
    args = parser.parse_args()
    config = plan(pathlib.Path('rosdistro'), args.ref)
    if args.command == 'plan':
        print(json.dumps(config))
    else:
        Build(config, args.deploy, args.force_mirror, args.rebuild, args.no_cache, args.graph_only).execute()


if __name__ == '__main__':
    main()
