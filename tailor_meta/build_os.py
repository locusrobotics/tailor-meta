"""The same per-OS package build command for production and smoke fixtures."""
import argparse
import json
import os
from pathlib import Path
import subprocess

import yaml

from .actions import file_digest


def run(*command):
    subprocess.run([str(arg) for arg in command], check=True)


def configure_apt(config, os_version):
    apt_repo = config['common'].get('apt_repo')
    if not apt_repo:
        return  # Credential-free fixture with only public/system dependencies.
    sources = [f'deb [arch=amd64] {apt_repo}/{config["label"]}/ubuntu {os_version}-mirror {os_version}']
    # A newly bootstrapped label may not yet have its main package repository.
    import boto3
    from botocore.exceptions import ClientError
    try:
        boto3.client('s3').head_object(Bucket=apt_repo.removeprefix('s3://'),
                                     Key=f'{config["label"]}/ubuntu/dists/{os_version}/InRelease')
    except ClientError as error:
        if error.response['Error']['Code'] not in ('404', 'NoSuchKey', 'NotFound'):
            raise
    else:
        sources.append(f'deb [arch=amd64] {apt_repo}/{config["label"]}/ubuntu {os_version} main')
    Path('/etc/apt/sources.list.d/tailor.list').write_text('\n'.join(sources) + '\n')


def build_os(config, os_version, root=Path('/work'), rebuild=False):
    if os_version not in config['os']:
        raise ValueError(f'{os_version} is not in the build plan')
    work = root / 'builds' / os_version
    (work / 'workspace').mkdir(parents=True, exist_ok=True)
    src = work / 'workspace/src'
    src.symlink_to(root / 'workspace/src', target_is_directory=True)
    graph = root / 'workspace/graphs' / f'ubuntu-{os_version}-graph.yaml'
    recipe = root / 'rosdistro/config/recipes.yaml'
    os.chdir(work)
    os.environ['CCACHE_DIR'] = str(work / 'ccache')
    Path(os.environ['CCACHE_DIR']).mkdir()
    configure_apt(config, os_version)
    dependencies = root / 'workspace/dependencies' / f'all-{os_version}-{config["label"]}-dependencies.txt'
    packages = dependencies.read_text().splitlines()
    if packages:
        run('apt-get', 'update')
        run('apt-get', 'install', '-y', '--no-install-recommends', *packages)
    flags = ['--rebuild-all'] if rebuild else []
    for distro in ('ros1', 'ros2'):
        run('build_packages', '--graph', graph, '--workspace', 'workspace', '--recipe', recipe,
            '--ros-distro', distro, '--build-report', f'build-report-{os_version}-{distro}.yaml', *flags)
    run('build_bundles', '--graph', graph, '--workspace', 'workspace', '--recipe', recipe, *flags)
    destination = root / 'packages' / os_version
    destination.mkdir(parents=True, exist_ok=True)
    for deb in work.glob('*.deb'):
        deb.rename(destination / deb.name)
    (work / 'package-checksums.json').write_text(json.dumps({
        path.name: file_digest(path) for path in destination.glob('*.deb')}, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--os', required=True)
    parser.add_argument('--rebuild', action='store_true')
    args = parser.parse_args()
    config = json.loads(args.plan.read_text())
    build_os(config, args.os, rebuild=args.rebuild)


if __name__ == '__main__':
    main()
