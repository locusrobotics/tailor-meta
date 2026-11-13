"""Prepare published build tools and provision all targets in one imager container."""
import argparse
import json
import os
import platform
from pathlib import Path

from tailor_image.create_image import create_image

from .build_os import configure_apt, run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    config = json.loads(parser.parse_args().plan.read_text())
    common = config['common']
    os.environ['RTI_NC_LICENSE_ACCEPTED'] = 'yes'
    configure_apt(config, platform.freedesktop_os_release()['VERSION_CODENAME'])
    run('apt-get', 'update')
    run('apt-get', 'install', '-y', '--no-install-recommends',
        f'{common["organization"]}-{common["build_flavour"]}-{config["label"]}')
    os.environ.update(BUNDLE_ROOT=f'/opt/{common["organization"]}/{config["label"]}', TAILOR_RUN=config['run'])
    for target in config['images']:
        os.chdir('/work')
        create_image(target['name'], target['os'], common['apt_repo'].removeprefix('s3://'), config['label'],
                     target['flavour'], common['organization'], common['docker_registry'], Path('/work/rosdistro'),
                     config['timestamp'], publish=True)
    if config['kind'] != 'final':
        os.chdir('/work')
        run('cleanup_images', '--release-label', config['label'], '--apt-repo', common['apt_repo'].removeprefix('s3://'),
            '--organization', common['organization'], '--num-to-keep', '30',
            *(['--days-to-keep', str(config['days'])] if config['days'] else []))


if __name__ == '__main__':
    main()
