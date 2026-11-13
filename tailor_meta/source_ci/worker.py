"""Uncredentialed hosted worker. No GitHub/AWS clients are imported here."""
import argparse
import json
import os
from pathlib import Path
import re
import secrets
import shutil
import subprocess

from tailor_meta.actions import file_digest
from .inputs import extract_source


SCRIPT = r'''
set -euo pipefail
# Keep rosdep's prepared cache but put all writable homes inside the container.
cache=$(python3 -c 'from rosdep2.sources_list import get_sources_cache_dir; print(get_sources_cache_dir())')
mkdir -p /tmp/tailor-home/.ros/rosdep
if [[ -d "$cache" ]]; then cp -a "$cache" /tmp/tailor-home/.ros/rosdep/sources.cache; fi
export HOME=/tmp/tailor-home ROS_HOME=/tmp/tailor-home/.ros CCACHE_DIR=/tmp/tailor-ccache
export COLCON_HOME=/tmp/tailor-colcon
setup="${BUNDLE_ROOT:?Published image must define BUNDLE_ROOT}/$ROS_DISTRO_NAME/setup.bash"
# Generated ROS/colcon setup scripts are not nounset-safe.
set +u
source "$setup"
set -u
export ROS_PACKAGE_PATH="/source:${ROS_PACKAGE_PATH:-}"
# No installation or downloads here. Missing dependencies fail the test and
# require a refreshed published environment, never fleet credentials.
rosdep check --from-paths /source --ignore-src
colcon build --merge-install --base-paths /source --test-result-base test_results \
  --cmake-args -DCMAKE_CXX_STANDARD=17 -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
colcon test --merge-install --base-paths /source --test-result-base test_results \
  --executor sequential --event-handlers console_direct+
colcon test-result --test-result-base test_results --verbose
'''


def command(image_id, source, results, distro):
    return ['docker', 'run', '--rm', '--network', 'none', '--cap-drop', 'ALL',
            '--security-opt', 'no-new-privileges', '--pids-limit', '512', '--memory', '4g', '--cpus', '2',
            '--tmpfs', '/tmp:rw,exec,nosuid,size=2g',
            '-v', f'{source}:/source:ro', '-v', f'{results}:/results', '-w', '/results',
            '-e', f'ROS_DISTRO_NAME={distro}', '--entrypoint', '/bin/bash', image_id, '-ec', SCRIPT]


def verify_bundle(bundle):
    checksums = json.loads((bundle / 'checksums.json').read_text())
    actual = {str(path.relative_to(bundle)) for path in bundle.rglob('*') if path.is_file()}
    if actual != set(checksums) | {'checksums.json'}:
        raise ValueError('Prepared input inventory changed')
    for relative, expected in checksums.items():
        path = bundle / relative
        if path.is_symlink() or not path.resolve().is_relative_to(bundle.resolve()) or file_digest(path) != expected:
            raise ValueError('Prepared input checksum/path mismatch')


def collect_reports(results, destination):
    """Never follow source-controlled symlinks when uploading host artifacts."""
    destination.mkdir()
    root = results / 'test_results'
    if not root.is_dir() or root.is_symlink():
        return
    count = 0
    for directory, folders, files in os.walk(root, followlinks=False):
        folders[:] = [name for name in folders if not (Path(directory) / name).is_symlink()]
        for name in files:
            path = Path(directory) / name
            if path.suffix != '.xml' or path.is_symlink() or path.stat().st_size > 10 * 1024 * 1024:
                continue
            count += 1
            if count > 10000:
                raise ValueError('Too many test reports')
            target = destination / path.relative_to(root)
            target.parent.mkdir(parents=True, exist_ok=True)
            with os.fdopen(os.open(path, os.O_RDONLY | os.O_NOFOLLOW), 'rb') as source, target.open('wb') as out:
                shutil.copyfileobj(source, out)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', type=Path, required=True)
    parser.add_argument('--request-id', required=True)
    parser.add_argument('--results', type=Path, required=True)
    parser.add_argument('--reports', type=Path, required=True)
    args = parser.parse_args()
    verify_bundle(args.bundle)
    request = json.loads((args.bundle / 'request.json').read_text())
    if request['id'] != args.request_id:
        raise ValueError('Unexpected prepared source request')
    if not re.fullmatch(r'(?:sha256:)?[a-f0-9]{64}', request['image_id']):
        raise ValueError('Prepared image must have an immutable local identity')
    subprocess.run(['docker', 'load', '-i', str(args.bundle / 'image.tar')], check=True)
    checkout = args.results.with_name(args.results.name + '-source')
    extract_source(args.bundle / 'source.zip', checkout)
    args.results.mkdir()
    args.results.chmod(0o777)  # Dedicated scratch mount; image UID need not match host UID.
    token = secrets.token_hex(32)
    print('::stop-commands::' + token, flush=True)
    try:
        subprocess.run(command(request['image_id'], checkout.resolve(), args.results.resolve(),
                               request['distro']), check=True)
    finally:
        print('::' + token + '::', flush=True)
        collect_reports(args.results, args.reports)


if __name__ == '__main__':
    main()
