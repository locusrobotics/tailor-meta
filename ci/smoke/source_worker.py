"""Exercise the uncredentialed source worker with a known public fixture image."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import zipfile

from tailor_meta.actions import file_digest
from tailor_meta.toolchains import inspect


ISOLATION = '''import os
from pathlib import Path
assert not any(os.environ.get(key) for key in ('AWS_ACCESS_KEY_ID', 'AWS_SECRET_ACCESS_KEY',
                                              'AWS_SESSION_TOKEN', 'GITHUB_TOKEN', 'GH_TOKEN',
                                              'SOURCE_CI_GITHUB_TOKEN', 'ACTIONS_RUNTIME_TOKEN'))
assert not Path('/var/run/docker.sock').exists()
status = dict(line.split(':', 1) for line in Path('/proc/self/status').read_text().splitlines() if ':' in line)
assert int(status['CapEff'].strip(), 16) == 0
assert status['NoNewPrivs'].strip() == '1'
assert not any(line.split()[1] == '00000000' for line in Path('/proc/net/route').read_text().splitlines()[1:])
try:
    Path(__file__).with_name('must-not-write').touch()
except OSError:
    pass
else:
    raise AssertionError('Source mount is writable')
'''


def run(*args):
    subprocess.run([str(arg) for arg in args], check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', required=True)
    parser.add_argument('--output', type=Path, default=Path('output'))
    parser.add_argument('--os', required=True)
    args = parser.parse_args()
    plan = json.loads((args.output / 'build-plan.json').read_text())
    root = Path('source-worker-smoke')
    root.mkdir()
    bundle = root / 'bundle'
    bundle.mkdir()
    shutil.copytree(args.output / 'workspace/src/ros1/smoke_base', root / 'source')
    (root / 'source/isolation.py').write_text(ISOLATION)
    with (root / 'source/CMakeLists.txt').open('a') as source:
        source.write('\nenable_testing()\nadd_test(NAME isolation COMMAND /usr/bin/python3 '
                     '${CMAKE_CURRENT_SOURCE_DIR}/isolation.py)\n')
    with zipfile.ZipFile(bundle / 'source.zip', 'w') as archive:
        for path in (root / 'source').rglob('*'):
            if path.is_file():
                archive.write(path, 'fixture/' + str(path.relative_to(root / 'source')))
    name = 'tailor-isolation-' + args.os
    image = 'tailor-isolation-image:' + args.os
    try:
        # This is public synthetic CI fixture construction, not production input
        # preparation. No AWS role, fleet token, or private input is provided.
        run('docker', 'run', '--name', name, '-v', f'{(args.output / "packages" / args.os).resolve()}:/packages:ro',
            '--entrypoint', '/bin/bash', args.image, '-ec', 'dpkg -i /packages/*.deb; rosdep update')
        run('docker', 'commit', '--change', f'ENV BUNDLE_ROOT=/opt/toydistro/{plan["label"]}', name, image)
        run('docker', 'save', '-o', bundle / 'image.tar', image)
        request = {'id': 'a' * 64, 'image_id': inspect(image)['Id'], 'distro': 'ros1'}
        (bundle / 'request.json').write_text(json.dumps(request))
        checksums = {str(path.relative_to(bundle)): file_digest(path) for path in bundle.rglob('*') if path.is_file()}
        (bundle / 'checksums.json').write_text(json.dumps(checksums))
        run(sys.executable, '-m', 'tailor_meta.source_ci.worker', '--bundle', bundle,
            '--request-id', request['id'], '--results', root / 'results', '--reports', root / 'reports')
    finally:
        subprocess.run(['docker', 'rm', '-f', name], check=False)
        subprocess.run(['docker', 'image', 'rm', image], check=False)


if __name__ == '__main__':
    main()
