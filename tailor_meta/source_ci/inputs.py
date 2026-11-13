"""Input validation and safe, non-executing source/archive inspection."""
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import stat
import xml.etree.ElementTree as ET
import zipfile


def identifier(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def extract_source(archive, destination):
    destination.mkdir(parents=True)
    total = 0
    with zipfile.ZipFile(archive) as source:
        entries = source.infolist()
        if len(entries) > 100000:
            raise ValueError('Too many files in source archive')
        roots = {PurePosixPath(item.filename).parts[0] for item in entries if item.filename}
        if len(roots) != 1:
            raise ValueError('Expected one GitHub archive root')
        for item in entries:
            path = PurePosixPath(item.filename)
            if path.is_absolute() or '..' in path.parts or '\\' in item.filename:
                raise ValueError('Unsafe source archive path')
            mode = item.external_attr >> 16
            if stat.S_ISLNK(mode):
                raise ValueError('Source symlinks are not supported by the initial isolated pilot')
            relative = Path(*path.parts[1:])
            if relative == Path('.'):
                continue
            target = destination / relative
            total += item.file_size
            if total > 2 * 1024 ** 3:
                raise ValueError('Expanded source archive exceeds pilot size limit')
            if item.is_dir():
                target.mkdir(parents=True, exist_ok=True)
            else:
                target.parent.mkdir(parents=True, exist_ok=True)
                with source.open(item) as incoming, target.open('xb') as outgoing:
                    import shutil
                    shutil.copyfileobj(incoming, outgoing)
                target.chmod(0o755 if mode & 0o111 else 0o644)


def package_names(root):
    names = set()
    for path in root.rglob('package.xml'):
        content = path.read_bytes()
        if len(content) > 1024 * 1024 or re.search(br'<!\s*(DOCTYPE|ENTITY)', content, re.I):
            raise ValueError('Unsafe package manifest')
        name = ET.fromstring(content).findtext('name')
        if not name or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', name):
            raise ValueError('Invalid ROS package name')
        names.add(name)
    return names


def check_removed(source, baseline, graph, distro):
    removed = package_names(baseline) - package_names(source)
    own = package_names(baseline) | package_names(source)
    for namespace, packages in graph['packages'].items():
        for name, package in packages.items():
            if namespace == distro and name in own:
                continue
            dependencies = []
            if namespace == distro:
                dependencies += [dep.split(':', 1)[-1] for dep in package.get('source_depends', [])]
            if distro == 'ros1' and namespace == 'ros2':
                dependencies += package.get('ros1_depends', [])
            if removed.intersection(dependencies):
                raise ValueError(f'{namespace}/{name} still depends on removed packages: '
                                 + ', '.join(sorted(removed.intersection(dependencies))))


def no_image_credentials(image):
    denied = ('AWS_ACCESS_KEY_ID=', 'AWS_SECRET_ACCESS_KEY=', 'AWS_SESSION_TOKEN=', 'AWS_WEB_IDENTITY_TOKEN_FILE=',
              'GITHUB_TOKEN=', 'GH_TOKEN=', 'ANSIBLE_VAULT_PASS=')
    for entry in image.get('Config', {}).get('Env', []):
        if entry.startswith(denied) and entry.partition('=')[2]:
            raise ValueError('Test image contains credential-bearing environment variables; rebuild it')


def credential_file(name, content):
    if (name.endswith(('/.vault_pass.txt', '/.git-tokens', '/.git-credentials',
                       '/.ssh/id_rsa', '/.ssh/id_ed25519', '/.ssh/id_ecdsa'))
            or '/.gnupg/private-keys-v1.d/' in name or '/.aws/sso/cache/' in name) and content.strip():
        raise ValueError('Test image contains a credential file; rebuild it')
    if name.endswith('/.docker/config.json'):
        value = json.loads(content)
        if any(auth.get('auth') or auth.get('identitytoken') for auth in value.get('auths', {}).values()):
            raise ValueError('Test image contains saved Docker credentials; rebuild it')
    keys = {'accesskeyid', 'secretaccesskey', 'token', 'aws_access_key_id', 'aws_secret_access_key', 'aws_session_token'}
    for line in content.splitlines():
        key, separator, value = line.partition('=')
        if separator and key.strip().lower() in keys and value.strip().strip('\'"'):
            raise ValueError('Test image contains baked AWS credentials; rebuild it')
    if re.search(r'https://[^\s/@]+:[^\s/@]+@|gh[pousr]_[A-Za-z0-9]+|github_pat_', content):
        raise ValueError('Test image contains baked GitHub credentials; rebuild it')
