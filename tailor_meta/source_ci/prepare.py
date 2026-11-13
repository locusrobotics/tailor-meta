"""Prepare private inputs without starting containers or executing source code."""
import base64
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tarfile
import tempfile

import yaml

from tailor_meta.actions import file_digest
from .inputs import extract_source, no_image_credentials, check_removed, credential_file
from .poll import ecr_location


def inspect_archive(path):
    # Inspect every saved layer, including credentials deleted by later layers.
    # Never extract image files on the trusted runner or execute their contents.
    with tarfile.open(path) as saved:
        manifests = json.load(saved.extractfile('manifest.json'))
        for manifest in manifests:
            config = json.load(saved.extractfile(manifest['Config']))
            no_image_credentials({'Config': config.get('config', {})})
            for entry in config.get('history', []):
                if re.search(r'(?:AWS_ACCESS_KEY_ID|AWS_SECRET_ACCESS_KEY|AWS_SESSION_TOKEN|GITHUB_TOKEN|GH_TOKEN|'
                             r'ANSIBLE_VAULT_PASS)=[^\s]+', entry.get('created_by', '')):
                    raise ValueError('Test image history may contain credentials; rebuild it')
            for layer in manifest['Layers']:
                with tarfile.open(fileobj=saved.extractfile(layer), mode='r|*') as archive:
                    inspect_layer(archive)


def inspect_layer(archive):
    for item in archive:
        name = item.name.lstrip('./')
        sensitive = (name.endswith(('/.aws/credentials', '/.aws/config', '/.gitconfig', '/.git-tokens',
                                    '/.vault_pass.txt', '/.git-credentials', '/.ssh/id_rsa', '/.ssh/id_ed25519',
                                    '/.ssh/id_ecdsa', '/.docker/config.json'))
                     or '/.gnupg/private-keys-v1.d/' in name or '/.aws/sso/cache/' in name
                     or name == 'etc/apt/s3auth.conf')
        if not sensitive or not item.isfile():
            continue
        if item.size > 1024 * 1024:
            raise ValueError('Suspicious authentication file in test image')
        content = archive.extractfile(item).read().decode(errors='replace')
        credential_file(name, content)


def prepare(request, github, s3, ecr, destination):
    destination.mkdir(parents=True)
    auth_parent = Path(os.environ.get('RUNNER_TEMP', tempfile.gettempdir())) / (
        'source-ci-auth-' + os.environ.get('GITHUB_RUN_ID', 'local') + '-' +
        os.environ.get('GITHUB_RUN_ATTEMPT', '1') + '-' + request['id'])
    auth_parent.mkdir(mode=0o700)
    with tempfile.TemporaryDirectory(dir=auth_parent) as temporary:
        work = Path(temporary)
        for key, name in (('sha', 'source'), ('baseline_sha', 'baseline')):
            archive = work / (name + '.zip')
            github.archive(request['repository'], request[key], archive)
            extract_source(archive, work / name)
        graph = s3.get_object(Bucket=request['apt_bucket'], Key=request['graph_key'])['Body'].read()
        (destination / 'graph.yaml').write_bytes(graph)
        check_removed(work / 'source', work / 'baseline', yaml.safe_load(graph), request['distro'])
        # Preserve executable bits across Actions artifact transfer. Only this
        # repository's tested revision, not the baseline/fleet, reaches the worker.
        shutil.copyfile(work / 'source.zip', destination / 'source.zip')
        account, region, repository = ecr_location(request['image'].split('@')[0])
        response = ecr(region).get_authorization_token(registryIds=[account])['authorizationData'][0]
        expected = 'https://' + request['image'].split('/')[0]
        if response['proxyEndpoint'] != expected:
            raise ValueError('Unexpected ECR authentication endpoint')
        # Docker login state is temporary and never uploaded. No image is started.
        docker_env = dict(os.environ, DOCKER_CONFIG=str(work / 'docker'))
        username, password = base64.b64decode(response['authorizationToken']).decode().split(':', 1)
        subprocess.run(['docker', 'login', '--username', username, '--password-stdin', expected],
                       input=password, text=True, env=docker_env, check=True, stdout=subprocess.DEVNULL)
        subprocess.run(['docker', 'pull', request['image']], env=docker_env, check=True)
        image = json.loads(subprocess.check_output(['docker', 'image', 'inspect', request['image']],
                                                   env=docker_env, text=True))[0]
        no_image_credentials(image)
        subprocess.run(['docker', 'save', '-o', str(destination / 'image.tar'), request['image']],
                       env=docker_env, check=True)
        inspect_archive(destination / 'image.tar')
        runtime = {key: request[key] for key in ('id', 'image', 'distro', 'os', 'sha', 'baseline_sha')}
        runtime['image_id'] = image['Id']
        (destination / 'request.json').write_text(json.dumps(runtime, indent=2))
        checksums = {str(path.relative_to(destination)): file_digest(path)
                     for path in destination.rglob('*') if path.is_file()}
        (destination / 'checksums.json').write_text(json.dumps(checksums, indent=2))
