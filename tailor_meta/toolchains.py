"""Resolve and verify immutable toolchain images before a distribution build."""
import json
import re
import subprocess

REGISTRY = 'ghcr.io/locusrobotics/tailor-meta'


def validate_reference(reference, allow_local=False):
    if re.fullmatch(r'[^\s]+@sha256:[0-9a-f]{64}', reference):
        return
    if allow_local and re.fullmatch(r'sha256:[0-9a-f]{64}', reference):
        return
    raise ValueError(f'Toolchain must be pinned by digest, not a mutable tag: {reference}')


def inspect(reference):
    image = json.loads(subprocess.check_output(['docker', 'image', 'inspect', reference], text=True))[0]
    if re.fullmatch(r'[0-9a-f]{64}', image.get('Id', '')):
        image['Id'] = 'sha256:' + image['Id']  # Podman-compatible Docker CLI.
    return image


def resolve(implementation_sha, roles, repository=None):
    resolved = {}
    for role in roles:
        if role.startswith('builder-'):
            if not repository or repository == 'local':
                raise ValueError('Builder images must be scoped to a configuration repository')
            image_repository = f'ghcr.io/{repository.lower()}-builder'
            tag = f'{image_repository}:{implementation_sha}-{role.removeprefix("builder-")}'
        else:
            image_repository = f'{REGISTRY}-{role}'
            tag = f'{image_repository}:{implementation_sha}'
        subprocess.run(['docker', 'pull', tag], check=True)
        image = inspect(tag)
        candidates = [ref for ref in image.get('RepoDigests', []) if ref.startswith(image_repository + '@')]
        if len(candidates) != 1:
            raise ValueError(f'Cannot identify one digest for {tag}')
        resolved[role] = candidates[0]
        verify(resolved[role], implementation_sha, role=role)
    return resolved


def verify(reference, implementation_sha, allow_local=False, role=None):
    validate_reference(reference, allow_local)
    image = inspect(reference)
    labels = image['Config'].get('Labels', {})
    if labels.get('org.opencontainers.image.revision') != implementation_sha:
        raise ValueError('Toolchain image does not match the pinned implementation revision')
    if role:
        target, _, os_version = role.partition('-')
        if labels.get('io.tailor.role') != target or (os_version and labels.get('io.tailor.os') != os_version):
            raise ValueError('Toolchain role/OS does not match the build plan')
