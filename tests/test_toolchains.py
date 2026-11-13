from unittest import mock

import pytest

from tailor_meta.toolchains import validate_reference, verify, resolve


def test_tags_are_not_accepted_as_pinned_toolchains():
    with pytest.raises(ValueError, match='digest'):
        validate_reference('ghcr.io/example/tools:master')
    validate_reference('ghcr.io/example/tools@sha256:' + 'a' * 64)
    with pytest.raises(ValueError):
        validate_reference('sha256:' + 'a' * 64)
    validate_reference('sha256:' + 'a' * 64, allow_local=True)


def test_image_revision_must_match_implementation():
    with mock.patch('tailor_meta.toolchains.inspect', return_value={'Config': {'Labels': {
        'org.opencontainers.image.revision': 'old'}}}):
        with pytest.raises(ValueError, match='revision'):
            verify('ghcr.io/example/tools@sha256:' + 'a' * 64, 'new')


def test_resolve_records_digest_not_sha_discovery_tag():
    reference = 'ghcr.io/locusrobotics/tailor-meta-tools@sha256:' + 'a' * 64
    image = {'RepoDigests': [reference], 'Config': {'Labels': {
        'org.opencontainers.image.revision': 'sha', 'io.tailor.role': 'tools'}}}
    with mock.patch('tailor_meta.toolchains.subprocess.run'), mock.patch('tailor_meta.toolchains.inspect', return_value=image):
        assert resolve('sha', ['tools']) == {'tools': reference}


def test_toolchain_cannot_substitute_the_wrong_os():
    labels = {'org.opencontainers.image.revision': 'sha', 'io.tailor.role': 'builder', 'io.tailor.os': 'jammy'}
    with mock.patch('tailor_meta.toolchains.inspect', return_value={'Config': {'Labels': labels}}):
        with pytest.raises(ValueError, match='role/OS'):
            verify('ghcr.io/example/builder@sha256:' + 'a' * 64, 'sha', role='builder-noble')


def test_builders_are_resolved_from_consumer_namespace_not_tailor_meta():
    reference = 'ghcr.io/locusrobotics/toydistro-builder@sha256:' + 'a' * 64
    image = {'RepoDigests': [reference], 'Config': {'Labels': {
        'org.opencontainers.image.revision': 'sha', 'io.tailor.role': 'builder', 'io.tailor.os': 'example-os'}}}
    with mock.patch('tailor_meta.toolchains.subprocess.run') as run, \
            mock.patch('tailor_meta.toolchains.inspect', return_value=image):
        result = resolve('sha', ['builder-example-os'], 'locusrobotics/toydistro')
    assert result == {'builder-example-os': reference}
    assert run.call_args.args[0] == ['docker', 'pull', 'ghcr.io/locusrobotics/toydistro-builder:sha-example-os']
