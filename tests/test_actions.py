import json
import pathlib
from unittest import mock

import pytest
import yaml

from tailor_meta.actions import Build, plan, release
from tailor_meta.create_pipelines import source_workflow
from tailor_image import find_package


@pytest.mark.parametrize('ref,kind,track,label,days', [
    ('refs/heads/master', 'hotdog', 'hotdog', 'hotdog', 30),
    ('refs/heads/feature/Example', 'feature', 'hotdog', 'feature-example', 30),
    ('refs/heads/release/26.1', 'candidate', '26.1', '26.1-rc', 0),
    ('refs/tags/26.1.0', 'final', '26.1', '26.1.0', 0),
    ('refs/heads/other', 'trivial', '', '', 1),
])
def test_release(ref, kind, track, label, days):
    assert release(ref) == dict(kind=kind, track=track, label=label, days=days)


@pytest.mark.parametrize('ref', ['refs/tags/not-a-release', 'refs/heads/feature/a/b',
                                 'refs/heads/release/bad_label'])
def test_invalid_release(ref):
    with pytest.raises(ValueError):
        release(ref)


def test_image_matrix(tmp_path):
    (tmp_path / 'config').mkdir()
    (tmp_path / 'config/recipes.yaml').write_text(yaml.safe_dump({
        'os': {'ubuntu': ['jammy', 'noble']}, 'common': {'testing_flavour': 'dev'},
    }))
    (tmp_path / 'config/images.yaml').write_text(yaml.safe_dump({'images': {
        'all': {}, 'limited': {'os_versions': ['noble', 'old'], 'bundle_flavour': 'support'},
    }}))
    with mock.patch('tailor_meta.actions.revision', return_value='a' * 40):
        result = plan(tmp_path, 'refs/heads/master', mode='validate')
    assert result['images'] == [
        {'name': 'all', 'os': 'jammy', 'flavour': 'dev'},
        {'name': 'all', 'os': 'noble', 'flavour': 'dev'},
        {'name': 'limited', 'os': 'noble', 'flavour': 'support'},
    ]


def test_secure_build_environment():
    text = pathlib.Path('environment/toolchains/Dockerfile').read_text()
    assert 'ARG AWS_ACCESS_KEY_ID' not in text
    assert 'ARG AWS_SECRET_ACCESS_KEY' not in text
    assert 'ARG ANSIBLE_VAULT_PASS' not in text
    assert 'tee /etc/apt/s3auth.conf' not in text
    assert 'tee /home/tailor/.vault_pass.txt' not in text
    assert 'aws configure set aws_secret_access_key' not in text
    assert 'ENV AWS_ACCESS_KEY_ID' not in text
    assert 'COPY . /tailor-meta' in text
    assert 'COPY rosdistro' not in text
    assert 'FROM base AS builder' in text
    assert 'FROM base AS tools' in text
    assert 'FROM base AS imager' in text


def test_source_stub():
    text = source_workflow([dict(source_branch='release/26.1/ros2', tailor_meta='abc123', repository_name='example',
                                rosdistro_name='ros2', release_label='26.1-rc', release_track='26.1',
                                distributions=['noble'], docker_registry='example.invalid/ci')])
    workflow = yaml.safe_load(text)
    assert workflow['on']['push']['branches'] == ['release/26.1/ros2']
    job = workflow['jobs']['ros2']
    assert job['uses'].endswith('/source.yaml@abc123')
    assert json.loads(job['with']['distributions']) == ['noble']
    assert 'steps' not in job


def test_internal_image_assets():
    assert find_package('/tailor-meta/environment/image', 'ansible.cfg', {}) == (
        '/tailor-meta/environment/image/ansible.cfg')


def test_no_publish_or_images_in_graph_mode(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv('RUNNER_TEMP', str(tmp_path))
    config = dict(kind='hotdog', label='hotdog', timestamp='20261113.000000', run='1', mode='graph',
                  completed=['fetch'], common={'apt_repo': 's3://example'})
    build = Build(config)
    for stage in ('mirror', 'packages', 'publish', 'images'):
        with pytest.raises(ValueError):
            build.execute(stage)


def test_secret_cleanup_on_failure(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv('RUNNER_TEMP', str(tmp_path))
    config = dict(kind='hotdog', label='hotdog', timestamp='20261113.000000', run='1', mode='graph',
                  completed=[], common={'apt_repo': 's3://example'})
    build = Build(config)
    with mock.patch.object(build, 'fetch', side_effect=RuntimeError('build failed')):
        with pytest.raises(RuntimeError):
            build.execute('fetch')
    assert not list(tmp_path.glob('tailor-auth-*'))
    assert config['completed'] == []
