import json
import pathlib
from unittest import mock

import pytest
import yaml

from tailor_meta.actions import Build, plan, release, secure_dockerfile
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
    assert plan(tmp_path, 'refs/heads/master')['images'] == [
        {'name': 'all', 'os': 'jammy', 'flavour': 'dev'},
        {'name': 'all', 'os': 'noble', 'flavour': 'dev'},
        {'name': 'limited', 'os': 'noble', 'flavour': 'support'},
    ]


@pytest.mark.parametrize('path', [
    'environment/distro/Dockerfile', 'environment/image/Dockerfile',
    'tailor_distro/debian_templates/Dockerfile.j2',
])
def test_secure_build_environment(path):
    text = secure_dockerfile(pathlib.Path(path).read_text())
    assert 'ARG AWS_ACCESS_KEY_ID' not in text
    assert 'ARG AWS_SECRET_ACCESS_KEY' not in text
    assert 'ARG ANSIBLE_VAULT_PASS' not in text
    assert 'tee /etc/apt/s3auth.conf' not in text
    assert 'tee /home/tailor/.vault_pass.txt' not in text
    assert 'aws configure set aws_secret_access_key' not in text
    assert 'ENV AWS_ACCESS_KEY_ID' not in text
    assert 'id=s3auth,target=/etc/apt/s3auth.conf' in text
    assert 'COPY tailor-meta /tailor-meta' in text


def test_source_stub():
    text = source_workflow(dict(source_branch='release/26.1/ros2', tailor_meta='abc123',
                                rosdistro_name='ros2', release_label='26.1-rc',
                                distributions=['noble'], docker_registry='example.invalid/ci'))
    workflow = yaml.safe_load(text)
    assert workflow['on']['push']['branches'] == ['release/26.1/ros2']
    job = workflow['jobs']['tailor']
    assert job['uses'].endswith('/source.yaml@abc123')
    assert json.loads(job['with']['distributions']) == ['noble']
    assert 'steps' not in job


def test_internal_image_assets():
    assert find_package('/tailor-meta/environment/image', 'ansible.cfg', {}) == (
        '/tailor-meta/environment/image/ansible.cfg')


def test_no_publish_or_images_in_graph_mode(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv('RUNNER_TEMP', str(tmp_path))
    config = dict(kind='hotdog', label='hotdog', timestamp='20261113.000000',
                  common={'apt_repo': 's3://example'})
    build = Build(config, deploy=True, graph_only=True)
    with mock.patch.object(build, 'distribution') as distribution, mock.patch.object(build, 'images') as images:
        build.execute()
    distribution.assert_called_once()
    images.assert_not_called()
    assert not list(build.secret_dir.iterdir())


def test_secret_cleanup_on_failure(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv('RUNNER_TEMP', str(tmp_path))
    config = dict(kind='hotdog', label='hotdog', timestamp='20261113.000000',
                  common={'apt_repo': 's3://example'})
    build = Build(config)
    with mock.patch.object(build, 'distribution', side_effect=RuntimeError('build failed')):
        with pytest.raises(RuntimeError):
            build.execute()
    assert not list(build.secret_dir.iterdir())
