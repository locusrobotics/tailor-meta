import io
import json
from pathlib import Path
from unittest import mock

import pytest
import yaml

from tailor_meta.actions import Build, claim_release, verify_release_claim, config_digest, save_plan
from tailor_meta.create_pipelines import source_workflow


def config(mode='release', kind='feature'):
    return dict(kind=kind, label='feature-ci', track='hotdog', timestamp='20261113.000000', days=30,
                os=['jammy', 'noble'], common={'apt_repo': 's3://example', 'organization': 'example'},
                run='123-1', mode=mode, completed=[], repository='example/toy',
                config_sha='a' * 40, implementation_sha='b' * 40)


def test_packages_must_all_succeed_before_publish(tmp_path, monkeypatch):
    monkeypatch.setenv('RUNNER_TEMP', str(tmp_path))
    build = Build(config())
    build.config['completed'] = ['fetch', 'mirror', 'graph']
    with mock.patch.object(build, 'packages', side_effect=RuntimeError('noble failed')):
        with pytest.raises(RuntimeError):
            build.execute('packages')
    with mock.patch.object(build, 'publish') as publish:
        with pytest.raises(ValueError, match='requires successful packages'):
            build.execute('publish')
        publish.assert_not_called()


def test_successful_stages_cannot_be_replayed():
    build = Build(config())
    build.config['completed'] = ['fetch', 'mirror', 'graph', 'packages', 'publish']
    with pytest.raises(ValueError, match='already completed'):
        build.execute('publish')


def test_packages_mode_never_mirrors_or_publishes():
    build = Build(config('packages'))
    for stage in ('mirror', 'publish', 'images'):
        with pytest.raises(ValueError, match='not allowed'):
            build.execute(stage)


def test_container_credential_scopes(tmp_path, monkeypatch):
    monkeypatch.setenv('RUNNER_TEMP', str(tmp_path))
    monkeypatch.setenv('TAILOR_GPG_DIR', '/signing')
    build = Build(config())
    with build.credentials(), mock.patch.object(build, 'run') as run:
        build.container('build', 'build_packages', apt=True, git_auth=True)
        command = ' '.join(run.call_args.args)
        assert '/etc/apt/s3auth.conf:ro' in command
        assert '/root/.gitconfig:ro' in command
        assert 'AWS_SECRET_ACCESS_KEY' not in command
        assert 'GITHUB_TOKEN' not in command
        assert '/gpg' not in command
        assert 'vault' not in command
        build.container('publish', 'publish_packages', aws=True, signing=True)
        command = ' '.join(run.call_args.args)
        assert '/signing:/gpg:ro' in command
        assert 'AWS_SECRET_ACCESS_KEY' in command
        assert '.gitconfig' not in command
        assert 'vault' not in command


def test_final_claim_is_atomic_and_legacy_releases_are_rejected():
    identity = config(kind='final')
    client = mock.Mock()
    client.list_objects_v2.return_value = {'KeyCount': 0}
    claim_release(identity, client)
    assert client.put_object.call_args.kwargs['IfNoneMatch'] == '*'
    client.list_objects_v2.return_value = {'KeyCount': 1}
    client.put_object.reset_mock()
    with pytest.raises(RuntimeError, match='refusing to overwrite'):
        claim_release(identity, client)
    client.put_object.assert_not_called()


def test_final_claim_cannot_be_adopted_by_rerun():
    identity = config(kind='final')
    client = mock.Mock()
    client.get_object.return_value = {'Body': io.BytesIO(json.dumps(identity).encode())}
    verify_release_claim(identity, client)
    client.get_object.return_value = {'Body': io.BytesIO(json.dumps(identity).encode())}
    with pytest.raises(RuntimeError, match='does not own'):
        verify_release_claim(dict(identity, run='123-2'), client)
    assert Build(identity).retention() == []


def test_final_claim_race_does_not_ignore_precondition_failure():
    from botocore.exceptions import ClientError
    client = mock.Mock()
    client.list_objects_v2.return_value = {'KeyCount': 0}
    client.put_object.side_effect = ClientError({'Error': {'Code': 'PreconditionFailed'}}, 'PutObject')
    with pytest.raises(ClientError):
        claim_release(config(kind='final'), client)


def test_feature_publication_does_not_claim_immutable_label():
    client = mock.Mock()
    claim_release(config(), client)
    client.put_object.assert_not_called()
    client.list_objects_v2.assert_not_called()


def test_snapshot_detects_configuration_change(tmp_path):
    (tmp_path / 'config').mkdir()
    path = tmp_path / 'config/recipes.yaml'
    path.write_text('foo: 1')
    before = config_digest(tmp_path)
    path.write_text('foo: 2')
    assert config_digest(tmp_path) != before
    identity = config()
    save_plan(tmp_path / 'plan.json', identity)
    assert json.loads((tmp_path / 'plan.json').read_text()) == identity


def test_publish_rejects_modified_package_set_before_remote_writes(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    identity = config()
    identity['packages'] = {'jammy': {'test.deb': 'wrong'}, 'noble': {}}
    build = Build(identity)
    with mock.patch.object(build, 'run') as run, mock.patch.object(build, 'container') as container:
        with pytest.raises(ValueError, match='snapshot changed'):
            build.publish()
        run.assert_not_called()
        container.assert_not_called()


def test_source_branch_jobs_are_aggregated_without_collisions():
    base = dict(source_branch='devel', tailor_meta='a' * 40, repository_name='example', release_label='hotdog',
                release_track='hotdog', distributions=['noble'], docker_registry='example.invalid/ci')
    text = source_workflow([dict(base, rosdistro_name='ros1'), dict(base, rosdistro_name='ros2')])
    assert '&id' not in text and '*id' not in text
    workflow = yaml.safe_load(text)
    assert set(workflow['jobs']) == {'ros1', 'ros2'}
    assert workflow['on']['pull_request']['branches'] == ['devel']
    assert all('tailor_meta' not in job['with'] for job in workflow['jobs'].values())


def test_build_script_preserves_underlay_order():
    script = Path('ci/build-os.sh').read_text()
    assert script.index('--ros-distro ros1') < script.index('--ros-distro ros2') < script.index('build_bundles')
