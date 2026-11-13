import copy
import io
import json
from pathlib import Path
import stat
import tarfile
from unittest import mock
import zipfile

import pytest
import yaml

from tailor_meta.source_ci.controller import enqueue, outcome, reconcile
from tailor_meta.source_ci.inputs import extract_source, no_image_credentials, credential_file, check_removed
from tailor_meta.source_ci.poll import discover, environment, ecr_location
from tailor_meta.source_ci.prepare import inspect_archive
from tailor_meta.source_ci.state import State
from tailor_meta.source_ci.worker import command, collect_reports, verify_bundle
from tailor_meta.source_ci.cli import artifact_policy, owned
from tailor_meta.actions import file_digest


class MemoryState:
    def __init__(self):
        self.data = {}
        self.etags = {}

    def get(self, key):
        return copy.deepcopy(self.data.get(key)), self.etags.get(key)

    def put(self, key, value, etag=None):
        assert self.etags.get(key) == etag, 'Conditional write conflict'
        self.data[key] = copy.deepcopy(value)
        self.etags[key] = str(int(etag or 0) + 1)

    def records(self):
        for key in list(self.data):
            if key.startswith('requests/'):
                value, etag = self.get(key)
                yield key, value, etag


def request(key='a', family='family'):
    return {'id': key * 64, 'family': family, 'repository': 'example/source', 'sha': key * 40,
            'context': 'tailor/toy/feature-ci/ros1/target'}


def test_queue_deduplicates_and_limits_claims():
    state, github, actions = MemoryState(), mock.Mock(), mock.Mock()
    rows = [request('a', 'one'), request('b', 'two')]
    assert enqueue(rows, state, github, actions, 'example/toy', '1', '1', limit=1) == ['a' * 64]
    assert enqueue(rows, state, github, actions, 'example/toy', '2', '1') == ['b' * 64]
    assert github.status.call_count == 2
    assert state.data['runs/2-1.json']['requests'] == ['b' * 64]


def test_superseded_work_is_cancelled_before_new_pending_status():
    state, github, actions = MemoryState(), mock.Mock(), mock.Mock()
    enqueue([request('a')], state, github, actions, 'example/toy', '1', '1')
    enqueue([request('b')], state, github, actions, 'example/toy', '2', '1')
    assert state.data['requests/' + 'a' * 64 + '.json']['state'] == 'superseded'
    actions.request.assert_called_once_with('POST', '/repos/example/toy/actions/runs/1/cancel')
    assert [call.args[1] for call in github.status.call_args_list] == ['pending', 'error', 'pending']


def test_run_with_other_live_requests_is_not_cancelled():
    state, github, actions = MemoryState(), mock.Mock(), mock.Mock()
    enqueue([request('a', 'one'), request('b', 'two')], state, github, actions, 'example/toy', '1', '1')
    enqueue([request('c', 'one')], state, github, actions, 'example/toy', '2', '1')
    actions.request.assert_not_called()


@pytest.mark.parametrize('conclusion,expected', [('success', 'success'), ('failure', 'failure'),
                                               ('cancelled', 'error'), ('timed_out', 'error'), ('skipped', 'error')])
def test_reporting_uses_native_job_conclusion(conclusion, expected):
    record = {'request': request(), 'state': 'prepared'}
    jobs = [{'name': 'poll / source-test:' + 'a' * 64, 'conclusion': conclusion}]
    assert outcome(record, jobs)[0] == expected
    assert outcome(dict(record, state='preparation_failed'), jobs)[0] == 'error'


def test_reconcile_closes_pending_status_after_interrupted_run():
    state, github, actions = MemoryState(), mock.Mock(), mock.Mock()
    enqueue([request()], state, github, actions, 'example/toy', '1', '1')
    actions.request.return_value = {'status': 'completed'}
    actions.pages.return_value = []
    reconcile(state, github, actions)
    assert state.data['requests/' + 'a' * 64 + '.json']['state'] == 'error'
    assert github.status.call_args.args[1] == 'error'


def test_s3_state_is_namespaced_and_conditional():
    client = mock.Mock()
    state = State(client, 's3://ci-state/pilot', 'example/toy')
    state.put('requests/id.json', {'state': 'queued'})
    assert client.put_object.call_args.kwargs['Key'] == 'pilot/example/toy/requests/id.json'
    assert client.put_object.call_args.kwargs['IfNoneMatch'] == '*'
    state.put('requests/id.json', {'state': 'success'}, 'etag')
    assert client.put_object.call_args.kwargs['IfMatch'] == 'etag'
    with pytest.raises(ValueError):
        State(client, 's3://ci-state', 'example/toy')


def published_fixture(tmp_path):
    (tmp_path / 'config').mkdir()
    registry = '123456789012.dkr.ecr.us-east-1.amazonaws.com/toy'
    (tmp_path / 'config/recipes.yaml').write_text(yaml.safe_dump({
        'common': {'apt_repo': 's3://toy', 'docker_registry': 'https://' + registry},
        'os': {'ubuntu': ['target-a', 'target-b']},
    }))
    published = {'label': 'feature-pilot', 'completed': ['images'], 'timestamp': '20261113.123456',
                 'config_sha': 'd' * 40, 'sources': {'ros1_repositories_data.jsonl': [
                     {'repo': 'test_meta', 'sha': 'b' * 40}]},
                 'test_images': {version: registry + '@sha256:' + 'f' * 64 for version in ('target-a', 'target-b')}}
    client = mock.Mock()
    client.get_object.side_effect = lambda **kwargs: {'Body': io.BytesIO(json.dumps(published).encode())}
    return published, client


def test_discovery_uses_configured_os_pinned_images_and_rejects_forks(tmp_path):
    published, s3 = published_fixture(tmp_path)
    github = mock.Mock()
    github.head.return_value = 'a' * 40
    github.pages.return_value = [
        {'head': {'repo': {'full_name': 'example/test_meta'}, 'sha': 'c' * 40},
         'number': 1, 'author_association': 'MEMBER'},
        {'head': {'repo': {'full_name': 'external/fork'}, 'sha': 'e' * 40},
         'number': 2, 'author_association': 'MEMBER'},
        {'head': {'repo': {'full_name': 'example/test_meta'}, 'sha': 'e' * 40},
         'number': 3, 'author_association': 'NONE'},
    ]
    grouped = {('example/test_meta', 'master'): [{'rosdistro_name': 'ros1', 'repository_name': 'test_meta',
                                               'distributions': ['target-a', 'target-b']}]}
    with mock.patch('tailor_meta.source_ci.poll.collect_sources', return_value=grouped):
        rows = list(discover(tmp_path, 'example/toy', '0' * 40, 'refs/heads/feature/pilot', github, s3,
                             ['example/test_meta']))
    assert len(rows) == 4
    assert {row['os'] for row in rows} == {'target-a', 'target-b'}
    assert {row['pull_request'] for row in rows} == {None, 1}
    assert len({row['id'] for row in rows}) == 4
    assert all(row['image'] == published['test_images'][row['os']] for row in rows)


def test_environment_must_be_complete_and_have_pinned_test_images(tmp_path):
    published, s3 = published_fixture(tmp_path)
    published['completed'] = ['publish']
    with pytest.raises(ValueError, match='successfully published'):
        environment(tmp_path, 'refs/heads/feature/pilot', s3)
    published['completed'] = ['images']
    published['test_images'] = {}
    with pytest.raises(ValueError, match='pinned test-image'):
        environment(tmp_path, 'refs/heads/feature/pilot', s3)


def test_ecr_registry_cannot_select_arbitrary_download_host():
    with pytest.raises(ValueError):
        ecr_location('https://metadata.internal/evil')


@pytest.mark.parametrize('name', ['root/../../escape', '/absolute/file', 'root/..\\escape'])
def test_source_archive_rejects_traversal(tmp_path, name):
    archive = tmp_path / 'source.zip'
    with zipfile.ZipFile(archive, 'w') as bundle:
        bundle.writestr(name, 'not executable')
    with pytest.raises(ValueError):
        extract_source(archive, tmp_path / 'out')


def test_source_archive_rejects_symlinks(tmp_path):
    archive = tmp_path / 'source.zip'
    entry = zipfile.ZipInfo('root/link')
    entry.external_attr = (stat.S_IFLNK | 0o777) << 16
    with zipfile.ZipFile(archive, 'w') as bundle:
        bundle.writestr(entry, '../../outside')
    with pytest.raises(ValueError, match='symlinks'):
        extract_source(archive, tmp_path / 'out')


def test_source_archive_preserves_executable_bits(tmp_path):
    archive = tmp_path / 'source.zip'
    entry = zipfile.ZipInfo('root/generate.sh')
    entry.external_attr = (stat.S_IFREG | 0o755) << 16
    with zipfile.ZipFile(archive, 'w') as bundle:
        bundle.writestr(entry, '#!/bin/sh\nexit 0\n')
    extract_source(archive, tmp_path / 'out')
    assert (tmp_path / 'out/generate.sh').stat().st_mode & 0o111


def test_removed_package_checks_use_graph_without_fetching_fleet(tmp_path):
    baseline, source = tmp_path / 'baseline', tmp_path / 'source'
    baseline.mkdir()
    source.mkdir()
    (baseline / 'package.xml').write_text('<package><name>removed</name></package>')
    graph = {'packages': {'ros2': {'other': {'ros1_depends': ['removed']}}}}
    with pytest.raises(ValueError, match='removed packages'):
        check_removed(source, baseline, graph, 'ros1')


def test_image_environment_allows_region_but_not_credentials():
    no_image_credentials({'Config': {'Env': ['AWS_DEFAULT_REGION=us-east-1']}})
    with pytest.raises(ValueError):
        no_image_credentials({'Config': {'Env': ['AWS_ACCESS_KEY_ID=test-only-placeholder']}})
    with pytest.raises(ValueError):
        credential_file('etc/apt/s3auth.conf', 'SecretAccessKey = "test-only-placeholder"')


def test_deleted_credential_layer_is_still_rejected(tmp_path):
    layer = io.BytesIO()
    with tarfile.open(fileobj=layer, mode='w') as archive:
        data = b'aws_access_key_id = test-only-placeholder'
        member = tarfile.TarInfo('root/.aws/credentials')
        member.size = len(data)
        archive.addfile(member, io.BytesIO(data))
    path = tmp_path / 'image.tar'
    with tarfile.open(path, 'w') as archive:
        files = {'manifest.json': json.dumps([{'Config': 'config.json', 'Layers': ['layer.tar']}]).encode(),
                 'config.json': b'{"config": {}, "history": []}', 'layer.tar': layer.getvalue()}
        for name, content in files.items():
            member = tarfile.TarInfo(name)
            member.size = len(content)
            archive.addfile(member, io.BytesIO(content))
    with pytest.raises(ValueError, match='baked AWS'):
        inspect_archive(path)


def test_worker_container_has_no_host_socket_network_or_credential_mounts(tmp_path):
    args = command('sha256:' + 'a' * 64, tmp_path / 'source', tmp_path / 'results', 'ros1')
    assert args[args.index('--network') + 1] == 'none'
    assert args[args.index('--cap-drop') + 1] == 'ALL'
    assert 'no-new-privileges' in args
    text = ' '.join(args)
    for denied in ('docker.sock', 'AWS_ACCESS_KEY_ID', 'GITHUB_TOKEN', '.gitconfig', '/gpg', '--privileged'):
        assert denied not in text


def test_report_collection_does_not_follow_untrusted_symlinks(tmp_path):
    root = tmp_path / 'results/test_results'
    root.mkdir(parents=True)
    (tmp_path / 'private.xml').write_text('must not upload')
    (root / 'leak.xml').symlink_to(tmp_path / 'private.xml')
    (root / 'valid.xml').write_text('<testsuite/>')
    collect_reports(tmp_path / 'results', tmp_path / 'reports')
    assert [path.name for path in (tmp_path / 'reports').iterdir()] == ['valid.xml']


def test_bundle_checksums_detect_tampering(tmp_path):
    path = tmp_path / 'request.json'
    path.write_text('{}')
    (tmp_path / 'checksums.json').write_text(json.dumps({'request.json': file_digest(path)}))
    verify_bundle(tmp_path)
    path.write_text('{"changed":true}')
    with pytest.raises(ValueError):
        verify_bundle(tmp_path)


def test_worker_job_receives_no_control_plane_secrets():
    workflow = yaml.safe_load(Path('.github/workflows/source-poll.yaml').read_text())
    worker = workflow['jobs']['test']
    assert worker['runs-on'] == 'ubuntu-24.04'
    assert 'SOURCE_CI_GITHUB_TOKEN' not in json.dumps(worker)
    assert 'SOURCE_CI_STATE_URI' not in json.dumps(worker)
    assert 'tailor-source-control' not in json.dumps(worker)


def test_public_artifacts_require_explicit_approval(monkeypatch):
    monkeypatch.setenv('GITHUB_REPOSITORY', 'example/toy')
    actions = mock.Mock()
    actions.request.return_value = {'private': False}
    with pytest.raises(ValueError, match='explicitly approve'):
        artifact_policy(actions, False)
    artifact_policy(actions, True)
    actions.request.return_value = {'private': True}
    artifact_policy(actions, False)


def test_another_run_cannot_prepare_or_report_the_request(monkeypatch):
    state, github, actions = MemoryState(), mock.Mock(), mock.Mock()
    enqueue([request()], state, github, actions, 'example/toy', '1', '1')
    monkeypatch.setenv('GITHUB_REPOSITORY', 'example/toy')
    monkeypatch.setenv('GITHUB_RUN_ID', '2')
    monkeypatch.setenv('GITHUB_RUN_ATTEMPT', '1')
    with pytest.raises(ValueError, match='does not own'):
        owned(state, 'a' * 64)
