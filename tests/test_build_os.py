import json
from pathlib import Path
from unittest import mock

import pytest

from tailor_meta.build_os import build_os


def test_same_command_installs_dependencies_and_builds_underlay_before_overlay(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    dependencies = tmp_path / 'workspace/dependencies'
    dependencies.mkdir(parents=True)
    (dependencies / 'all-noble-feature-ci-dependencies.txt').write_text('cmake\nlibextra-dev\n')
    identity = {'os': ['noble'], 'label': 'feature-ci', 'common': {'apt_repo': ''}}
    with mock.patch('tailor_meta.build_os.run') as run:
        build_os(identity, 'noble', root=tmp_path, rebuild=True)
    calls = [call.args for call in run.call_args_list]
    assert calls[0] == ('apt-get', 'update')
    assert calls[1][-2:] == ('cmake', 'libextra-dev')
    builds = [call for call in calls if call[0] == 'build_packages']
    assert [call[call.index('--ros-distro') + 1] for call in builds] == ['ros1', 'ros2']
    assert all('--rebuild-all' in call for call in builds)
    assert calls[-1][0] == 'build_bundles'
    assert (tmp_path / 'builds/noble/workspace/src').is_symlink()
    assert json.loads((tmp_path / 'builds/noble/package-checksums.json').read_text()) == {}


def test_no_bundle_build_after_overlay_failure(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    dependencies = tmp_path / 'workspace/dependencies'
    dependencies.mkdir(parents=True)
    (dependencies / 'all-noble-feature-ci-dependencies.txt').write_text('')
    identity = {'os': ['noble'], 'label': 'feature-ci', 'common': {'apt_repo': ''}}

    def command(*args):
        if 'ros2' in args:
            raise RuntimeError('overlay failed')

    with mock.patch('tailor_meta.build_os.run', side_effect=command) as run:
        with pytest.raises(RuntimeError, match='overlay failed'):
            build_os(identity, 'noble', root=tmp_path)
    assert not any(call.args[0] == 'build_bundles' for call in run.call_args_list)
    assert not (tmp_path / 'builds/noble/package-checksums.json').exists()
