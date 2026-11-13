from unittest import mock

import pytest

from tailor_meta.actions import Build


@pytest.mark.parametrize('deploy', [False, True])
def test_order_and_publication_boundary(tmp_path, monkeypatch, deploy):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv('RUNNER_TEMP', str(tmp_path))
    monkeypatch.setenv('TAILOR_GPG_DIR', str(tmp_path / 'keys'))
    environment = tmp_path / 'tailor-meta/environment/distro'
    environment.mkdir(parents=True)
    (environment / 'Dockerfile').write_text('FROM ubuntu:noble\nRUN true\n')
    debian = tmp_path / 'workspace/debian'
    debian.mkdir(parents=True)
    (debian / 'Dockerfile-noble').write_text('FROM ubuntu:noble\nRUN true\n')
    dependencies = tmp_path / 'workspace/dependencies'
    dependencies.mkdir()
    (dependencies / 'dev-noble-hotdog-dependencies.txt').write_text('cmake\ngit\n')
    config = dict(kind='hotdog', label='hotdog', track='hotdog', timestamp='20261113.000000', days=30,
                  os=['noble'], common={'apt_repo': 's3://example', 'organization': 'example'})
    build = Build(config, deploy=deploy, rebuild=True, no_cache=True)
    with mock.patch.object(build, 'run') as run, mock.patch.object(build, 'images') as images:
        build.execute()
    calls = [call.args for call in run.call_args_list]
    package_calls = [call for call in calls if any('build_packages --graph' in str(arg) for arg in call)]
    assert len(package_calls) == 1
    script = next(arg for arg in package_calls[0] if 'build_packages --graph' in str(arg))
    assert script.index('--ros-distro ros1') < script.index('--ros-distro ros2') < script.index('build_bundles')
    assert '--rebuild-all' in script
    assert '/work/builds/noble' in package_calls[0]
    assert (tmp_path / 'builds/noble/workspace/src').is_symlink()
    assert all('--no-cache' in call for call in calls if call[:2] == ('docker', 'build'))
    mutations = [call for call in calls if call[:3] == ('aws', 's3', 'sync') or
                 's3_directory_cleanup' in call or any('publish_packages' in str(arg) for arg in call)]
    assert bool(mutations) is deploy
    assert images.call_count == int(deploy)
