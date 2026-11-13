from tailor_distro.blossom import Graph, GraphPackage


def test_dependency_export_covers_packages_outside_flavour_roots(tmp_path, monkeypatch):
    import sys
    import yaml
    from unittest import mock
    from tailor_distro.get_dependency_list import main

    recipe = tmp_path / 'recipes.yaml'
    recipe.write_text(yaml.safe_dump({'flavours': {'test': {'distributions': {
        'ros1': {'root_packages': ['root']}}}}}))
    graph = mock.Mock(os_version='noble', release_label='feature-ci', packages={'ros1': {'root': {}, 'other': {}}})
    graph.all_apt_depends.side_effect = lambda name, distro: {'cmake'} if name == 'root' else {'libextra-dev'}
    monkeypatch.setattr(sys, 'argv', ['get_dependency_list', '--recipe', str(recipe),
                                    '--graph', 'unused.yaml', '--workspace', str(tmp_path)])
    with mock.patch('tailor_distro.get_dependency_list.Graph.from_yaml', return_value=graph):
        main()
    assert (tmp_path / 'dependencies/all-noble-feature-ci-dependencies.txt').read_text().splitlines() == [
        'cmake', 'libextra-dev']

# Arbitrary dates to test
OLD_BUILD_DATE = "20260506.000000"
NEW_BUILD_DATE = "20260507.000000"


def test_all_apt_depends():
    """
    Tests that all apt depends are included in the query, even nested depends.
    """
    a = GraphPackage(
        "pkg_a",
        "0.0.0",
        "abc1234",
        ros_version="ros1",
        apt_candidate_version=f"0.0.0-{OLD_BUILD_DATE}+git1234567",
        path="",
        apt_depends=["b:apt_depend1"],
        source_depends=["b:pkg_b"],
    )
    b = GraphPackage(
        "pkg_b",
        "0.0.0",
        "abc1234",
        ros_version="ros1",
        apt_candidate_version=f"0.0.0-{OLD_BUILD_DATE}+git1234567",
        path="",
        apt_depends=["b:apt_depend2"],
        source_depends=[],
    )

    graph = Graph(
        "ubuntu",
        "jammy",
        "test",
        NEW_BUILD_DATE,
        apt_repo="",
        init_apt=False,
        packages={"ros1": {"pkg_a": a, "pkg_b": b}},
    )
    graph.finalize()

    apt_depends = graph.all_apt_depends("pkg_a", "ros1")
    assert "apt_depend1" in apt_depends
    assert "apt_depend2" in apt_depends


def test_run_and_build_depends():
    """
    Tests that run/build depends are included and can be queried separately.
    """
    a = GraphPackage(
        "pkg_a",
        "0.0.0",
        "abc1234",
        ros_version="ros1",
        apt_candidate_version=f"0.0.0-{OLD_BUILD_DATE}+git1234567",
        path="",
        apt_depends=["r:apt_run_depend1", "b:apt_build_depend1"],
        source_depends=["r:source_run_depend1", "b:source_build_depend1"],
    )

    source_run_depends = a.run_depends(types=["source"])
    assert "source_run_depend1" in source_run_depends

    source_build_depends = a.build_depends(types=["source"])
    assert "source_build_depend1" in source_build_depends

    apt_run_depends = a.run_depends(types=["apt"])
    assert "apt_run_depend1" in apt_run_depends

    apt_build_depends = a.build_depends(types=["apt"])
    assert "apt_build_depend1" in apt_build_depends


if __name__ == "__main__":
    test_all_apt_depends()
    test_run_and_build_depends()
