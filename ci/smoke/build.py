"""Fixture inputs → the production build-os path → install and execute packages."""
import argparse
import datetime
import json
import os
from pathlib import Path
import subprocess

import yaml

from tailor_distro.blossom import Graph, GraphPackage
from tailor_meta.actions import Build, release, revision, save_plan
from tailor_meta import toolchains


def source(root, distro, name, cmake, files):
    directory = root / 'workspace/src' / distro / name
    directory.mkdir(parents=True)
    (directory / 'CMakeLists.txt').write_text(cmake)
    (directory / 'package.xml').write_text(f'''<package format="3">
  <name>{name}</name><version>1.0.0</version><description>Tailor smoke fixture</description>
  <maintainer email="ci@example.invalid">Tailor CI</maintainer><license>MIT</license>
  <export><build_type>cmake</build_type></export>
</package>''')
    for path, text in files.items():
        (directory / path).write_text(text)
    return GraphPackage(name, '1.0.0', 'abcdef0', str(directory), distro, [], [],
                        description='Tailor smoke fixture', maintainers='Tailor CI <ci@example.invalid>')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', required=True)
    parser.add_argument('--os', required=True)
    parser.add_argument('--configuration', type=Path, required=True)
    args = parser.parse_args()
    if (args.configuration / 'config/recipes.yaml').exists():
        subprocess.run(['python3', str(Path(__file__).resolve().parents[1] / 'validate.py'),
                        str(args.configuration)], check=True)
    image = toolchains.inspect(args.image)['Id']
    implementation = revision(Path(__file__).resolve().parents[2])
    toolchains.verify(image, implementation, allow_local=True, role='builder-' + args.os)
    identity = release(os.environ.get('GITHUB_REF', 'refs/heads/feature/local-smoke'))
    if not identity['label']:
        identity['label'], identity['kind'], identity['track'] = 'feature-ci-smoke', 'feature', 'hotdog'
    identity.update(schema=1, mode='packages', os=[args.os], fixture=True, local_toolchains=True,
                    timestamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%d.%H%M%S'),
                    run='smoke', config_sha=revision(args.configuration), implementation_sha=implementation,
                    toolchains={'builder-' + args.os: image}, completed=['fetch', 'graph'],
                    common={'apt_repo': '', 'organization': 'toydistro'}, sources={})
    root = Path('output')
    root.mkdir()
    # Graph paths must be valid inside the same /work mount production uses.
    base = source(root, 'ros1', 'smoke_base', '''cmake_minimum_required(VERSION 3.16)
project(smoke_base)
install(FILES smoke_value.h DESTINATION include)
install(FILES smoke_baseConfig.cmake DESTINATION share/smoke_base/cmake)
''', {'smoke_value.h': '#define SMOKE_VALUE 42\n',
      'smoke_baseConfig.cmake': 'get_filename_component(SMOKE_PREFIX "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)\n'
                               'set(SMOKE_INCLUDE "${SMOKE_PREFIX}/include")\n'})
    overlay = source(root, 'ros2', 'smoke_overlay', '''cmake_minimum_required(VERSION 3.16)
project(smoke_overlay)
find_package(smoke_base REQUIRED CONFIG)
add_executable(smoke-check main.cpp)
target_include_directories(smoke-check PRIVATE ${SMOKE_INCLUDE})
install(TARGETS smoke-check DESTINATION bin)
''', {'main.cpp': '#include <smoke_value.h>\nint main() { return SMOKE_VALUE == 42 ? 0 : 1; }\n'})
    overlay.ros1_depends = ['smoke_base']
    for package in (base, overlay):
        package.path = '/work/' + str(Path(package.path).relative_to(root))
    graph = Graph('ubuntu', args.os, identity['label'], identity['timestamp'], '',
                  organization='toydistro', init_apt=False,
                  packages={'ros1': {'smoke_base': base}, 'ros2': {'smoke_overlay': overlay}})
    graph.finalize()
    (root / 'workspace/graphs').mkdir()
    graph.write_yaml(root / 'workspace/graphs')
    recipe = {
        'common': {'organization': 'toydistro', 'cxx_flags': [], 'cxx_standard': 17, 'python_version': '3',
                   'default_build_depends': [], 'distributions': {
                       'ros1': {'env': {'ROS_VERSION': '1'}, 'os': []},
                       'ros2': {'env': {'ROS_VERSION': '2'}, 'underlays': ['ros1'], 'os': []}}},
        'flavours': {'smoke': {'description': 'Smoke installation set', 'distributions': {
            'ros1': {'root_packages': ['smoke_base']}, 'ros2': {'root_packages': ['smoke_overlay']}}}},
    }
    (root / 'rosdistro/config').mkdir(parents=True)
    (root / 'rosdistro/config/recipes.yaml').write_text(yaml.safe_dump(recipe))
    (root / 'workspace/dependencies').mkdir()
    (root / 'workspace/dependencies' / f'all-{args.os}-{identity["label"]}-dependencies.txt').write_text('cmake\n')
    os.chdir(root)
    save_plan(Path('build-plan.json'), identity)
    build = Build(identity, rebuild=True)
    with build.credentials():
        build.build_os(args.os)
        identity['completed'].append('packages')
        save_plan(Path('build-plan.json'), identity)
        build.container(image, 'bash', '-ec',
                        'dpkg -i /work/packages/"$1"/*.deb; exec /opt/toydistro/"$2"/ros2/bin/smoke-check',
                        'verify', args.os, identity['label'])
    debs = sorted((Path('packages') / args.os).glob('*.deb'))
    assert len(debs) >= 5, 'Expected source, environment and bundle Debian packages'
    for distro in ('ros1', 'ros2'):
        report = yaml.safe_load(Path(f'builds/{args.os}/build-report-{args.os}-{distro}.yaml').read_text())
        assert report['rebuilt_count'] == 1, report
    Path('smoke-result.json').write_text(json.dumps({
        'config_sha': identity['config_sha'], 'implementation_sha': implementation,
        'label': identity['label'], 'os': args.os, 'toolchain': image,
        'packages': [path.name for path in debs], 'installed_binary_exit_code': 0,
    }, indent=2))


if __name__ == '__main__':
    main()
