"""Credential-free integration test of real colcon, Debian packaging and underlays.

Uses the caller's release label and identity. The two tiny CMake packages are
synthetic fixtures; this does not fetch/build the entire private distribution.
"""
import json
import os
from pathlib import Path
import subprocess

import yaml

from tailor_distro.blossom import Graph, GraphPackage


def run(*args):
    subprocess.run([str(arg) for arg in args], check=True)


def source(distro, name, cmake, files):
    directory = Path('workspace/src') / distro / name
    directory.mkdir(parents=True)
    (directory / 'CMakeLists.txt').write_text(cmake)
    (directory / 'package.xml').write_text(f'''<package format="3">
  <name>{name}</name><version>1.0.0</version><description>Tailor smoke fixture</description>
  <maintainer email="ci@example.invalid">Tailor CI</maintainer><license>MIT</license>
  <export><build_type>cmake</build_type></export>
</package>''')
    for path, text in files.items():
        (directory / path).write_text(text)
    return GraphPackage(name, '1.0.0', 'abcdef0', str(directory.resolve()), distro, [], [],
                        description='Tailor smoke fixture', maintainers='Tailor CI <ci@example.invalid>')


def main():
    identity = json.loads(Path('/identity/build-plan.json').read_text())
    base = source('ros1', 'smoke_base', '''cmake_minimum_required(VERSION 3.16)
project(smoke_base)
install(FILES smoke_value.h DESTINATION include)
install(FILES smoke_baseConfig.cmake DESTINATION share/smoke_base/cmake)
''', {'smoke_value.h': '#define SMOKE_VALUE 42\n',
      'smoke_baseConfig.cmake': 'get_filename_component(SMOKE_PREFIX "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)\n'
                               'set(SMOKE_INCLUDE "${SMOKE_PREFIX}/include")\n'})
    overlay = source('ros2', 'smoke_overlay', '''cmake_minimum_required(VERSION 3.16)
project(smoke_overlay)
find_package(smoke_base REQUIRED CONFIG)
add_executable(smoke-check main.cpp)
target_include_directories(smoke-check PRIVATE ${SMOKE_INCLUDE})
install(TARGETS smoke-check DESTINATION bin)
''', {'main.cpp': '#include <smoke_value.h>\nint main() { return SMOKE_VALUE == 42 ? 0 : 1; }\n'})
    overlay.ros1_depends = ['smoke_base']
    graph = Graph('ubuntu', os.environ['OS_VERSION'], identity['label'], identity['timestamp'], '',
                  organization='toydistro', init_apt=False,
                  packages={'ros1': {'smoke_base': base}, 'ros2': {'smoke_overlay': overlay}})
    graph.finalize()
    graph.write_yaml(Path('graphs'))
    recipe = {
        'common': {'organization': 'toydistro', 'cxx_flags': [], 'cxx_standard': 17, 'python_version': '3',
                   'default_build_depends': [], 'distributions': {
                       'ros1': {'env': {'ROS_VERSION': '1'}, 'os': []},
                       'ros2': {'env': {'ROS_VERSION': '2'}, 'underlays': ['ros1'], 'os': []}}},
        'flavours': {'smoke': {'description': 'Smoke installation set', 'distributions': {
            'ros1': {'root_packages': ['smoke_base']}, 'ros2': {'root_packages': ['smoke_overlay']}}}},
    }
    Path('recipes.yaml').write_text(yaml.safe_dump(recipe))
    run('bash', '/tailor-meta/ci/build-os.sh', f'graphs/ubuntu-{os.environ["OS_VERSION"]}-graph.yaml',
        'recipes.yaml', os.environ['OS_VERSION'], 'true')
    debs = sorted(Path('.').glob('*.deb'))
    assert len(debs) >= 5, 'Expected source, environment and bundle Debian packages'
    run('dpkg', '-i', *debs)
    run(f'/opt/toydistro/{identity["label"]}/ros2/bin/smoke-check')
    for distro in ('ros1', 'ros2'):
        report = yaml.safe_load(Path(f'build-report-{os.environ["OS_VERSION"]}-{distro}.yaml').read_text())
        assert report['rebuilt_count'] == 1, report
    Path('smoke-result.json').write_text(json.dumps({
        'config_sha': identity['config_sha'], 'implementation_sha': identity['implementation_sha'],
        'label': identity['label'], 'os': os.environ['OS_VERSION'],
        'packages': [path.name for path in debs], 'installed_binary_exit_code': 0,
    }, indent=2))


if __name__ == '__main__':
    main()
