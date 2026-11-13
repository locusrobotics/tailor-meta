# Python cleanup

Turn the imported command-line scripts into small libraries with thin CLI
wrappers, then remove duplicated infrastructure and obsolete publishing paths.
Keep the existing Python namespaces initially; the repository consolidation is
already complete.

## 1. Clarify namespace responsibilities

- [ ] Extract shared process, APT, storage, and hashing helpers from build-specific
      modules. In particular, `build_images.py` should not import APT setup from
      `build_os.py`, and `build_os.py` should not import hashing from `actions.py`.
- [ ] Separate planning, orchestration, container execution, and repository
      maintenance within `tailor_meta`.
- [ ] Keep release-management commands separate from buildfarm execution.
- [ ] Preserve the `debian_packager` colcon extension boundary.

Suggested destination layout:

```text
tailor_meta/
  cli.py
  plan.py                 # build identity, release classification, stage state
  pipeline.py             # workflow-level orchestration
  containers.py           # container execution and scoped mounts
  toolchains.py
  runtime/
    process.py
    apt.py
    storage.py
  repositories/
    workflows.py
    settings.py

tailor_distro/
  graph.py
  resolve.py
  sources.py
  recipes.py
  build.py
  bundles.py
  publish.py
  retention.py
  manage/

tailor_image/
  build.py
  docker.py
  disk.py                 # shared bare-metal/LXD preparation
  ami.py
  index.py
  retention.py

debian_packager/
```

Treat this as a destination, not a bulk file-renaming exercise. Extract
responsibilities as their callers are simplified.

## 2. Make CLI entry points thin

- [ ] Extract library functions from commands that currently combine argument
      parsing, filesystem changes, subprocess execution, and business logic.
- [ ] Have `build_os` call Python functions instead of spawning other installed
      Python CLI commands where practical.
- [ ] Keep subprocesses for external tools such as colcon, Docker, apt, and Packer.
- [ ] Replace library-level `exit()` calls with exceptions or returned results.
- [ ] Test library functions directly rather than patching `sys.argv`.
- [ ] Retain existing console-script names as compatibility wrappers while
      introducing a consistent public CLI.

Example boundary:

```python
def build_packages(context, graph, distro) -> BuildResult:
    ...


def main():
    args = parse_args()
    result = build_packages(...)
    return result.exit_code
```

## 3. Reduce oversized package initializers

- [ ] Move implementation out of `tailor_distro/__init__.py` and
      `tailor_image/__init__.py`, then review `debian_packager/__init__.py`.
- [ ] Leave package metadata and a small set of intentional exports in
      `__init__.py` files.
- [ ] Consolidate duplicated `YamlLoadAction`, subprocess wrappers, S3 URL parsing,
      and CloudFront invalidation.
- [ ] Provide one subprocess helper supporting `cwd`, explicit environments,
      checked failures, and redacted logging.

## 4. Remove obsolete package-publication code

The active package publisher uses deb-s3. `tailor_distro/__init__.py` still
contains an aptly package-publication implementation; several repository
add/list/remove/publish helpers have no production callers in this checkout,
and some remain referenced only by tests.

- [ ] Check compatibility consumers before removing unused helpers.
- [ ] Retain aptly functionality used for upstream mirroring.
- [ ] Remove the obsolete aptly package-publication path and its obsolete tests.
- [ ] Remove ignored parameters such as `cleanup_stale_s3`.
- [ ] Trim stale `PublishPlan` fields and misleading documentation.

## 5. Separate graph data from dependency resolution

`blossom.Graph` currently combines serializable data, APT/rosdep initialization,
dependency resolution, rebuild selection, and YAML persistence.

- [ ] Introduce a `PackageGraph` containing data and graph algorithms.
- [ ] Extract package discovery, rosdep/APT queries, and candidate-version lookup
      into a resolver.
- [ ] Separate serialization helpers from resolution.
- [ ] Ensure loading an existing graph does not initialize rosdep merely to
      inspect dependencies or calculate rebuilds.
- [ ] Replace the opaque `blossom.py` name with a discoverable module name.

## 6. Simplify image backends around explicit results

- [ ] Decompose `tailor_image/create_image.py` into straightforward backend
      functions, for example:

      ```python
      build_docker_image(spec, context) -> ImageArtifact
      build_disk_image(spec, context) -> ImageArtifact
      build_ami(spec, context) -> ImageArtifact
      ```

- [ ] Separate building, publication, and index updates where the backend permits
      it. AMI creation inherently creates AWS resources; make that explicit.
- [ ] Replace global `os.chdir()` with subprocess `cwd` arguments.
- [ ] Manage temporary containers and files with context managers or
      `try/finally` cleanup.
- [ ] Normalize defaults once before backend execution.
- [ ] Replace long positional argument lists with `ImageSpec` and `BuildContext`.
- [ ] Pass Packer variables through a temporary JSON file rather than assembling
      long command lines; clean up sensitive variable files reliably.
- [ ] Define consistent nonpublishing behavior across backends instead of
      surprising no-ops.

## 7. Simplify image-index updates

The current S3 tag-based lock is non-atomic and eventually proceeds after a
timeout. Image builds are now sequential within a distribution run.

- [ ] Collect image results and update the index once per run.
- [ ] Use an ETag-conditional write with conflict retry to handle other writers.
- [ ] Remove lock-tag polling, randomized waits, and unconditional lock takeover.

## 8. Modernize Python packaging

- [ ] Move `setup.cfg`/`setup.py` metadata into `pyproject.toml`.
- [ ] Remove `pytest-runner`, `setup_requires`, and `tests_require`.
- [ ] Keep one installable project with optional dependency groups for
      orchestration/management, package building, imaging, and development.
- [ ] Avoid installing colcon, bloom, and AWS CLI merely to generate source
      workflow stubs.

## Recommended implementation order

1. Extract shared process/APT helpers and reduce `__init__.py` files.
2. Introduce library functions beneath existing CLI wrappers.
3. Remove obsolete publication code.
4. Split image backends and simplify index updates.
5. Separate graph data from resolution.
6. Modernize packaging.

Prioritize removing coupling and duplicate behavior over merging all three
namespaces into one. Keep target OS selection in toydistro/rosdistro configuration.
