# tailor-meta

Centralized Actions, toolchains, and builders for Tailor. Consumers contain
configuration and thin workflow callers. Imported tools retain their Python
namespaces and CLI names; sibling repositories remain untouched. See
[ci/IMPORTS.md](ci/IMPORTS.md) for provenance.

## Architecture

```text
tailor-meta change
  ├─ ci.yaml: unit/lint tests + native-host packaging smoke test
  └─ toolchain-images.yaml: publish generic tools/imager to GHCR

rosdistro configuration change
  ├─ toolchain-images.yaml: caller-scoped builders for recipe-selected OSes
  └─ distribution.yaml
       plan + pull immutable digests
       fetch → mirror → graph → build every OS → publish → images

source repositories → source.yaml
source stub management → maintain-sources.yaml
```

These are the five workflows in `.github/workflows/`. No standalone smoke
workflow or check-to-Python workflow chain remains. CI can also be called by
toydistro's hosted pilot. Source maintenance is independent of build credentials.

## Toolchains, built once per implementation

`environment/toolchains/Dockerfile` has three targets:

| GHCR package/tag | Contents |
| --- | --- |
| `tailor-meta:tools-<sha>` | Source/graph tools, aptly, deb-s3 |
| `<configuration-repository>:builder-<sha>-<os>` | Compilers, colcon, Debian packaging, Rust; OS tags selected by caller recipes |
| `tailor-meta:imager-<sha>` | Ansible, Packer, Docker provisioning tools on the host's native OS |

There is one GHCR package per existing repository, not separate `-builder`,
`-tools`, or `-imager` packages. Toydistro uses `ghcr.io/locusrobotics/toydistro`
with builder tags; rosdistro uses its own package. **No target OS list or OS-specific builder package
lives in tailor-meta.** The reusable image factory reads the caller's
`config/recipes.yaml`; the generic Dockerfile only takes an OS parameter.
Tools/imager use the CI host's native OS independently of distribution targets.

Images contain no distro configuration,
source archives, bundles, signing keys, or credentials. `toolchain-images.yaml`
publishes SHA discovery tags using the Actions token. Existing SHA tags are not
overwritten; rebuilding toolchains requires a new implementation commit. GHA
BuildKit caches accelerate publication.

Set the GHCR packages **public** before distribution cutover. GitHub may create
them as private even though the source repository is public. Consumers should
pull toolchains anonymously rather than needing ECR or registry secrets.

Planning resolves discovery tags to immutable `@sha256:...` references and
verifies implementation/role/OS labels. The build plan records those digests;
later stages use them, never mutable tags. Explicit local image IDs are allowed
only for CI fixtures.

Distribution runs no longer build environment images or generate packaging
Dockerfiles. They install distro dependencies inside disposable containers.
Private packages and authentication are not committed back to toolchain images.

## One build-os path

`tailor_actions build-os --plan build-plan.json --os <configured-os>` starts one pinned
builder container. Its `tailor-build-os` command owns workspace preparation,
dependency installation, ROS 1 → ROS 2 → bundles, and package output/checksums.

`tailor_actions packages` invokes this sequentially for all configured OS targets.
All must succeed before publication starts. The commands share one container per
OS to preserve installed underlays; build/install/optinstall trees remain
OS-specific. Sources use a stable `/work` mount. Dependencies cover every graph
package and do not require a `dev` flavour.

CI smoke tests prepare two small CMake fixtures and resolved graph inputs, then
use **the same Build.build_os path and production builder Dockerfile**. They
install seven generated debs in a fresh builder container and execute the ROS 2
binary requiring the ROS 1 package. Caller CI tests OSes from its recipes;
tailor-meta's own CI tests only the runner's native OS. This tests runtime dependency
setup, filesystem paths, packaging, installation, and underlays. Private source
fetching, signing, AWS publication, and production image provisioning still need
buildfarm verification.

## Actions structure

Pin the caller `uses` ref and `tailor_meta` input to the **same commit**. Call the
image factory first with `packages: write` so caller-scoped builders are ready.
Generic tools/imager must already have finished publishing from tailor-meta:

```yaml
jobs:
  builders:
    uses: locusrobotics/tailor-meta/.github/workflows/toolchain-images.yaml@<sha>
    permissions:
      contents: read
      packages: write
    with:
      tailor_meta: <sha>
  tailor:
    needs: builders
    uses: locusrobotics/tailor-meta/.github/workflows/distribution.yaml@<sha>
    with:
      tailor_meta: <sha>
      mode: release
    secrets: inherit
```

`.github/actions/setup` handles Python installation and run-local workspace/plan
preparation. Actions owns triggers, concurrency, runners, scoped credentials,
visible stage steps, and artifacts. Python owns configuration interpretation,
container execution, checksums, and final-release claims.

| Mode | Work | Remote writes |
| --- | --- | --- |
| `validate` | Safe YAML/sandboxed Jinja checks on hosted runners | None |
| `graph` | Fetch and graph using existing published dependencies | None |
| `packages` | Graph plus every OS package set | None |
| `release` | Mirror, graph, packages, publish, provision images | Yes |

Read-only modes need existing dependency mirrors. `force_mirror` and
`invalidate_colcon_cache` are independent. Docker-cache invalidation is no longer
a distribution input: toolchains have their own build workflow. Dependencies
install afresh at runtime; compiler caches are run-local for now.

One `build-plan.json` records timestamp, configuration/implementation SHAs,
configuration digest, toolchain digests, resolved sources, package checksums,
and completed stages. It is archived on failure and uploaded after publication.
All OS builds finish before publication, but multiple APT publications are not
an atomic transaction: a publish-stage failure can still leave a partial label.

## Publication and credentials

`master` publishes hotdog automatically (also nightly at 02:00 UTC);
`feature/*` publishes artifacts for testing; `release/*` publishes candidates;
tags publish immutable finals. PRs never run on privileged buildfarm runners.
Publication approval remains a follow-up.

Final publication reserves an S3 `release-claim.json` with conditional
`If-None-Match: *`, rejecting existing legacy artifacts and competing/rerun
claims. Later writes verify ownership. Failed claims remain for deliberate
operator recovery; they are not automatically removed. Disable legacy/manual
writers and scope IAM so they cannot bypass this contract. Final deletion is
disabled. Hotdog/features retain 30 days/30 versions; candidates retain 30
versions without an age limit. GitHub artifacts have separate retention.

Buildfarm stages require dedicated, preferably ephemeral Ubuntu amd64 runners
labelled `self-hosted`, `linux`, `x64`, `tailor`, Python 3.12+, Docker, and the
host modules/devices required by image provisioning. CI/source tests use isolated
hosted runners. The buildfarm pulls toolchains; it does not build them.

| Secret/variable | Scope |
| --- | --- |
| `TAILOR_SOURCE_TOKEN` | Read-only private source fetching/build auth |
| `AWS_READ_ACCESS_KEY_ID`, `AWS_READ_SECRET_ACCESS_KEY`, optional `AWS_READ_SESSION_TOKEN` | Graph/package dependency reads |
| `AWS_ACCESS_KEY_ID`, `AWS_SECRET_ACCESS_KEY`, optional `AWS_SESSION_TOKEN` | Mirror/package/image publication |
| `ANSIBLE_VAULT_PASS` | Image provisioning only |
| `TAILOR_GPG_DIR` | Runner-local keys, mounted only for mirror/package signing |
| `TAILOR_MAINTENANCE_TOKEN` | Independent source-stub write job |

APT/Git/vault authentication uses temporary runtime mounts. Build containers
never receive publication credentials, vault passwords, or signing keys. The
imager installs the published build-flavour bundle at runtime before provisioning,
instead of rebuilding an environment image. Product images still publish to the
configured ECR destinations; GHCR is only for buildfarm tooling.

## Maintenance and cutover

`maintain-sources.yaml` previews by default; `apply: true` directly updates
configured source branches. `source.test_commits` selects repositories, not distro
contents. Jobs aggregate by repository/branch to avoid ROS distro collisions.
Source CI must use its own read-only credentials, not publication/maintenance
identities. Fork private tests require a reviewed manual dispatch on a hosted runner.

1. Publish the implementation and generic toolchains; call the image factory
   from configuration repositories to publish their builders. Make GHCR packages public.
2. Run CI/toydistro hosted checks, then graph/packages with read identities.
3. Configure signing/publication identities and run a real toydistro feature release.
4. Preview/apply source stubs, update required checks, and disable Jenkins jobs/hooks.

Explicit follow-ups: publication approvals, auto-merging source-update PRs once
tests pass, external integration tests, distro-triggered source fan-out, Slack,
OIDC/App tokens, and shared build caches. Sequential OS builds remain deliberate;
a matrix can follow once an artifact-transfer/storage contract is chosen.

## Local checks

```bash
python3 -m venv --system-site-packages .venv
.venv/bin/pip install -e . pytest==8.4.1
.venv/bin/pytest -o addopts= tests
.venv/bin/python ci/validate.py ../toydistro
actionlint -shellcheck= .github/workflows/*.yaml
```

Use Ubuntu system Python with `python3-apt` for graph tests. Python 3.10 remains
supported inside Jammy toolchains. Keep generated outputs outside configuration
repositories; the local checks above never publish remotely.
