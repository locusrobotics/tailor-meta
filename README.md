# tailor-meta

Tailor's centralized GitHub Actions buildfarm: distribution packaging, image
provisioning, publication, and source CI. `rosdistro` remains the release source
of truth. Consumer repositories contain only reusable-workflow caller stubs.

The former `tailor-distro` and `tailor-image` implementations live here under
their original Python namespaces and CLI names. The sibling repositories remain
untouched. See [ci/IMPORTS.md](ci/IMPORTS.md) for provenance.

## Workflows

| Reusable workflow | Purpose |
| --- | --- |
| `distribution.yaml` | Validate, plan, fetch, mirror, graph, package, publish, image |
| `maintain-sources.yaml` | Generate/apply source stubs independently of build credentials |
| `source.yaml` | Dependency checks, colcon build/test, test-result enforcement |
| `smoke.yaml` | Hosted Jammy/Noble packaging integration test without private credentials |
| `python.yaml` | Unit tests and Actions linting |

All live under `.github/workflows/`. Pin the caller's `uses` ref and
`tailor_meta` input to the **same immutable implementation SHA**:

```yaml
jobs:
  tailor:
    uses: locusrobotics/tailor-meta/.github/workflows/distribution.yaml@<sha>
    with:
      tailor_meta: <sha>
      mode: release
    secrets: inherit
```

`tailor-meta` is public, so hosted validation and smoke tests can check out its
pinned implementation without cross-repository secrets. PR configuration is
parsed with safe YAML and sandboxed Jinja; it never runs on the buildfarm.

## Modes and stages

| Mode | Work performed | Remote writes |
| --- | --- | --- |
| `validate` | Offline configuration/template parsing on a hosted runner | None |
| `graph` | Plan, fetch sources, resolve graphs using published dependencies | None |
| `packages` | Graph mode plus all OS package sets | None |
| `release` | Fetch, publish mirrors, graph, build every OS, publish packages, build/publish images | Yes |

`force_mirror`, `invalidate_docker_cache`, and `invalidate_colcon_cache` remain
independent controls. Read-only modes need an existing dependency mirror;
`release` bootstraps new labels. Images consume newly published packages, so
image provisioning belongs to `release` mode.

Each stage is a separate Actions step and `tailor_actions` command. One
`build-plan.json` records the timestamp, configuration/implementation commits,
configuration digest, resolved source revisions, package checksums, and completed
stages. Subsequent stages load it instead of recomputing build identity, reject
configuration changes, and enforce predecessor success. The plan is archived
on failure and uploaded to S3 after publication. A failed stage is not marked
complete. Rerunning the workflow creates a new run attempt, not an implicit
resume of partially published artifacts.

All OS builds finish **before package publication starts**. Publishing two APT
distributions is not an atomic transaction: a publication failure can still
leave a partially published label. Build failures, however, cannot publish a
successful OS early. ROS 1, ROS 2, and bundles share one container per OS;
build/install/optinstall trees never cross OS boundaries. Dependency collection
covers all graph packages, without requiring a flavour named `dev`.

## Branch and release policy

* `master` → `hotdog`; automatic push and 02:00 UTC nightly publication.
* `feature/name` → `feature-name`; **automatic publication for testing**.
* `release/26.1` → `26.1-rc`; automatic candidate publication.
* `26.1.0` tag → `26.1.0`; immutable final publication.
* PRs → hosted validation only.

Publication approval is intentionally deferred. The workflow currently has no
GitHub environment approval gate.

### Immutable final releases

Before a final release writes its mirror, it checks for existing objects under
the release label and atomically creates `release-claim.json` with S3
`If-None-Match: *`. Legacy published labels and concurrent claims are rejected.
Later write stages verify ownership against repository, run attempt, and both
commit SHAs. Failed claims remain: retrying the same tag must not overwrite a
partially published final release. Recovery requires explicit operator action,
or a new release version. Do not remove claims automatically.

Final publication disables package/image retention deletion. Hotdog/features
retain 30 days/30 versions; candidates retain 30 versions without an age limit.
These are separate from GitHub artifact retention (30 days diagnostics, 7 days
nonpublishing packages/smoke results). The claim is enforced by this pipeline;
legacy Jenkins/manual publishers must be disabled and IAM access scoped so they
cannot bypass it. Publication roles require conditional S3 PutObject access.

## Credentials and runners

Buildfarm stages require dedicated, preferably ephemeral Ubuntu amd64 runners
with `self-hosted`, `linux`, `x64`, `tailor` labels, Python 3.12+, Docker/BuildKit,
and the host modules/devices required by image provisioning. Source PR tests run
on isolated GitHub-hosted runners, never these privileged runners.

Credentials are scoped to Actions steps **and** container capabilities:

| Secret/variable | Scope |
| --- | --- |
| `TAILOR_SOURCE_TOKEN` | Read-only private source access in fetch/package stages |
| `AWS_READ_ACCESS_KEY_ID`, `AWS_READ_SECRET_ACCESS_KEY`, optional `AWS_READ_SESSION_TOKEN` | Graph/package dependency reads |
| `AWS_ACCESS_KEY_ID`, `AWS_SECRET_ACCESS_KEY`, optional `AWS_SESSION_TOKEN` | Mirror/package/image publication only |
| `ANSIBLE_VAULT_PASS` | Image provisioning only |
| `TAILOR_GPG_DIR` variable | Runner-local signing-key directory, mounted only for mirror/package publication |
| `TAILOR_MAINTENANCE_TOKEN` | Separate maintenance job: write source contents/workflows; no buildfarm access |

Use separate read/write AWS identities. Never put signing keys, credentials, or
authenticated URLs into Git. Dockerfiles declare secret mounts directly;
there is no runtime Dockerfile rewriting. APT credentials and Git URL rewriting
are temporary runtime mounts, with Git auth mounted where the package builder's
clean environment expects it. Build containers do not receive publishing AWS
credentials, vault passwords, or signing keys. Build contexts use an allowlist
and exclude source/build trees, `.git`, and caches.

OIDC roles and short-lived GitHub App token issuance are a follow-up deployment
improvement; the current interface uses separate secret-backed identities.
Docker keeps its normal local layer cache. Shared compiler/registry caches are
also follow-up work; ephemeral runners currently use run-local compiler caches.

## Source maintenance

`maintain-sources.yaml` generates a preview by default; `apply: true` writes
directly to configured source branches. This is intentional for the large
repository fleet. `source.test_commits` selects repositories; it does not select
distribution contents. Jobs are aggregated by GitHub repository **and source
branch**, retaining one job per ROS distribution so shared branches do not
overwrite each other's stubs. The generator is offline without `--deploy`:

```bash
create_pipelines --rosdistro-index ../rosdistro/rosdistro/index.yaml \
  --recipes ../rosdistro/config/recipes.yaml --meta-branch <sha> \
  --release-track hotdog --release-label hotdog --output-dir /tmp/opencode/stubs
```

Source CI needs its own read-only AWS credentials (named `AWS_ACCESS_KEY_ID` and
`AWS_SECRET_ACCESS_KEY`) and `TAILOR_SOURCE_TOKEN`. Never share publication or
maintenance identities with source tests. Fork PRs skip private-environment
tests automatically; after review a maintainer can dispatch `source_ref:
pull/123/merge` on an isolated hosted runner. Native Actions checks replace the
Jenkins status context; update branch protections during cutover.

## Hosted toydistro pilot

`locusrobotics/toydistro` branch `feature/github-actions-smoke` calls validation
and `smoke.yaml` with an immutable implementation pin. Until infrastructure is
configured, this pilot deliberately runs hosted checks rather than publishing.
Its manual dispatch supports switching to the normal modes.

The smoke test builds two tiny CMake fixture packages using the real
`build_packages`/colcon Debian extension and `build_bundles`. The ROS 2 fixture
requires a header installed by the ROS 1 fixture. It installs the generated
debs, executes the resulting binary, and archives reports and artifacts on
Jammy and Noble. It uses the caller's release label and configuration identity.
This verifies container packaging/underlay behavior, not the full private
toydistro source set, AWS signing/publication, or production image provisioning.

## Cutover and follow-ups

1. Publish the implementation SHA before pushing caller stubs that reference it.
2. Run hosted validation/smoke tests, then graph and package builds with read-only
   identities. Configure signing keys, publication identities, and buildfarm runners.
3. Run a feature release in toydistro and inspect APT, images, plans, and reports
   before production cutover. Final-tag reruns must be rejected.
4. Apply source stubs directly after preview. Disable old Jenkins jobs/hooks and
   update required checks; source Jenkinsfiles are not blindly deleted.

Explicit follow-ups: publication approval gates; source-update PRs that
**auto-merge once checks pass** (not manual merging across the fleet); external
integration-test migration; distro-triggered source-test fan-out; Slack
notifications; OIDC/App token provisioning; shared build caches.

## Local checks

```bash
python3 -m venv --system-site-packages .venv
.venv/bin/pip install -e . pytest==8.4.1
.venv/bin/pytest -o addopts= tests
.venv/bin/python ci/validate.py ../toydistro
actionlint -shellcheck= .github/workflows/*.yaml
```

Use Ubuntu system Python with `python3-apt` for graph tests. Python 3.10 remains
supported in Jammy containers. Keep generated recipes/build outputs outside
configuration repositories. No test command above publishes remotely.
