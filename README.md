# tailor-meta

Tailor's GitHub Actions buildfarm: distribution packaging, publishing, image
provisioning, and source-repository CI. `rosdistro` remains the release source
of truth; it contains configuration and a thin reusable-workflow caller.

The former `tailor-distro` and `tailor-image` implementations now live here,
with their Python namespaces and CLI names preserved. Those sibling repositories
are not modified by this migration. See [ci/IMPORTS.md](ci/IMPORTS.md) for the
imported versions.

## Reusable workflows

| Workflow | Caller | Responsibility |
| --- | --- | --- |
| `.github/workflows/distribution.yaml` | rosdistro | Validate configuration, build packages, publish, provision images, update source stubs |
| `.github/workflows/source.yaml` | Opted-in source repositories | Dependency checks, removed-package checks, colcon build/test and test-result enforcement |
| `.github/workflows/python.yaml` | tailor-meta | Python/unit checks on Ubuntu, including system `python3-apt` |

All executable CI steps live in this repository. `create_pipelines` generates
only `.github/workflows/tailor.yaml` caller stubs for manifest sources with
`source.test_commits: true`. It is inspection-only unless `--deploy` is supplied.
The generator preserves each source branch and distro OS restrictions; it does
not alter release pins or package allowlists.

Caller `uses` refs and the `tailor_meta` input **must identify the same revision**:

```yaml
jobs:
  tailor:
    uses: locusrobotics/tailor-meta/.github/workflows/distribution.yaml@<tailor-meta-sha>
    with:
      tailor_meta: <tailor-meta-sha>
      deploy: true
    secrets: inherit
```

Use an immutable commit SHA for production. The workflow input selects the
implementation checkout; do not use `github.workflow_sha`, which can identify
the caller rather than the reusable implementation. GitHub must permit caller
repositories to access tailor-meta's reusable workflows.

## Build policy

* `master`: `hotdog` track/label; nightly at 02:00 UTC.
* `feature/name`: `hotdog` track, lowercase `feature-name` label.
* `release/26.1`: `26.1` track, `26.1-rc` label.
* `26.1.0` tag: `26.1` track, `26.1.0` label.
* Other branches and PRs: offline YAML/template validation only; no privileged
  buildfarm execution. Labels with unsupported Debian punctuation are rejected.

Trusted pushes and schedules publish automatically through the
`tailor-production` environment. Manual dispatch defaults to no publication and
supports `deploy`, `graph_only`, `force_mirror`, `invalidate_docker_cache`,
`invalidate_colcon_cache`, and `update_sources`. GitHub environment approval
rules are an operational prerequisite, **not** something YAML creates.

The initial port deliberately builds OS targets sequentially on one dedicated
runner. ROS 1 is built before ROS 2, preserving the underlay and graph-based
rebuild behavior. The master recipes file remains the graph/package/bundle
input. Each OS's debs are isolated before publication. Image selection preserves
`os_versions` and `bundle_flavour`/`testing_flavour`, including the configured test
and dependency-checker images.

Nonpublishing builds do not provision images: they would otherwise consume the
previous published APT snapshot instead of the just-built debs. Graph-only mode
skips mirroring, packaging, images, publication, and remote cleanup. Neither mode
is a full offline build; source retrieval and published dependency reads remain.

APT/S3 cleanup retains 30 versions and, for hotdog/feature labels, 30 days.
Candidate/final labels have no age cutoff. GitHub diagnostics retain 30 days;
nonpublishing deb artifacts retain 7 days. These are distinct retention systems.
CDN invalidation and cleanup run only when publishing.

## Runner and credential setup

Distribution builds require dedicated, preferably ephemeral, **Ubuntu amd64**
self-hosted runners with labels `self-hosted`, `linux`, `x64`, `tailor`, Python
3.12+, Docker with BuildKit, AWS CLI, jq, and the host kernel/provisioning tools
needed by the configured image types. Bare-metal/LXD builds require privileged
containers, host devices, modules, and Docker socket access. Never attach public
or unreviewed PR workflows to these runners.

Create these environments in **rosdistro**, before enabling the caller:

* `tailor-production`: required reviewers and approved deployment branch/tag
  rules; hold publishing credentials here.
* `tailor-build`: read-only access to private sources/APT dependencies.

Secrets available to the reusable distribution workflow:

* `TAILOR_GITHUB_TOKEN`: read private checkouts/source archives; when source stub
  management is enabled, write repository contents and workflows on opted-in
  source branches. Prefer a short-lived GitHub App token scoped to those repos.
* `AWS_ACCESS_KEY_ID`, `AWS_SECRET_ACCESS_KEY`, optional `AWS_SESSION_TOKEN`:
  access to the configured S3/ECR/CloudFront resources. Production image recipes
  can additionally need EC2/AMI permissions. Build-only credentials must not
  permit writes.
* `ANSIBLE_VAULT_PASS`: configured provisioning vault access.
* Environment/repository variable `TAILOR_GPG_DIR`: runner-local directory of
  signing `*.key` files, mounted read-only at runtime. Never put key files in Git.

Environment images are local-only. `tailor_meta.actions.secure_dockerfile` adapts
the imported legacy Dockerfiles to BuildKit secret mounts and removes legacy
credential ENV/auth/vault/git-config layers. Runtime APT auth and vault data use
temporary read-only mounts, removed on completion/failure. Published Docker
image recipes also mount APT auth as a secret; credentials are not copied into
image layers. Use fresh test images from this migration before source CI cutover;
old cached images can contain credentials.

Source CI runs on disposable GitHub-hosted Ubuntu runners. Provide **separate
read-only** AWS ECR/APT credentials and `TAILOR_SOURCE_TOKEN` with read-only
source access in source repositories. Production credentials must not be
inherited by source tests. Fork PRs cannot receive repository secrets and are
skipped automatically. After review, a maintainer can dispatch the source stub
with `source_ref: pull/123/merge`; it runs on an isolated hosted runner with only
read-only access. Tests enforce `colcon test-result`, not merely `colcon test`.

## Cutover

1. Publish this implementation revision, enable reusable-workflow access, and
   configure environments, runners, signing keys, and scoped credentials.
2. Point both rosdistro caller refs at that revision. Run configuration/unit
   checks, then a manual `graph_only` build, then a nonpublishing package build.
3. Approve a hotdog publishing run. Inspect package graphs, SHA logs, build
   reports, APT candidates, all image types, and rebuilt CI environments.
4. Enable `update_sources` after reviewing generated source stubs. Updating
   stubs is a remote write; do not use `--deploy` just to validate. Existing
   source Jenkinsfiles are not deleted automatically; remove only generated
   Tailor Jenkinsfiles deliberately during source-repository cutover.
5. Update branch-protection required check names to the native Actions checks;
   disable old Jenkins jobs/hooks externally. Jenkins definitions are removed
   from tailor-meta and rosdistro. The untouched sibling repos still contain
   historical Jenkins definitions and must no longer be scheduled by Jenkins.

The external `ci_integration_tests` pipeline and comment-triggered integration
tests are explicitly **not migrated** in this change. Source builds triggered
by a completed distro build (rather than source pushes/manual dispatch) also
need a cross-repository Actions trigger as a follow-up. Slack notification
integration is not ported; failures are visible in native Actions job results.

## Local checks (no deployment)

```bash
python3 -m venv --system-site-packages .venv
.venv/bin/pip install -e . pytest==8.4.1
.venv/bin/pytest -o addopts= tests
.venv/bin/python ci/validate.py ../rosdistro
actionlint -shellcheck= .github/workflows/*.yaml
```

Use Ubuntu system Python with `python3-apt` installed for graph tests. Python
3.10 remains supported for Jammy packaging containers; Actions checks use 3.12.
`tailor_manage query` and `create_recipes` retain their existing CLI semantics.
Keep generated recipes/build outputs outside rosdistro. Parsing/unit checks do
not verify source refs, package existence, real builds, signing, AWS permissions,
or image provisioning; those require the configured buildfarm.
