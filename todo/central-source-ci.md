# Central source CI: polling first, GitHub App later

## Decision

Replace secret-bearing source-repository workflow callers with centrally
executed source CI. Start with polling and `locusrobotics/test_meta`; later
replace polling with GitHub App events without changing the execution boundary.

Source repositories must not hold AWS credentials or tokens granting access to
the source fleet. Keep `source.test_commits: true` as the configuration opt-in.
Toydistro/rosdistro remain authoritative for source branches, ROS distributions,
target OSes, and published testing environments.

Do not bake long-lived credentials into runner images. Trusted EC2 runners can
use IAM instance roles; GitHub credentials stay central and are injected only
into trusted jobs. Untrusted source builds run on separate disposable workers.

## Architecture

```text
central schedule / manual dispatch
  → discover opted-in branch commits and PR revisions
  → validate and persist a test request
  → trusted preparation downloads exact source and published dependencies
  → separate disposable worker builds/tests without infrastructure credentials
  → trusted reporting posts status on the original source commit

future GitHub App event
  → same validated request and execution path
```

Use tailor-meta as the initial central execution repository. No source CI stub
or webhook is required for polling. Keep existing source-workflow generation
separate during transition; do not install secret-inheriting hooks as the new
testing mechanism.

## Runner and credential boundaries

### Trusted discovery/preparation/reporting

- [ ] Restrict the central runner/group to the central CI repository; do not
      expose it to arbitrary source-repository workflows.
- [ ] Give an EC2 runner a scoped AWS read instance role instead of static keys.
- [ ] Store a central PAT in Actions secrets or a secret manager, with read access
      to selected source repositories/PR metadata and commit-status write access.
- [ ] Inject the PAT only into trusted discovery, preparation, and reporting jobs.
- [ ] Do not execute source code, repository-provided scripts, dependency install
      hooks, or untrusted build configuration on these runners.
- [ ] Keep publication credentials and signing keys off the source-CI preparation
      runner; use a separate publication identity/runner.

### Disposable source-test workers

- [ ] Use separate hosted runners or ephemeral self-hosted workers with no AWS
      instance role, fleet token, publication access, or signing keys.
- [ ] Transfer prepared source and dependency inputs, not authentication files.
- [ ] Do not share the preparation runner's filesystem, Docker daemon, or socket.
- [ ] Ensure workers cannot obtain credentials through instance metadata, secret
      stores, shared caches, or leftover files.
- [ ] Destroy worker state after completion, failure, or cancellation.
- [ ] Treat source code and build scripts as untrusted even for internal PRs.

Unsetting environment variables or separating steps on the same credentialed
runner is not sufficient isolation. A privileged container or Docker socket can
expose the host; workers must not reach a credentialed host.

## Phase 1: discovery and validated requests

- [ ] Read opted-in sources from pinned configuration repository/ref snapshots.
- [ ] Discover configured branch heads and relevant PR revisions; start with
      test_meta only.
- [ ] Poll on a central schedule, initially around five minutes, and support
      explicit manual dispatch. Account for GitHub schedule delays.
- [ ] Batch/stagger discovery, use conditional requests where appropriate, and
      respect API limits/backoff as the repository fleet grows.
- [ ] Define a stable request identity including repository, exact source SHA,
      ROS distribution, configuration SHA, and published environment identity.
- [ ] Persist deduplication and queued/running/completed state outside ephemeral
      runner workspaces.
- [ ] Validate repository/branch opt-in and derive images/commands from trusted
      configuration; reject arbitrary repositories, URLs, commands, or credential
      requests supplied through dispatch inputs.
- [ ] Define policy for PR merge versus head revisions and tests targeting
      different source/release branches.
- [ ] Queue only newly discovered or deliberately requested test identities.

## Phase 2: trusted preparation

- [ ] Download the exact source SHA being tested using central source access.
- [ ] Resolve immutable test-image digests and published dependency snapshots.
- [ ] Download required private images/packages using the central AWS read role.
- [ ] Produce a credential-free input bundle containing source, dependency
      artifacts, graph/configuration metadata, and checksums.
- [ ] Choose transfer/storage and worker access mechanisms that do not require
      fleet-wide credentials on workers. Avoid publicly exposing private inputs.
- [ ] Replace source-CI scripts that fetch the entire source fleet with prepared
      published dependencies and graph artifacts where possible.
- [ ] Define dependency/removed-package checks without granting the worker access
      to unrelated repositories.
- [ ] Avoid running fetched source code while creating the input bundle.

## Phase 3: isolated execution and reporting

- [ ] Implement one central source-test workflow with trusted preparation,
      isolated execution, and trusted reporting jobs.
- [ ] Run dependency checks, colcon build/test, and explicit test-result
      enforcement on the worker using prepared inputs.
- [ ] Preserve ROS underlay and per-OS isolation semantics.
- [ ] Post pending/success/failure/error commit statuses with stable contexts such
      as `tailor/ros1`; link to the central Actions run and test reports.
- [ ] Complete statuses on preparation failure, worker failure, cancellation,
      timeout, and reporting retries; do not leave permanent pending statuses.
- [ ] Cancel superseded PR revisions and report their cancellation without
      conflating distinct configuration/environment test identities.
- [ ] Apply an explicit review/approval policy for fork PRs and private inputs.
- [ ] Ensure source workers cannot forge authoritative results by obtaining the
      reporting identity.

## Phase 4: test_meta pilot and cutover

- [ ] Preserve test_meta's `test_commits` opt-in and feature-environment selection
      in toydistro configuration.
- [ ] Configure the central runner, read role, PAT, and uncredentialed worker.
- [ ] Publish/verify the selected test environment before requiring source checks.
- [ ] Test a push and PR, then confirm polling queues the exact revision and posts
      status on that revision.
- [ ] Test build failure, preparation failure, cancellation, duplicate discovery,
      and a superseding PR commit.
- [ ] Confirm no AWS credentials or fleet-access token are configured in test_meta
      or observable by its build scripts.
- [ ] Verify input artifacts/caches do not leak credentials or expose private
      dependencies to unauthorized readers.
- [ ] Update required branch-protection contexts only after result reporting is
      reliable.
- [ ] Retire obsolete secret-inheriting source hooks and legacy Jenkins triggers
      deliberately; preserve unrelated CI and repository changes.
- [ ] Expand polling to other opted-in repositories after the pilot passes.

## Phase 5: migrate discovery to a GitHub App

- [ ] Introduce an App/webhook receiver for push, PR, and optional rerun events.
- [ ] Verify signatures, deduplicate deliveries, and validate requests against the
      same configuration opt-in rules.
- [ ] Feed events into the existing persisted request queue rather than creating
      a second execution path.
- [ ] Replace the central PAT with narrowly scoped short-lived App tokens.
- [ ] Optionally replace commit statuses with App check runs while coordinating
      required-check context changes.
- [ ] Disable polling after event delivery/recovery is verified, or retain it as
      a low-frequency reconciliation mechanism.
- [ ] Keep preparation, worker isolation, publication separation, and artifact
      handling unchanged.

## Acceptance criteria

- Source repositories contain no AWS secrets or fleet-access tokens.
- Source code executes only on workers without infrastructure credentials.
- Every test is attributable to exact source/configuration/environment identities.
- The pilot produces reliable commit statuses, including failures/cancellations.
- Polling can be replaced by App events without redesigning the build worker.

Related plans: `source-ci.md`, `credentials-and-approvals.md`, `cutover.md`, and
`reproducibility.md`. Direct source-stub maintenance and its future auto-merging
PR workflow remain separate from central source-test execution.
