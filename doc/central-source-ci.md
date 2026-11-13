# Central polling source CI

`source-poll.yaml` is a reusable workflow executed in the calling configuration
repository. All input/result artifacts and run logs belong to that repository,
not tailor-meta. Source repositories need only their manifest `test_commits`
opt-in: no workflow hook, AWS credential, or fleet-access token is required.

## Execution boundary

1. A trusted `tailor-source-control` runner resolves the configuration snapshot,
   discovers opted-in branch heads and approved same-repository PR heads, and
   claims new source/OS requests in an S3 ledger.
2. Trusted preparation downloads only the tested repository's exact source and
   published baseline SHAs, its published dependency graph, and its pinned test
   image. It never starts that image or executes fetched source.
3. Prepared bundles are uploaded as caller-owned Actions artifacts. Every saved
   image layer, config, and history is checked for known credential locations;
   images with detected baked credentials are rejected. This is a defensive
   check, not a proof that arbitrary images contain no secrets: operators must
   approve sanitized test environments.
4. A separate disposable GitHub-hosted worker downloads its bundle and runs
   offline in a container without AWS/fleet tokens, a host Docker socket,
   privileged mode, network access, or Linux capabilities. The worker does not
   run on the control machine or receive its IAM role.
5. Trusted reporting reads native Actions job conclusions, not source-controlled
   result JSON, then writes commit statuses on the original SHA. Later polls
   reconcile interrupted runs. Discovery/reporting share a control-plane lock;
   preparation/workers can overlap with later polls.

Requests include source/baseline SHAs, configuration SHA/ref, ROS namespace,
target OS, published environment identity, and immutable test-image digest.
Status contexts include configuration repository, label, ROS namespace, and OS
to avoid cross-distribution collisions. Superseded requests receive an error
status and obsolete runs are cancelled only when they have no other live work.

## Requirements

### Configuration and environment

- Call from toydistro/rosdistro with a pinned `tailor_meta` implementation.
- Select the configuration ref explicitly; OS selection and opt-in remain in
  that configuration, not tailor-meta.
- The feature environment must have completed publication using the updated
  publisher. Its S3 `build-plan.json` must contain `test_images` pinned by digest
  and baseline source SHAs. Its graph must exist under
  `<label>/graphs/<environment-timestamp>/ubuntu-<configured-os>-graph.yaml`.
- Images must contain all test/build dependencies and a usable rosdep cache.
  Workers do not install dependencies or fetch the source fleet. A new dependency
  absent from the published environment fails the check and requires an
  environment refresh.

### Trusted runner

Provide a dedicated, preferably ephemeral Ubuntu amd64 runner labelled
`self-hosted`, `linux`, `x64`, `tailor-source-control`, with Python 3.12+, venv,
Docker, and sufficient disk for image archives. Restrict its runner group to the
trusted configuration repositories. Do not let source repositories or PR jobs
schedule arbitrary work there. No publication credentials or signing keys belong
on this runner.

Use an instance role rather than baked access keys. It needs:

- Read access to the selected published environment's S3 plans/graphs.
- ECR authorization and image pull access, not ECR push.
- List/Get/conditional Put access to a **dedicated CI ledger prefix**. This small
  state-write permission is separate from distribution/publication permissions.

No infrastructure is created automatically. Provision the runner, role, and
private state bucket/prefix deliberately. Do not expire live ledger records.

### Caller secret and variables

| Setting | Meaning |
| --- | --- |
| Secret `SOURCE_CI_GITHUB_TOKEN` | Source/configuration contents read, PR metadata read, and commit-status write on selected repositories |
| Variable `SOURCE_CI_STATE_URI` | Dedicated private `s3://bucket/prefix`; state is further namespaced by caller repository |
| Variable `SOURCE_CI_ENABLED` | Set to `true` to enable scheduled polling after acceptance |
| Variable `SOURCE_CI_PUBLIC_INPUTS` | Explicit public-artifact approval; keep unset/false for private data |

The GitHub token may be an organization secret restricted to configuration
repositories, rather than duplicated through the source fleet. It needs no
source-workflow write permission. The caller's built-in token handles its own
Actions metadata/cancellation; neither token reaches the test container.

**Public repository warning:** GitHub artifacts and logs in a public caller may
be accessible to other users. Non-dry-run polling and preparation fail closed
unless the caller is private or `allow_public_inputs` is explicitly true. Do
not approve public storage for private source/images; use a private configuration
repository instead. Attaching artifacts to the caller does not make them private.

## Pilot caller

The toydistro branch `feature/github-actions-smoke` provides a thin caller limited
to `locusrobotics/test_meta` and its feature environment. It schedules every five
minutes only when `SOURCE_CI_ENABLED=true`; manual dispatch defaults to dry-run.
GitHub schedules run from the default branch and may be delayed, so merge the
caller there before expecting scheduled operation. A controlled feature-branch
push can exercise the caller after explicit enablement.

Use `dry_run` to resolve configuration, baseline, image identities, and candidate
requests without publishing statuses, changing the ledger, or uploading bundles.
It still requires GitHub/AWS read access and the published environment. A normal
run claims at most four source/OS requests; later polls pick up the remainder.
Use explicit `retry_failed` to requeue a failed/error request with the same
identity. Successful requests deduplicate; a new source/configuration/environment
identity automatically creates new work.

Input artifacts retain one day; curated XML reports retain seven. Source output
cannot inject Actions commands during container execution, and report collection
does not follow source-controlled symlinks. The reporting job does not execute or
interpret worker-provided status code/files as authoritative outcomes.

## Initial limitations and App migration

- Fork PRs and same-repository PRs from non-members/non-collaborators are not
  automatically given private inputs. Explicit reviewed approval is future work.
- The pilot tests PR **head** SHAs, not synthetic merge commits.
- Source symlinks are rejected; archives contain neither Git metadata nor
  submodule content. Repositories requiring those need a later prepared-Git-bundle
  extension, not worker repository credentials.
- Published-environment dependency checks and static removed-package graph checks
  replace fleet source fetching. Runtime downloads/installation are prohibited.
- The small S3 ledger lists request objects during reconciliation; batched API
  discovery, state retention/indexing, and higher-scale queue storage are later
  optimizations. Use one polling invocation per caller run.
- A cancelled workflow may not run its reporting job; the next enabled poll
  reconciles its pending statuses. Runner availability and the schedule must be
  monitored operationally.

A future GitHub App receiver can validate events and feed the same request
ledger/execution boundary. Replace the central PAT with scoped App tokens then;
do not give workers the App identity. See `todo/central-source-ci.md` for the
remaining operational acceptance checklist.
