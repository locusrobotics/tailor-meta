# Source CI and cross-repository orchestration

The chosen next architecture is central polling with trusted preparation and
uncredentialed workers, followed by GitHub App events. See `central-source-ci.md`.
The source-secret/stub model below describes the transition, not the desired
long-term execution boundary.

Source CI currently runs on source pushes or explicit manual dispatch. Stub
updates are applied directly to configured branches, aggregated by repository
and source branch to avoid ROS distribution collisions.

## Distribution-triggered source tests

- [ ] Add fan-out after successful hotdog/candidate distribution publication.
- [ ] Select only opted-in sources and their configured branches/OS restrictions.
- [ ] Carry the exact published release/build identity into dispatched tests,
      rather than implicitly consuming a later mutable label.
- [ ] Define required Actions permissions, rate limiting, duplicate suppression,
      and reporting of dispatch failures.
- [ ] Decide which source-test failures affect distribution/release status and
      how operators discover cross-repository results.

## External integration tests

- [ ] Migrate the external integration-test pipeline explicitly; it was not part
      of the initial four-repository migration.
- [ ] Restore appropriate distribution-triggered integration tests.
- [ ] Migrate supported comment-triggered tests with explicit actor authorization
      and reviewed source refs.
- [ ] Avoid executing unreviewed PR code on persistent privileged runners.
- [ ] Record test inputs and link results back to the exact source/release build.

## Fork PRs and private environments

- [ ] Document which checks run without secrets on fork PRs.
- [ ] Define the approval/dispatch process for reviewed fork code that needs
      private sources, APT repositories, or test environments.
- [ ] Use disposable hosted/isolated runners and read-only credentials for those
      tests; never reuse publishing or maintenance identities.
- [ ] Distinguish skipped private tests from successful tests in required checks
      and maintainer-facing results.

## Source-update PR automation

- [ ] Keep direct source-stub updates as the current supported fleet-wide path.
- [ ] Design an alternative workflow that opens update PRs and **automatically
      merges once required checks pass**.
- [ ] Avoid requiring manual maintainer-driven merging across the repository
      fleet.
- [ ] Define branch-protection compatibility, retry behavior, deduplication,
      stale PR updates, and reporting for repositories that cannot auto-merge.
- [ ] Preserve per-branch source choices and aggregated multi-distribution jobs.

See `credentials-and-approvals.md` for token scopes and `cutover.md` for required
check migration and legacy Jenkins shutdown.
