# Release recovery and partial publication

All OS package builds complete before publication, but mirror, APT, image, and
index publication do not form one atomic transaction. Define how operators
recover when a write stage fails partway through.

## Classify publication state

- [ ] Document the artifacts and metadata created by each mirror, package, image,
      index, and CDN stage.
- [ ] Distinguish a completed release, a partially published release, and an
      abandoned reservation without relying only on Actions job status.
- [ ] Record sufficient stage/result metadata alongside the persisted build plan.
- [ ] Define a release completion marker and when it becomes authoritative.
- [ ] Provide read-only inspection commands for claims, package sets, image
      results, and configuration/toolchain identities.

## Immutable final releases

- [ ] Document the lifetime and ownership checks of `release-claim.json`.
- [ ] Preserve the rule that an existing final label cannot be silently adopted
      or overwritten by another run or run attempt.
- [ ] Define operator actions for a claim created before any artifact publication.
- [ ] Define operator actions for a claim followed by partial publication; prefer
      a new release version when safe repair cannot be established.
- [ ] Require explicit authorization and an audit trail for any exceptional
      claim/artifact repair. Never automatically delete failed claims.
- [ ] Test concurrent claims, interrupted publication, reruns, and legacy labels
      without claims.
- [ ] Disable or constrain legacy/manual publishers that could bypass the final
      release contract.

## Mutable feature and candidate labels

- [ ] Define safe retry behavior after one OS publishes and another fails.
- [ ] Define recovery after packages publish but one or more image builds fail.
- [ ] Ensure index repair and CDN refresh do not advertise incomplete results as
      complete releases.
- [ ] Decide whether retrying a failed stage uses its original persisted snapshot
      or requires a fresh build; never silently recompute source identity.
- [ ] Document cleanup behavior for orphaned packages, images, and temporary AWS
      resources, separately from normal retention.

See `cutover.md` for acceptance testing and `reproducibility.md` for snapshot
requirements. Recovery tooling must default to inspection, not remote writes.
