# Buildfarm cutover and acceptance testing

The hosted pilot verifies fixture packaging and underlays. It does not yet
verify the complete toydistro source set, signing, S3/ECR publication, or real
image provisioning. Complete these checks before production cutover.

## Published-toolchain acceptance

- [ ] Confirm generic toolchains and caller-selected builders have published
      successfully for the pinned implementation revision.
- [ ] Configure the single GHCR package per repository for anonymous read access.
- [ ] From a clean consumer runner with no registry login or cached images, pull
      the published toolchain images and resolve their immutable digests.
- [ ] Verify implementation, role, and OS labels against the build plan.
- [ ] Run the packaging fixture through the published builder digest, rather than
      rebuilding its Dockerfile locally.
- [ ] Install generated packages in a fresh container and execute the
      underlay-dependent binary.
- [ ] Retain the plan, digests, reports, and acceptance results as evidence.

## Real toydistro validation

- [ ] Configure dedicated buildfarm runners and separate source/read/publication
      identities; verify each stage receives only its required access.
- [ ] Run graph mode against actual manifest sources and inspect resolved refs,
      dependency graphs, package names, and flavour roots.
- [ ] Run packages mode for every configured OS; verify all package sets finish
      before any publication is attempted.
- [ ] Configure signing keys and verify signed repository metadata.
- [ ] Run a feature release, preserving automatic `feature/*` publication for
      testing.
- [ ] Verify S3 APT contents, ECR product images, CDN invalidation, and retention.
- [ ] Provision every enabled image backend and check OS restrictions and flavour
      selection.
- [ ] Install the published installation sets on clean targets and exercise
      representative runtime functionality.
- [ ] Verify a final-tag rerun is rejected without overwriting artifacts.
- [ ] Record the tested implementation/configuration SHAs, toolchain digests, and
      Actions run links.

## Provisioning-host OS versus target OS

The generic imager uses its host's native OS and installs a published
build-flavour bundle for that OS. This can fail when the configuration repository
does not build that OS, even if its requested image targets are otherwise valid.

- [ ] Define whether provisioning requires an explicitly configured host OS or
      should use tooling independent of the distribution's published bundles.
- [ ] Validate this requirement before publication/provisioning begins.
- [ ] Test a configuration that does not build the imager host's native OS.
- [ ] Keep target OS selection in toydistro/rosdistro; do not introduce an implicit
      target matrix in tailor-meta.

## Production transition

- [ ] Update production caller pins only after toolchains and acceptance checks
      succeed.
- [ ] Preview source stubs before applying direct updates across the repository
      fleet.
- [ ] Replace Jenkins required status contexts with the appropriate native
      Actions checks.
- [ ] Disable old Jenkins schedules, jobs, and hooks, including historical jobs
      associated with untouched companion repositories.
- [ ] Remove legacy generated source Jenkinsfiles deliberately, without deleting
      unrelated repository CI.
- [ ] Document operators, rollback boundaries, and the final cutover decision.

Publication approval remains a separate follow-up. Do not claim the hosted
fixture checks alone establish production readiness.
