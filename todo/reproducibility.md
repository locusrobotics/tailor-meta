# Reproducible toolchains and distribution inputs

Toolchain image digests fix the environment used by a particular run. They do
not by themselves make rebuilding that environment or release reproducible.
Record and pin the external inputs that currently remain mutable.

## Toolchain inputs

- [ ] Pin Ubuntu base image digests, with target OS choices still supplied by the
      configuration repository.
- [ ] Pin the Rust toolchain version instead of relying on the installer's
      current default toolchain.
- [ ] Pin the deb-s3 source revision rather than only a branch name.
- [ ] Review Python dependencies across role-specific environments and lock
      transitive versions where reproducible rebuilding is required.
- [ ] Verify checksums for downloaded tools such as aptly, Packer, plugins,
      restic, and Docker installation inputs.
- [ ] Record resolved base/tool versions and source revisions in toolchain
      publication metadata.
- [ ] Define a deliberate security-update process that produces a new immutable
      implementation/toolchain identity without overwriting existing SHA tags.

## Distribution inputs

- [ ] Preserve resolved source SHAs, release pins, package allowlists, patches,
      and configuration/implementation digests in the build plan.
- [ ] Record the actual APT package versions installed into builder and imager
      containers, not merely their dependency names.
- [ ] Decide how upstream mirror snapshots are identified, retained, and selected
      for a build.
- [ ] Determine whether final releases need immutable dependency snapshots or
      another mechanism to reproduce runtime dependency installation.
- [ ] Record provisioning package/playbook identity and external base-image/AMI
      identity for each image result.
- [ ] Test rebuilding from recorded inputs and distinguish functional
      reproducibility from byte-for-byte reproducibility.

## Metadata and retention

- [ ] Keep build plans, toolchain manifests, package checksums, and image results
      available for the intended release lifetime, independently of short-lived
      GitHub artifacts.
- [ ] Document which data is sufficient to rebuild after runner caches disappear.
- [ ] Exclude credentials, authenticated URLs, vault contents, and private keys
      from provenance records and logs.

See `release-recovery.md` for interrupted releases and `build-performance.md`
for caches. A cache is an optimization, not an authoritative dependency snapshot.
