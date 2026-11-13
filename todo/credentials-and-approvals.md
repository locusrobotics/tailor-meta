# Credentials, runner isolation, and publication approval

Keep the existing stage-level credential separation while replacing long-lived
identities and documenting operational access requirements.

## AWS OIDC

- [ ] Replace static read/publication AWS access keys with short-lived OIDC role
      sessions where supported.
- [ ] Define separate read, mirror/package publication, and image-provisioning
      roles with only their required S3/ECR/CloudFront/EC2 permissions.
- [ ] Constrain trust policies by organization, repository, workflow/ref, and
      environment where applicable.
- [ ] Carry temporary session tokens correctly into APT authentication and
      container stages.
- [ ] Verify read-only stages cannot publish or provision resources.
- [ ] Document conditional S3 access needed for immutable release claims.

## GitHub App identities

- [ ] Replace long-lived source and maintenance tokens with short-lived GitHub
      App installation tokens.
- [ ] Keep read-only source access separate from contents/workflows maintenance
      writes and any future Actions dispatch permissions.
- [ ] Scope installations to the intended repositories and document provisioning,
      rotation, revocation, and failure handling.
- [ ] Keep source PR tests isolated from maintenance and publication tokens.

## Publication approval

- [ ] Add approval at the publication boundary rather than unnecessarily gating
      source fetching and package building.
- [ ] Decide how approval interacts with dependency mirroring, which can already
      write before package publication.
- [ ] Bind approval to the persisted configuration/source/package snapshot so
      approved artifacts cannot be replaced before publication.
- [ ] Define applicable branch/tag rules while preserving automatic feature
      artifact publication for testing.
- [ ] Document which protections require GitHub environment/ruleset setup outside
      workflow YAML.

## Runner and secret handling

- [ ] Document the privileged capabilities required by each image backend.
- [ ] Prefer ephemeral buildfarm runners and prohibit unreviewed source/PR code
      from persistent privileged runners.
- [ ] Verify temporary credential mounts are cleaned up on normal completion,
      failure, cancellation, and runner restart.
- [ ] Review logging/redaction, Docker contexts, generated metadata, and artifacts
      for accidental credential exposure.
- [ ] Document GHCR public read visibility and package publication permissions;
      use the single package per existing repository with role tags.

See `source-ci.md` for dispatch/auto-merge permissions and `release-recovery.md`
for exceptional repair authorization.
