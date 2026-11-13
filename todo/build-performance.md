# Build performance and caching

Measure the simplified pipeline before adding infrastructure. Prebuilt
toolchains remove repeated environment builds; distribution dependencies still
install at runtime and compiler caches are currently run-local.

## Measurements

- [ ] Measure source fetching, image pulls, graph resolution, dependency
      installation, compilation, packaging, image provisioning, and publication
      separately.
- [ ] Record peak disk/memory use and artifact sizes per configured OS.
- [ ] Compare cold ephemeral runners with warm runners and identify the dominant
      bottlenecks.

## Shared compiler caches

- [ ] Evaluate shared ccache with explicit size, retention, and cache-key policy.
- [ ] Include relevant compiler/toolchain/architecture/build-setting identity in
      cache isolation; keep target selection in configuration repositories.
- [ ] Prevent untrusted source tests from poisoning trusted release caches.
- [ ] Verify cache loss or eviction changes performance, not correctness.

## Dependency and image caching

- [ ] Evaluate caching downloaded APT packages or a dependency layer when runtime
      installation is a measured bottleneck.
- [ ] Key caches by dependency versions/mirror snapshots and toolchain identity,
      not only mutable release labels.
- [ ] Keep authentication and private artifacts out of publicly published
      toolchain images and cache exports.
- [ ] Define freshness and security-update behavior rather than silently using
      stale cached dependencies.

## OS-parallel distribution builds

Sequential OS builds are deliberate today. A matrix can improve throughput but
adds artifact transfer, storage, access, and lifecycle requirements.

- [ ] Choose an artifact-transfer/storage contract for resolved sources, graphs,
      plans, debs, and reports before introducing a matrix.
- [ ] Quantify transfer costs and check GitHub artifact/storage limits against
      actual build sizes.
- [ ] Keep ROS 1, ROS 2, and bundles together within each OS container.
- [ ] Preserve the barrier that every configured OS package set must succeed
      before any package publication starts.
- [ ] Define failure/cancellation cleanup and retry behavior across runners.
- [ ] Keep cross-OS state isolated and avoid concurrent writers to shared plans
      or release indexes.

See `reproducibility.md`: caches must not replace authoritative input snapshots.
