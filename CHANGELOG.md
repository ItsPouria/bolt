# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

### Added
- Ray queries: `PhysicsWorld::ray_cast(origin, direction) -> Option<RayHit>` in
  bolt-core with a delegating method on bolt-bevy's wrapper (#3). `RayHit`
  carries a slot-table-validated `BodyHandle`, the body's `user_data` payload,
  the world-space impact point, and the fraction along the ray; the direction's
  magnitude is the ray's maximum distance. Hits map back to entities via
  `Entity::from_bits(hit.user_data)`. First fully-safe (no `unsafe`) path to
  Jolt state, built on rolt's checked query wrappers. Deferred capabilities
  (surface normals, filters, all-hits) tracked in #9.
- `PhysicsWorld::user_data(handle)` in bolt-core: the first public reader of
  the per-body payload supplied in `BodySpawn::user_data`, validated against
  the slot table like every other body API. bolt-bevy stores
  `entity.to_bits()` there, so physics results can be mapped back to entities.
- bolt-bevy now re-exports `BodyHandle` and `RayHit` (crate root and prelude):
  types returned by its public API are nameable without a direct bolt-core
  dependency.

### Safety
- Removed bolt-bevy's `PhysicsWorld::physics_system()` passthrough (#3):
  `rolt` exposes body mutators as safe `&self` methods, so through the
  passthrough safe code could destroy bodies behind the slot table's back
  (a double-destroy use-after-free at world drop) and mutate Jolt state
  through a shared reference, invalidating the crate's `Sync` guarantee.
- bolt-bevy no longer depends on `rolt` or `joltc-sys`; the "no Jolt types
  in bolt-bevy's public API" rule is now enforced by the dependency graph
  rather than by convention. **Breaking:** callers of the passthrough must
  go through bolt-core's validated API (safe read-only query wrappers are
  planned in #3).
- bolt-core: removed the public `PhysicsWorld::physics_system()` getter, the
  last remaining door to raw Jolt state (#3). Internal capabilities will be
  added as validated `PhysicsWorld` methods instead. **Breaking:** escape-
  hatch users must migrate; `user_data` above is the first replacement.

### Internal
- Test-suite expansion (#3): slot-recycle and handle-uniqueness tripwires,
  mixed-body-type teardown, substep clamping (bolt-core white-box);
  `user_data` contract, capacity exhaustion, and full-surface cross-world
  safety (bolt-core black-box); `user_data`→entity mapping, static-transform
  authority, mid-run gravity changes, hierarchical despawn, config
  precedence, and dynamic-body velocity writes (bolt-bevy ECS).
- The velocity and gravity tests document that all Bolt bodies carry Jolt's
  default linear damping (0.05/s); exposing or configuring damping is
  future work.

## [0.2.0] - 2026-10-12

### Architecture
- Split into `bolt-core` and `bolt-bevy`: core owns the Jolt runtime with
  **zero Bevy dependencies** and is usable standalone; bolt-bevy is a thin
  ECS translator, and no Jolt/`rolt` type appears in its public API.
- Engine-agnostic spawn vocabulary (`WorldConfig`, `BodySpawn`, `BodyType`,
  `ShapeSpec`) with boundary conversions for Bevy component data
  (scale-baking lives on `Collider::to_spec`).

### Performance
- Body validation is now a masked index into a dense generation slot table
  (O(1), no hashing, ~40 KB at default capacity), replacing the
  `HashMap` registry; stale handles are rejected by generation compare.
- New headless benchmark example drives `bolt-core` standalone:
  `cargo run --release -p bolt-examples --bin headless_bench -- 1000 1`.

### Safety
- Bodies are addressed by **opaque `BodyHandle`s** (private field, unforgeable
  outside the crate); every world API rejects stale handles. In-crate tests
  still forge IDs through the private constructor to validate the slot table,
  and the first integration suite (`bolt-core/tests/handles.rs`) verifies the
  contract from outside the crate.
- Clean body teardown on world drop; temp-allocator overflow guard.

### Internal
- Migrated to Bevy 0.20; the dependency tree carries a single glam (0.33),
  unified with `rolt`'s.
- Collision layers, Jolt bootstrap, and the shape factory live in bolt-core,
  `pub(crate)` where possible.
- CI gains a docs gate: `RUSTDOCFLAGS="-D warnings" cargo doc --workspace
  --no-deps` — broken intra-doc links fail the build.

## [0.1.0] - 2026-08-07

- Initial implementation of the Bolt physics engine integration for Bevy.
- Support for dynamic, static, and kinematic bodies.
- Integration with Jolt Physics via `rolt`.
