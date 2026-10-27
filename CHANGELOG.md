# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

### Added
- Spawn-time damping control (#8): bolt-bevy gains `LinearDamping`/
  `AngularDamping` components and bolt-core's `BodySpawn` gains
  `linear_damping`/`angular_damping` fields (units 1/s; decay law
  `v(t) = v₀·e^(−d·t)`). Both default to 0.05 — Jolt's own default, via an
  explicit `Default` impl so `#[require]`-inserted components preserve the
  behavior bodies previously inherited silently (a derived `Default` would be
  0.0 and un-damp every existing scene). Damping is **read once at spawn**,
  like collider scale and CCD; runtime mutation is pending an upstream
  JoltC API (no `JPC_*Damping` functions exist at the pinned revision —
  tracked upstream as [SecondHalfGames/JoltC#28](https://github.com/SecondHalfGames/JoltC/issues/28)).
- `spawn_body` and setter validation (#7, #8): non-finite or negative damping
  on either field returns `None` instead of reaching Jolt — Jolt asserts on
  negative damping (a debug-build process abort), and negative damping adds
  energy. Shared predicate: `is_finite_and_non_negative`. Non-finite positions,
  rotations, and velocities, as well as unnormalized rotations, are likewise
  rejected across `spawn_body`, `set_position_and_rotation`,
  `set_linear_velocity`, `set_angular_velocity`, and `set_gravity`.
- Thin-primitive support in `bolt-core/src/shapes.rs`: `create_box_shape` and
  `create_cylinder_shape` clamp `ConvexRadius` (`0.05` m default) down to the
  shape's smallest half-extent/radius so thin geometry (`< 0.05` m) succeeds
  instead of failing Jolt's convex-radius check.
- In-place `RigidBody` and `Collider` mutation/replacement in `bolt-bevy`:
  `spawn_physics_bodies` reacts to `Changed<RigidBody>` / `Changed<Collider>`,
  destroying the previous Jolt body and respawning with updated motion type,
  collider geometry, and `StaticMarker` state. The replacement inherits the
  previous body's Jolt-side velocities unless the entity carries explicit
  `LinearVelocity`/`AngularVelocity` components, and the previous body is
  destroyed only after its replacement exists: a rejected mutation (e.g. an
  invalid collider) keeps the last-known-good body simulating with a logged
  warning (fixing the component retries the respawn), while a failed fresh
  spawn still strips the physics components — now with a warning instead of
  silence.
- Negative scale (mirroring) support in `Collider::to_spec`: applies
  `scale.abs()` so mirrored entities produce positive shape dimensions.
- `PhysicsWorld`'s `FromWorld` impl picks up a `Gravity` resource inserted
  before the plugin is added (the plugin now initializes `Gravity` before
  `PhysicsWorld`); gravity changes after construction continue to flow
  through `apply_gravity` each fixed tick.
- Ray queries: `PhysicsWorld::ray_cast(origin, direction) -> Option<RayHit>` in
  bolt-core with a delegating method on bolt-bevy's wrapper (#3). `RayHit`
  carries a slot-table-validated `BodyHandle`, the body's `user_data` payload,
  the world-space impact point, and the fraction along the ray; the direction's
  magnitude is the ray's maximum distance. Hits map back to entities via
  `Entity::from_bits(hit.user_data)`. First fully-safe (no `unsafe`) path to
  Jolt state, built on rolt's checked query wrappers. Deferred capabilities
  (surface normals, filters, all-hits) tracked in #9.
- `PhysicsWorld::user_data(handle)` in bolt-core: lock-free O(1) reader of
  the per-body payload supplied in `BodySpawn::user_data`, backed by a dense
  table parallel to `slots` and validated against the slot table like every
  other body API. bolt-bevy stores `entity.to_bits()` there, so physics
  results can be mapped back to entities.
- `PhysicsWorld::get_linear_and_angular_velocity(handle)` in bolt-core:
  both velocities behind a single lock acquisition instead of two FFI
  round-trips; bolt-bevy's transform-sync loop uses it.
- `PhysicsWorld::move_kinematic` in bolt-core (#4, Fix B core layer):
  displacement-driven kinematic motion — a target pose plus a time
  interval in, a velocity Jolt computes internally (displacement /
  delta_time) and that **persists until re-issued**; the position lands on
  the next `step`. Guards, silent no-ops per house setter style: stale
  handles, non-finite or unnormalized targets, non-positive or non-finite
  `delta_time` (Jolt divides by it), and non-kinematic bodies — verified
  against the vendored Jolt source that statics hold a null
  motion-properties pointer and `MoveKinematic` carries no guard, so the
  motion type is checked before the FFI call (a release-mode null
  dereference otherwise). Completes the movement family: teleport
  (`set_position_and_rotation`), velocity (`set_*_velocity`), and
  displacement (`move_kinematic`). Pinned by three white-box tests —
  landing-on-next-step, the refusal sweep, and velocity persistence, the
  semantics that will oblige the planned ECS wiring (#4) to re-issue
  every fixed tick. Logging of the
  silent refusals is deliberately deferred to the observability-policy
  issue (#12); the ECS layer (Transform-driven kinematics) follows as
  the #4 Fix B remainder.
- Transform-driven kinematic bodies in bolt-bevy (#4, Fix B ECS layer):
  a kinematic body whose `Transform` is edited by user code now moves
  with real velocity instead of teleporting. The first genuine edit
  claims ownership via a new `TransformDriven` marker component; the new
  `move_transform_driven_kinematics` system then issues
  `PhysicsWorld::move_kinematic` toward the ECS-derived world pose every
  fixed tick — unconditional by design, because the velocity
  MoveKinematic sets persists until re-issued: when the script stops
  editing, target ≡ current and the velocity decays to zero, halting the
  body gracefully. Parented targets compose the parent's world frame via
  the #4 Fix A machinery (on the parent — the entity's own Jolt pose
  would discard the edit); scale is dropped, as `move_kinematic` takes
  only a pose. bolt-bevy's `PhysicsWorld` wrapper gains the delegating
  `move_kinematic` method. Ownership arbitration is complete: a user
  velocity write reclaims the body for the velocity path, and a respawn
  strips the marker (fresh bodies start unowned — a Kinematic → Dynamic
  switch leaves no stale note). A marked body's local `Transform` is
  user-authoritative — `sync_transforms` skips mirroring into it (that
  mirror-write laundered the parent's motion out of the drive target for
  parented bodies, making them hold a fixed world pose instead of
  tracking the parent frame); velocity components still mirror. Riders
  are carried by friction: the acceptance test's crate tracks a 1.5 m/s
  sliding platform at exactly the predicted μ-limited arithmetic
  (0.93 m × tick quantization = 0.837 m measured), and a parented
  kinematic tracks a moving parent's frame to sub-millimeter.
- bolt-bevy now re-exports `BodyHandle` and `RayHit` (crate root and prelude):
  types returned by its public API are nameable without a direct bolt-core
  dependency.

### Changed
- Kinematic `Transform` edits no longer teleport the Jolt body (#4, Fix
  B): they claim `TransformDriven` ownership and move with real velocity
  instead (see Added). Two consequences: the edit lands on the same tick
  it is made (the marker flushes at the auto sync point on the claiming
  system's ordering edge, and that tick's step lands the pose), and the
  spawn-time bundle insert
  is placement, never an edit: a Transform write that predates the body's
  creation tick does not claim ownership (otherwise every kinematic would
  be marked at birth and the per-tick move would cancel velocity-path
  motion every tick). The 5 mm nudge test now pins the eat-window
  invariant on a dynamic body, and a new unit test pins the ownership
  claim itself (marker inserted, Jolt unmoved until the per-tick system).
- bolt-bevy's wrapper `spawn_body` collapsed its grown 10-argument signature
  into a `BodySpawnRequest<'a>` parameter object (#8): a borrowed struct with
  named fields (entity, collider & scale, pose, velocities, damping, CCD),
  exported from the prelude. **Breaking:** direct callers construct a
  `BodySpawnRequest` instead of passing positional arguments. This retires
  the "Revisit in the Phase 4 API pass" note.
- `BodySpawn` gained the two damping fields (#8). **Breaking:** full-literal
  constructions outside the crate must add them (Bolt's own helpers use
  0.05); struct-update call sites (`..base`) compile untouched.
- `ShapeSpec::create` is now `pub(crate)`, keeping raw `joltc-sys` shape
  pointers internal to `bolt-core`.

### Fixed
- Despawning a physics entity no longer logs two spurious warnings (#6):
  the `Remove<RigidBody>`/`Remove<Collider>` observers queue their
  `JoltBody`/`StaticMarker` removal mid-despawn while the entity still
  exists, but the command applies after `despawn()` has finished — exactly
  the failure mode Bevy 0.20's `EntityCommands::remove` warns on. The
  observers now use `try_remove` (identical semantics, no warning when the
  entity is legitimately gone); the spawn path deliberately keeps plain
  `remove`, where a warning signals a real bug. bolt-bevy's `despawn_quiet`
  test pins the zero-warnings contract via log capture.
- Children of moving parents no longer drift with them (#4):
  `sync_transforms` converted Jolt world coordinates into the parent's
  local frame using the parent's `GlobalTransform` component — one
  propagation behind inside `FixedUpdate` — and `apply_user_transforms`
  fed each tick's resulting error back into Jolt as teleports, compounding
  it (a force-free dynamic child drifted ~0.48 m along with a 1 m/s
  parent over 1 s). The parent frame is now computed fresh inside the tick
  by `compute_world_affine`: a parent that is a physics body contributes
  its authoritative Jolt pose directly (already world-space, so ancestor
  walks stop at the first body), and plain non-physics parents compose up
  a `Without<RigidBody>`-filtered query — disjoint from the system's
  `&mut Transform` by construction. That disjointness is what makes a
  fresh frame possible at all: a `TransformHelper` in this system panics
  with B0001 (it reads every `Transform` in the world), so the old stale-
  component read was the only conflict-free option a single-pass writer
  had. Pinned by `test_child_of_moving_parent_holds_world_position`.
- Sleeping bodies keep their ECS mirror synced (#4): `sync_transforms`
  used to skip inactive bodies entirely, freezing a sleeping child's
  local `Transform` while its parent's frame kept moving — invisible
  until the drift fix landed, because the stale-frame bug's spurious
  teleports had kept children artificially awake (a force-free dynamic
  body sleeps after 0.5 s). Transform sync now runs for sleeping bodies
  too — their Jolt pose is fixed but the parent's frame may be moving —
  while only velocity mirroring stays gated on `is_active`.

### Performance
- bolt-core's body getters drop their per-call `IsAdded` FFI pre-checks: a
  slot-table-valid handle (full index+generation compare) is always an
  added body — `spawn_body` creates-and-adds and `destroy_body` is the sole
  removal path — so the extra round-trips were dead checks.

### Safety
- `PhysicsWorld::new` validates `max_bodies` and `num_threads` before any FFI
  work (#7): `max_bodies` of `0` or above Jolt's body ceiling (`JOLT_MAX_BODIES`
  = 8,388,608) and `num_threads < -1` now produce readable Rust panics instead
  of a message-less process abort inside Jolt's initialization. A `# Panics`
  section on `new` documents all panic conditions (`max_bodies` bounds,
  `num_threads < -1`, temp-allocator overflow, allocation failure).
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
- Test-suite expansion (#3, #7, #8): slot-recycle and handle-uniqueness
  tripwires, mixed-body-type teardown, substep clamping, non-finite pose/velocity
  rejection, thin-shape convex-radius clamping, and `num_threads` validation
  (bolt-core white-box); `user_data` contract, capacity exhaustion, and
  full-surface cross-world safety (bolt-core black-box); `user_data`→entity
  mapping, static-transform authority, static-to-dynamic transitions (both
  removal and in-place mutation), mirrored negative scale colliders, mid-run
  gravity changes, hierarchical despawn, config precedence, and dynamic-body
  velocity writes (bolt-bevy ECS).
- The velocity and gravity tests document that all Bolt bodies carry Jolt's
  default linear damping (0.05/s); #8 exposed that default as spawn-time
  configuration (see Added).
- Kinematic-ownership coverage (#4, Fix B): respawn strips `TransformDriven`;
  the per-tick drive carries a marked body to its target through the wired
  pipeline; the velocity path never gets marked; a velocity write reclaims
  ownership; and a transform-driven platform halts when its script stops —
  the per-tick decay pinned at the ECS level.
- Determinism probe (#7, item 5): twin worlds — identical config, spawns,
  and 120 steps — agree bit-exactly on every body transform, both
  single-threaded and across a 4-thread job system (0 flakes in 50 runs).
  The pinned guarantee is deliberately scoped: same-process, same-machine,
  same-build agreement (replays, lockstep on a fixed platform);
  cross-machine/cross-platform determinism is not claimed.
- ASan CI job (#7, item 4): bolt-core's suite runs on nightly under
  AddressSanitizer with **both sides of the FFI boundary instrumented** —
  `RUSTFLAGS` covers the Rust side, `CFLAGS`/`CXXFLAGS` flow through
  joltc-sys's CMake build into Jolt itself, and both share one ASan
  runtime at link time. LeakSanitizer rides along by default on Linux, so
  clean exits double as leak checks for the teardown tests. The explicit
  (host) `--target` keeps proc-macros and build scripts uninstrumented —
  rustc rejects sanitizer-instrumented proc-macros. Nightly is a visitor
  (this job only), not a migration.
- CI hardening: the dependency guards now fail closed (a `cargo tree` error
  fails the job instead of the guard silently passing) and enforce both
  README invariants — bolt-bevy takes no direct `rolt`/`joltc-sys` edge of
  any kind, and bolt-core stays Bevy-free; cargo invocations run `--locked`
  so CI tests exactly the committed lockfile; jobs carry explicit timeouts
  and least-privilege permissions (only the test job may write checks).

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
