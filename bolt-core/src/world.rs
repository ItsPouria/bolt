//! The engine-agnostic Jolt physics world and its construction vocabulary.
//!
//! This module is home to the [`PhysicsWorld`](world::PhysicsWorld) type — the Jolt
//! `PhysicsSystem`, its allocators and job system, and the body management
//! API — together with the engine-agnostic types that engine integrations
//! convert their component data into before touching the world:
//! [`WorldConfig`](world::WorldConfig), [`BodyType`](world::BodyType), and
//! [`BodySpawn`](world::BodySpawn).

use std::mem::ManuallyDrop;
use std::ptr::NonNull;

use crate::init::ensure_jolt_initialized;
use crate::layers::{
    OBJECT_LAYER_DYNAMIC, OBJECT_LAYER_STATIC, SimpleBroadPhaseLayer, SimpleObjectLayerPairFilter,
    SimpleObjectVsBroadPhaseLayerFilter,
};
use crate::shapes::ShapeSpec;
use glam::{Quat, Vec3};
use joltc_sys::{
    JPC_JobSystemThreadPool, JPC_JobSystemThreadPool_delete, JPC_JobSystemThreadPool_new3,
    JPC_MAX_PHYSICS_BARRIERS, JPC_MAX_PHYSICS_JOBS, JPC_PhysicsSystem_SetGravity,
    JPC_TempAllocatorImpl, JPC_TempAllocatorImpl_delete, JPC_TempAllocatorImpl_new, JPC_Vec3,
};
use rolt::{BodyId, PhysicsSystem, RRayCast, RayCastArgs};

/// Constant representing an invalid or unallocated Jolt body ID (0xFFFFFFFF).
pub const INVALID_BODY_ID: u32 = 0xffff_ffff;

/// Jolt packs a body ID as a 23-bit index plus an 8-bit sequence number
/// (`BodyID.h`: `cMaxBodyIndex = 0x7fffff`, shift 23). The index addresses
/// the slot tables; the sequence detects stale handles.
const BODY_INDEX_MASK: u32 = 0x007f_ffff;

/// Jolt's hard ceiling on `max_bodies`. `BodyID` packs a 23-bit body index
/// plus an 8-bit sequence number (`BodyID.h`: `cMaxBodyIndex = 0x7fffff`,
/// shift 23), so body indices run `0..=cMaxBodyIndex`; per the header,
/// `cMaxBodyIndex` is "the maximum amount of bodies supported − 1", which
/// makes the largest usable `max_bodies` exactly one more than the index
/// mask.
const JOLT_MAX_BODIES: u32 = BODY_INDEX_MASK + 1;

/// Runtime parameters for constructing a [`PhysicsWorld`].
///
/// This is the engine-agnostic parameter set: plain data with no engine
/// attached. Engine integrations typically expose their own user-facing
/// configuration resource (with editor and serialization support) and convert
/// it into this type at world creation time.
#[derive(Clone, Debug)]
pub struct WorldConfig {
    /// The maximum number of physics bodies allowed in the world.
    ///
    /// This value sizes Jolt's internal body storage; it is a hard cap, not
    /// a preallocation of that magnitude.
    pub max_bodies: u32,
    /// The maximum number of body pairs that can potentially collide in a
    /// single simulation step.
    pub max_body_pairs: u32,
    /// The maximum number of contact constraints Jolt will solve in a single
    /// simulation step.
    pub max_contact_constraints: u32,
    /// The number of worker threads Jolt's job system should use.
    ///
    /// `-1` requests one thread per logical core.
    pub num_threads: i32,
    /// The size of Jolt's temporary allocator, in megabytes.
    ///
    /// This allocator backs per-step scratch allocations; roughly 10 MB per
    /// few hundred active bodies is a reasonable starting point.
    pub temp_allocator_size_mb: u32,
}

impl Default for WorldConfig {
    fn default() -> Self {
        Self {
            max_bodies: 10240,
            max_body_pairs: 65536,
            max_contact_constraints: 10240,
            num_threads: -1,
            temp_allocator_size_mb: 10,
        }
    }
}

/// The motion behavior of a physics body.
///
/// This is the engine-agnostic twin of an integration's rigid-body component
/// (bolt-bevy's `RigidBody`): integrations convert their component into this
/// type at the crate boundary when spawning bodies, which keeps `bolt-core`
/// free of engine types.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BodyType {
    /// A body affected by forces, gravity, and collisions (e.g. crates, debris).
    Dynamic,
    /// A body that never moves and has infinite mass (e.g. terrain, floors,
    /// walls).
    Static,
    /// A body unaffected by forces but moved manually, which can push dynamic
    /// bodies around (e.g. platforms, elevators).
    Kinematic,
}

/// The complete description of a body to spawn into the physics world.
///
/// This is the parameter object of [`PhysicsWorld`]'s body-spawning API: everything
/// needed to create one body in a single call, with geometry fully baked
/// (entity scale applied by the caller) and expressed in world space.
#[derive(Clone, Debug)]
pub struct BodySpawn {
    /// The collision shape, with final baked dimensions in meters.
    pub shape: ShapeSpec,
    /// How the body moves and responds to forces.
    pub body_type: BodyType,
    /// The world-space position of the body's origin, in meters.
    pub position: Vec3,
    /// The world-space rotation of the body.
    pub rotation: Quat,
    /// The initial linear velocity, in meters per second.
    pub linear_velocity: Vec3,
    /// The initial angular velocity, in radians per second.
    pub angular_velocity: Vec3,
    /// The linear (translational) damping coefficient, in inverse seconds.
    ///
    /// Velocity decays exponentially as `v(t) = v₀·e^(−d·t)`: with Jolt's own
    /// default of `0.05`, a body retains ~97.5% of its speed after half a
    /// second, while `0.0` leaves analytic gameplay code (projectile arcs,
    /// aiming, replays) exact. Passing `0.05` therefore preserves the
    /// behavior bodies previously inherited silently. Only meaningful for
    /// [`BodyType::Dynamic`] bodies; must be finite and non-negative.
    ///
    /// Read once at spawn: later changes have no effect on an existing body
    /// (runtime damping control is pending an upstream JoltC API).
    pub linear_damping: f32,
    /// The angular (rotational) damping coefficient, in inverse seconds.
    ///
    /// Governs spin decay with the same exponential law and the same rules
    /// as [`BodySpawn::linear_damping`]: finite, non-negative, meaningful
    /// only for [`BodyType::Dynamic`] bodies, and read once at spawn. Jolt's
    /// default is likewise `0.05`.
    pub angular_damping: f32,
    /// An opaque per-body payload stored inside Jolt and retrievable from
    /// query results.
    ///
    /// Jolt identifies bodies by internal IDs; engine integrations use this
    /// field to map bodies back to their own game objects. Bevy stores
    /// `entity.to_bits()` here so that contact and query results can be
    /// translated back into entities.
    pub user_data: u64,
    /// Whether to enable continuous collision detection for this body.
    ///
    /// Only meaningful for [`BodyType::Dynamic`] bodies: it prevents
    /// fast-moving bodies from tunneling through thin obstacles.
    pub use_ccd: bool,
}

/// Returns `true` when `value` is finite and non-negative — the validity
/// rule shared by [`BodySpawn`]'s scalar material fields (damping today;
/// friction, which Jolt likewise asserts non-negative, when it is
/// exposed — restitution instead clamps, so it will not reuse this).
///
/// Both checks are required: `value >= 0.0` alone already rejects `NaN`
/// (every comparison with `NaN` is false) but still accepts
/// [`f32::INFINITY`], which would freeze a body on its first step.
#[must_use]
fn is_finite_and_non_negative(value: f32) -> bool {
    value.is_finite() && value >= 0.0
}

/// An opaque handle to a physics body in a [`PhysicsWorld`].
///
/// Minted by `spawn_body`; cannot be constructed from raw bits. Handles
/// may be freely copied, stored, and compared — the world rejects any
/// handle that is stale (its slot was freed or recycled).
///
/// Handles are scoped to the world that minted them: presenting a handle
/// to a different world is not memory-unsafe (validation still runs), but
/// it may silently operate on whatever body occupies the same slot there.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct BodyHandle(BodyId);

/// The result of a closest-hit ray query against the physics world.
///
/// Produced by [`PhysicsWorld::ray_cast`]. The query reports
/// the **single closest** body intersected by the ray segment: Jolt walks
/// its broadphase trees, tests surviving candidates exactly, and keeps the
/// smallest fraction, pruning anything farther as it goes. Bodies beyond
/// the ray's length are never reported.
///
/// Every field is validated or derived before the hit is returned: the
/// body's ID is checked against the slot table (so the handle is safe to
/// use with any world method), the payload is read from the body, and the
/// impact point is computed from the fraction.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RayHit {
    /// The body that was hit, as a validated handle.
    ///
    /// Because the hit's body ID was checked against the slot table before
    /// this hit was constructed, the handle is guaranteed live at query
    /// time and can be passed to any `PhysicsWorld` method.
    pub body: BodyHandle,
    /// The opaque per-body payload the body was spawned with
    /// ([`BodySpawn::user_data`]).
    ///
    /// Engine integrations store their own object identifier here so that
    /// query results can be mapped back to game objects: bolt-bevy fills
    /// it with `entity.to_bits()`, so a hit translates back to an entity
    /// with `Entity::from_bits(hit.user_data)`.
    pub user_data: u64,
    /// The world-space point where the ray impacted the body's surface.
    ///
    /// Computed at query time as `origin + fraction * direction`.
    pub hit_point: Vec3,
    /// Where along the ray the hit occurred: `0.0` at the ray's origin,
    /// `1.0` at its far end.
    ///
    /// This is a **parameter of the ray, not a distance in meters**: the
    /// ray's length is the magnitude of the direction vector passed to the
    /// query. Multiply the fraction by that magnitude to obtain the
    /// distance traveled.
    pub fraction: f32,
}

/// The Jolt physics world: owns the `PhysicsSystem`, its allocators and
/// worker threads, and the slot-table bookkeeping for validated body access.
pub struct PhysicsWorld {
    /// The Jolt physics system: bodies, broadphase, and solver.
    ///
    /// Wrapped in [`ManuallyDrop`] because destructor timing is
    /// load-bearing — it must run after the body cleanup in [`Drop`] and
    /// *before* the C++ allocator deletions, so [`Drop`] triggers it
    /// manually instead of relying on implicit field-drop order.
    physics_system: ManuallyDrop<PhysicsSystem>,
    /// Jolt's temporary (scratch) allocator, consumed during each `update`.
    ///
    /// An owned C++ allocation held as a guaranteed-non-null pointer; freed
    /// in [`Drop`], after the physics system.
    temp_allocator: NonNull<JPC_TempAllocatorImpl>,
    /// Jolt's worker thread pool, executing simulation jobs across cores.
    ///
    /// An owned C++ allocation held as a guaranteed-non-null pointer; freed
    /// in [`Drop`], after the physics system.
    job_system: NonNull<JPC_JobSystemThreadPool>,
    /// Dense slot table: body index → full raw `BodyID` (index + generation).
    /// A slot is free if it holds [`INVALID_BODY_ID`]. Sized to `max_bodies`
    /// at construction (~40 KB at the default), so hot-path validation is a
    /// masked compare on contiguous memory instead of a hash lookup.
    slots: Vec<u32>,
    /// Dense table: body index → the `user_data` payload stored in Jolt.
    user_data: Vec<u64>,
}

impl PhysicsWorld {
    /// Creates a new physics world with the given configuration.
    ///
    /// Initializes the Jolt runtime on first use and starts the job system's
    /// worker threads.
    ///
    /// # Panics
    ///
    /// - if `max_bodies` is 0 or exceeds Jolt's body ceiling of 8,388,608
    ///   (`BodyID` packs a 23-bit index: `cMaxBodyIndex` = 8,388,607 is the
    ///   largest index, not the largest count) — both otherwise abort the
    ///   process inside Jolt's `PhysicsSystem::Init` with no message
    /// - if `temp_allocator_size_mb` in bytes overflows `u32`
    /// - if Jolt's temp allocator or job system allocation fails (out of
    ///   memory)
    ///
    /// `max_body_pairs` and `max_contact_constraints` are legal at 0
    /// (probed: construction and drop survive).
    pub fn new(config: WorldConfig) -> Self {
        assert!(
            config.max_bodies > 0,
            "max_bodies must be at least 1, got 0: Jolt's PhysicsSystem::Init \
             asserts on zero and aborts the process",
        );
        assert!(
            config.max_bodies <= JOLT_MAX_BODIES,
            "max_bodies {} exceeds Jolt's hard limit of {} bodies (BodyID \
             packs a 23-bit index: cMaxBodyIndex = 8,388,607 is the largest \
             index, not the largest count)",
            config.max_bodies,
            JOLT_MAX_BODIES,
        );
        ensure_jolt_initialized();
        let mut physics_system = PhysicsSystem::new();

        physics_system.init(
            config.max_bodies,
            0, // num_body_mutexes (0 = default) calculated automatically by Jolt
            config.max_body_pairs,
            config.max_contact_constraints,
            SimpleBroadPhaseLayer,
            SimpleObjectVsBroadPhaseLayerFilter,
            SimpleObjectLayerPairFilter,
        );

        let temp_allocator_bytes = config
            .temp_allocator_size_mb
            .checked_mul(1024 * 1024)
            .expect("temp_allocator_size_mb in bytes overflows u32");

        // SAFETY: 10MB is a valid size for the Jolt temp allocator.
        let temp_allocator_ptr = unsafe { JPC_TempAllocatorImpl_new(temp_allocator_bytes) };
        let temp_allocator = NonNull::new(temp_allocator_ptr)
            .expect("Failed to allocate Jolt TempAllocator: Out of memory");
        // SAFETY: Thread counts and max jobs constants are valid parameters for Jolt.
        let job_system_ptr = unsafe {
            JPC_JobSystemThreadPool_new3(
                JPC_MAX_PHYSICS_JOBS as u32,
                JPC_MAX_PHYSICS_BARRIERS as u32,
                config.num_threads,
            )
        };
        let job_system =
            NonNull::new(job_system_ptr).expect("Failed to allocate Jolt JobSystem: Out of memory");

        Self {
            physics_system: ManuallyDrop::new(physics_system),
            temp_allocator,
            job_system,
            slots: vec![INVALID_BODY_ID; config.max_bodies as usize],
            user_data: vec![0; config.max_bodies as usize],
        }
    }

    /// Retrieves the opaque per-body payload supplied at spawn time
    /// ([`BodySpawn::user_data`]).
    ///
    /// Engine integrations typically store their own entity identifier here
    /// so that physics results (queries, contacts) can be mapped back to
    /// game objects.
    ///
    /// Returns `None` for unknown or already-destroyed IDs.
    pub fn user_data(&self, handle: BodyHandle) -> Option<u64> {
        if !self.is_valid(handle) {
            return None;
        }
        // SAFETY: We verify the body ID is valid and added before querying its state.
        unsafe {
            let body_interface = self.body_interface();

            if !joltc_sys::JPC_BodyInterface_IsAdded(body_interface, handle.0.raw()) {
                return None;
            }

            let data = joltc_sys::JPC_BodyInterface_GetUserData(body_interface, handle.0.raw());
            Some(data)
        }
    }

    /// Casts a ray into the simulation and returns the closest body it
    /// intersects, if any.
    ///
    /// The ray starts at `origin` and travels along `direction`.
    /// **The magnitude of `direction` is the ray's maximum distance**: a
    /// normalized direction casts a 1-meter ray. To cast `max_distance`
    /// meters along a unit vector `dir`, pass `dir * max_distance`.
    ///
    /// Returns the single closest hit along the segment, with the hit
    /// body's handle validated against the slot table, its
    /// [`user_data`](Self::user_data) payload included, and the world-space
    /// impact point computed. Returns `None` when the ray misses
    /// everything, when it is shorter than the distance to the nearest
    /// hit, when the inputs are non-finite or the direction has zero
    /// length, or — defensively — when the hit body cannot be validated.
    ///
    /// This is a read-only query: it does not wake bodies or advance the
    /// simulation. It is also the crate's first fully-safe path to Jolt
    /// state (no `unsafe` block), built on rolt's checked query wrappers.
    ///
    /// A ray whose origin lies on or inside a body reports that body itself
    /// at `fraction` 0.0, and closest-hit semantics then shadow everything
    /// beyond it. When casting from inside your own geometry, start the
    /// origin past it; body-exclusion filters are not yet available.
    pub fn ray_cast(&self, origin: Vec3, direction: Vec3) -> Option<RayHit> {
        if !origin.is_finite() || !direction.is_finite() || direction.length_squared() == 0.0 {
            return None;
        }

        let ray = RRayCast { origin, direction };
        let args = RayCastArgs {
            ray,
            ..Default::default()
        };
        let result = self.physics_system.narrow_phase_query().cast_ray(args)?;

        // The query reports a raw Jolt body ID; the unforgeability rule
        // applies to query hits too, so mint a handle and validate it
        // against the slot table before handing it out.
        let handle = BodyHandle(result.body_id);
        if !self.is_valid(handle) {
            return None;
        }

        let payload = self.user_data(handle)?;

        Some(RayHit {
            body: handle,
            user_data: payload,
            hit_point: origin + direction * result.fraction,
            fraction: result.fraction,
        })
    }

    /// Helper to get the Jolt BodyInterface.
    ///
    /// # Safety
    /// The caller must ensure that the `PhysicsSystem` is still valid.
    unsafe fn body_interface(&self) -> *mut joltc_sys::JPC_BodyInterface {
        // SAFETY: The raw pointer from `physics_system` is valid.
        unsafe { joltc_sys::JPC_PhysicsSystem_GetBodyInterface(self.physics_system.raw()) }
    }

    /// Validates a body ID against the slot table in O(1), without hashing.
    ///
    /// Compares the full raw ID — index *and* generation — so a stale handle
    /// to a destroyed-and-recycled slot is rejected.
    ///
    /// The explicit [`INVALID_BODY_ID`] check looks redundant: its index
    /// (8,388,607) sits far beyond any sane `max_bodies`. But if a world
    /// were ever configured with more slots than Jolt's own `cMaxBodyIndex`,
    /// that slot could exist and hold a matching value — validating the
    /// invalid ID. Defensive completeness costs one compare.
    #[inline]
    fn is_valid(&self, handle: BodyHandle) -> bool {
        let raw = handle.0.raw();
        if raw == INVALID_BODY_ID {
            return false;
        }
        let index = (raw & BODY_INDEX_MASK) as usize;
        index < self.slots.len() && self.slots[index] == raw
    }

    /// Advances the physics simulation by `delta_time`, performing `collision_steps` substeps.
    pub fn step(&mut self, delta_time: f32, collision_steps: i32) {
        if delta_time <= 0.0 || !delta_time.is_finite() {
            return;
        }

        // SAFETY: physics_system, allocator, and job_system pointers are valid for the lifetime of PhysicsWorld.
        unsafe {
            self.physics_system.update(
                delta_time,
                collision_steps.max(1),
                self.temp_allocator.as_ptr(),
                self.job_system.as_ptr(),
            );
        }
    }

    /// Sets the world's global gravity acceleration, in m/s².
    ///
    /// Integrations commonly default to `(0.0, -9.81, 0.0)`. The new value
    /// affects all dynamic bodies on subsequent [`step`](Self::step) calls.
    pub fn set_gravity(&mut self, gravity: Vec3) {
        // SAFETY: `physics_system.raw()` returns a valid pointer to the initialized
        // JPC_PhysicsSystem. The JPC_Vec3 struct is correctly initialized with padding.
        unsafe {
            let raw_physics_system = self.physics_system.raw();
            let gravity_vec = JPC_Vec3 {
                x: gravity.x,
                y: gravity.y,
                z: gravity.z,
                _w: 0.0,
            };
            JPC_PhysicsSystem_SetGravity(raw_physics_system, gravity_vec);
        }
    }

    /// Spawns a body into the world as described by `spawn`.
    ///
    /// Returns a handle to the new body, or `None` if either damping value
    /// is negative or non-finite, the shape could not be built, or the
    /// world's body capacity is full.
    pub fn spawn_body(&mut self, spawn: BodySpawn) -> Option<BodyHandle> {
        // Damping must be finite and non-negative: Jolt asserts on negative
        // damping (a debug-build process abort), and negative damping adds
        // energy — always a bug, never a tuning choice.
        if !is_finite_and_non_negative(spawn.linear_damping)
            || !is_finite_and_non_negative(spawn.angular_damping)
        {
            return None;
        }

        let shape_ptr = spawn.shape.create()?;

        let motion_type = match spawn.body_type {
            BodyType::Dynamic => joltc_sys::JPC_MOTION_TYPE_DYNAMIC,
            BodyType::Static => joltc_sys::JPC_MOTION_TYPE_STATIC,
            BodyType::Kinematic => joltc_sys::JPC_MOTION_TYPE_KINEMATIC,
        };

        let motion_quality = if spawn.use_ccd && spawn.body_type == BodyType::Dynamic {
            joltc_sys::JPC_MOTION_QUALITY_LINEAR_CAST
        } else {
            joltc_sys::JPC_MOTION_QUALITY_DISCRETE
        };

        let position = joltc_sys::JPC_Vec3 {
            x: spawn.position.x,
            y: spawn.position.y,
            z: spawn.position.z,
            _w: 0.0,
        };

        let rotation = joltc_sys::JPC_Quat {
            x: spawn.rotation.x,
            y: spawn.rotation.y,
            z: spawn.rotation.z,
            w: spawn.rotation.w,
        };

        let lin_vel = joltc_sys::JPC_Vec3 {
            x: spawn.linear_velocity.x,
            y: spawn.linear_velocity.y,
            z: spawn.linear_velocity.z,
            _w: 0.0,
        };

        let ang_vel = joltc_sys::JPC_Vec3 {
            x: spawn.angular_velocity.x,
            y: spawn.angular_velocity.y,
            z: spawn.angular_velocity.z,
            _w: 0.0,
        };

        let object_layer = match spawn.body_type {
            BodyType::Static => OBJECT_LAYER_STATIC.raw(),
            BodyType::Dynamic | BodyType::Kinematic => OBJECT_LAYER_DYNAMIC.raw(),
        };

        let settings = joltc_sys::JPC_BodyCreationSettings {
            Position: position,
            Rotation: rotation,
            MotionType: motion_type,
            MotionQuality: motion_quality,
            ObjectLayer: object_layer,
            Shape: shape_ptr,
            LinearVelocity: lin_vel,
            AngularVelocity: ang_vel,
            LinearDamping: spawn.linear_damping,
            AngularDamping: spawn.angular_damping,
            UserData: spawn.user_data,
            ..Default::default()
        };

        // SAFETY: settings and shape_ptr are valid, and motion types match Jolt requirements.
        let body_id = unsafe {
            let body_interface = self.body_interface();

            let id = joltc_sys::JPC_BodyInterface_CreateAndAddBody(
                body_interface,
                &settings,
                joltc_sys::JPC_ACTIVATION_ACTIVATE,
            );

            joltc_sys::JPC_Shape_Release(shape_ptr);

            id
        };

        // Check for invalid body ID from Jolt (cInvalidBodyID = 0xFFFFFFFF)
        if body_id == INVALID_BODY_ID {
            return None;
        }
        let index = (body_id & BODY_INDEX_MASK) as usize;
        if index >= self.slots.len() {
            // Jolt handed out an index beyond our `max_bodies`; only possible
            // if the world was init'd with a different capacity than ours.
            return None;
        }
        self.slots[index] = body_id;
        self.user_data[index] = spawn.user_data;
        let handle = BodyHandle(BodyId::new(body_id));
        Some(handle)
    }

    /// Returns whether the body with the given ID is active (awake and
    /// simulated) in Jolt.
    ///
    /// Unknown, invalid, or already-destroyed IDs report `false`.
    #[must_use]
    pub fn is_active(&self, handle: BodyHandle) -> bool {
        if !self.is_valid(handle) {
            return false;
        }
        // SAFETY: The body ID occupies a live slot (verified by `is_valid`)
        // and is confirmed added before querying.
        unsafe {
            let body_interface = self.body_interface();
            if !joltc_sys::JPC_BodyInterface_IsAdded(body_interface, handle.0.raw()) {
                return false;
            }
            joltc_sys::JPC_BodyInterface_IsActive(body_interface, handle.0.raw())
        }
    }

    /// Teleports the body to a new world-space position and rotation,
    /// forcefully waking it.
    ///
    /// Unknown or destroyed IDs are ignored.
    pub fn set_position_and_rotation(
        &mut self,
        handle: BodyHandle,
        position: Vec3,
        rotation: Quat,
    ) {
        if !self.is_valid(handle) {
            return;
        }
        // SAFETY: body_id is valid and registered.
        unsafe {
            let body_interface = self.body_interface();
            let pos = joltc_sys::JPC_Vec3 {
                x: position.x,
                y: position.y,
                z: position.z,
                _w: 0.0,
            };
            let rot = joltc_sys::JPC_Quat {
                x: rotation.x,
                y: rotation.y,
                z: rotation.z,
                w: rotation.w,
            };
            joltc_sys::JPC_BodyInterface_SetPositionAndRotation(
                body_interface,
                handle.0.raw(),
                pos,
                rot,
                joltc_sys::JPC_ACTIVATION_ACTIVATE,
            );
        }
    }

    /// Retrieves the body's current world-space position and rotation.
    ///
    /// Returns `None` for unknown or already-destroyed IDs.
    pub fn get_transform(&self, handle: BodyHandle) -> Option<(Vec3, Quat)> {
        if !self.is_valid(handle) {
            return None;
        }
        // SAFETY: body_id is verified to be valid and allocated in this PhysicsWorld.
        unsafe {
            let body_interface = self.body_interface();

            if !joltc_sys::JPC_BodyInterface_IsAdded(body_interface, handle.0.raw()) {
                return None;
            }

            let mut pos = joltc_sys::JPC_Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                _w: 0.0,
            };
            let mut rot = joltc_sys::JPC_Quat {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 0.0,
            };

            joltc_sys::JPC_BodyInterface_GetPositionAndRotation(
                body_interface,
                handle.0.raw(),
                &mut pos,
                &mut rot,
            );

            Some((
                Vec3::new(pos.x, pos.y, pos.z),
                Quat::from_xyzw(rot.x, rot.y, rot.z, rot.w),
            ))
        }
    }

    /// Retrieves the body's linear velocity, in meters per second.
    ///
    /// Returns `None` for unknown or already-destroyed IDs.
    pub fn get_linear_velocity(&self, handle: BodyHandle) -> Option<Vec3> {
        if !self.is_valid(handle) {
            return None;
        }
        // SAFETY: We verify the body ID is valid and added before querying its state.
        unsafe {
            let body_interface = self.body_interface();

            if !joltc_sys::JPC_BodyInterface_IsAdded(body_interface, handle.0.raw()) {
                return None;
            }

            let lin_vel =
                joltc_sys::JPC_BodyInterface_GetLinearVelocity(body_interface, handle.0.raw());

            Some(Vec3::new(lin_vel.x, lin_vel.y, lin_vel.z))
        }
    }

    /// Retrieves the body's angular velocity, in radians per second.
    ///
    /// Returns `None` for unknown or already-destroyed IDs.
    pub fn get_angular_velocity(&self, handle: BodyHandle) -> Option<Vec3> {
        if !self.is_valid(handle) {
            return None;
        }
        // SAFETY: We verify the body ID is valid and added before querying its state.
        unsafe {
            let body_interface = self.body_interface();

            if !joltc_sys::JPC_BodyInterface_IsAdded(body_interface, handle.0.raw()) {
                return None;
            }

            let ang_vel =
                joltc_sys::JPC_BodyInterface_GetAngularVelocity(body_interface, handle.0.raw());

            Some(Vec3::new(ang_vel.x, ang_vel.y, ang_vel.z))
        }
    }

    /// Sets the body's linear velocity, in meters per second.
    ///
    /// Unknown or already-destroyed IDs are ignored.
    pub fn set_linear_velocity(&mut self, handle: BodyHandle, linear_velocity: Vec3) {
        if !self.is_valid(handle) {
            return;
        }
        // SAFETY: We verify the body ID is valid and added before mutating its state.
        unsafe {
            let body_interface = self.body_interface();

            if !joltc_sys::JPC_BodyInterface_IsAdded(body_interface, handle.0.raw()) {
                return;
            }

            let jolt_vel = joltc_sys::JPC_Vec3 {
                x: linear_velocity.x,
                y: linear_velocity.y,
                z: linear_velocity.z,
                _w: 0.0,
            };

            joltc_sys::JPC_BodyInterface_SetLinearVelocity(
                body_interface,
                handle.0.raw(),
                jolt_vel,
            );
        }
    }

    /// Sets the body's angular velocity, in radians per second.
    ///
    /// Unknown or already-destroyed IDs are ignored.
    pub fn set_angular_velocity(&mut self, handle: BodyHandle, angular_velocity: Vec3) {
        if !self.is_valid(handle) {
            return;
        }
        // SAFETY: We verify the body ID is valid and added before mutating its state.
        unsafe {
            let body_interface = self.body_interface();

            if !joltc_sys::JPC_BodyInterface_IsAdded(body_interface, handle.0.raw()) {
                return;
            }

            let jolt_vel = joltc_sys::JPC_Vec3 {
                x: angular_velocity.x,
                y: angular_velocity.y,
                z: angular_velocity.z,
                _w: 0.0,
            };

            joltc_sys::JPC_BodyInterface_SetAngularVelocity(
                body_interface,
                handle.0.raw(),
                jolt_vel,
            );
        }
    }

    /// Destroys the body with the given ID and releases its Jolt resources.
    ///
    /// Returns whether a registered body was destroyed; unknown or
    /// already-destroyed IDs return `false` without side effects.
    pub fn destroy_body(&mut self, handle: BodyHandle) -> bool {
        if !self.is_valid(handle) {
            return false;
        }
        let index = (handle.0.raw() & BODY_INDEX_MASK) as usize;

        // SAFETY: `is_valid` guarantees the ID occupies a live slot — a body
        // allocated by this PhysicsWorld that has not yet been destroyed.
        unsafe {
            let body_interface = self.body_interface();
            if joltc_sys::JPC_BodyInterface_IsAdded(body_interface, handle.0.raw()) {
                joltc_sys::JPC_BodyInterface_RemoveBody(body_interface, handle.0.raw());
            }
            joltc_sys::JPC_BodyInterface_DestroyBody(body_interface, handle.0.raw());
        }

        self.slots[index] = INVALID_BODY_ID;
        self.user_data[index] = 0;
        true
    }
}

impl Drop for PhysicsWorld {
    fn drop(&mut self) {
        // SAFETY: Clean up all live bodies before tearing down the physics
        // system; every non-invalid slot holds a body this world allocated.
        unsafe {
            let body_interface = self.body_interface();
            for &raw in &self.slots {
                if raw == INVALID_BODY_ID {
                    continue;
                }
                if joltc_sys::JPC_BodyInterface_IsAdded(body_interface, raw) {
                    joltc_sys::JPC_BodyInterface_RemoveBody(body_interface, raw);
                }
                joltc_sys::JPC_BodyInterface_DestroyBody(body_interface, raw);
            }
        }

        // SAFETY: Drop ordering is critical. The `physics_system` MUST be dropped first,
        // as Jolt internally references the job_system and temp_allocator during its shutdown.
        unsafe {
            ManuallyDrop::drop(&mut self.physics_system);
        }

        // SAFETY: Now that the physics system is destroyed, it is safe to delete the allocator and job system.
        unsafe {
            JPC_JobSystemThreadPool_delete(self.job_system.as_ptr());
            JPC_TempAllocatorImpl_delete(self.temp_allocator.as_ptr());
        }
    }
}

// SAFETY: `PhysicsWorld` owns every C++ resource it touches — the
// `PhysicsSystem`, the temp allocator, and the job system's worker threads
// (allocated in `new`, freed only in `Drop`) — and none of them have thread
// affinity. The Rust-side state (`slots`, `user_data`) is plain data. Moving
// the world between threads therefore moves everything it depends on with it.
unsafe impl Send for PhysicsWorld {}

// SAFETY: Sharing `&PhysicsWorld` across threads is sound because no public
// API mutates Jolt state through `&self`:
// - Every mutating method takes `&mut self`, so the borrow checker rules out
//   any overlap between mutation and anything else from the Rust side.
// - The `&self` surface is read-only: body queries (`get_transform`,
//   `user_data`, `is_active`, ...) go through Jolt's *locking* body interface
//   (`JPC_PhysicsSystem_GetBodyInterface`, internally mutex-protected), and
//   `ray_cast` goes through rolt's `NarrowPhaseQuery`, a read-only view.
// - The one raw door — the private `body_interface()` helper — never leaves
//   the crate: only this impl's own methods reach it, readers and writers
//   alike, each validating handles first (the reads are safe because the
//   helper returns Jolt's *locking* interface). The public `physics_system()`
//   escape hatch was removed precisely because it let safe code bypass all
//   of this.
// - `step` drives Jolt's own worker threads, but only while the caller
//   holds `&mut self`, so no `&self` reader can overlap a step.
// Rust-level exclusivity is the primary guarantee; Jolt's locks are defense
// in depth.
unsafe impl Sync for PhysicsWorld {}

#[cfg(test)]
mod tests {
    use super::*;

    /// Builds a minimal one-meter box spawn with sensible defaults; tests
    /// override individual fields (e.g. `position`, `shape`) as needed.
    fn box_spawn(body_type: BodyType, user_data: u64) -> BodySpawn {
        BodySpawn {
            shape: ShapeSpec::Box {
                half_extents: Vec3::splat(1.0),
            },
            body_type,
            position: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            linear_damping: 0.05,
            angular_damping: 0.05,
            user_data,
            use_ccd: false,
        }
    }

    /// Pins the `Send`/`Sync` promise at compile time: Bevy requires both for
    /// a `Resource`, and the `unsafe impl`s on `PhysicsWorld` are the manual
    /// guarantee behind them. If either impl is ever removed, this fails to
    /// compile.
    #[test]
    fn physics_world_is_send_and_sync() {
        fn assert_send_sync<T: Send + Sync>() {}
        assert_send_sync::<PhysicsWorld>();
    }

    #[test]
    fn test_set_position_and_rotation_valid_body() {
        let mut physics_world = PhysicsWorld::new(WorldConfig::default());
        let body_id = physics_world
            .spawn_body(box_spawn(BodyType::Dynamic, 0))
            .expect("Failed to spawn body");

        let new_pos = Vec3::new(10.0, 20.0, 30.0);
        let new_rot = Quat::from_rotation_x(std::f32::consts::PI / 2.0);

        physics_world.set_position_and_rotation(body_id, new_pos, new_rot);

        let (fetched_pos, fetched_rot) = physics_world.get_transform(body_id).unwrap();
        assert!(fetched_pos.distance(new_pos) < 1e-4);
        // Quat::angle_between requires normalized quats, and angle might be slightly off.
        // Dot product is safe.
        assert!((1.0 - fetched_rot.dot(new_rot).abs()) < 1e-4);
    }

    #[test]
    fn test_set_position_and_rotation_invalid_body_noop() {
        let mut physics_world = PhysicsWorld::new(WorldConfig::default());
        let invalid_id = BodyHandle(BodyId::new(INVALID_BODY_ID));
        // Should not panic or crash
        physics_world.set_position_and_rotation(invalid_id, Vec3::ZERO, Quat::IDENTITY);

        let fake_id = BodyHandle(BodyId::new(9999));
        physics_world.set_position_and_rotation(fake_id, Vec3::ZERO, Quat::IDENTITY);
    }

    #[test]
    fn test_spawn_static_box() {
        let mut physics_world = PhysicsWorld::new(WorldConfig::default());
        let result = physics_world.spawn_body(box_spawn(BodyType::Static, 0));
        assert!(result.is_some());
    }

    #[test]
    fn test_spawn_kinematic_box() {
        let mut physics_world = PhysicsWorld::new(WorldConfig::default());
        let result = physics_world.spawn_body(box_spawn(BodyType::Kinematic, 0));
        assert!(result.is_some());
    }

    #[test]
    fn test_spawn_with_invalid_shape_returns_none() {
        let mut physics_world = PhysicsWorld::new(WorldConfig::default());
        let mut spawn = box_spawn(BodyType::Static, 0);
        spawn.shape = ShapeSpec::Box {
            half_extents: Vec3::splat(-1.0),
        };
        assert!(physics_world.spawn_body(spawn).is_none());
    }

    #[test]
    fn test_is_finite_and_non_negative_truth_table() {
        // The table is the contract: finiteness and sign, never magnitude.
        let cases: &[(f32, bool)] = &[
            // Valid: zero boundary, the Jolt default, ordinary tuning
            // values, and the finite extremes (no epsilon floor, no
            // upper bound — huge damping is a user choice, not a bug).
            (0.0, true),
            (-0.0, true), // IEEE: -0.0 >= 0.0 — negative zero *is* zero
            (0.05, true),
            (1.0, true),
            (5.0, true),
            (f32::MIN_POSITIVE, true),
            (f32::MAX, true),
            // Invalid: NaN, infinities, and negatives at every magnitude.
            (f32::NAN, false),
            (f32::INFINITY, false), // passes `>= 0.0`; only is_finite catches it
            (f32::NEG_INFINITY, false),
            (-0.1, false),
            (-1e-9, false), // no tolerance: any negative adds energy
            (f32::MIN, false),
        ];

        // Collect-then-assert so one run reports every misbehaving input.
        let failures: Vec<f32> = cases
            .iter()
            .filter(|&&(value, expected)| is_finite_and_non_negative(value) != expected)
            .map(|&(value, _)| value)
            .collect();

        assert!(
            failures.is_empty(),
            "predicate returned wrong verdicts for: {failures:?}"
        );
    }

    #[test]
    fn test_spawn_rejects_negative_damping() {
        let mut physics_world = PhysicsWorld::new(WorldConfig::default());

        let mut spawn = box_spawn(BodyType::Dynamic, 0);
        spawn.linear_damping = -0.1;
        assert!(physics_world.spawn_body(spawn).is_none());

        let mut spawn = box_spawn(BodyType::Dynamic, 0);
        spawn.angular_damping = -0.1;
        assert!(physics_world.spawn_body(spawn).is_none());
    }

    #[test]
    fn test_spawn_rejects_non_finite_damping() {
        let mut physics_world = PhysicsWorld::new(WorldConfig::default());

        let mut spawn = box_spawn(BodyType::Dynamic, 0);
        spawn.linear_damping = f32::NAN;
        assert!(physics_world.spawn_body(spawn).is_none());

        let mut spawn = box_spawn(BodyType::Dynamic, 0);
        spawn.angular_damping = f32::NAN;
        assert!(physics_world.spawn_body(spawn).is_none());

        // +inf passes `>= 0.0`; only the finiteness half of the guard
        // rejects it (the truth table pins this at the predicate level).
        let mut spawn = box_spawn(BodyType::Dynamic, 0);
        spawn.linear_damping = f32::INFINITY;
        assert!(physics_world.spawn_body(spawn).is_none());
    }

    #[test]
    fn test_high_linear_damping_decelerates_faster() {
        let mut physics_world = PhysicsWorld::new(WorldConfig::default());
        physics_world.set_gravity(Vec3::ZERO);

        // Two dynamic boxes, offset so they never touch, with identical
        // initial velocities and differing only in linear damping.
        let default_body = physics_world
            .spawn_body(box_spawn(BodyType::Dynamic, 0))
            .expect("default-damping body must spawn");
        let damped_body = physics_world
            .spawn_body(BodySpawn {
                linear_damping: 5.0,
                position: Vec3::new(0.0, 10.0, 0.0),
                ..box_spawn(BodyType::Dynamic, 1)
            })
            .expect("high-damping body must spawn");

        physics_world.set_linear_velocity(default_body, Vec3::new(10.0, 0.0, 0.0));
        physics_world.set_linear_velocity(damped_body, Vec3::new(10.0, 0.0, 0.0));

        physics_world.step(0.5, 1);

        let default_speed = physics_world
            .get_linear_velocity(default_body)
            .expect("default-damping body must be queryable")
            .length();
        let damped_speed = physics_world
            .get_linear_velocity(damped_body)
            .expect("high-damping body must be queryable")
            .length();

        // v(0.5) = v0·e^(−d·0.5): ~9.75 m/s at d = 0.05 vs ~0.82 m/s at
        // d = 5.0. The 25% threshold sits far from both — and if the
        // write-through ever regresses, both bodies decay identically and
        // the ratio approaches 1.
        assert!(
            damped_speed < 0.25 * default_speed,
            "high-damping speed {damped_speed} should be far below default-damping speed {default_speed}"
        );
    }

    #[test]
    fn test_get_transform_invalid_and_destroyed_body() {
        let mut physics_world = PhysicsWorld::new(WorldConfig::default());

        // 1. Querying an invalid body ID returns None
        let invalid_id = BodyHandle(BodyId::new(INVALID_BODY_ID));
        assert!(physics_world.get_transform(invalid_id).is_none());

        // 2. Querying an unallocated body ID returns None
        let fake_id = BodyHandle(BodyId::new(9999));
        assert!(physics_world.get_transform(fake_id).is_none());

        // 3. Spawning a valid box returns Some(...)
        let mut spawn = box_spawn(BodyType::Dynamic, 0);
        spawn.position = Vec3::new(1.0, 2.0, 3.0);
        let body_id = physics_world
            .spawn_body(spawn)
            .expect("Failed to spawn box");
        assert!(physics_world.get_transform(body_id).is_some());

        // 4. Destroying the body causes get_transform to return None
        physics_world.destroy_body(body_id);
        assert!(physics_world.get_transform(body_id).is_none());
    }

    #[test]
    fn test_spawn_body_stores_user_data() {
        let mut physics_world = PhysicsWorld::new(WorldConfig::default());
        let body_id = physics_world
            .spawn_body(box_spawn(BodyType::Dynamic, 42))
            .expect("Failed to spawn box");

        // 1. The payload round-trips through the public getter while the body lives
        let user_data = physics_world.user_data(body_id).unwrap();
        assert_eq!(user_data, 42);

        // 2. Destroying the body makes the handle stale: the getter must reject it
        physics_world.destroy_body(body_id);
        assert!(physics_world.user_data(body_id).is_none());
    }

    #[test]
    fn test_ray_cast_returns_closest_hit() {
        let mut physics_world = PhysicsWorld::new(WorldConfig::default());
        let box_id = physics_world
            .spawn_body(box_spawn(BodyType::Static, 42))
            .expect("Failed to spawn box");

        // Ray from 4 m above the box's top face (the face sits at y = 1),
        // with a total length of 10 m: the hit is 40% along the ray.
        let hit = physics_world
            .ray_cast(Vec3::new(0.0, 5.0, 0.0), Vec3::new(0.0, -10.0, 0.0))
            .expect("Ray must hit the box");

        assert_eq!(hit.body, box_id);
        assert_eq!(hit.user_data, 42);
        assert!((hit.fraction - 0.4).abs() < 1e-4);
        assert!(hit.hit_point.distance(Vec3::new(0.0, 1.0, 0.0)) < 1e-4);
    }

    #[test]
    fn test_ray_cast_reports_the_nearest_of_multiple_hits() {
        let mut physics_world = PhysicsWorld::new(WorldConfig::default());
        physics_world
            .spawn_body(box_spawn(BodyType::Static, 1))
            .expect("Failed to spawn lower box");
        let upper = BodySpawn {
            position: Vec3::new(0.0, 4.0, 0.0),
            ..box_spawn(BodyType::Static, 2)
        };
        physics_world
            .spawn_body(upper)
            .expect("Failed to spawn upper box");

        // Both boxes intersect the ray; the query must report the nearer
        // (upper) one: its top face is 5 m from the origin on a 20 m ray.
        let hit = physics_world
            .ray_cast(Vec3::new(0.0, 10.0, 0.0), Vec3::new(0.0, -20.0, 0.0))
            .expect("Ray must hit a box");

        assert_eq!(hit.user_data, 2);
        assert!((hit.fraction - 0.25).abs() < 1e-4);
        assert!(hit.hit_point.distance(Vec3::new(0.0, 5.0, 0.0)) < 1e-4);
    }

    #[test]
    fn test_ray_cast_miss_and_short_ray_return_none() {
        let mut physics_world = PhysicsWorld::new(WorldConfig::default());
        physics_world
            .spawn_body(box_spawn(BodyType::Static, 42))
            .expect("Failed to spawn box");

        // 1. Pointing away from the box: a miss.
        assert!(
            physics_world
                .ray_cast(Vec3::new(0.0, 5.0, 0.0), Vec3::new(0.0, 10.0, 0.0))
                .is_none()
        );

        // 2. Pointing at the box but only 2 m long: the ray ends at y = 3,
        //    while the box's top face is at y = 1. The direction's magnitude
        //    is the ray's maximum distance.
        assert!(
            physics_world
                .ray_cast(Vec3::new(0.0, 5.0, 0.0), Vec3::new(0.0, -2.0, 0.0))
                .is_none()
        );

        // 3. A normalized direction is a 1 m ray: the classic gotcha.
        assert!(
            physics_world
                .ray_cast(Vec3::new(0.0, 5.0, 0.0), Vec3::new(0.0, -1.0, 0.0))
                .is_none()
        );
    }

    #[test]
    fn test_ray_cast_rejects_invalid_inputs() {
        let physics_world = PhysicsWorld::new(WorldConfig::default());

        // Non-finite origin, non-finite direction, infinite direction, and a
        // zero-length direction are all rejected as None before touching Jolt.
        assert!(
            physics_world
                .ray_cast(Vec3::new(f32::NAN, 0.0, 0.0), Vec3::new(0.0, -10.0, 0.0))
                .is_none()
        );
        assert!(
            physics_world
                .ray_cast(Vec3::new(0.0, 5.0, 0.0), Vec3::new(0.0, f32::NAN, 0.0))
                .is_none()
        );
        assert!(
            physics_world
                .ray_cast(Vec3::new(0.0, 5.0, 0.0), Vec3::new(f32::INFINITY, 0.0, 0.0))
                .is_none()
        );
        assert!(
            physics_world
                .ray_cast(Vec3::new(0.0, 5.0, 0.0), Vec3::ZERO)
                .is_none()
        );
    }

    #[test]
    fn test_step_handles_invalid_delta_time() {
        let mut physics_world = PhysicsWorld::new(WorldConfig::default());

        // Calling step with 0.0, negative dt, or NaN should safely return without panic
        physics_world.step(0.0, 1);
        physics_world.step(-1.0 / 60.0, 1);
        physics_world.step(f32::NAN, 1);
    }

    #[test]
    fn test_step_clamps_non_positive_collision_steps() {
        let mut physics_world = PhysicsWorld::new(WorldConfig::default());
        let body_id = physics_world
            .spawn_body(box_spawn(BodyType::Dynamic, 0))
            .expect("Failed to spawn box");

        // Zero and negative substep counts clamp to a single collision step:
        // no panic, and the simulation still advances under default gravity.
        physics_world.step(1.0 / 60.0, 0);
        physics_world.step(1.0 / 60.0, -3);

        let (position, _) = physics_world.get_transform(body_id).unwrap();
        assert!(
            position.y < 0.0,
            "body should have fallen under default gravity"
        );
    }

    #[test]
    fn test_velocity_getters_and_setters() {
        let mut physics_world = PhysicsWorld::new(WorldConfig::default());
        let body_id = physics_world
            .spawn_body(box_spawn(BodyType::Dynamic, 0))
            .expect("Failed to spawn box");

        let target_lin_vel = Vec3::new(1.0, 2.0, 3.0);
        let target_ang_vel = Vec3::new(4.0, 5.0, 6.0);

        physics_world.set_linear_velocity(body_id, target_lin_vel);
        physics_world.set_angular_velocity(body_id, target_ang_vel);

        assert_eq!(
            physics_world.get_linear_velocity(body_id).unwrap(),
            target_lin_vel
        );
        assert_eq!(
            physics_world.get_angular_velocity(body_id).unwrap(),
            target_ang_vel
        );
    }

    #[test]
    fn test_is_active_lifecycle() {
        let mut physics_world = PhysicsWorld::new(WorldConfig::default());

        // 1. Invalid and unallocated IDs return false
        assert!(!physics_world.is_active(BodyHandle(BodyId::new(INVALID_BODY_ID))));
        assert!(!physics_world.is_active(BodyHandle(BodyId::new(9999))));

        // 2. Newly spawned dynamic box is active
        let body_id = physics_world
            .spawn_body(box_spawn(BodyType::Dynamic, 0))
            .expect("Failed to spawn box");
        assert!(physics_world.is_active(body_id));

        // 3. Destroyed body returns false
        physics_world.destroy_body(body_id);
        assert!(!physics_world.is_active(body_id));
    }

    #[test]
    fn test_stale_generation_handle_is_invalid() {
        let mut physics_world = PhysicsWorld::new(WorldConfig::default());
        let body_id = physics_world
            .spawn_body(box_spawn(BodyType::Dynamic, 0))
            .expect("Failed to spawn box");
        physics_world.destroy_body(body_id);

        // Same index, bumped sequence number: a stale handle to a recycled
        // slot. The slot table must reject it because the full raw ID —
        // including the generation bits — no longer matches.
        let stale = BodyHandle(BodyId::new(body_id.0.raw() + (1 << 23)));
        assert!(!physics_world.is_active(stale));
        assert!(physics_world.get_transform(stale).is_none());
    }

    #[test]
    fn test_recycled_slot_rejects_original_handle() {
        let mut physics_world = PhysicsWorld::new(WorldConfig::default());
        let first = physics_world
            .spawn_body(box_spawn(BodyType::Dynamic, 1))
            .expect("Failed to spawn first box");
        physics_world.destroy_body(first);

        // Jolt recycles freed slots: the next body re-occupies the same index
        // with a fresh sequence number. This is the real-world version of
        // `test_stale_generation_handle_is_invalid` — no hand-forged bits.
        let second = physics_world
            .spawn_body(box_spawn(BodyType::Dynamic, 2))
            .expect("Failed to spawn second box");

        assert_eq!(
            first.0.raw() & BODY_INDEX_MASK,
            second.0.raw() & BODY_INDEX_MASK,
            "Jolt did not recycle the freed slot; adjust this test"
        );
        assert_ne!(first.0.raw(), second.0.raw());

        // The stale handle must be rejected even though a live body now
        // occupies its slot — and the new handle must be fully valid.
        assert!(!physics_world.is_active(first));
        assert!(physics_world.get_transform(first).is_none());
        assert!(physics_world.user_data(first).is_none());
        assert!(physics_world.is_active(second));
        assert_eq!(physics_world.user_data(second), Some(2));
    }

    #[test]
    fn test_spawned_handles_are_unique() {
        let mut physics_world = PhysicsWorld::new(WorldConfig::default());
        let mut seen = std::collections::HashSet::new();
        for i in 0..64u64 {
            let handle = physics_world
                .spawn_body(box_spawn(BodyType::Dynamic, i))
                .expect("Failed to spawn box");
            assert!(seen.insert(handle.0), "duplicate body handle handed out");
        }
        assert_eq!(seen.len(), 64);
    }

    #[test]
    fn test_physics_world_drop_destroys_live_bodies_cleanly() {
        let mut physics_world = PhysicsWorld::new(WorldConfig::default());

        // Spawn multiple dynamic bodies
        for i in 0..10 {
            let spawn = BodySpawn {
                position: Vec3::new(i as f32, 0.0, 0.0),
                ..box_spawn(BodyType::Dynamic, i as u64)
            };
            physics_world.spawn_body(spawn);
        }

        // Dropping physics_world with active bodies must not trigger JPH_ASSERT(mNumBodies == 0)
        drop(physics_world);
    }

    #[test]
    fn test_drop_destroys_mixed_body_types_cleanly() {
        let mut physics_world = PhysicsWorld::new(WorldConfig::default());

        // Static, kinematic, and dynamic bodies take different layer and
        // motion-type paths through teardown; sweep all of them.
        for (body_type, lane) in [
            (BodyType::Static, 0.0),
            (BodyType::Kinematic, 10.0),
            (BodyType::Dynamic, 20.0),
        ] {
            for layer in 0..4 {
                let spawn = BodySpawn {
                    position: Vec3::new(lane, layer as f32, 0.0),
                    ..box_spawn(body_type, layer as u64)
                };
                physics_world
                    .spawn_body(spawn)
                    .expect("Failed to spawn body");
            }
        }

        // Dropping with live bodies of every motion type must not trip
        // Jolt's teardown assertions.
        drop(physics_world);
    }

    #[test]
    fn test_getters_return_none_for_unallocated_body_id() {
        let physics_world = PhysicsWorld::new(WorldConfig::default());
        let unallocated = BodyHandle(BodyId::new(99_999));

        assert!(physics_world.get_transform(unallocated).is_none());
        assert!(physics_world.get_linear_velocity(unallocated).is_none());
        assert!(physics_world.get_angular_velocity(unallocated).is_none());
    }

    #[test]
    fn test_setters_noop_for_unallocated_body_id() {
        let mut physics_world = PhysicsWorld::new(WorldConfig::default());
        let unallocated = BodyHandle(BodyId::new(99_999));

        // Mutating unallocated ID must safely no-op without memory corruption
        physics_world.set_linear_velocity(unallocated, Vec3::X);
        physics_world.set_angular_velocity(unallocated, Vec3::Y);
    }

    #[test]
    #[should_panic(expected = "temp_allocator_size_mb in bytes overflows u32")]
    fn test_temp_allocator_overflow_panics_safely() {
        let config = WorldConfig {
            temp_allocator_size_mb: u32::MAX,
            ..Default::default()
        };
        let _ = PhysicsWorld::new(config);
    }

    #[test]
    #[should_panic(expected = "max_bodies must be at least 1")]
    fn test_zero_max_bodies_panics_readably() {
        let config = WorldConfig {
            max_bodies: 0,
            ..Default::default()
        };
        let _ = PhysicsWorld::new(config);
    }

    #[test]
    #[should_panic(expected = "exceeds Jolt's hard limit")]
    fn test_max_bodies_above_jolts_limit_panics_readably() {
        let config = WorldConfig {
            max_bodies: JOLT_MAX_BODIES + 1,
            ..Default::default()
        };
        let _ = PhysicsWorld::new(config);
    }

    #[test]
    fn test_one_max_bodies_is_the_legal_floor() {
        let mut physics_world = PhysicsWorld::new(WorldConfig {
            max_bodies: 1,
            ..Default::default()
        });
        // The floor must be usable, not merely accepted: one body spawns
        // into the single slot and reads back through its handle.
        let handle = physics_world
            .spawn_body(box_spawn(BodyType::Dynamic, 0))
            .expect("the single slot must spawn a body");
        assert!(physics_world.get_transform(handle).is_some());
    }
}
