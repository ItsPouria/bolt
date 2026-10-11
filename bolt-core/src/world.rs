//! The engine-agnostic Jolt physics world and its construction vocabulary.
//!
//! This module is the future home of the `PhysicsWorld` type itself — the Jolt
//! `PhysicsSystem`, its allocators and job system, and the body management
//! API — together with the engine-agnostic types that engine integrations
//! convert their component data into before touching the world:
//! [`WorldConfig`], [`BodyType`], and [`BodySpawn`].

use std::collections::HashMap;
use std::mem::ManuallyDrop;
use std::ptr::NonNull;

use crate::layers::{
    SimpleBroadPhaseLayer, SimpleObjectLayerPairFilter, SimpleObjectVsBroadPhaseLayerFilter,
};
use crate::shapes::ShapeSpec;
use glam::{Quat, Vec3};
use joltc_sys::{
    JPC_JobSystemThreadPool, JPC_JobSystemThreadPool_delete, JPC_JobSystemThreadPool_new3,
    JPC_MAX_PHYSICS_BARRIERS, JPC_MAX_PHYSICS_JOBS, JPC_PhysicsSystem_SetGravity,
    JPC_TempAllocatorImpl, JPC_TempAllocatorImpl_delete, JPC_TempAllocatorImpl_new, JPC_Vec3,
};
use rolt::{BodyId, PhysicsSystem};

/// Constant representing an invalid or unallocated Jolt body ID (0xFFFFFFFF).
pub const INVALID_BODY_ID: u32 = 0xffff_ffff;

/// Runtime parameters for constructing a physics world.
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
/// This is the parameter object of the world's body-spawning API: everything
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

/// The Jolt physics world: owns the `PhysicsSystem`, the temporary allocator,
/// and the job system required to step the simulation.
pub struct PhysicsWorld {
    physics_system: ManuallyDrop<PhysicsSystem>,
    temp_allocator: NonNull<JPC_TempAllocatorImpl>,
    job_system: NonNull<JPC_JobSystemThreadPool>,
    body_registry: HashMap<BodyId, u64>,
}

impl PhysicsWorld {
    /// Creates a new physics world with the given configuration.
    ///
    /// Initializes the Jolt runtime on first use and starts the job system's
    /// worker threads.
    pub fn new(config: WorldConfig) -> Self {
        crate::init::ensure_jolt_initialized();
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
            body_registry: HashMap::new(),
        }
    }

    /// Returns a shared reference to the underlying Jolt [`PhysicsSystem`].
    ///
    /// This is an escape hatch for Jolt capabilities `bolt-core` does not yet
    /// wrap — narrow-phase queries, contact listeners, constraints — reachable
    /// through the safe `rolt` API. Prefer the methods on [`PhysicsWorld`]
    /// where available: they validate body IDs against the registry before
    /// touching Jolt state, while anything done through this reference is the
    /// caller's responsibility.
    ///
    /// The reference borrows the world (`&self`), so it cannot outlive it.
    pub fn physics_system(&self) -> &PhysicsSystem {
        &self.physics_system
    }

    /// Helper to get the Jolt BodyInterface.
    ///
    /// # Safety
    /// The caller must ensure that the `PhysicsSystem` is still valid.
    unsafe fn body_interface(&self) -> *mut joltc_sys::JPC_BodyInterface {
        // SAFETY: The raw pointer from `physics_system` is valid.
        unsafe { joltc_sys::JPC_PhysicsSystem_GetBodyInterface(self.physics_system.raw()) }
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
}

impl Drop for PhysicsWorld {
    fn drop(&mut self) {
        //SAFETY: Clean up all live bodies before tearing down the physics system.
        unsafe {
            let body_interface = self.body_interface();
            for (body_id, _) in self.body_registry.drain() {
                if joltc_sys::JPC_BodyInterface_IsAdded(body_interface, body_id.raw()) {
                    joltc_sys::JPC_BodyInterface_RemoveBody(body_interface, body_id.raw());
                }
                joltc_sys::JPC_BodyInterface_DestroyBody(body_interface, body_id.raw());
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

// SAFETY: The Jolt `PhysicsSystem` is designed for multi-threaded access.
// SAFETY: The Jolt `PhysicsSystem` and `BodyInterface` are internally synchronized
// via mutexes (num_body_mutexes). We enforce that `PhysicsWorld` only exposes
// Jolt state mutation through `&mut self` (exclusive access), ensuring no
// data races can occur from the Rust side.
unsafe impl Send for PhysicsWorld {}
// SAFETY: See Send justification. Jolt's C++ locks make `&self` reads safe across threads.
unsafe impl Sync for PhysicsWorld {}
