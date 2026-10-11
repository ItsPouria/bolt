//! The engine-agnostic Jolt physics world and its construction vocabulary.
//!
//! This module is the future home of the `PhysicsWorld` type itself — the Jolt
//! `PhysicsSystem`, its allocators and job system, and the body management
//! API — together with the engine-agnostic types that engine integrations
//! convert their component data into before touching the world:
//! [`WorldConfig`], [`BodyType`], and [`BodySpawn`].

use glam::{Quat, Vec3};

use crate::shapes::ShapeSpec;

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
