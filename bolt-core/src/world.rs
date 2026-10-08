//! The engine-agnostic Jolt physics world and its construction vocabulary.
//!
//! This module is the future home of the `PhysicsWorld` type itself — the Jolt
//! `PhysicsSystem`, its allocators and job system, and the body management
//! API — together with the engine-agnostic types that engine integrations
//! convert their component data into before touching the world
//! ([`WorldConfig`], and the upcoming body-spawning vocabulary).

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
