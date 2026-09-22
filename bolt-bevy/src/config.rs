use bevy::prelude::*;

#[derive(Resource, Debug, Clone)]
pub struct PhysicsConfig {
    /// The maximum number of physics objects allowed in the world.
    pub max_bodies: u32,
    /// The maximum number of pairs of objects that can potentially collide in a single frame.
    pub max_body_pairs: u32,
    /// The maximum number of contact constraints Jolt will solve in a single frame.
    pub max_contact_constraints: u32,
    /// Number of worker threads Jolt's JobSystem should use.
    pub num_threads: i32,
    /// Global gravity acceleration.
    pub gravity: Vec3,
    /// Number of collision sub-steps per physics update (default: 1).
    pub collision_steps: u32,
}

impl Default for PhysicsConfig {
    fn default() -> Self {
        Self {
            max_bodies: 10240,
            max_body_pairs: 65536,
            max_contact_constraints: 10240,
            num_threads: 2,
            gravity: Vec3::new(0.0, -9.81, 0.0),
            collision_steps: 1,
        }
    }
}
