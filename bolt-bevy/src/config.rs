use bevy::ecs::resource::Resource;

#[derive(Resource, Debug, Clone)]
pub struct PhysicsConfig {
    max_bodies: u32,     //The maximum number of physics objects allowed in the world.
    max_body_pairs: u32, //The maximum number of pairs of objects that can potentially collide in a
    //single frame.
    max_contact_constraints: u32, //The maximum number of contact poits Jolt will try to solve in a
    //single frame.
    num_threads: u32, //How many background threads Jolt's JobSystem should use.
}

impl Default for PhysicsConfig {
    fn default() -> Self {
        Self {
            max_bodies: 10240,
            max_body_pairs: 65536,
            max_contact_constraints: 10240,
            num_threads: 2,
        }
    }
}
