use bevy::ecs::resource::Resource;

#[derive(Resource, Debug, Clone)]
pub struct PhysicsConfig {
    pub max_bodies: u32, //The maximum number of physics objects allowed in the world.
    pub max_body_pairs: u32, //The maximum number of pairs of objects that can potentially collide in a
    //single frame.
    pub max_contact_constraints: u32, //The maximum number of contact poits Jolt will try to solve in a
    //single frame.
    pub num_threads: i32, //How many background threads Jolt's JobSystem should use.
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
