//! Bevy integration for the Jolt Physics engine.
pub mod layers;
pub mod prelude;
pub mod world;
pub use joltc_sys;
pub use rolt;
pub mod config;
pub mod plugin;

#[cfg(test)]
mod tests {
    use crate::config::PhysicsConfig;
    use crate::world::PhysicsWorld;

    #[test]
    fn hello_world_physics() {
        let world = PhysicsWorld::new(PhysicsConfig::default());

        let delta_time = 1.0 / 60.0; // 60 FPS
        let collision_steps = 1;

        unsafe {
            world.physics_system.update(
                delta_time,
                collision_steps,
                world.temp_allocator.as_ptr(),
                world.job_system.as_ptr(),
            );
        }
    }
}
