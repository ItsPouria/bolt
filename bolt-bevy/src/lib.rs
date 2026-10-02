//! Bevy integration for the Jolt Physics engine.
#![deny(unsafe_op_in_unsafe_fn)]
#![warn(missing_docs)]
#![warn(clippy::undocumented_unsafe_blocks)]
/// The `layers` module.
pub mod layers;
/// The `prelude` module.
pub mod prelude;
/// The `world` module.
pub mod world;
pub use joltc_sys;
pub use rolt;
/// The `components` module.
pub mod components;
/// The `config` module.
pub mod config;
/// The `gravity` module.
pub mod gravity;
/// The `plugin` module.
pub mod plugin;
/// The `systems` module.
pub mod systems;

#[cfg(test)]
mod tests {
    use crate::config::PhysicsConfig;
    use crate::world::PhysicsWorld;

    #[test]
    fn hello_world_physics() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());

        let delta_time = 1.0 / 60.0; // 60 FPS
        let collision_steps = 1;

        world.step(delta_time, collision_steps);
    }
}
