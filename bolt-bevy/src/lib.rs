//! Bevy integration for the Jolt Physics engine.
#![deny(unsafe_op_in_unsafe_fn)]
#![warn(missing_docs)]
#![warn(clippy::undocumented_unsafe_blocks)]
/// Bevy components for physics bodies and shapes
pub mod components;
/// Configuration resources for the physics engine
pub mod config;
/// Gravity resources and systems
pub mod gravity;
/// Main Bevy plugin for the physics engine
pub mod plugin;
/// Re-exports of common items
pub mod prelude;
/// Internal physics simulation systems
pub mod systems;
/// Core physics world and simulation
pub mod world;

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
