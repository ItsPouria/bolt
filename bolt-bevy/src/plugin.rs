use bevy::prelude::*;

use crate::gravity::{Gravity, apply_gravity};
use crate::registry::PhysicsRegistry;
use crate::world::PhysicsWorld;

#[derive(Default, Debug)]
pub struct BoltPlugin {}

impl Plugin for BoltPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<PhysicsWorld>();
        app.init_resource::<Gravity>(); // Initialize the gravity resource
        app.init_resource::<PhysicsRegistry>();

        // Spawn bodies, apply gravity, and step the physics world in order
        app.add_systems(Update, (crate::systems::spawn_physics_bodies, apply_gravity, step_physics).chain());
    }
}

fn step_physics(mut world: ResMut<PhysicsWorld>, time: Res<Time>) {
    let delta_time = time.delta_secs();
    let collision_steps = 1;
    world.step(delta_time, collision_steps);
}
