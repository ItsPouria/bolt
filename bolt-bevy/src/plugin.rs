use bevy::prelude::*;

use crate::gravity::{Gravity, apply_gravity};
use crate::systems::spawn_bodies;
use crate::world::PhysicsWorld;

#[derive(Default, Debug)]
pub struct BoltPlugin {}

impl Plugin for BoltPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<PhysicsWorld>();
        app.init_resource::<Gravity>(); // Initialize the gravity resource

        // Run gravity application BEFORE we step the physics world
        app.add_systems(Update, (spawn_bodies, apply_gravity, step_physics).chain());
        app.add_systems(Update, step_physics);
    }
}

fn step_physics(mut world: ResMut<PhysicsWorld>, time: Res<Time>) {
    let delta_time = time.delta_secs();
    let collision_steps = 1;
    world.step(delta_time, collision_steps);
}
