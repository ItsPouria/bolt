use bevy::prelude::*;

use crate::world::PhysicsWorld;

#[derive(Default, Debug)]
pub struct BoltPlugin {}

impl Plugin for BoltPlugin {
    fn build(&self, app: &mut bevy::app::App) {
        app.init_resource::<PhysicsWorld>();
        app.add_systems(Update, step_physics);
    }
}

fn step_physics(mut world: ResMut<PhysicsWorld>, time: Res<Time>) {
    let delta_time = time.delta_secs();
    let collision_steps = 1;
    world.step(delta_time, collision_steps);
}
