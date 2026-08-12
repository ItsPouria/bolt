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

pub fn step_physics(world: ResMut<PhysicsWorld>, time: Res<Time>) {
    let delta_time = time.delta_secs();
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
