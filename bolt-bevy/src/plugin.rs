use bevy::prelude::*;

use crate::config::PhysicsConfig;
use crate::gravity::{Gravity, apply_gravity};
use crate::systems::sync_transforms;
use crate::world::PhysicsWorld;

#[derive(Default, Debug)]
pub struct BoltPlugin {}

impl Plugin for BoltPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<PhysicsConfig>();
        app.init_resource::<PhysicsWorld>();
        app.init_resource::<Gravity>();

        app.add_systems(Update, (crate::systems::spawn_physics_bodies,));
        app.add_systems(
            FixedUpdate,
            (apply_gravity, step_physics, sync_transforms).chain(),
        );
        app.add_observer(crate::systems::cleanup_despawned_physics_bodies);
    }
}

fn step_physics(
    mut world: ResMut<PhysicsWorld>,
    config: Res<PhysicsConfig>,
    time: Res<Time<Fixed>>,
) {
    let delta_time = time.delta_secs();
    world.step(delta_time, config.collision_steps as i32);
}
