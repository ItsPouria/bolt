use bevy::prelude::*;

use crate::config::PhysicsConfig;
use crate::gravity::{Gravity, apply_gravity};
use crate::systems::{apply_velocities, sync_transforms};
use crate::world::PhysicsWorld;

#[derive(Default, Debug)]
/// Boltplugin struct.
pub struct BoltPlugin {
    /// The configuration for the physics world.
    pub config: PhysicsConfig,
}

impl Plugin for BoltPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(self.config.clone());
        app.init_resource::<PhysicsWorld>();
        app.init_resource::<Gravity>();

        app.configure_sets(FixedUpdate, (
            crate::systems::PhysicsSet::Spawn,
            crate::systems::PhysicsSet::ApplyVelocities,
            crate::systems::PhysicsSet::ApplyGravity,
            crate::systems::PhysicsSet::Step,
            crate::systems::PhysicsSet::SyncTransforms,
        ).chain());

        app.add_systems(
            FixedUpdate,
            (
                crate::systems::spawn_physics_bodies.in_set(crate::systems::PhysicsSet::Spawn),
                apply_velocities.in_set(crate::systems::PhysicsSet::ApplyVelocities),
                apply_gravity.in_set(crate::systems::PhysicsSet::ApplyGravity),
                crate::systems::step_physics.in_set(crate::systems::PhysicsSet::Step),
                sync_transforms.in_set(crate::systems::PhysicsSet::SyncTransforms),
            ),
        );
        app.add_observer(crate::systems::cleanup_despawned_physics_bodies);
    }
}
