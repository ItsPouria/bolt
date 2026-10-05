use bevy::prelude::*;

use crate::config::PhysicsConfig;
use crate::gravity::{Gravity, apply_gravity};
use crate::systems::{apply_velocities, spawn_physics_bodies, step_physics, sync_transforms};
use crate::world::PhysicsWorld;

#[derive(Default, Debug)]
/// Boltplugin struct.
pub struct BoltPlugin {
    /// The configuration for the physics world.
    pub config: PhysicsConfig,
}

impl Plugin for BoltPlugin {
    fn build(&self, app: &mut App) {
        if !app.world().contains_resource::<PhysicsConfig>() {
            app.insert_resource(self.config.clone());
        }
        app.init_resource::<PhysicsWorld>();
        app.init_resource::<Gravity>();

        // Register all reflected physics types
        app.register_type::<crate::components::RigidBody>()
            .register_type::<crate::components::Collider>()
            .register_type::<crate::components::LinearVelocity>()
            .register_type::<crate::components::AngularVelocity>()
            .register_type::<crate::components::StaticMarker>()
            .register_type::<crate::config::PhysicsConfig>()
            .register_type::<crate::gravity::Gravity>();

        app.configure_sets(
            FixedUpdate,
            (
                crate::systems::PhysicsSet::Spawn,
                crate::systems::PhysicsSet::ApplyVelocities,
                crate::systems::PhysicsSet::ApplyGravity,
                crate::systems::PhysicsSet::Step,
                crate::systems::PhysicsSet::SyncTransforms,
            )
                .chain(),
        );

        // Flush spawn commands immediately so JoltBody is queryable in the same tick
        app.add_systems(
            FixedUpdate,
            ApplyDeferred
                .after(crate::systems::PhysicsSet::Spawn)
                .before(crate::systems::PhysicsSet::ApplyVelocities),
        );

        app.add_systems(
            FixedUpdate,
            (
                spawn_physics_bodies.in_set(crate::systems::PhysicsSet::Spawn),
                apply_velocities.in_set(crate::systems::PhysicsSet::ApplyVelocities),
                crate::systems::apply_user_transforms
                    .in_set(crate::systems::PhysicsSet::ApplyVelocities)
                    .after(apply_velocities),
                apply_gravity.in_set(crate::systems::PhysicsSet::ApplyGravity),
                step_physics.in_set(crate::systems::PhysicsSet::Step),
                sync_transforms.in_set(crate::systems::PhysicsSet::SyncTransforms),
            ),
        );

        app.add_observer(crate::systems::cleanup_despawned_physics_bodies);
        app.add_observer(crate::systems::remove_jolt_body_on_rigidbody_removal);
        app.add_observer(crate::systems::remove_jolt_body_on_collider_removal);
    }
}
