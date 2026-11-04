use bevy::prelude::*;

use crate::components::{
    AngularDamping, AngularVelocity, Collider, ContinuousCollision, ExternalForce, ExternalImpulse,
    ExternalTorque, LinearDamping, LinearVelocity, RigidBody, StaticMarker, TransformDriven,
};
use crate::config::PhysicsConfig;
use crate::gravity::{Gravity, apply_gravity};
use crate::systems::{
    LastPhysicsSyncTick, PhysicsSet, apply_external_forces, apply_external_impulses,
    apply_user_transforms, apply_velocities, cleanup_despawned_physics_bodies,
    move_transform_driven_kinematics, remove_jolt_body_on_collider_removal,
    remove_jolt_body_on_rigidbody_removal, spawn_physics_bodies, step_physics, sync_transforms,
};
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
        // Initialize `Gravity` first so `PhysicsWorld`'s `FromWorld` impl
        // sees a gravity resource at construction time (the plugin default,
        // or a user-inserted one); later changes flow through `apply_gravity`.
        app.init_resource::<Gravity>();
        app.init_resource::<PhysicsWorld>();
        app.init_resource::<LastPhysicsSyncTick>();

        // Register all reflected physics types
        app.register_type::<RigidBody>()
            .register_type::<Collider>()
            .register_type::<LinearVelocity>()
            .register_type::<AngularVelocity>()
            .register_type::<LinearDamping>()
            .register_type::<AngularDamping>()
            .register_type::<ExternalForce>()
            .register_type::<ExternalTorque>()
            .register_type::<ExternalImpulse>()
            .register_type::<StaticMarker>()
            .register_type::<ContinuousCollision>()
            .register_type::<TransformDriven>()
            .register_type::<PhysicsConfig>()
            .register_type::<Gravity>();

        app.configure_sets(
            FixedUpdate,
            (
                PhysicsSet::Spawn,
                PhysicsSet::ApplyVelocities,
                PhysicsSet::ApplyGravity,
                PhysicsSet::ApplyForces,
                PhysicsSet::Step,
                PhysicsSet::SyncTransforms,
            )
                .chain(),
        );

        // Flush spawn commands immediately so JoltBody is queryable in the same tick
        app.add_systems(
            FixedUpdate,
            ApplyDeferred
                .after(PhysicsSet::Spawn)
                .before(PhysicsSet::ApplyVelocities),
        );

        app.add_systems(
            FixedUpdate,
            (
                spawn_physics_bodies.in_set(PhysicsSet::Spawn),
                apply_velocities.in_set(PhysicsSet::ApplyVelocities),
                apply_user_transforms
                    .in_set(PhysicsSet::ApplyVelocities)
                    .after(apply_velocities),
                move_transform_driven_kinematics
                    .in_set(PhysicsSet::ApplyVelocities)
                    .after(apply_user_transforms),
                apply_gravity.in_set(PhysicsSet::ApplyGravity),
                apply_external_forces.in_set(PhysicsSet::ApplyForces),
                // Ordered explicitly: the two systems share ResMut<PhysicsWorld>
                // (commutative in effect — one accumulates, the other kicks —
                // but Bevy would otherwise flag the pair as ambiguously
                // ordered, invisibly under MinimalPlugins and fatally for
                // apps that escalate ambiguity_detection to Error).
                apply_external_impulses
                    .in_set(PhysicsSet::ApplyForces)
                    .after(apply_external_forces),
                step_physics.in_set(PhysicsSet::Step),
                sync_transforms.in_set(PhysicsSet::SyncTransforms),
            ),
        );

        app.add_observer(cleanup_despawned_physics_bodies);
        app.add_observer(remove_jolt_body_on_rigidbody_removal);
        app.add_observer(remove_jolt_body_on_collider_removal);
    }
}
