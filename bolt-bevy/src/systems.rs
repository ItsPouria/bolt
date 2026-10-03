use bevy::math::Affine3A;
use bevy::prelude::*;

use crate::components::{AngularVelocity, JoltBody, LinearVelocity};
use crate::config::PhysicsConfig;
use crate::prelude::{Collider, RigidBody};
use crate::world::PhysicsWorld;

/// System sets for ordering physics execution.
#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub enum PhysicsSet {
    /// Spawns new bodies.
    Spawn,
    /// Applies velocity changes from ECS to Jolt.
    ApplyVelocities,
    /// Applies gravity forces.
    ApplyGravity,
    /// Steps the physics simulation.
    Step,
    /// Synchronizes transforms from Jolt back to ECS.
    SyncTransforms,
}

/// System that queries newly added [`RigidBody`] entities and creates their corresponding
/// Jolt physics bodies in the [`PhysicsWorld`].
///
/// Coordinates are resolved in world space:
/// - If [`GlobalTransform`] is available, its world translation and rotation are used.
/// - Otherwise, the entity's local [`Transform`] is used as a fallback.
///
/// On successful body creation, a [`JoltBody`] component is inserted onto the entity.
#[allow(clippy::type_complexity)]
pub fn spawn_physics_bodies(
    mut commands: Commands,
    query: Query<
        (
            Entity,
            &RigidBody,
            &Collider,
            Option<&LinearVelocity>,
            Option<&AngularVelocity>,
        ),
        (Or<(Added<RigidBody>, Added<Collider>)>, Without<JoltBody>),
    >,
    transform_helper: bevy::transform::helper::TransformHelper,
    mut physics_world: ResMut<PhysicsWorld>,
) {
    for (entity, rigidbody, collider, linear_velocity, angular_velocity) in query.iter() {
        let global = transform_helper
            .compute_global_transform(entity)
            .unwrap_or(GlobalTransform::IDENTITY);
        let (position, rotation) = (global.translation(), global.rotation());

        let lin_vel = linear_velocity.map(|v| **v).unwrap_or(Vec3::ZERO);
        let ang_vel = angular_velocity.map(|v| **v).unwrap_or(Vec3::ZERO);

        let body_id = match collider {
            Collider::Box { half_extents } => physics_world.spawn_box(
                entity,
                *half_extents,
                (position, rotation),
                rigidbody,
                lin_vel,
                ang_vel,
            ),
        };

        if let Some(id) = body_id {
            commands.entity(entity).insert(JoltBody(id));
        } else {
            error!("Failed to spawn physics body for entity {:?}", entity);
            commands
                .entity(entity)
                .remove::<RigidBody>()
                .remove::<Collider>();
        }
    }
}

/// Synchronizes physics state from the Jolt physics engine back into the Bevy ECS.
///
/// Runs during [`FixedUpdate`] after physics simulation stepping.
///
/// ### Transforms & Hierarchy Handling:
/// - **Root Entities**: Jolt world-space coordinates are written directly to [`Transform`].
/// - **Child Entities** (entities with [`ChildOf`]): Jolt world coordinates are transformed
///   into the parent's local coordinate frame using the inverse of the parent's [`GlobalTransform`].
///   This prevents double-transformation when Bevy propagates transforms down the hierarchy.
///
/// ### Velocities:
/// If the entity possesses a [`LinearVelocity`] or [`AngularVelocity`] component, it will be
/// overwritten with the current velocity of the physics body from the simulation step.
#[allow(clippy::type_complexity)]
#[allow(clippy::collapsible_if)]
pub fn sync_transforms(
    mut query: Query<
        (
            &mut Transform,
            Option<&ChildOf>,
            &JoltBody,
            Option<&mut LinearVelocity>,
            Option<&mut AngularVelocity>,
        ),
        With<RigidBody>,
    >,
    parents: Query<&GlobalTransform>,
    physics_world: Res<PhysicsWorld>,
) {
    for (mut transform, child_of, body, lin_vel, ang_vel) in query.iter_mut() {
        if let Some((world_pos, world_rot)) = physics_world.get_transform(body.0) {
            if let Some(parent_global) = child_of.and_then(|c| parents.get(c.parent()).ok()) {
                let world_affine = Affine3A::from_rotation_translation(world_rot, world_pos);
                let local_affine = parent_global.affine().inverse() * world_affine;
                let (_, local_rotation, local_translation) =
                    local_affine.to_scale_rotation_translation();

                transform.translation = local_translation;
                transform.rotation = local_rotation;
            } else {
                transform.translation = world_pos;
                transform.rotation = world_rot;
            }

            if let Some(mut velocity) = lin_vel {
                if let Some(jolt_vel) = physics_world.get_linear_velocity(body.0) {
                    if velocity.0 != jolt_vel {
                        velocity.0 = jolt_vel;
                    }
                }
            }

            if let Some(mut velocity) = ang_vel {
                if let Some(jolt_vel) = physics_world.get_angular_velocity(body.0) {
                    if velocity.0 != jolt_vel {
                        velocity.0 = jolt_vel;
                    }
                }
            }
        }
    }
}

/// Lifecycle observer that removes and destroys Jolt physics bodies when their
/// Bevy entity or [`JoltBody`] component is despawned / removed.
pub fn cleanup_despawned_physics_bodies(
    event: On<Remove, JoltBody>,
    query: Query<&JoltBody>,
    mut physics_world: ResMut<PhysicsWorld>,
) {
    if let Ok(body) = query.get(event.entity) {
        physics_world.destroy_body(body.0);
        debug!(
            "Successfully cleaned up Jolt body for despawned entity {:?}",
            event.entity
        );
    }
}

/// Scans the ECS for any physics bodies whose [`LinearVelocity`] or [`AngularVelocity`]
/// components were modified by the user this frame, and synchronizes those changes
/// down into the internal Jolt physics engine before the next simulation step.
#[allow(clippy::type_complexity)]
pub fn apply_velocities(
    mut physics_world: ResMut<PhysicsWorld>,
    query: Query<
        (&JoltBody, Option<&LinearVelocity>, Option<&AngularVelocity>),
        Or<(Changed<LinearVelocity>, Changed<AngularVelocity>)>,
    >,
) {
    for (body, lin_vel, ang_vel) in query.iter() {
        if let Some(vel) = lin_vel {
            physics_world.set_linear_velocity(body.0, **vel);
        }
        if let Some(vel) = ang_vel {
            physics_world.set_angular_velocity(body.0, **vel);
        }
    }
}

/// Steps the internal physics simulation.
pub fn step_physics(
    mut world: ResMut<PhysicsWorld>,
    config: Res<PhysicsConfig>,
    time: Res<Time<Fixed>>,
) {
    let delta_time = time.delta_secs();
    world.step(delta_time, config.collision_steps as i32);
}

/// Observer that removes the JoltBody when RigidBody is removed.
pub fn remove_jolt_body_on_rigidbody_removal(event: On<Remove, RigidBody>, mut commands: Commands) {
    if let Ok(mut entity) = commands.get_entity(event.entity) {
        entity.remove::<JoltBody>();
    }
}

/// Observer that removes the JoltBody when Collider is removed.
pub fn remove_jolt_body_on_collider_removal(event: On<Remove, Collider>, mut commands: Commands) {
    if let Ok(mut entity) = commands.get_entity(event.entity) {
        entity.remove::<JoltBody>();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plugin::BoltPlugin;

    #[test]
    fn test_spawn_physics_bodies_failure() {
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(BoltPlugin::default());

        let broken_entity = app
            .world_mut()
            .spawn((
                Transform::from_xyz(0.0, 10.0, 0.0),
                RigidBody::Dynamic,
                Collider::Box {
                    half_extents: Vec3::splat(-1.0),
                },
            ))
            .id();

        app.update();

        assert!(app.world().get::<JoltBody>(broken_entity).is_none());
    }
}
