use bevy::math::Affine3A;
use bevy::prelude::*;

use crate::components::{AngularVelocity, JoltBody, LinearVelocity, StaticMarker};
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
            Option<&crate::components::ContinuousCollision>,
        ),
        (Or<(Added<RigidBody>, Added<Collider>)>, Without<JoltBody>),
    >,
    transform_helper: bevy::transform::helper::TransformHelper,
    mut physics_world: ResMut<PhysicsWorld>,
) {
    for (entity, rigidbody, collider, linear_velocity, angular_velocity, ccd) in query.iter() {
        let global = transform_helper
            .compute_global_transform(entity)
            .unwrap_or(GlobalTransform::IDENTITY);
        let (scale, rotation, position) = global.to_scale_rotation_translation();

        let lin_vel = linear_velocity.map(|v| **v).unwrap_or(Vec3::ZERO);
        let ang_vel = angular_velocity.map(|v| **v).unwrap_or(Vec3::ZERO);

        let body_id = physics_world.spawn_body(
            entity,
            collider,
            scale,
            (position, rotation),
            rigidbody,
            lin_vel,
            ang_vel,
            ccd.is_some(),
        );

        if let Some(id) = body_id {
            let mut entity_cmds = commands.entity(entity);
            entity_cmds.insert(JoltBody(id));
            if *rigidbody == RigidBody::Static {
                entity_cmds.insert(crate::components::StaticMarker);
            }
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
        (With<RigidBody>, Without<StaticMarker>),
    >,
    parents: Query<&GlobalTransform>,
    physics_world: Res<PhysicsWorld>,
) {
    for (mut transform, child_of, body, lin_vel, ang_vel) in query.iter_mut() {
        if let Some((world_pos, world_rot)) = physics_world.get_transform(body.0) {
            const POS_EPSILON_SQ: f32 = 1e-6;
            const ROT_EPSILON: f32 = 1e-5;

            if let Some(parent_global) = child_of.and_then(|c| parents.get(c.parent()).ok()) {
                let world_affine = Affine3A::from_rotation_translation(world_rot, world_pos);
                let local_affine = parent_global.affine().inverse() * world_affine;
                let (_, local_rotation, local_translation) =
                    local_affine.to_scale_rotation_translation();

                if transform.translation.distance_squared(local_translation) > POS_EPSILON_SQ
                    || (1.0 - transform.rotation.dot(local_rotation).abs()) > ROT_EPSILON
                {
                    transform.translation = local_translation;
                    transform.rotation = local_rotation;
                }
            } else {
                if transform.translation.distance_squared(world_pos) > POS_EPSILON_SQ
                    || (1.0 - transform.rotation.dot(world_rot).abs()) > ROT_EPSILON
                {
                    transform.translation = world_pos;
                    transform.rotation = world_rot;
                }
            }

            if let Some(mut velocity) = lin_vel {
                if let Some(jolt_vel) = physics_world.get_linear_velocity(body.0) {
                    if velocity.0 != jolt_vel {
                        velocity.bypass_change_detection().0 = jolt_vel;
                    }
                }
            }

            if let Some(mut velocity) = ang_vel {
                if let Some(jolt_vel) = physics_world.get_angular_velocity(body.0) {
                    if velocity.0 != jolt_vel {
                        velocity.bypass_change_detection().0 = jolt_vel;
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

/// Scans the ECS for any physics bodies whose [`Transform`] was manually modified
/// by the user this frame, and teleports the corresponding Jolt body.
#[allow(clippy::type_complexity)]
pub fn apply_user_transforms(
    mut physics_world: ResMut<PhysicsWorld>,
    query: Query<(Entity, &JoltBody, &Transform, Option<&GlobalTransform>), Changed<Transform>>,
    transform_helper: bevy::transform::helper::TransformHelper,
) {
    for (entity, body, transform, global_transform) in query.iter() {
        let (pos, rot) = if let Some(global) = global_transform {
            // Compute accurate world transform for child entities immediately
            let global = transform_helper
                .compute_global_transform(entity)
                .unwrap_or(*global);
            let (_, rot, pos) = global.to_scale_rotation_translation();
            (pos, rot)
        } else {
            (transform.translation, transform.rotation)
        };

        if let Some((world_pos, world_rot)) = physics_world.get_transform(body.0) {
            // If the ECS transform diverges from Jolt's authoritative state by a margin,
            // the user must have modified it manually.
            if world_pos.distance_squared(pos) > 1e-4 || (1.0 - world_rot.dot(rot).abs()) > 1e-4 {
                physics_world.set_position_and_rotation(body.0, pos, rot);
            }
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
    fn test_apply_user_transforms_teleports_body() {
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(BoltPlugin::default());
        // Insert time update strategy so FixedUpdate runs reliably in tests
        app.world_mut()
            .insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(
                std::time::Duration::from_secs_f32(1.0 / 60.0),
            ));
        // In case step 2.5 is not done yet, add it manually
        app.add_systems(FixedUpdate, apply_user_transforms);

        let entity = app
            .world_mut()
            .spawn((
                Transform::from_xyz(0.0, 10.0, 0.0),
                RigidBody::Dynamic,
                Collider::Box {
                    half_extents: Vec3::splat(1.0),
                },
            ))
            .id();

        // Manually run spawn and flush commands
        use bevy::ecs::system::RunSystemOnce;
        let _ = app.world_mut().run_system_once(spawn_physics_bodies);
        app.world_mut().flush();

        // Mutate the transform
        {
            let mut transform = app.world_mut().get_mut::<Transform>(entity).unwrap();
            transform.translation = Vec3::new(100.0, 200.0, 300.0);
        }

        // Ensure FixedUpdate runs again
        let _ = app.world_mut().run_system_once(apply_user_transforms);
        // Step physics manually so get_transform returns the updated value if needed
        // Actually set_position_and_rotation updates Jolt immediately.

        let physics_world = app.world().get_resource::<PhysicsWorld>().unwrap();
        let jolt_body = app.world().get::<JoltBody>(entity).unwrap();

        let (pos, _) = physics_world.get_transform(jolt_body.0).unwrap();
        assert!(pos.distance(Vec3::new(100.0, 200.0, 300.0)) < 1e-4);
    }

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

/// Draws wireframes representing the authoritative physics state directly from Jolt.
pub fn debug_draw_colliders(
    mut gizmos: Gizmos,
    physics_world: Res<crate::world::PhysicsWorld>,
    query: Query<(&crate::components::JoltBody, &crate::components::Collider, &GlobalTransform)>,
) {
    for (body, collider, global_transform) in query.iter() {
        // Bypass Bevy's transform and ask Jolt exactly where the body is right now
        if let Some((pos, rot)) = physics_world.get_transform(body.0) {
            let scale = global_transform.compute_transform().scale;

            match collider {
                crate::components::Collider::Box { half_extents } => {
                    let scaled_extents = *half_extents * scale;
                    // Draw a bright green wireframe box
                    gizmos.cube(
                        Transform::from_translation(pos).with_rotation(rot).with_scale(scaled_extents * 2.0),
                        Color::srgb(0.0, 1.0, 0.0),
                    );
                }
                #[allow(unreachable_patterns)]
                _ => {}
            }
        }
    }
}
