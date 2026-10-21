use bevy::math::Affine3A;
use bevy::prelude::*;

use crate::components::{
    AngularDamping, AngularVelocity, JoltBody, LinearDamping, LinearVelocity, StaticMarker,
};
use crate::config::PhysicsConfig;
use crate::prelude::{Collider, RigidBody};
use crate::world::{BodySpawnRequest, PhysicsWorld};

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

/// System that queries newly added or modified [`RigidBody`] / [`Collider`] entities and creates
/// (or recreates) their corresponding Jolt physics bodies in the [`PhysicsWorld`].
///
/// Coordinates are resolved in world space:
/// - If [`GlobalTransform`] is available, its world translation and rotation are used.
/// - Otherwise, the entity's local [`Transform`] is used as a fallback.
///
/// On successful body creation, a [`JoltBody`] component is inserted onto the entity.
///
/// Respawn semantics (an existing body is being replaced):
/// - The previous body is destroyed only after its replacement exists, so a
///   rejected mutation never tears down a working body.
/// - The replacement inherits the previous body's Jolt-side velocities unless
///   the entity carries explicit [`LinearVelocity`]/[`AngularVelocity`]
///   components.
/// - If a respawn fails, the previous body and components are kept (a warning
///   is logged) so the entity keeps simulating; fixing the component retries.
///   A failed *fresh* spawn removes the physics components instead.
#[allow(clippy::type_complexity)]
pub fn spawn_physics_bodies(
    mut commands: Commands,
    query: Query<
        (
            Entity,
            &RigidBody,
            &Collider,
            Option<&JoltBody>,
            Option<&LinearVelocity>,
            Option<&AngularVelocity>,
            Option<&LinearDamping>,
            Option<&AngularDamping>,
            Option<&crate::components::ContinuousCollision>,
        ),
        Or<(Changed<RigidBody>, Changed<Collider>)>,
    >,
    transform_helper: bevy::transform::helper::TransformHelper,
    mut physics_world: ResMut<PhysicsWorld>,
) {
    for (
        entity,
        rigidbody,
        collider,
        existing_body,
        linear_velocity,
        angular_velocity,
        linear_damping,
        angular_damping,
        ccd,
    ) in query.iter()
    {
        // Capture the Jolt-side velocity before any teardown so a respawn
        // stays velocity-continuous even when the entity carries no
        // `LinearVelocity`/`AngularVelocity` components.
        let (fallback_lin_vel, fallback_ang_vel) = existing_body
            .and_then(|old| physics_world.get_linear_and_angular_velocity(old.0))
            .unwrap_or((Vec3::ZERO, Vec3::ZERO));

        let global = transform_helper
            .compute_global_transform(entity)
            .unwrap_or(GlobalTransform::IDENTITY);
        let (scale, rotation, position) = global.to_scale_rotation_translation();

        let lin_vel = linear_velocity.map(|v| **v).unwrap_or(fallback_lin_vel);
        let ang_vel = angular_velocity.map(|v| **v).unwrap_or(fallback_ang_vel);
        // `#[require]` guarantees presence on spawned bodies; the fallback
        // mirrors the components' Jolt-faithful default of 0.05.
        let lin_damp = linear_damping.map(|d| **d).unwrap_or(0.05);
        let ang_damp = angular_damping.map(|d| **d).unwrap_or(0.05);

        let body_id = physics_world.spawn_body(BodySpawnRequest {
            entity,
            collider,
            scale,
            position,
            rotation,
            rigidbody,
            linear_velocity: lin_vel,
            angular_velocity: ang_vel,
            linear_damping: lin_damp,
            angular_damping: ang_damp,
            use_ccd: ccd.is_some(),
        });

        match body_id {
            Some(id) => {
                // Destroy the previous body only after its replacement exists:
                // a failed respawn must never tear down a working body.
                if let Some(old_body) = existing_body {
                    physics_world.destroy_body(old_body.0);
                }
                let mut entity_cmds = commands.entity(entity);
                entity_cmds.insert(JoltBody(id));
                if *rigidbody == RigidBody::Static {
                    entity_cmds.insert(StaticMarker);
                } else {
                    entity_cmds.remove::<StaticMarker>();
                }
            }
            None if existing_body.is_some() => {
                // A mutation was rejected (e.g. an invalid collider). Keep the
                // last-known-good body and components so the entity keeps
                // simulating; fixing the component later triggers another
                // `Changed<...>`-driven respawn attempt.
                warn!(
                    "Failed to respawn physics body for entity {:?}; keeping the previous body",
                    entity
                );
            }
            None => {
                warn!(
                    "Failed to spawn physics body for entity {:?}; removing RigidBody and Collider",
                    entity
                );
                commands
                    .entity(entity)
                    .remove::<(RigidBody, Collider, JoltBody, StaticMarker)>();
            }
        }
    }
}

/// Tracks the system change tick of the most recent [`sync_transforms`] execution
/// so [`apply_user_transforms`] can skip [`Changed<Transform>`] notifications
/// that were caused by physics synchronization rather than user modifications.
///
/// The exact-tick comparison in [`apply_user_transforms`] is sound because
/// Bevy allocates a distinct change tick to every system run: a write made by
/// any system other than `sync_transforms` necessarily lands on a different
/// tick than the one recorded here.
/// The type is public only because the systems taking it as a parameter are;
/// the field is crate-private so callers cannot fabricate or mutate the tick.
#[derive(Resource, Default, Debug, Clone, Copy)]
pub struct LastPhysicsSyncTick(pub(crate) Option<bevy::ecs::change_detection::Tick>);

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
    last_sync_tick: Option<ResMut<LastPhysicsSyncTick>>,
    system_change_tick: bevy::ecs::system::SystemChangeTick,
) {
    if let Some(mut sync_tick) = last_sync_tick {
        sync_tick.0 = Some(system_change_tick.this_run());
    }

    for (mut transform, child_of, body, lin_vel, ang_vel) in query.iter_mut() {
        if !physics_world.is_active(body.0) {
            if let Some(mut velocity) = lin_vel {
                if velocity.0 != Vec3::ZERO {
                    velocity.bypass_change_detection().0 = Vec3::ZERO;
                }
            }
            if let Some(mut velocity) = ang_vel {
                if velocity.0 != Vec3::ZERO {
                    velocity.bypass_change_detection().0 = Vec3::ZERO;
                }
            }
            continue;
        }

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

            if lin_vel.is_some() || ang_vel.is_some() {
                if let Some((jolt_lin_vel, jolt_ang_vel)) =
                    physics_world.get_linear_and_angular_velocity(body.0)
                {
                    if let Some(mut velocity) = lin_vel {
                        if velocity.0 != jolt_lin_vel {
                            velocity.bypass_change_detection().0 = jolt_lin_vel;
                        }
                    }
                    if let Some(mut velocity) = ang_vel {
                        if velocity.0 != jolt_ang_vel {
                            velocity.bypass_change_detection().0 = jolt_ang_vel;
                        }
                    }
                }
            }
        }
    }
}

/// Lifecycle observer that removes and destroys Jolt physics bodies when their
/// Bevy entity or [`JoltBody`] component is despawned / removed.
pub fn cleanup_despawned_physics_bodies(
    event: On<Remove<JoltBody>>,
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
        (
            Or<(Changed<LinearVelocity>, Changed<AngularVelocity>)>,
            Without<StaticMarker>,
        ),
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
    query: Query<(Entity, &JoltBody, Ref<Transform>, Option<&ChildOf>), Changed<Transform>>,
    transform_helper: bevy::transform::helper::TransformHelper,
    last_sync_tick: Option<Res<LastPhysicsSyncTick>>,
) {
    let sync_tick = last_sync_tick.and_then(|t| t.0);
    for (entity, body, transform, child_of) in query.iter() {
        // Ignore Transform changes that were written by `sync_transforms`
        if sync_tick == Some(transform.last_changed()) {
            continue;
        }

        let (pos, rot) = if child_of.is_some() {
            // Compute accurate world transform for child entities immediately
            if let Ok(global) = transform_helper.compute_global_transform(entity) {
                let (_, rot, pos) = global.to_scale_rotation_translation();
                (pos, rot)
            } else {
                (transform.translation, transform.rotation)
            }
        } else {
            (transform.translation, transform.rotation)
        };

        if let Some((world_pos, world_rot)) = physics_world.get_transform(body.0)
            && (world_pos.distance_squared(pos) > 1e-6 || (1.0 - world_rot.dot(rot).abs()) > 1e-5)
        {
            physics_world.set_position_and_rotation(body.0, pos, rot);
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
pub fn remove_jolt_body_on_rigidbody_removal(event: On<Remove<RigidBody>>, mut commands: Commands) {
    if let Ok(mut entity) = commands.get_entity(event.entity) {
        entity.try_remove::<(JoltBody, StaticMarker)>();
    }
}

/// Observer that removes the JoltBody when Collider is removed.
pub fn remove_jolt_body_on_collider_removal(event: On<Remove<Collider>>, mut commands: Commands) {
    if let Ok(mut entity) = commands.get_entity(event.entity) {
        entity.try_remove::<(JoltBody, StaticMarker)>();
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
    fn test_apply_user_transforms_small_nudge_after_sync() {
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(BoltPlugin::default());

        let entity = app
            .world_mut()
            .spawn((
                Transform::from_xyz(0.0, 10.0, 0.0),
                RigidBody::Kinematic,
                Collider::Box {
                    half_extents: Vec3::splat(1.0),
                },
            ))
            .id();

        use bevy::ecs::system::RunSystemOnce;
        let _ = app.world_mut().run_system_once(spawn_physics_bodies);
        app.world_mut().flush();
        let _ = app.world_mut().run_system_once(sync_transforms);

        // Nudge by 5 mm (< 1 cm, which the old 1e-4 squared distance threshold ignored)
        {
            let mut transform = app.world_mut().get_mut::<Transform>(entity).unwrap();
            transform.translation.x += 0.005;
        }

        let _ = app.world_mut().run_system_once(apply_user_transforms);

        let physics_world = app.world().get_resource::<PhysicsWorld>().unwrap();
        let jolt_body = app.world().get::<JoltBody>(entity).unwrap();
        let (pos, _) = physics_world.get_transform(jolt_body.0).unwrap();
        assert!((pos.x - 0.005).abs() < 1e-5);
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
