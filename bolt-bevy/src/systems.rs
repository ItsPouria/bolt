use bevy::math::Affine3A;
use bevy::prelude::*;

use crate::components::{
    AngularDamping, AngularVelocity, JoltBody, LinearDamping, LinearVelocity, StaticMarker,
    TransformDriven,
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
    transform_helper: TransformHelper,
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
                // A respawn is a fresh body: a marker from the previous one
                // must not survive (e.g. a kinematic switched to dynamic
                // would keep a stale TransformDriven the move refuses anyway).
                entity_cmds.remove::<TransformDriven>();
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
                commands.entity(entity).remove::<(
                    RigidBody,
                    Collider,
                    JoltBody,
                    StaticMarker,
                    TransformDriven,
                )>();
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
/// - **Child Entities** (entities with [`ChildOf`]): Jolt world coordinates are
///   transformed into the parent's local coordinate frame using the inverse of the
///   parent's world frame, computed fresh inside the tick by the private
///   `compute_world_affine` helper: a parent that is a physics body contributes its
///   authoritative Jolt pose (already world-space), while a plain non-physics
///   parent is composed up its ancestor chain, stopping at the first physics
///   body. The [`GlobalTransform`] component is deliberately never read here —
///   propagation refreshes it only in [`PostUpdate`], so during [`FixedUpdate`]
///   it is one frame behind a moving parent, and dividing out a stale frame
///   drags children along with the parent (fed back into Jolt as teleports).
///   The plain-ancestor query is filtered `Without<RigidBody>`, keeping its
///   reads disjoint from this system's `&mut Transform` — fresh data with no
///   borrow conflict.
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
            &RigidBody,
            Option<&mut LinearVelocity>,
            Option<&mut AngularVelocity>,
            Option<&TransformDriven>,
        ),
        Without<StaticMarker>,
    >,
    // Parents that are physics bodies: their world pose comes from Jolt
    // directly — fresher than any ECS read, and it touches no `Transform`,
    // so it cannot conflict with the `&mut Transform` above.
    parent_bodies: Query<&JoltBody, With<RigidBody>>,
    // Physics bodies' ECS scale (Jolt never sees it; spawn bakes it into
    // colliders). GlobalTransform is untouched by every other param, so
    // this read cannot conflict — it exists to keep scaled parents' frames
    // invertible, matching what propagation will compose with later.
    body_globals: Query<&GlobalTransform, With<RigidBody>>,
    // Non-physics ancestors only. `Without<RigidBody>` keeps this query's
    // `&Transform` reads disjoint from the `&mut Transform` above by
    // construction — a fresh parent frame without a B0001 borrow conflict
    // and without a `ParamSet`.
    plain_hierarchy: Query<(Option<&ChildOf>, &Transform), Without<RigidBody>>,
    physics_world: Res<PhysicsWorld>,
    last_sync_tick: Option<ResMut<LastPhysicsSyncTick>>,
    system_change_tick: bevy::ecs::system::SystemChangeTick,
) {
    if let Some(mut sync_tick) = last_sync_tick {
        sync_tick.0 = Some(system_change_tick.this_run());
    }

    for (mut transform, child_of, body, rigidbody, mut lin_vel, mut ang_vel, transform_driven) in
        query.iter_mut()
    {
        let active = physics_world.is_active(body.0);

        if !active {
            // A sleeping (or otherwise inactive) body has no Jolt-side
            // velocity to mirror: zero the components without tripping
            // change detection, as user code never wrote them.
            if let Some(velocity) = lin_vel.as_mut() {
                if velocity.0 != Vec3::ZERO {
                    velocity.bypass_change_detection().0 = Vec3::ZERO;
                }
            }
            if let Some(velocity) = ang_vel.as_mut() {
                if velocity.0 != Vec3::ZERO {
                    velocity.bypass_change_detection().0 = Vec3::ZERO;
                }
            }
        }

        // Transform sync runs for sleeping bodies too: a sleeping body's
        // Jolt pose is fixed, but its parent's frame may be moving, and the
        // local re-expression must continue or the ECS mirror drifts along
        // with the parent (#4). Stale handles return None and skip all of
        // this; a sleeping root body compares equal and writes nothing.
        //
        // EXCEPT Transform-driven kinematics: their local Transform is
        // user-authoritative (the drive's target is parent frame × local),
        // and mirroring the Jolt pose back into it would launder the
        // parent's motion out of the target — the drive would chase its
        // own tail and hold the status-quo world pose instead of tracking
        // the parent frame. Velocities still mirror; the drive lands the
        // body at parent × local within a sub-step (the target is computed
        // from the parent's pre-step pose, so a fast parent shows one
        // sub-step of lag — v·dt), and propagation renders from the same
        // frame, so the ECS view is exact.
        // Kinematic-gated: a hand-inserted marker on a dynamic body must
        // not freeze its ECS mirror (the drive refuses non-kinematics, so
        // nothing would ever write the local) — the invariant "marker
        // present ⇒ the drive is actually driving" holds only for
        // kinematics, so only they get the mirror skip.
        let transform_driven_kinematic =
            transform_driven.is_some() && *rigidbody == RigidBody::Kinematic;
        if !transform_driven_kinematic
            && let Some((world_pos, world_rot)) = physics_world.get_transform(body.0)
        {
            const POS_EPSILON_SQ: f32 = 1e-6;
            const ROT_EPSILON: f32 = 1e-5;

            let parent_affine = child_of.and_then(|c| {
                compute_world_affine(
                    c.parent(),
                    &parent_bodies,
                    &body_globals,
                    &plain_hierarchy,
                    &physics_world,
                )
            });
            if let Some(parent_affine) = parent_affine {
                let world_affine = Affine3A::from_rotation_translation(world_rot, world_pos);
                let local_affine = parent_affine.inverse() * world_affine;
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
        }

        // Velocity mirroring runs for marked kinematics too (it cannot
        // launder intent — it writes the velocity components, not the
        // Transform): the stop test reads the drive's decay through it.
        if active && (lin_vel.is_some() || ang_vel.is_some()) {
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

/// Computes an entity's world-space [`Affine3A`] for parent-frame conversion
/// in [`sync_transforms`], without borrow conflicts: an entity carrying a
/// [`JoltBody`] contributes its authoritative Jolt pose — already
/// world-space, so the walk stops there — while a plain (non-physics) entity
/// composes its local [`Transform`] up through ancestors until it reaches
/// the root or the first physics body.
///
/// The `plain_hierarchy` query is filtered `Without<RigidBody>`, so its
/// `&Transform` reads are disjoint from `sync_transforms`' `&mut Transform`
/// by construction — the disjointness that makes a fresh parent frame
/// possible without a `ParamSet` (a `TransformHelper` here would panic with
/// B0001: it reads every `Transform` in the world).
fn compute_world_affine(
    entity: Entity,
    parent_bodies: &Query<&JoltBody, With<RigidBody>>,
    body_globals: &Query<&GlobalTransform, With<RigidBody>>,
    plain_hierarchy: &Query<(Option<&ChildOf>, &Transform), Without<RigidBody>>,
    physics_world: &PhysicsWorld,
) -> Option<Affine3A> {
    // Physics body: its Jolt pose *is* its world transform. Jolt never
    // sees ECS scale (spawn bakes it into the collider), so the ancestor's
    // GlobalTransform contributes the scale alone — reading that component
    // is conflict-free (no other param touches GlobalTransform) and its
    // one-tick staleness is at worst a transient for animating scale.
    if let Ok(body) = parent_bodies.get(entity) {
        let (pos, rot) = physics_world.get_transform(body.0)?;
        let scale = body_globals
            .get(entity)
            .map(|global| global.scale())
            .unwrap_or(Vec3::ONE);
        return Some(Affine3A::from_scale_rotation_translation(scale, rot, pos));
    }

    // Plain entity: compose its local transform up the ancestor chain.
    let (child_of, transform) = plain_hierarchy.get(entity).ok()?;
    let mut affine = transform.compute_affine();
    let mut ancestor = child_of.map(|child_of| child_of.parent());

    while let Some(entity) = ancestor {
        if let Ok(body) = parent_bodies.get(entity) {
            // A physics ancestor's Jolt pose is world-space: compose it in
            // front (with its ECS scale, as above) and stop — nothing above
            // it can change the result.
            let (pos, rot) = physics_world.get_transform(body.0)?;
            let scale = body_globals
                .get(entity)
                .map(|global| global.scale())
                .unwrap_or(Vec3::ONE);
            affine = Affine3A::from_scale_rotation_translation(scale, rot, pos) * affine;
            break;
        }

        let (ancestor_child_of, ancestor_transform) = plain_hierarchy.get(entity).ok()?;
        affine = ancestor_transform.compute_affine() * affine;
        ancestor = ancestor_child_of.map(|child_of| child_of.parent());
    }

    Some(affine)
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
    mut commands: Commands,
    mut physics_world: ResMut<PhysicsWorld>,
    query: Query<
        (
            Entity,
            &JoltBody,
            Option<&LinearVelocity>,
            Option<&AngularVelocity>,
            Option<&TransformDriven>,
        ),
        (
            Or<(Changed<LinearVelocity>, Changed<AngularVelocity>)>,
            Without<StaticMarker>,
        ),
    >,
) {
    for (entity, body, lin_vel, ang_vel, transform_driven) in query.iter() {
        if transform_driven.is_some() {
            // The user wrote a velocity by hand: the velocity path takes over,
            // and the per-tick MoveKinematic drive must stop — otherwise next
            // tick's move (target == current) would cancel this velocity.
            commands.entity(entity).remove::<TransformDriven>();
        }
        if let Some(vel) = lin_vel {
            physics_world.set_linear_velocity(body.0, **vel);
        }
        if let Some(vel) = ang_vel {
            physics_world.set_angular_velocity(body.0, **vel);
        }
    }
}

/// Scans the ECS for any physics bodies whose [`Transform`] was manually
/// modified by the user this frame. Dynamic and static bodies are teleported
/// to the new pose. Kinematic bodies instead claim [`TransformDriven`]
/// ownership: the per-tick MoveKinematic path drives them toward the
/// ECS-derived pose with real velocity, so riders get carried. The edit
/// lands on the SAME tick it is made: Bevy's auto-inserted sync point on
/// this system's explicit ordering edge flushes the marker before
/// `move_transform_driven_kinematics` runs, and that tick's `step` lands
/// the pose (any latency a user feels is FixedUpdate scheduling, not the
/// marker chain).
#[allow(clippy::type_complexity)]
pub fn apply_user_transforms(
    mut commands: Commands,
    mut physics_world: ResMut<PhysicsWorld>,
    query: Query<
        (
            Entity,
            &RigidBody,
            &JoltBody,
            Ref<JoltBody>,
            Ref<Transform>,
            Option<&ChildOf>,
        ),
        Changed<Transform>,
    >,
    transform_helper: TransformHelper,
    last_sync_tick: Option<Res<LastPhysicsSyncTick>>,
) {
    let sync_tick = last_sync_tick.and_then(|t| t.0);
    for (entity, rigidbody, body, jolt_body_ref, transform, child_of) in query.iter() {
        // Ignore Transform changes that were written by `sync_transforms`
        if sync_tick == Some(transform.last_changed()) {
            continue;
        }

        // A Transform write that predates (or shares the tick with) the
        // JoltBody's creation is spawn placement, not user motion — the
        // body was just created at that pose. Claiming ownership for it
        // would mark every kinematic at birth, and the per-tick move
        // (target == current) would cancel velocity-path motion every tick.
        // Tick precedence (not `is_added`, whose "since this system last
        // ran" semantics treat everything as new on a system's first run)
        // works both in the pipeline and under run_system_once tests.
        //
        // Two known edges, both accepted: the raw u32 compare is not
        // wraparound-aware, and the u32 straddle across a change-tick wrap
        // can misfire in BOTH directions — an ancient spawn read as newer
        // than a fresh edit (skipped: a lost claim), or a placement write
        // at u32::MAX−k read as older than a post-wrap flush (a false
        // claim at birth). The vulnerable window per wrap equals the
        // spawn-to-edit gap (any pair straddling the wrap misfires, however
        // far apart); long-running servers that spawn kinematics
        // continuously will eventually hit one. A wrap-aware compare
        // (half-range wrapping_sub, like Tick::is_newer) would close it if
        // this ever matters in practice. And a Commands-queued Transform
        // write landing on the JoltBody's own flush tick is treated as
        // placement forever — "placement wins" is the safe reading of an
        // ambiguous case.
        if transform.last_changed().get() <= jolt_body_ref.last_changed().get() {
            continue;
        }

        if *rigidbody == RigidBody::Kinematic {
            // Don't teleport: hand ownership to the per-tick MoveKinematic
            // path. From here on the entity's local Transform is
            // user-authoritative (sync stops mirroring into it), so the
            // drive reads the user's intent directly. The auto sync point
            // on this system's ordering edge flushes the marker before the
            // move system runs, so the edit lands on this same tick's step.
            // Tiebreak: a velocity write and a Transform edit on the same
            // tick means this claim runs AFTER apply_velocities' strip —
            // the Transform edit wins, ownership following the most recent
            // authoring intent.
            commands.entity(entity).insert(TransformDriven);
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

/// Drives [`TransformDriven`] kinematic bodies toward their ECS-derived
/// world pose every fixed tick via [`PhysicsWorld::move_kinematic`], giving
/// them real velocity (displacement / delta_time) so friction can carry
/// riders and contacts respond to genuine motion.
///
/// Issued unconditionally every tick — not on `Changed<Transform>` —
/// because the velocity `move_kinematic` sets **persists until re-issued**:
/// once the user stops editing, target ≡ current pose and the computed
/// velocity decays to zero, halting the body gracefully. Issue-on-change
/// would leave platforms drifting forever after their script stops (pinned
/// by `test_move_kinematic_velocity_persists_until_reissued` in bolt-core).
///
/// The target pose is ECS-derived (the user's authority): root bodies take
/// their own [`Transform`]; parented bodies compose the parent's world
/// frame — computed on the *parent*, never the entity, whose [`JoltBody`]
/// would make `compute_world_affine` return its Jolt pose and silently
/// discard the user's edit. Scale is dropped in the decomposition:
/// `move_kinematic` takes only position and rotation.
#[allow(clippy::type_complexity)]
pub fn move_transform_driven_kinematics(
    mut physics_world: ResMut<PhysicsWorld>,
    // Reads the entity's local Transform, which for marked bodies is pure
    // user intent (sync does not mirror into it — that mirroring is what
    // laundered the parent frame out of the target for parented bodies).
    query: Query<(&JoltBody, &RigidBody, &Transform, Option<&ChildOf>), With<TransformDriven>>,
    parent_bodies: Query<&JoltBody, With<RigidBody>>,
    body_globals: Query<&GlobalTransform, With<RigidBody>>,
    plain_hierarchy: Query<(Option<&ChildOf>, &Transform), Without<RigidBody>>,
    time: Res<Time<Fixed>>,
) {
    let delta_time = time.delta_secs();
    for (body, rigidbody, transform, child_of) in query.iter() {
        // ECS-side kinematic gate: a hand-inserted marker on a dynamic or
        // static body is inert (the drive refuses non-kinematics in
        // bolt-core), so skip it here rather than paying a futile
        // GetMotionType FFI refusal every tick.
        if *rigidbody != RigidBody::Kinematic {
            continue;
        }

        let target_affine = match child_of {
            Some(child_of) => compute_world_affine(
                child_of.parent(),
                &parent_bodies,
                &body_globals,
                &plain_hierarchy,
                &physics_world,
            )
            .map(|parent_affine| parent_affine * transform.compute_affine()),
            None => Some(transform.compute_affine()),
        };

        let Some(target_affine) = target_affine else {
            continue;
        };
        // Scale is deliberately dropped: MoveKinematic takes only the pose.
        let (_, rotation, position) = target_affine.to_scale_rotation_translation();

        // Refusal semantics, checked on the FINISHED target: NaN can enter
        // through any ingredient — the entity's local, a plain parent's
        // transform, a NaN scale via the parent frame — and all routes
        // converge here, after the composition. Sending a non-finite
        // target would be refused by move_kinematic's pre-FFI guard,
        // leaving the body holding its LAST drive velocity forever
        // (bolt-core's persistence contract) — a runaway, potentially
        // triggered by a sibling script's bad parent transform rather
        // than this body's own code. Halt instead. The warn fires only
        // when there is actually a runaway to stop (velocity non-zero),
        // so a persistently-poisoned script logs once, not per tick.
        if !position.is_finite() || !rotation.is_finite() {
            if physics_world
                .get_linear_velocity(body.0)
                .is_some_and(|v| v != Vec3::ZERO)
            {
                warn!("TransformDriven target is non-finite; halting body to prevent a runaway");
            }
            physics_world.set_linear_velocity(body.0, Vec3::ZERO);
            physics_world.set_angular_velocity(body.0, Vec3::ZERO);
            continue;
        }

        physics_world.move_kinematic(body.0, position, rotation, delta_time);
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
        entity.try_remove::<(JoltBody, StaticMarker, TransformDriven)>();
    }
}

/// Observer that removes the JoltBody when Collider is removed.
pub fn remove_jolt_body_on_collider_removal(event: On<Remove<Collider>>, mut commands: Commands) {
    if let Ok(mut entity) = commands.get_entity(event.entity) {
        entity.try_remove::<(JoltBody, StaticMarker, TransformDriven)>();
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
        // Advance the change tick between spawn and the user edit, as the
        // real schedule does between system runs — otherwise both writes
        // share a tick and the spawn-placement discriminator (correctly)
        // treats the edit as part of spawning.
        let _ = app.world_mut().run_system_once(sync_transforms);

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
        // Dynamic body: the teleport path, whose eat-window invariant this
        // test pins. Kinematic Transform edits no longer teleport — they
        // claim TransformDriven ownership (see the test below).
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(BoltPlugin::default());

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

    #[test]
    fn test_kinematic_transform_edit_claims_ownership_not_teleport() {
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
        // Advance the change tick between spawn and the edit (the real
        // schedule does this between system runs); see the teleport test.
        let _ = app.world_mut().run_system_once(sync_transforms);

        // A genuine user edit: nudge, then run the claiming system.
        {
            let mut transform = app.world_mut().get_mut::<Transform>(entity).unwrap();
            transform.translation.x += 0.005;
        }
        let _ = app.world_mut().run_system_once(apply_user_transforms);
        app.world_mut().flush();

        // New contract, part 1: the marker is claimed…
        assert!(
            app.world()
                .get::<crate::components::TransformDriven>(entity)
                .is_some(),
            "kinematic Transform edit must claim TransformDriven ownership"
        );
        // …and part 2: no teleport — Jolt is unmoved at this instant
        // because only the claiming system has run; the move system and
        // the landing happen later this same tick.
        let pw = app.world().get_resource::<PhysicsWorld>().unwrap();
        let handle = app.world().get::<JoltBody>(entity).unwrap().0;
        let (pos, _) = pw.get_transform(handle).unwrap();
        assert!(
            (pos.x - 0.0).abs() < 1e-6,
            "kinematic edit must not teleport; Jolt still at spawn, got {pos}"
        );
    }

    #[test]
    fn test_velocity_write_reclaims_ownership_from_transform_driven() {
        // The ownership tug-of-war, pinned: a Transform-driven kinematic
        // (marked) whose user then hand-writes a velocity component must
        // lose the marker — otherwise the per-tick move (target == current)
        // cancels that velocity back to zero one tick later.
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
        // Advance the change tick between spawn and the edit, as the real
        // schedule does between system runs.
        let _ = app.world_mut().run_system_once(sync_transforms);

        // Claim ownership: a genuine Transform edit.
        {
            let mut transform = app.world_mut().get_mut::<Transform>(entity).unwrap();
            transform.translation.x += 0.005;
        }
        let _ = app.world_mut().run_system_once(apply_user_transforms);
        app.world_mut().flush();
        assert!(
            app.world()
                .get::<crate::components::TransformDriven>(entity)
                .is_some(),
            "precondition: the Transform edit must claim ownership first"
        );

        // The user hand-writes a velocity: apply_velocities must strip the
        // marker so the velocity path takes over.
        {
            let mut velocity = app.world_mut().get_mut::<LinearVelocity>(entity).unwrap();
            velocity.x = 2.0;
        }
        let _ = app.world_mut().run_system_once(apply_velocities);
        app.world_mut().flush();

        assert!(
            app.world()
                .get::<crate::components::TransformDriven>(entity)
                .is_none(),
            "a user velocity write must reclaim ownership from TransformDriven"
        );
    }

    #[test]
    fn test_respawn_strips_transform_driven() {
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

        // Claim ownership first, so the strip has something to strip.
        {
            let mut transform = app.world_mut().get_mut::<Transform>(entity).unwrap();
            transform.translation.x += 0.005;
        }
        let _ = app.world_mut().run_system_once(apply_user_transforms);
        app.world_mut().flush();
        assert!(
            app.world()
                .get::<crate::components::TransformDriven>(entity)
                .is_some()
        );

        // A genuine collider change triggers a respawn: fresh body, so the
        // marker must not survive it.
        {
            let mut collider = app.world_mut().get_mut::<Collider>(entity).unwrap();
            *collider = Collider::Box {
                half_extents: Vec3::splat(2.0),
            };
        }
        let _ = app.world_mut().run_system_once(spawn_physics_bodies);
        app.world_mut().flush();

        assert!(
            app.world()
                .get::<crate::components::TransformDriven>(entity)
                .is_none(),
            "a respawned body must start with fresh ownership"
        );
        // And the respawn itself worked.
        assert!(app.world().get::<JoltBody>(entity).is_some());
    }

    #[test]
    fn test_per_tick_drive_moves_marked_body_to_target() {
        // End-to-end through the wired pipeline: claim ownership with a
        // Transform edit, let the per-tick system drive, and the body
        // arrives at the target with real velocity.
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(TransformPlugin);
        app.add_plugins(BoltPlugin::default());
        app.world_mut()
            .insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(
                std::time::Duration::from_secs_f32(1.0 / 64.0),
            ));

        let entity = app
            .world_mut()
            .spawn((
                Transform::from_xyz(0.0, 10.0, 0.0),
                RigidBody::Kinematic,
                Collider::Box {
                    half_extents: Vec3::splat(0.5),
                },
            ))
            .id();

        // Warm-up: with a manual clock the FIRST update can run zero fixed
        // ticks, so pump until the body actually exists — otherwise the
        // edit below lands on the spawn tick and is absorbed as placement
        // (a previous version of this test passed vacuously exactly that
        // way: pos arrived via spawn, the marker was never claimed).
        for _ in 0..2 {
            app.update();
        }
        assert!(
            app.world()
                .get::<crate::components::TransformDriven>(entity)
                .is_none(),
            "spawn placement must not claim ownership"
        );

        // A genuine edit: move the target 3 m sideways.
        {
            let mut transform = app.world_mut().get_mut::<Transform>(entity).unwrap();
            transform.translation.x = 3.0;
        }

        // ONE update must be enough: the marker flushes at the auto sync
        // point on apply_user_transforms' ordering edge, the move system
        // runs later in the same tick, and that tick's step lands the
        // pose (velocity = displacement / delta_time, so arrival is exact
        // and immediate). This pins the same-tick landing the docs
        // describe — if it ever regresses to a multi-tick chain, this
        // fails.
        app.update();

        let pw = app.world().get_resource::<PhysicsWorld>().unwrap();
        let handle = app.world().get::<JoltBody>(entity).unwrap().0;
        let (pos, _) = pw.get_transform(handle).unwrap();
        assert!(
            pos.x > 2.9,
            "marked kinematic should have arrived at the target, got x = {}",
            pos.x
        );
        // The claim itself is pinned too: vacuous arrival via spawn
        // placement is no longer possible after the warm-up.
        assert!(
            app.world()
                .get::<crate::components::TransformDriven>(entity)
                .is_some(),
            "the edit must have claimed ownership (vacuous-pass guard)"
        );

        // Decay needs the NEXT tick's re-issue (arrival leaves the body
        // with velocity = displacement / dt), so run one more update
        // before asserting the velocity has returned toward zero.
        app.update();
        let velocity = app.world().get::<LinearVelocity>(entity).unwrap();
        assert!(
            velocity.x.abs() < 0.1,
            "velocity should decay after arrival, got {}",
            velocity.x
        );
    }

    #[test]
    fn test_non_finite_target_halts_instead_of_runaway() {
        // The S2 regression pin: without the halt, a script writing a NaN
        // local leaves the body holding its last drive velocity forever
        // (persistence contract) — a runaway — while its ECS pose froze.
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(TransformPlugin);
        app.add_plugins(BoltPlugin::default());
        app.world_mut()
            .insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(
                std::time::Duration::from_secs_f32(1.0 / 64.0),
            ));

        let entity = app
            .world_mut()
            .spawn((
                Transform::from_xyz(0.0, 10.0, 0.0),
                RigidBody::Kinematic,
                Collider::Box {
                    half_extents: Vec3::splat(0.5),
                },
            ))
            .id();

        // Warm up (first update may run zero fixed ticks), then drive to
        // establish a real velocity.
        for _ in 0..2 {
            app.update();
        }
        {
            let mut transform = app.world_mut().get_mut::<Transform>(entity).unwrap();
            transform.translation.x = 3.0;
        }
        app.update();
        let (handle, driven_pos) = {
            let pw = app.world().get_resource::<PhysicsWorld>().unwrap();
            let handle = app.world().get::<JoltBody>(entity).unwrap().0;
            let (pos, _) = pw.get_transform(handle).unwrap();
            (handle, pos)
        };
        assert!(driven_pos.x > 2.9, "precondition: body was driven");

        // The script bug: a non-finite target. The body must halt, not
        // keep its drive velocity.
        {
            let mut transform = app.world_mut().get_mut::<Transform>(entity).unwrap();
            transform.translation.x = f32::NAN;
        }
        for _ in 0..4 {
            app.update();
        }

        let velocity = app.world().get::<LinearVelocity>(entity).unwrap();
        assert!(
            velocity.length() < 0.1,
            "non-finite target must halt the body, velocity = {:?}",
            velocity.0
        );
        let (pos_after, _) = app
            .world()
            .get_resource::<PhysicsWorld>()
            .unwrap()
            .get_transform(handle)
            .unwrap();
        assert!(
            (pos_after.x - driven_pos.x).abs() < 0.05,
            "halted body must not drift: {pos_after} vs {driven_pos}"
        );
    }

    #[test]
    fn test_non_finite_parent_halts_driven_child_instead_of_runaway() {
        // The parent-route pin: NaN can enter the drive target through the
        // PARENT's transform, not just the entity's own local — the
        // composed target is parent frame x local, and move_kinematic's
        // pre-FFI guard refusing the NaN result leaves the body holding
        // its last drive velocity forever (persistence contract). The
        // child's own code is healthy; it gets poisoned through its
        // parent. Review-probe confirmed the pre-fix runaway: 12 m in
        // four ticks at last drive velocity.
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(TransformPlugin);
        app.add_plugins(BoltPlugin::default());
        app.world_mut()
            .insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(
                std::time::Duration::from_secs_f32(1.0 / 64.0),
            ));

        let parent = app
            .world_mut()
            .spawn(Transform::from_xyz(0.0, 10.0, 0.0))
            .id();
        let child = app
            .world_mut()
            .spawn((
                Transform::from_xyz(2.0, 0.0, 0.0),
                ChildOf(parent),
                RigidBody::Kinematic,
                Collider::Box {
                    half_extents: Vec3::splat(0.5),
                },
            ))
            .id();

        // Warm up, then drive the child sideways to establish velocity.
        for _ in 0..2 {
            app.update();
        }
        {
            let mut transform = app.world_mut().get_mut::<Transform>(child).unwrap();
            transform.translation.x = 5.0;
        }
        app.update();
        let (handle, driven_pos) = {
            let pw = app.world().get_resource::<PhysicsWorld>().unwrap();
            let handle = app.world().get::<JoltBody>(child).unwrap().0;
            let (pos, _) = pw.get_transform(handle).unwrap();
            (handle, pos)
        };
        assert!(
            driven_pos.x > 4.9,
            "precondition: child was driven, got {driven_pos}"
        );

        // The sibling script's bug: poison the PARENT. The child's own
        // local stays finite throughout.
        {
            let mut transform = app.world_mut().get_mut::<Transform>(parent).unwrap();
            transform.translation.x = f32::NAN;
        }
        for _ in 0..4 {
            app.update();
        }

        let velocity = app.world().get::<LinearVelocity>(child).unwrap();
        assert!(
            velocity.length() < 0.1,
            "non-finite parent must halt the driven child, velocity = {:?}",
            velocity.0
        );
        let pos_after = app
            .world()
            .get_resource::<PhysicsWorld>()
            .unwrap()
            .get_transform(handle)
            .unwrap()
            .0;
        assert!(
            (pos_after.x - driven_pos.x).abs() < 0.05,
            "halted child must not drift: {pos_after} vs {driven_pos}"
        );
    }

    #[test]
    fn test_velocity_path_never_gets_marked() {
        // The elevator's authoring path: only velocity writes, never a
        // Transform edit. The marker must stay absent so the per-tick
        // drive (which would cancel the velocity) never engages.
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(TransformPlugin);
        app.add_plugins(BoltPlugin::default());
        app.world_mut()
            .insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(
                std::time::Duration::from_secs_f32(1.0 / 64.0),
            ));

        let entity = app
            .world_mut()
            .spawn((
                Transform::from_xyz(0.0, 10.0, 0.0),
                RigidBody::Kinematic,
                Collider::Box {
                    half_extents: Vec3::splat(0.5),
                },
                LinearVelocity(Vec3::new(0.0, 2.0, 0.0)),
            ))
            .id();

        for _ in 0..8 {
            app.update();
            assert!(
                app.world()
                    .get::<crate::components::TransformDriven>(entity)
                    .is_none(),
                "velocity writes alone must never claim ownership"
            );
        }

        // And the body actually rose under its own velocity the whole time.
        let pw = app.world().get_resource::<PhysicsWorld>().unwrap();
        let handle = app.world().get::<JoltBody>(entity).unwrap().0;
        let (pos, _) = pw.get_transform(handle).unwrap();
        assert!(pos.y > 10.1, "velocity path broken: y = {}", pos.y);
    }

    // --- compute_world_affine: unit tests for every branch of the helper ---

    /// Runs the private helper through a one-shot system so its exact query
    /// parameters can be constructed, and returns the computed affine.
    fn affine_of(app: &mut App, entity: Entity) -> Option<bevy::math::Affine3A> {
        use bevy::ecs::system::RunSystemOnce;
        app.world_mut()
            .run_system_once(
                move |parent_bodies: Query<&JoltBody, With<RigidBody>>,
                      body_globals: Query<&GlobalTransform, With<RigidBody>>,
                      plain_hierarchy: Query<
                    (Option<&ChildOf>, &Transform),
                    Without<RigidBody>,
                >,
                      pw: Res<PhysicsWorld>| {
                    compute_world_affine(
                        entity,
                        &parent_bodies,
                        &body_globals,
                        &plain_hierarchy,
                        &pw,
                    )
                },
            )
            .unwrap()
    }

    #[test]
    fn test_compute_world_affine_physics_body_returns_jolt_pose() {
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(BoltPlugin::default());

        let entity = app
            .world_mut()
            .spawn((
                Transform::from_xyz(5.0, 2.0, -3.0)
                    .with_rotation(Quat::from_rotation_y(std::f32::consts::FRAC_PI_2)),
                RigidBody::Dynamic,
                Collider::Box {
                    half_extents: Vec3::splat(0.5),
                },
            ))
            .id();

        use bevy::ecs::system::RunSystemOnce;
        let _ = app.world_mut().run_system_once(spawn_physics_bodies);
        app.world_mut().flush();

        // Diverge Jolt from the ECS: if the helper returned the entity's
        // ECS Transform instead of querying Jolt, it could never notice.
        // This pins the branch's actual contract — Jolt is the source of
        // truth — rather than the spawn round-trip they both agree on.
        let handle = app.world().get::<JoltBody>(entity).unwrap().0;
        {
            let mut pw = app.world_mut().get_resource_mut::<PhysicsWorld>().unwrap();
            pw.set_position_and_rotation(handle, Vec3::new(9.0, 1.0, 4.0), Quat::IDENTITY);
        }
        let ecs_translation = app.world().get::<Transform>(entity).unwrap().translation;

        let affine = affine_of(&mut app, entity).expect("physics body affine");

        // Translation must match the JOLT pose, not the ECS one.
        assert!(affine.translation.distance(Vec3::new(9.0, 1.0, 4.0).into()) < 1e-4);
        assert_ne!(
            affine.translation.to_vec3(),
            ecs_translation,
            "helper returned the ECS transform, not the Jolt pose"
        );
        // And the rotation follows Jolt too: the teleport set IDENTITY, so
        // +X maps to +X (the spawn rotation is gone, and with it any
        // chance of passing on spawn-time state).
        let mapped = affine.transform_vector3(Vec3::X);
        assert!(mapped.distance(Vec3::X) < 1e-4);
    }

    #[test]
    fn test_compute_world_affine_composes_plain_chain_and_stops_at_physics_ancestor() {
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(BoltPlugin::default());

        // Physics grandparent at (10, 0, 0), rotated +90° about Y.
        let physics_grandparent = app
            .world_mut()
            .spawn((
                Transform::from_xyz(10.0, 0.0, 0.0)
                    .with_rotation(Quat::from_rotation_y(std::f32::consts::FRAC_PI_2)),
                RigidBody::Dynamic,
                Collider::Box {
                    half_extents: Vec3::splat(0.5),
                },
            ))
            .id();
        // Plain parent: local offset (2, 0, 0) under the physics body.
        let plain_parent = app
            .world_mut()
            .spawn((
                Transform::from_xyz(2.0, 0.0, 0.0),
                ChildOf(physics_grandparent),
            ))
            .id();
        // Plain child: local offset (0, 1, 0) under the plain parent.
        let plain_child = app
            .world_mut()
            .spawn((Transform::from_xyz(0.0, 1.0, 0.0), ChildOf(plain_parent)))
            .id();

        use bevy::ecs::system::RunSystemOnce;
        let _ = app.world_mut().run_system_once(spawn_physics_bodies);
        app.world_mut().flush();

        // Composition: grandparent's Jolt pose × (2, 0, 0) × (0, 1, 0).
        // The +90° Y rotation maps the accumulated (2, 1, 0) offset to
        // (0, 1, -2), so the world translation is (10, 1, -2). An inverted
        // composition order produces a different answer, so this pins the
        // ancestor-on-the-left rule. (Mutation-verified: swapping the
        // multiply order in the walk fails here with (12, 1, 0).)
        let affine = affine_of(&mut app, plain_child).expect("plain-chain affine");
        assert!(
            affine
                .translation
                .distance(Vec3::new(10.0, 1.0, -2.0).into())
                < 1e-3,
            "expected (10, 1, -2), got {}",
            affine.translation
        );
    }

    #[test]
    fn test_compute_world_affine_root_plain_entity_returns_own_transform() {
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(BoltPlugin::default());

        let entity = app
            .world_mut()
            .spawn(Transform::from_xyz(3.0, 4.0, 5.0).with_scale(Vec3::new(2.0, 1.0, 1.0)))
            .id();

        let affine = affine_of(&mut app, entity).expect("root plain affine");
        assert!(affine.translation.distance(Vec3::new(3.0, 4.0, 5.0).into()) < 1e-5);
        // Scale rides along: local +X is doubled.
        let mapped = affine.transform_vector3(Vec3::X);
        assert!(mapped.distance(Vec3::new(2.0, 0.0, 0.0)) < 1e-5);
    }

    #[test]
    fn test_compute_world_affine_unspawned_rigidbody_yields_none() {
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(BoltPlugin::default());

        // RigidBody present but spawn_physics_bodies never ran: no JoltBody,
        // yet Without<RigidBody> keeps it out of the plain query too. The
        // helper must report "not computable" — the child branch then falls
        // back to world coordinates, same as a stale handle.
        let entity = app
            .world_mut()
            .spawn((
                Transform::from_xyz(0.0, 10.0, 0.0),
                RigidBody::Dynamic,
                Collider::Box {
                    half_extents: Vec3::splat(0.5),
                },
            ))
            .id();

        assert!(affine_of(&mut app, entity).is_none());
    }
}
