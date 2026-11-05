use bevy::math::Affine3A;
use bevy::prelude::*;

use crate::components::{
    AngularDamping, AngularVelocity, Collider, ContinuousCollision, ExternalForce, ExternalImpulse,
    ExternalTorque, JoltBody, LinearDamping, LinearVelocity, RigidBody, StaticMarker,
    TransformDriven,
};
use crate::config::PhysicsConfig;
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
    /// Re-issues continuous external forces/torques and applies one-shot
    /// impulses from ECS components into Jolt, just before the step.
    /// The slot is a latency choice, not a correctness requirement — the
    /// accumulator would treat a later re-issue exactly the same.
    ApplyForces,
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
///   A failed *fresh* spawn removes the physics components and the dead
///   `ExternalImpulse`; `ExternalForce`/`ExternalTorque` are live user data
///   that auto-resumes when a body exists (presence is their re-issue) and
///   are kept. Unfireable impulses generally are discarded by
///   [`apply_external_impulses`] when the body comes into existence.
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
            Option<&ContinuousCollision>,
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
                    "Failed to spawn physics body for entity {:?}; removing RigidBody, \
                     Collider, and ExternalImpulse (any ExternalForce/ExternalTorque is kept)",
                    entity
                );
                commands.entity(entity).remove::<(
                    RigidBody,
                    Collider,
                    JoltBody,
                    StaticMarker,
                    TransformDriven,
                    // The impulse is dead data without a body (its change
                    // was consumed or will be discarded at the door when a
                    // retry succeeds) — strip it. ExternalForce/Torque are
                    // LIVE user data that auto-resumes the moment a body
                    // exists (presence is the re-issue): they stay.
                    ExternalImpulse,
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

/// Re-issues [`ExternalForce`] and [`ExternalTorque`] into Jolt every
/// fixed tick, unconditionally — deliberately *not* gated on `Changed`
/// — because Jolt clears its per-body force and torque accumulators
/// after every step: a change-gated force would apply for exactly one
/// tick and then silently vanish while its component sits unchanged
/// (core's `test_add_force_is_one_step_lived` pins this). The
/// component's presence *is* the re-issue: a sustained push is inserted
/// once and left alone.
///
/// Zero vectors are skipped rather than sent: Jolt's `AddForce` wakes
/// the body even for a zero force (its activation default fires
/// unconditionally inside the dynamic branch), so forwarding a calm
/// wind's `Vec3::ZERO` every tick would pin holders awake forever.
/// Skipping preserves the physics exactly — a zero force does nothing —
/// while letting bodies sleep.
///
/// Non-dynamic bodies are skipped ECS-side rather than paying a futile
/// FFI call Jolt would no-op itself — the same gate
/// [`move_transform_driven_kinematics`] uses. Sleeping bodies are *not*
/// skipped: a push is how they wake.
#[allow(clippy::type_complexity)]
pub fn apply_external_forces(
    mut physics_world: ResMut<PhysicsWorld>,
    query: Query<
        (
            &JoltBody,
            &RigidBody,
            Option<&ExternalForce>,
            Option<&ExternalTorque>,
        ),
        Or<(With<ExternalForce>, With<ExternalTorque>)>,
    >,
) {
    for (body, rigidbody, force, torque) in query.iter() {
        if *rigidbody != RigidBody::Dynamic {
            continue;
        }
        if let Some(force) = force
            && **force != Vec3::ZERO
        {
            physics_world.add_force(body.0, **force);
        }
        if let Some(torque) = torque
            && **torque != Vec3::ZERO
        {
            physics_world.add_torque(body.0, **torque);
        }
    }
}

/// Applies [`ExternalImpulse`] as a one-shot kick: every body whose
/// component changed this tick receives the impulse — velocity changes
/// immediately, `Δv = impulse / mass` — after which the component is
/// reset to `Vec3::ZERO` through `bypass_change_detection`, so the
/// reset never looks like a user write. A plain write would trip
/// `Changed` again next tick and re-fire the same kick forever,
/// turning the one-shot into a machine gun; the bypass keeps "a fresh
/// kick is a fresh component write" true.
///
/// Unfireable impulses are discarded here, at the door — this system
/// is the consumer of changes, so it alone can judge "can this still
/// fire?": the `Or` filter also admits bodies whose `JoltBody` is
/// fresh (spawn or respawn), and a non-zero impulse that is *not*
/// changed by this system's own view had its tick consumed before the
/// body existed (written while bodyless or non-dynamic, or bundled
/// into a `FixedUpdate` command-spawn whose flush lands after this
/// system's window). Such a value can never fire; it is zeroed with a
/// warning — impulses are consumed or discarded, never carried — when
/// the body is (or becomes) dynamic; a non-dynamic holder skips it,
/// and the discard lands at the dynamic flip. A fresh write rides the
/// ordinary path and fires, including same-frame world-level inserts
/// and systems ordered before the `Spawn` stage.
///
/// Zero vectors and non-dynamic bodies are skipped without an FFI call
/// (Jolt ignores impulses on statics and kinematics anyway, and a zero
/// impulse would still wake a sleeping body for nothing).
#[allow(clippy::type_complexity)]
pub fn apply_external_impulses(
    mut physics_world: ResMut<PhysicsWorld>,
    mut query: Query<
        (&JoltBody, &RigidBody, &mut ExternalImpulse),
        Or<(Changed<ExternalImpulse>, Changed<JoltBody>)>,
    >,
) {
    for (body, rigidbody, mut impulse) in query.iter_mut() {
        if *rigidbody != RigidBody::Dynamic || **impulse == Vec3::ZERO {
            continue;
        }
        if !impulse.is_changed() {
            warn!(
                "Discarded an unfireable ExternalImpulse for entity {:?} — its change was \
                 consumed before the body existed; impulses are consumed or discarded, never \
                 carried",
                body
            );
            impulse.bypass_change_detection().0 = Vec3::ZERO;
            continue;
        }
        physics_world.add_impulse(body.0, **impulse);
        impulse.bypass_change_detection().0 = Vec3::ZERO;
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
    use crate::gravity::Gravity;
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
            app.world().get::<TransformDriven>(entity).is_some(),
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
            app.world().get::<TransformDriven>(entity).is_some(),
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
            app.world().get::<TransformDriven>(entity).is_none(),
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
        assert!(app.world().get::<TransformDriven>(entity).is_some());

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
            app.world().get::<TransformDriven>(entity).is_none(),
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
            app.world().get::<TransformDriven>(entity).is_none(),
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
            app.world().get::<TransformDriven>(entity).is_some(),
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
                app.world().get::<TransformDriven>(entity).is_none(),
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

    // --- ApplyForces: the push family's ECS contracts ---

    /// A minimal app with a manual 1/64 s clock, zero gravity, and a
    /// dynamic 0.5 m half-extent box (a 1 m cube = 1 m³ = 1000 kg at
    /// Jolt's default density) spawned at the given offset — the mule
    /// the ApplyForces tests build their numbers on. Returns the app
    /// and the box's entity.
    fn forces_app(position: Vec3) -> (App, Entity) {
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(TransformPlugin);
        app.add_plugins(BoltPlugin::default());
        app.world_mut()
            .insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(
                std::time::Duration::from_secs_f32(1.0 / 64.0),
            ));
        app.world_mut().insert_resource(Gravity(Vec3::ZERO));
        let entity = app
            .world_mut()
            .spawn((
                Transform::from_translation(position),
                RigidBody::Dynamic,
                Collider::Box {
                    half_extents: Vec3::splat(0.5),
                },
            ))
            .id();
        (app, entity)
    }

    #[test]
    fn test_unchanged_external_force_keeps_applying() {
        // The component is inserted once and NEVER touched again, so a
        // change-gated system would fire for exactly one tick. 1000 N on
        // 1000 kg is 1 m/s² — after ~1 s of fixed ticks the velocity must
        // be near 1 m/s, not parked at one tick's worth (~0.016).
        let (mut app, entity) = forces_app(Vec3::new(0.0, 10.0, 0.0));
        app.world_mut()
            .get_entity_mut(entity)
            .unwrap()
            .insert(ExternalForce(Vec3::new(0.0, 1000.0, 0.0)));

        for _ in 0..3 {
            app.update(); // warm-up: first update may run zero fixed ticks
        }
        let early = app.world().get::<LinearVelocity>(entity).unwrap().y;

        for _ in 0..64 {
            app.update(); // one simulated second, component untouched
        }
        let late = app.world().get::<LinearVelocity>(entity).unwrap().y;

        assert!(
            late > early + 0.5,
            "an unchanged ExternalForce must keep applying every tick \
             (the accumulator is cleared per step); got {early} then {late}"
        );
    }

    #[test]
    fn test_external_torque_spins_without_translating() {
        // Continuous torque through the ECS: angular velocity grows
        // while the center of mass stays put.
        let (mut app, entity) = forces_app(Vec3::new(0.0, 10.0, 0.0));
        app.world_mut()
            .get_entity_mut(entity)
            .unwrap()
            .insert(ExternalTorque(Vec3::new(0.0, 500.0, 0.0)));

        for _ in 0..3 {
            app.update();
        }
        let early = app.world().get::<AngularVelocity>(entity).unwrap().y.abs();

        for _ in 0..64 {
            app.update();
        }
        let (late, linear) = {
            let world = app.world();
            let ang = world.get::<AngularVelocity>(entity).unwrap().y.abs();
            let lin = world.get::<LinearVelocity>(entity).unwrap().length();
            (ang, lin)
        };

        assert!(
            late > early + 0.5,
            "an unchanged ExternalTorque must keep spinning the body up; \
             got {early} then {late}"
        );
        assert!(
            linear < 1e-3,
            "a pure torque must not translate the body, got {linear}"
        );
    }

    #[test]
    fn test_push_family_ignores_non_dynamic_bodies() {
        // Static and kinematic bodies carry all three push components;
        // Jolt's IsDynamic() gate (plus the ECS-side skip) must leave
        // both exactly where they spawned.
        // The helper's dynamic mule floats untouched here (zero gravity);
        // the actual subjects spawn below.
        let (mut app, _mule) = forces_app(Vec3::new(0.0, 10.0, 0.0));
        let static_e = app
            .world_mut()
            .spawn((
                Transform::from_xyz(20.0, 0.0, 0.0),
                RigidBody::Static,
                Collider::Box {
                    half_extents: Vec3::splat(0.5),
                },
                ExternalForce(Vec3::new(0.0, 100_000.0, 0.0)),
                ExternalTorque(Vec3::Y * 100_000.0),
                ExternalImpulse(Vec3::new(0.0, 100_000.0, 0.0)),
            ))
            .id();
        let kinematic_e = app
            .world_mut()
            .spawn((
                Transform::from_xyz(30.0, 0.0, 0.0),
                RigidBody::Kinematic,
                Collider::Box {
                    half_extents: Vec3::splat(0.5),
                },
                ExternalForce(Vec3::new(0.0, 100_000.0, 0.0)),
                ExternalTorque(Vec3::Y * 100_000.0),
                ExternalImpulse(Vec3::new(0.0, 100_000.0, 0.0)),
            ))
            .id();

        for _ in 0..16 {
            app.update();
        }

        for (entity, expected) in [
            (static_e, Vec3::new(20.0, 0.0, 0.0)),
            (kinematic_e, Vec3::new(30.0, 0.0, 0.0)),
        ] {
            let transform = app.world().get::<Transform>(entity).unwrap();
            assert!(
                transform.translation.distance(expected) < 1e-3,
                "non-dynamic body must ignore the push family, got {}",
                transform.translation
            );
        }
    }

    #[test]
    fn test_external_impulse_fires_once_and_resets() {
        // 1000 N·s on the 1000 kg mule = 1 m/s. The kick lands once;
        // the component reads ZERO afterwards and stays zero; the
        // velocity does NOT keep growing on later ticks.
        let (mut app, entity) = forces_app(Vec3::new(0.0, 10.0, 0.0));
        app.world_mut()
            .get_entity_mut(entity)
            .unwrap()
            .insert(ExternalImpulse(Vec3::new(0.0, 1000.0, 0.0)));

        for _ in 0..3 {
            app.update();
        }
        let kicked = app.world().get::<LinearVelocity>(entity).unwrap().y;
        assert!(
            (kicked - 1.0).abs() < 0.2,
            "impulse must deliver Δv = J/m = 1 m/s, got {kicked}"
        );
        assert_eq!(
            *app.world().get::<ExternalImpulse>(entity).unwrap(),
            ExternalImpulse(Vec3::ZERO),
            "impulse must be reset to zero after firing"
        );

        for _ in 0..8 {
            app.update();
        }
        let settled = app.world().get::<LinearVelocity>(entity).unwrap().y;
        assert!(
            (settled - kicked).abs() < 0.2,
            "the one-shot must not re-fire on later ticks: {kicked} then {settled}"
        );
        assert_eq!(
            *app.world().get::<ExternalImpulse>(entity).unwrap(),
            ExternalImpulse(Vec3::ZERO)
        );
    }

    #[test]
    fn test_impulse_rewrite_fires_again() {
        // The other half of the one-shot contract: a fresh component
        // write is a fresh kick. After the first kick (1 m/s) and the
        // reset, writing the SAME value must fire again (2 m/s total).
        let (mut app, entity) = forces_app(Vec3::new(0.0, 10.0, 0.0));
        app.world_mut()
            .get_entity_mut(entity)
            .unwrap()
            .insert(ExternalImpulse(Vec3::new(0.0, 1000.0, 0.0)));
        for _ in 0..3 {
            app.update();
        }
        let first = app.world().get::<LinearVelocity>(entity).unwrap().y;
        assert!((first - 1.0).abs() < 0.2);

        {
            let mut impulse = app.world_mut().get_mut::<ExternalImpulse>(entity).unwrap();
            impulse.0 = Vec3::new(0.0, 1000.0, 0.0); // same value, fresh write
        }
        for _ in 0..3 {
            app.update();
        }
        let second = app.world().get::<LinearVelocity>(entity).unwrap().y;
        assert!(
            (second - 2.0).abs() < 0.3,
            "a rewritten impulse must fire again: 1 m/s + 1 m/s expected, got {second}"
        );
    }

    #[test]
    fn test_zero_external_force_lets_body_sleep() {
        // Jolt's AddForce wakes the body even for a zero force (activation
        // fires unconditionally in the dynamic branch), so forwarding a
        // calm wind's ZERO every tick would keep the body awake forever.
        // The precondition assert is the point — only a genuinely
        // sleeping body proves the guard.
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(TransformPlugin);
        app.add_plugins(BoltPlugin::default()); // default gravity: it must land
        app.world_mut()
            .insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(
                std::time::Duration::from_secs_f32(1.0 / 64.0),
            ));
        // Wide floor — a cube-sized floor under an off-center box is the
        // "falls forever, never sleeps" trap the core wake test taught.
        app.world_mut().spawn((
            Transform::from_xyz(0.0, -1.0, 0.0),
            RigidBody::Static,
            Collider::Box {
                half_extents: Vec3::new(50.0, 1.0, 50.0),
            },
        ));
        let entity = app
            .world_mut()
            .spawn((
                Transform::from_xyz(0.0, 0.5, 0.0), // resting on the floor
                RigidBody::Dynamic,
                Collider::Box {
                    half_extents: Vec3::splat(0.5),
                },
                ExternalForce(Vec3::ZERO), // present, but calm
            ))
            .id();

        for _ in 0..192 {
            // 3 simulated seconds to settle into sleep (the box spawns
            // already resting — nothing falls).
            app.update();
        }
        let (physics_world, jolt_body) = {
            let world = app.world();
            (
                world.get_resource::<PhysicsWorld>().unwrap(),
                world.get::<JoltBody>(entity).unwrap().handle(),
            )
        };
        assert!(
            !physics_world.is_active(jolt_body),
            "a zero ExternalForce must not keep its body awake — the system \
             must skip zero vectors rather than wake Jolt for nothing"
        );
    }

    #[test]
    fn test_stale_impulse_is_discarded_when_body_spawns() {
        // Impulses are consumed or discarded, never carried. A kick
        // written while the entity is bodyless has its Changed tick
        // consumed; when the body finally spawns, the stale value must
        // be discarded (zeroed) — not left sitting unfireable, not fired.
        let (mut app, mule) = forces_app(Vec3::new(0.0, 10.0, 0.0));
        // Remove the mule's physics identity, arm an impulse while it is
        // bodyless, then let the body come into existence.
        app.world_mut()
            .get_entity_mut(mule)
            .unwrap()
            .remove::<(RigidBody, Collider)>();
        app.world_mut()
            .get_entity_mut(mule)
            .unwrap()
            .insert(ExternalImpulse(Vec3::new(0.0, 1000.0, 0.0)));
        for _ in 0..3 {
            app.update();
        }
        // Precondition: the kick is still sitting non-zero, unconsumed as
        // data — otherwise the discard below would pass vacuously.
        assert_ne!(
            *app.world().get::<ExternalImpulse>(mule).unwrap(),
            ExternalImpulse(Vec3::ZERO)
        );
        app.world_mut().get_entity_mut(mule).unwrap().insert((
            RigidBody::Dynamic,
            Collider::Box {
                half_extents: Vec3::splat(0.5),
            },
        ));
        for _ in 0..5 {
            app.update();
        }

        let velocity = app.world().get::<LinearVelocity>(mule).unwrap().y;
        assert!(
            velocity.abs() < 1e-6,
            "the stale kick must not fire when the body spawns: {velocity}"
        );
        assert_eq!(
            *app.world().get::<ExternalImpulse>(mule).unwrap(),
            ExternalImpulse(Vec3::ZERO),
            "the stale kick must be discarded, not carried"
        );
    }

    #[test]
    fn test_consumed_impulse_is_discarded_on_dynamic_flip() {
        // Impulses are consumed or discarded, never carried. A kick
        // written while the body is kinematic is skipped but its Changed
        // tick is consumed; flipping to Dynamic rebuilds the body, and
        // the now-unfireable value must be discarded — never carried,
        // never fired.
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(TransformPlugin);
        app.add_plugins(BoltPlugin::default());
        app.world_mut()
            .insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(
                std::time::Duration::from_secs_f32(1.0 / 64.0),
            ));
        app.world_mut()
            .insert_resource(crate::gravity::Gravity(Vec3::ZERO));
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
        for _ in 0..3 {
            app.update();
        }

        // The kick aimed at what was not yet a dynamic body: consumed.
        app.world_mut()
            .get_entity_mut(entity)
            .unwrap()
            .insert(ExternalImpulse(Vec3::new(0.0, 1000.0, 0.0)));
        for _ in 0..3 {
            app.update();
        }
        assert_ne!(
            *app.world().get::<ExternalImpulse>(entity).unwrap(),
            ExternalImpulse(Vec3::ZERO),
            "precondition: the kinematic write must be sitting unconsumed-as-data"
        );

        // Flip to Dynamic — a respawn. The zombie must be discarded.
        {
            let mut rigidbody = app.world_mut().get_mut::<RigidBody>(entity).unwrap();
            *rigidbody = RigidBody::Dynamic;
        }
        for _ in 0..5 {
            app.update();
        }

        let velocity = app.world().get::<LinearVelocity>(entity).unwrap().y;
        assert!(
            velocity.abs() < 1e-6,
            "the consumed kick must not fire on the rebuilt body: {velocity}"
        );
        assert_eq!(
            *app.world().get::<ExternalImpulse>(entity).unwrap(),
            ExternalImpulse(Vec3::ZERO),
            "the consumed kick must be discarded at the door"
        );
    }

    #[test]
    fn test_same_frame_spawn_impulse_still_fires() {
        // The discriminator's other edge: an impulse written in the same
        // frame as the spawn (a bundle insert, or any write before the
        // first FixedUpdate) is Changed at the body's creation and must
        // ride the ordinary path — fire, not discard. 1000 N·s on the
        // 1000 kg mule = 1 m/s.
        let (mut app, entity) = forces_app(Vec3::new(0.0, 10.0, 0.0));
        app.world_mut()
            .get_entity_mut(entity)
            .unwrap()
            .insert(ExternalImpulse(Vec3::new(0.0, 1000.0, 0.0)));
        for _ in 0..3 {
            app.update();
        }
        let velocity = app.world().get::<LinearVelocity>(entity).unwrap().y;
        assert!(
            (velocity - 1.0).abs() < 0.2,
            "same-frame kick must fire, got {velocity}"
        );
        assert_eq!(
            *app.world().get::<ExternalImpulse>(entity).unwrap(),
            ExternalImpulse(Vec3::ZERO)
        );
    }

    #[test]
    fn test_failed_spawn_strips_push_components() {
        // A failed fresh spawn strips the physics bundle — the push family
        // included (added with the discard work): the entity leaves physics
        // exactly as it came, no orphaned pushes or kicks.
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(BoltPlugin::default());
        app.world_mut()
            .insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(
                std::time::Duration::from_secs_f32(1.0 / 64.0),
            ));
        let entity = app
            .world_mut()
            .spawn((
                Transform::from_xyz(0.0, 10.0, 0.0),
                RigidBody::Dynamic,
                Collider::Box {
                    half_extents: Vec3::splat(-1.0), // invalid: spawn fails
                },
                ExternalForce(Vec3::Y),
                ExternalTorque(Vec3::Y),
                ExternalImpulse(Vec3::Y),
            ))
            .id();
        // Warm-up: the strip is a deferred command inside FixedUpdate, and
        // a manual clock's first update may run zero fixed ticks.
        for _ in 0..3 {
            app.update();
        }

        let world = app.world();
        assert!(world.get::<JoltBody>(entity).is_none());
        // The impulse is dead data — stripped with the bundle.
        assert!(world.get::<ExternalImpulse>(entity).is_none());
        // Forces are LIVE user data that auto-resumes when a body exists
        // (presence is their re-issue) — they must survive the failure.
        assert!(world.get::<ExternalForce>(entity).is_some());
        assert!(world.get::<ExternalTorque>(entity).is_some());
    }

    #[test]
    fn test_external_force_survives_respawn() {
        // Presence is the re-issue — and re-issue means across body
        // rebuilds too: a collider-change respawn must not interrupt a
        // continuous push.
        let (mut app, entity) = forces_app(Vec3::new(0.0, 10.0, 0.0));
        app.world_mut()
            .get_entity_mut(entity)
            .unwrap()
            .insert(ExternalForce(Vec3::new(0.0, 1000.0, 0.0)));
        for _ in 0..3 {
            app.update();
        }
        let before = app.world().get::<LinearVelocity>(entity).unwrap().y;

        // A genuine collider change: destroy and rebuild the body mid-push.
        {
            let mut collider = app.world_mut().get_mut::<Collider>(entity).unwrap();
            *collider = Collider::Box {
                half_extents: Vec3::splat(0.4),
            };
        }
        for _ in 0..32 {
            app.update(); // half a simulated second of pushing
        }
        let after = app.world().get::<LinearVelocity>(entity).unwrap().y;

        assert!(
            after > before + 0.3,
            "the push must resume on the replacement body: {before} then {after}"
        );
        assert!(
            app.world().get::<JoltBody>(entity).is_some(),
            "precondition sanity: the respawn produced a body"
        );
    }

    #[test]
    fn test_impulse_reset_is_invisible_to_changed_consumers() {
        // The bypass's only observable value: a downstream system
        // filtering Changed<ExternalImpulse> must see exactly the user's
        // writes. The reset after a kick — and the discard at the door —
        // write through bypass_change_detection and must never masquerade
        // as user intent (a plain-write regression would count 2+).
        #[derive(Resource, Default)]
        struct ObservedWrites(usize);
        fn observer(mut seen: ResMut<ObservedWrites>, query: Query<(), Changed<ExternalImpulse>>) {
            seen.0 += query.iter().count();
        }

        let (mut app, entity) = forces_app(Vec3::new(0.0, 10.0, 0.0));
        app.init_resource::<ObservedWrites>();
        // Placement is the discriminator: the observer must run BEFORE
        // ApplyForces each update, so a phantom reset (a plain write, made
        // after the observer's run but before its next one) becomes
        // visible on the following update. An observer placed after
        // ApplyForces would collapse user write and phantom into one
        // change-window and pass vacuously.
        app.add_systems(FixedUpdate, observer.before(PhysicsSet::ApplyForces));
        for _ in 0..3 {
            app.update(); // warm-up; no writes yet
        }

        // The user writes once. The kick fires, the reset must be silent.
        app.world_mut()
            .get_entity_mut(entity)
            .unwrap()
            .insert(ExternalImpulse(Vec3::new(0.0, 1000.0, 0.0)));
        for _ in 0..8 {
            app.update();
        }

        let seen = app.world().get_resource::<ObservedWrites>().unwrap().0;
        assert_eq!(
            seen, 1,
            "observer must see exactly the one user write, saw {seen}"
        );
        assert_eq!(
            *app.world().get::<ExternalImpulse>(entity).unwrap(),
            ExternalImpulse(Vec3::ZERO)
        );
    }

    #[test]
    fn test_fixedupdate_bundle_impulse_is_discarded_at_the_door() {
        // An unordered FixedUpdate system spawning a body WITH its kick is
        // the idiomatic "projectile" pattern — and its command flush can
        // land after the apply system's window, consuming the change
        // before the body exists. The kick must be discarded with a
        // warning, never left sitting unfireable.
        fn spawn_projectile(mut commands: Commands, mut done: Local<bool>) {
            if *done {
                return;
            }
            *done = true;
            commands.spawn((
                Transform::from_xyz(0.0, 10.0, 0.0),
                RigidBody::Dynamic,
                Collider::Box {
                    half_extents: Vec3::splat(0.5),
                },
                ExternalImpulse(Vec3::new(0.0, 1000.0, 0.0)),
            ));
        }

        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(TransformPlugin);
        app.add_plugins(BoltPlugin::default());
        app.world_mut()
            .insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(
                std::time::Duration::from_secs_f32(1.0 / 64.0),
            ));
        app.world_mut()
            .insert_resource(crate::gravity::Gravity(Vec3::ZERO));
        app.add_systems(
            FixedUpdate,
            // Explicit window edges make the dead-window flush
            // deterministic rather than executor-timing luck: the spawner
            // runs after Spawn's tick and before ApplyForces', so its
            // command batch lands squarely in the window where the kick's
            // change is consumed while the body cannot yet be seen.
            spawn_projectile
                .after(PhysicsSet::Spawn)
                .before(PhysicsSet::ApplyForces),
        );

        for _ in 0..6 {
            app.update();
        }

        let mut query = app
            .world_mut()
            .query::<(&LinearVelocity, &ExternalImpulse)>();
        let (velocity, impulse) = query.single(app.world()).unwrap();
        assert!(
            velocity.y.abs() < 1e-6,
            "the in-window kick must not fire: {}",
            velocity.y
        );
        assert_eq!(
            *impulse,
            ExternalImpulse(Vec3::ZERO),
            "the in-window kick must be discarded at the door, not zombified"
        );
    }

    #[test]
    fn test_pre_spawn_ordered_kick_fires() {
        // A writer ordered before the Spawn stage flushes its bundle
        // early enough that the kick's change is alive when the body
        // exists — it fires normally.
        fn spawn_projectile(mut commands: Commands, mut done: Local<bool>) {
            if *done {
                return;
            }
            *done = true;
            commands.spawn((
                Transform::from_xyz(0.0, 10.0, 0.0),
                RigidBody::Dynamic,
                Collider::Box {
                    half_extents: Vec3::splat(0.5),
                },
                ExternalImpulse(Vec3::new(0.0, 1000.0, 0.0)),
            ));
        }

        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(TransformPlugin);
        app.add_plugins(BoltPlugin::default());
        app.world_mut()
            .insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(
                std::time::Duration::from_secs_f32(1.0 / 64.0),
            ));
        app.world_mut()
            .insert_resource(crate::gravity::Gravity(Vec3::ZERO));
        app.add_systems(FixedUpdate, spawn_projectile.before(PhysicsSet::Spawn));

        for _ in 0..6 {
            app.update();
        }

        let mut query = app
            .world_mut()
            .query::<(&LinearVelocity, &ExternalImpulse)>();
        let (velocity, impulse) = query.single(app.world()).unwrap();
        assert!(
            (velocity.y - 1.0).abs() < 0.2,
            "a pre-Spawn-ordered kick must fire: {}",
            velocity.y
        );
        assert_eq!(*impulse, ExternalImpulse(Vec3::ZERO));
    }

    #[test]
    fn test_discard_is_invisible_to_changed_consumers() {
        // The discard-at-the-door also writes through the bypass. An
        // observer (placed before ApplyForces, the discriminating window)
        // must see exactly the user's original write — never the
        // discard's zeroing; a plain write would surface as a second
        // sighting.
        #[derive(Resource, Default)]
        struct ObservedWrites(usize);
        fn observer(mut seen: ResMut<ObservedWrites>, query: Query<(), Changed<ExternalImpulse>>) {
            seen.0 += query.iter().count();
        }

        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(TransformPlugin);
        app.add_plugins(BoltPlugin::default());
        app.world_mut()
            .insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(
                std::time::Duration::from_secs_f32(1.0 / 64.0),
            ));
        app.world_mut()
            .insert_resource(crate::gravity::Gravity(Vec3::ZERO));
        app.init_resource::<ObservedWrites>();
        app.add_systems(FixedUpdate, observer.before(PhysicsSet::ApplyForces));

        // The zombie: a kick written while bodyless, its change consumed.
        let entity = app
            .world_mut()
            .spawn((
                Transform::from_xyz(0.0, 10.0, 0.0),
                ExternalImpulse(Vec3::new(0.0, 1000.0, 0.0)),
            ))
            .id();
        for _ in 0..3 {
            app.update();
        }
        assert_eq!(app.world().get_resource::<ObservedWrites>().unwrap().0, 1);

        // The body arrives; the discard fires this tick.
        app.world_mut().get_entity_mut(entity).unwrap().insert((
            RigidBody::Dynamic,
            Collider::Box {
                half_extents: Vec3::splat(0.5),
            },
        ));
        for _ in 0..5 {
            app.update();
        }

        assert_eq!(
            app.world().get_resource::<ObservedWrites>().unwrap().0,
            1,
            "the discard must be invisible: exactly the user's one write"
        );
        assert_eq!(
            *app.world().get::<ExternalImpulse>(entity).unwrap(),
            ExternalImpulse(Vec3::ZERO)
        );
    }

    #[test]
    fn test_fixed_update_schedule_has_no_ambiguous_pairs() {
        // Any two Bolt systems sharing ResMut<PhysicsWorld> without an
        // ordering edge make the FixedUpdate schedule ambiguous: the
        // warning is invisible under MinimalPlugins (no LogPlugin to
        // print it), and apps that escalate ambiguity_detection to
        // Error cannot build at all. Settings must be installed BEFORE
        // the first schedule build, and real updates must follow --
        // settings on an already-built schedule never re-check, and
        // FixedUpdate initializes lazily — each is a vacuous-pass trap
        // in its own right.
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(TransformPlugin);
        app.add_plugins(BoltPlugin::default());
        app.world_mut()
            .insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(
                std::time::Duration::from_secs_f32(1.0 / 64.0),
            ));
        app.world_mut().schedule_scope(FixedUpdate, |_, schedule| {
            schedule.set_build_settings(bevy::ecs::schedule::ScheduleBuildSettings {
                ambiguity_detection: bevy::ecs::schedule::LogLevel::Error,
                ..Default::default()
            });
        });
        for _ in 0..3 {
            app.update(); // panics if any pair has indeterminate order
        }
    }
}
