use bevy::prelude::*;
use bevy::time::TimeUpdateStrategy::ManualDuration;
use bolt_bevy::prelude::*;

fn physics_test_app() -> App {
    let mut app = App::new();
    app.add_plugins(MinimalPlugins);
    app.add_plugins(TransformPlugin);
    app.add_plugins(bolt_bevy::plugin::BoltPlugin::default());
    app
}
use std::time::Duration;

#[test]
fn test_gravity_pulls_dynamic_bodies() {
    let mut app = physics_test_app();

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

    app.update();
    advance_time_safely(&mut app, Duration::from_secs(1));

    let transform = app.world().get::<Transform>(entity).unwrap();

    assert!(
        transform.translation.y < 10.0,
        "Box did not fall! Y is still {}",
        transform.translation.y
    );
}

/// Advances the simulation time safely by the given duration.
///
/// **Why use a loop instead of one big time jump?**
/// Our physics engine runs inside Bevy's `FixedUpdate` schedule (usually 60 or 64 times per second).
/// If you tell Bevy that exactly 1 full second passed in a single frame, `FixedUpdate` realizes it
/// missed 60 frames and tries to run 60 physics steps back-to-back immediately to "catch up".
///
/// To prevent actual games from freezing if lag spikes occur, Bevy has a hardcoded "catch-up limit"
/// (it will refuse to run more than a certain number of fixed updates in a single frame). If we jump time
/// by too much at once, we hit that limit, and the physics simulation gets artificially truncated.
///
/// By looping and advancing time by 100 milliseconds per loop, we safely spoon-feed time to the engine.
/// 100ms is about 6 fixed physics steps per `app.update()`, which easily stays under Bevy's
/// catch-up limit, perfectly simulating the requested duration of physics!
fn advance_time_safely(app: &mut App, total_duration: Duration) {
    let step = std::time::Duration::from_millis(100);
    let mut remaining = total_duration;

    while remaining > std::time::Duration::ZERO {
        let current_step = remaining.min(step);
        app.world_mut()
            .insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(current_step));
        app.update();
        remaining -= current_step;
    }
}

#[test]
fn test_fixed_timestep_prevents_micro_updates() {
    let mut app = physics_test_app();

    // Freeze wall-clock time so test timing is deterministic
    app.world_mut()
        .insert_resource(ManualDuration(Duration::ZERO));
    // 1. Spawn a box exactly at Y = 10.0
    let start_y = 10.0;
    let entity = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, start_y, 0.0),
            RigidBody::Dynamic,
            Collider::Box {
                half_extents: Vec3::splat(1.0),
            },
        ))
        .id();

    // 2. Run one initial frame to let the ECS spawn the physics body
    app.update();

    // 3. Advance time by a microscopic amount (1 millisecond)
    app.world_mut()
        .resource_mut::<Time>()
        .advance_by(std::time::Duration::from_millis(1));

    // 4. Update the app.
    app.update();

    // 5. Check the box's position
    let transform = app.world().get::<Transform>(entity).unwrap();

    // TDD ASSERTION: If physics is correctly using FixedUpdate (64Hz = 16ms),
    // a 1ms advance should NOT trigger a physics step. The box should not have moved!
    assert!(
        (transform.translation.y - start_y).abs() < 0.001,
        "The box moved! Physics is running on variable Update instead of FixedUpdate!"
    );
}

#[test]
fn test_despawn_cleans_up_physics_body() {
    let mut app = physics_test_app();

    // 1. Spawn a box
    let entity = app
        .world_mut()
        .spawn((
            Transform::default(),
            RigidBody::Dynamic,
            Collider::Box {
                half_extents: Vec3::splat(1.0),
            },
        ))
        .id();

    // 2. Run an Update so the physics body is created in Jolt
    app.update();
    app.world_mut()
        .insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(
            std::time::Duration::from_secs_f32(1.0 / 60.0),
        ));
    app.update();

    // 3. Verify JoltBody component exists and is live in Jolt
    let jolt_body = app
        .world()
        .get::<bolt_bevy::prelude::JoltBody>(entity)
        .expect("JoltBody was not attached to entity!");
    let body_id = jolt_body.handle();

    let physics_world = app.world().resource::<bolt_bevy::prelude::PhysicsWorld>();
    assert!(
        physics_world.get_transform(body_id).is_some(),
        "Body was not live in Jolt!"
    );

    // 4. DESPAWN the entity from Bevy!
    app.world_mut().despawn(entity);

    // 5. TDD ASSERTION: The Jolt body should be destroyed from Jolt Physics
    let physics_world = app.world().resource::<bolt_bevy::prelude::PhysicsWorld>();
    assert!(
        physics_world.get_transform(body_id).is_none(),
        "MEMORY LEAK! The entity was despawned in Bevy, but the Jolt physics body was not destroyed!"
    );
}

#[test]
fn test_child_entity_spawns_at_world_coordinates() {
    let mut app = physics_test_app();

    let parent = app
        .world_mut()
        .spawn((
            Transform::from_xyz(10.0, 20.0, 30.0),
            GlobalTransform::IDENTITY,
        ))
        .id();

    let child = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 5.0, 0.0),
            RigidBody::Static,
            Collider::Box {
                half_extents: Vec3::splat(1.0),
            },
            ChildOf(parent),
        ))
        .id();

    // 1. Run an update so TransformPlugin propagates transforms and Bolt spawns bodies
    app.update();
    app.world_mut()
        .insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(
            std::time::Duration::from_secs_f32(1.0 / 60.0),
        ));
    app.update();

    let jolt_body = app
        .world()
        .get::<bolt_bevy::prelude::JoltBody>(child)
        .expect("JoltBody component missing on child");

    let physics_world = app.world().resource::<bolt_bevy::prelude::PhysicsWorld>();
    let (pos, _) = physics_world
        .get_transform(jolt_body.handle())
        .expect("Body transform not found");

    // Child world pos should be (10.0, 25.0, 30.0), not local (0.0, 5.0, 0.0)
    assert_eq!(pos, Vec3::new(10.0, 25.0, 30.0));
}

#[test]
fn test_child_entity_dynamic_sync_transforms() {
    let mut app = physics_test_app();

    let parent = app
        .world_mut()
        .spawn((
            Transform::from_xyz(10.0, 20.0, 30.0),
            GlobalTransform::IDENTITY,
        ))
        .id();

    let child = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 0.0, 0.0),
            RigidBody::Dynamic,
            Collider::Box {
                half_extents: Vec3::splat(0.5),
            },
            ChildOf(parent),
        ))
        .id();

    // 1. Initial frame: spawn body in Jolt
    app.update();

    // 2. Advance time by 1 second to let gravity pull the child down
    advance_time_safely(&mut app, Duration::from_secs(1));

    let child_transform = app.world().get::<Transform>(child).unwrap();

    // The child fell below the parent, so its local Y translation relative to parent must be negative
    assert!(
        child_transform.translation.y < 0.0,
        "Child local Y translation did not sync relative to parent! Y is {}",
        child_transform.translation.y
    );
}

#[test]
fn test_kinematic_body_defies_gravity() {
    let mut app = physics_test_app();

    // 1. Spawn a Kinematic box high in the air
    let start_y = 10.0;
    let entity = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, start_y, 0.0),
            RigidBody::Kinematic,
            Collider::Box {
                half_extents: Vec3::splat(1.0),
            },
        ))
        .id();

    // 2. Run one initial frame to let the ECS spawn the physics body in Jolt
    app.update();

    // 3. Advance time by 1 full second
    app.world_mut()
        .insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(
            std::time::Duration::from_secs(1),
        ));

    // 4. Update the app to step the physics simulation
    app.update();

    // 5. Check the box's position
    let transform = app.world().get::<Transform>(entity).unwrap();

    // 6. ASSERTION: Kinematic bodies ignore gravity. It should still be exactly at start_y.
    assert!(
        (transform.translation.y - start_y).abs() < 0.001,
        "Kinematic body moved! It should not be affected by gravity."
    );
}

#[test]
fn test_apply_velocities_syncs_with_physics_engine() {
    let mut app = physics_test_app();

    let entity = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 0.0, 0.0),
            RigidBody::Kinematic,
            Collider::Box {
                half_extents: Vec3::splat(1.0),
            },
            LinearVelocity(Vec3::new(0.0, 10.0, 0.0)),
        ))
        .id();

    app.update();
    advance_time_safely(&mut app, std::time::Duration::from_secs(1));

    let transform = app.world().get::<Transform>(entity).unwrap();

    assert!(
        (transform.translation.y - 10.0).abs() < 0.001,
        "Kinematic body did not move at the correct velocity! Expected 10.0, got {}!",
        transform.translation.y
    );
}

#[test]
fn test_sphere_dynamic_body_falls() {
    let mut app = physics_test_app();
    let entity = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 10.0, 0.0),
            RigidBody::Dynamic,
            Collider::Sphere { radius: 1.0 },
        ))
        .id();

    app.update();
    advance_time_safely(&mut app, std::time::Duration::from_secs(1));

    let transform = app.world().get::<Transform>(entity).unwrap();
    assert!(
        transform.translation.y < 10.0,
        "Sphere did not fall! Jolt physics body may not have been created."
    );
}

#[test]
fn test_capsule_dynamic_body_falls() {
    let mut app = physics_test_app();
    let entity = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 10.0, 0.0),
            RigidBody::Dynamic,
            Collider::Capsule {
                half_height: 1.0,
                radius: 0.5,
            },
        ))
        .id();

    app.update();
    advance_time_safely(&mut app, std::time::Duration::from_secs(1));

    let transform = app.world().get::<Transform>(entity).unwrap();
    assert!(
        transform.translation.y < 10.0,
        "Capsule did not fall! Jolt physics body may not have been created."
    );
}

#[test]
fn test_cylinder_dynamic_body_falls() {
    let mut app = physics_test_app();
    let entity = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 10.0, 0.0),
            RigidBody::Dynamic,
            Collider::Cylinder {
                half_height: 1.0,
                radius: 0.5,
            },
        ))
        .id();

    app.update();
    advance_time_safely(&mut app, std::time::Duration::from_secs(1));

    let transform = app.world().get::<Transform>(entity).unwrap();
    assert!(
        transform.translation.y < 10.0,
        "Cylinder did not fall! Jolt physics body may not have been created."
    );
}

#[test]
fn test_user_data_maps_bodies_back_to_entities() {
    let mut app = physics_test_app();

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

    // First update initializes the clock; FixedUpdate (and the spawn system
    // inside it) runs once time advances past a fixed timestep.
    app.update();
    advance_time_safely(&mut app, std::time::Duration::from_millis(100));

    // bolt-bevy stores entity.to_bits() in Jolt's per-body payload; the
    // public core getter must return it so query hits can be mapped back
    // to entities (the contract raycasts will depend on).
    let jolt_body = app
        .world()
        .get::<JoltBody>(entity)
        .expect("JoltBody was not attached");
    let physics_world = app.world().resource::<PhysicsWorld>();

    assert_eq!(
        physics_world.user_data(jolt_body.handle()),
        Some(entity.to_bits()),
        "user_data must carry the entity's bits for query-to-entity mapping"
    );
}

#[test]
fn test_static_body_transform_stays_user_authoritative() {
    let mut app = physics_test_app();

    let entity = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 10.0, 0.0),
            RigidBody::Static,
            Collider::Box {
                half_extents: Vec3::splat(1.0),
            },
        ))
        .id();

    app.update();
    advance_time_safely(&mut app, Duration::from_secs(1));

    // Statics are excluded from transform sync: simulation must never
    // rewrite the user's Transform.
    assert_eq!(
        app.world().get::<Transform>(entity).unwrap().translation,
        Vec3::new(0.0, 10.0, 0.0)
    );

    // The user remains the authority: moving the Transform teleports the
    // Jolt body, and sync never fights back.
    {
        let mut transform = app.world_mut().get_mut::<Transform>(entity).unwrap();
        transform.translation = Vec3::new(5.0, 5.0, 5.0);
    }
    advance_time_safely(&mut app, Duration::from_millis(200));

    let jolt_body = app.world().get::<JoltBody>(entity).unwrap();
    let (position, _) = app
        .world()
        .resource::<PhysicsWorld>()
        .get_transform(jolt_body.handle())
        .expect("static body must still be live");
    assert!(
        position.distance(Vec3::new(5.0, 5.0, 5.0)) < 1e-3,
        "Jolt body must follow the user's teleport"
    );
    assert_eq!(
        app.world().get::<Transform>(entity).unwrap().translation,
        Vec3::new(5.0, 5.0, 5.0),
        "sync must not clobber a static's Transform"
    );
}

#[test]
fn test_gravity_change_applies_mid_simulation() {
    let mut app = physics_test_app();

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

    app.update();
    advance_time_safely(&mut app, Duration::from_millis(300));
    assert!(
        app.world().get::<Transform>(entity).unwrap().translation.y < 10.0,
        "body must be falling under default gravity"
    );

    // Freeze gravity mid-run: apply_gravity must notice the resource change
    // and push it into Jolt before the next step.
    app.world_mut().insert_resource(Gravity(Vec3::ZERO));

    let velocity_at_freeze = app
        .world()
        .get::<LinearVelocity>(entity)
        .expect("LinearVelocity is required on RigidBody")
        .y;

    advance_time_safely(&mut app, Duration::from_millis(500));

    let velocity_after = app.world().get::<LinearVelocity>(entity).unwrap().y;
    // Jolt's default linear damping (0.05/s) may bleed a little velocity
    // (~0.1 m/s over 0.5 s); continued gravity would change it by ~4.9 m/s.
    assert!(
        (velocity_after - velocity_at_freeze).abs() < 0.2,
        "vertical velocity must stop changing once gravity is zero (was {}, now {})",
        velocity_at_freeze,
        velocity_after
    );
}

#[test]
fn test_despawning_parent_destroys_child_physics_bodies() {
    let mut app = physics_test_app();

    let parent = app
        .world_mut()
        .spawn(Transform::from_xyz(10.0, 20.0, 30.0))
        .id();
    let child = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 5.0, 0.0),
            RigidBody::Dynamic,
            Collider::Box {
                half_extents: Vec3::splat(0.5),
            },
            ChildOf(parent),
        ))
        .id();

    app.update();
    advance_time_safely(&mut app, Duration::from_millis(100));

    let child_body = app
        .world()
        .get::<JoltBody>(child)
        .expect("child must have a physics body")
        .handle();

    // Despawning the parent recursively despawns the child; the observer
    // cascade must destroy the child's Jolt body with it.
    app.world_mut().despawn(parent);

    assert!(app.world().get_entity(child).is_err());
    assert!(
        app.world()
            .resource::<PhysicsWorld>()
            .get_transform(child_body)
            .is_none(),
        "child's Jolt body leaked: despawning the parent must destroy it"
    );
}

#[test]
fn test_physics_config_inserted_before_plugin_wins() {
    let mut app = App::new();
    app.add_plugins(MinimalPlugins);
    app.add_plugins(TransformPlugin);
    // User-provided config must beat the plugin's default.
    app.insert_resource(PhysicsConfig {
        max_bodies: 1,
        ..Default::default()
    });
    app.add_plugins(BoltPlugin::default());

    app.world_mut().spawn((
        Transform::from_xyz(0.0, 5.0, 0.0),
        RigidBody::Dynamic,
        Collider::Box {
            half_extents: Vec3::splat(0.5),
        },
    ));
    app.world_mut().spawn((
        Transform::from_xyz(5.0, 5.0, 0.0),
        RigidBody::Dynamic,
        Collider::Box {
            half_extents: Vec3::splat(0.5),
        },
    ));

    app.update();
    advance_time_safely(&mut app, Duration::from_millis(200));

    let with_bodies: Vec<Entity> = app
        .world_mut()
        .query_filtered::<Entity, With<JoltBody>>()
        .iter(app.world())
        .collect();
    assert_eq!(
        with_bodies.len(),
        1,
        "max_bodies = 1 from the user's PhysicsConfig must cap the world at one body"
    );
}

#[test]
fn test_velocity_write_on_dynamic_body_moves_it() {
    let mut app = physics_test_app();
    // Zero gravity isolates the velocity signal from acceleration.
    app.insert_resource(Gravity(Vec3::ZERO));

    let entity = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 10.0, 0.0),
            RigidBody::Dynamic,
            Collider::Box {
                half_extents: Vec3::splat(0.5),
            },
            LinearDamping(0.0),
        ))
        .id();

    app.update();

    // A component write on a dynamic body is an engine input, not just a
    // kinematic feature: apply_velocities must push it into Jolt.
    {
        let mut velocity = app.world_mut().get_mut::<LinearVelocity>(entity).unwrap();
        velocity.y = -5.0;
    }

    advance_time_safely(&mut app, Duration::from_millis(500));

    let transform = app.world().get::<Transform>(entity).unwrap();
    // Zero damping makes the travel analytic: -5 m/s * 0.5 s = -2.5 m
    // exactly — the fixed timestep is 1/64 s (binary-exact in f32) and
    // apply_velocities re-writes the velocity every tick, so no tolerance
    // is needed. A dropped velocity write leaves the body at y = 10.
    assert_eq!(
        transform.translation.y, 7.5,
        "zero damping must make travel analytic: -5 m/s * 0.5 s"
    );
    let velocity = app.world().get::<LinearVelocity>(entity).unwrap();
    // With zero damping the engine preserves the written velocity exactly;
    // any deviation is a bug, not numerical noise.
    assert_eq!(
        velocity.y, -5.0,
        "zero damping must preserve the velocity write bit-for-bit"
    );
}

#[test]
fn test_spawned_body_carries_default_damping() {
    let mut app = physics_test_app();

    // Spawned without explicit damping components.
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

    app.update();

    // RigidBody's #[require] injects the damping components at spawn, and
    // their explicit Default pins Jolt's own 0.05 — a derived Default would
    // be 0.0 and silently un-damp every existing scene. This is the ECS-side
    // guarantee behind the zero-behavior-change promise.
    let linear = app.world().get::<LinearDamping>(entity).unwrap();
    assert_eq!(**linear, 0.05, "linear damping must default to Jolt's 0.05");
    let angular = app.world().get::<AngularDamping>(entity).unwrap();
    assert_eq!(
        **angular, 0.05,
        "angular damping must default to Jolt's 0.05"
    );
}

#[test]
fn test_raycast_maps_hit_back_to_entity() {
    let mut app = physics_test_app();

    let floor = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 0.0, 0.0),
            RigidBody::Static,
            Collider::Box {
                half_extents: Vec3::splat(1.0),
            },
        ))
        .id();

    app.update();
    advance_time_safely(&mut app, std::time::Duration::from_millis(100));

    // Cast a 10 m ray straight down from 5 m above the floor's top face.
    let hit = app
        .world()
        .resource::<PhysicsWorld>()
        .ray_cast(Vec3::new(0.0, 5.0, 0.0), Vec3::new(0.0, -10.0, 0.0))
        .expect("ray must hit the floor");

    // The payload is the spawn-time entity's bits: the hit translates
    // back to exactly the entity that was struck.
    assert_eq!(
        Entity::from_bits(hit.user_data),
        floor,
        "raycast hit must map back to the struck entity"
    );
    // And the geometry traveled with it: 4 m down a 10 m ray.
    assert!((hit.fraction - 0.4).abs() < 1e-4);
    assert!(hit.hit_point.distance(Vec3::new(0.0, 1.0, 0.0)) < 1e-4);
}

#[test]
fn test_static_to_dynamic_transition_removes_static_marker_and_syncs() {
    let mut app = physics_test_app();

    // 1. Spawn a static box at Y = 10.0
    let entity = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 10.0, 0.0),
            RigidBody::Static,
            Collider::Box {
                half_extents: Vec3::splat(1.0),
            },
        ))
        .id();

    app.update();
    advance_time_safely(&mut app, Duration::from_millis(100));

    // Must have StaticMarker
    assert!(
        app.world()
            .get::<bolt_bevy::components::StaticMarker>(entity)
            .is_some()
    );
    assert_eq!(
        app.world().get::<Transform>(entity).unwrap().translation.y,
        10.0
    );

    // 2. Remove RigidBody and Collider, then replace with Dynamic
    app.world_mut()
        .entity_mut(entity)
        .remove::<RigidBody>()
        .remove::<Collider>();
    app.update();

    // StaticMarker must be cleaned up
    assert!(
        app.world()
            .get::<bolt_bevy::components::StaticMarker>(entity)
            .is_none()
    );

    // 3. Re-add as Dynamic
    app.world_mut().entity_mut(entity).insert((
        RigidBody::Dynamic,
        Collider::Box {
            half_extents: Vec3::splat(1.0),
        },
    ));

    app.update();
    advance_time_safely(&mut app, Duration::from_secs(1));

    // Dynamic body must now fall and sync transforms
    assert!(
        app.world().get::<Transform>(entity).unwrap().translation.y < 10.0,
        "Transitioned dynamic body did not fall! StaticMarker may have lingered."
    );
}

#[test]
fn test_mirrored_negative_scale_collider_spawns_and_falls() {
    let mut app = physics_test_app();

    // Spawn an entity with single-axis negative scale (mirrored on X: negative
    // determinant so GlobalTransform::to_scale_rotation_translation preserves
    // the negative scale component rather than folding a 2-axis reflection
    // into a 180-degree rotation).
    let entity = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 10.0, 0.0).with_scale(Vec3::new(-1.0, 1.0, 1.0)),
            RigidBody::Dynamic,
            Collider::Box {
                half_extents: Vec3::splat(0.5),
            },
        ))
        .id();

    app.update();
    advance_time_safely(&mut app, Duration::from_millis(500));

    // Body should spawn successfully and fall under gravity
    let jolt_body = app.world().get::<JoltBody>(entity);
    assert!(
        jolt_body.is_some(),
        "Mirrored body failed to spawn into Jolt"
    );
    assert!(
        app.world().get::<Transform>(entity).unwrap().translation.y < 10.0,
        "Mirrored body did not fall"
    );
}

#[test]
fn test_collider_mutation_preserves_jolt_velocity_without_velocity_components() {
    let mut app = physics_test_app();

    // No LinearVelocity/AngularVelocity components: Jolt is the only place
    // this body's velocity lives.
    let entity = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 50.0, 0.0),
            RigidBody::Dynamic,
            Collider::Box {
                half_extents: Vec3::splat(0.5),
            },
        ))
        .id();

    // Fall for 0.5 s to build up downward velocity.
    advance_time_safely(&mut app, Duration::from_millis(500));
    let y_before = app.world().get::<Transform>(entity).unwrap().translation.y;
    assert!(y_before < 50.0, "Body must have started falling");

    // Mutate the collider in place: the body is destroyed and respawned.
    *app.world_mut().get_mut::<Collider>(entity).unwrap() = Collider::Box {
        half_extents: Vec3::splat(0.6),
    };
    advance_time_safely(&mut app, Duration::from_millis(500));

    let y_after = app.world().get::<Transform>(entity).unwrap().translation.y;
    // A respawn that lost the Jolt-side velocity would freeze the body at
    // `y_before` (gravity only acts through velocity); it must keep falling.
    assert!(
        y_after < y_before - 1.0,
        "Respawn must preserve Jolt-side velocity: y_before = {y_before}, y_after = {y_after}"
    );
}

#[test]
fn test_failed_respawn_keeps_previous_body_simulating() {
    let mut app = physics_test_app();

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

    app.update();
    advance_time_safely(&mut app, Duration::from_millis(100));

    let original_handle = app
        .world()
        .get::<JoltBody>(entity)
        .expect("Body must have spawned")
        .handle();

    // Mutate to an invalid collider: the respawn must fail...
    *app.world_mut().get_mut::<Collider>(entity).unwrap() = Collider::Box {
        half_extents: Vec3::new(1.0, -1.0, 1.0),
    };
    advance_time_safely(&mut app, Duration::from_millis(300));

    // ...but the previous body survives and keeps simulating.
    let jolt_body = app
        .world()
        .get::<JoltBody>(entity)
        .expect("Previous JoltBody must be kept after a failed respawn");
    assert_eq!(
        jolt_body.handle(),
        original_handle,
        "A failed respawn must not replace or destroy the previous body"
    );
    assert!(
        app.world().get::<Transform>(entity).unwrap().translation.y < 10.0,
        "The previous body must keep falling after a failed respawn"
    );

    // Fixing the collider retries the respawn with a fresh handle.
    *app.world_mut().get_mut::<Collider>(entity).unwrap() = Collider::Box {
        half_extents: Vec3::splat(1.0),
    };
    advance_time_safely(&mut app, Duration::from_millis(300));
    assert_ne!(
        app.world().get::<JoltBody>(entity).unwrap().handle(),
        original_handle,
        "A valid later mutation must respawn a fresh body"
    );
}

#[test]
fn test_in_place_rigidbody_mutation_respawns_body() {
    let mut app = physics_test_app();

    // 1. Spawn a static box at Y = 10.0
    let entity = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 10.0, 0.0),
            RigidBody::Static,
            Collider::Box {
                half_extents: Vec3::splat(1.0),
            },
        ))
        .id();

    app.update();
    advance_time_safely(&mut app, Duration::from_millis(100));

    let original_handle = app
        .world()
        .get::<JoltBody>(entity)
        .expect("Static body must have a JoltBody")
        .handle();
    assert!(
        app.world()
            .get::<bolt_bevy::components::StaticMarker>(entity)
            .is_some()
    );

    // 2. Mutate RigidBody in-place (without removing it first)
    *app.world_mut().get_mut::<RigidBody>(entity).unwrap() = RigidBody::Dynamic;

    advance_time_safely(&mut app, Duration::from_secs(1));

    // Old handle must be destroyed (or replaced), StaticMarker removed, and body falling
    assert!(
        app.world()
            .get::<bolt_bevy::components::StaticMarker>(entity)
            .is_none(),
        "StaticMarker must be removed when RigidBody mutates to Dynamic"
    );
    assert!(
        app.world().get::<Transform>(entity).unwrap().translation.y < 10.0,
        "In-place transitioned dynamic body must fall under gravity"
    );
    let new_handle = app.world().get::<JoltBody>(entity).unwrap().handle();
    assert_ne!(
        original_handle, new_handle,
        "Respawned body must receive a fresh generation handle"
    );
}

#[test]
fn test_child_of_moving_parent_holds_world_position() {
    let mut app = physics_test_app();

    app.world_mut().insert_resource(Gravity(Vec3::ZERO));

    // Parent: a dynamic box drifting at 1 m/s in +x. The drift happens
    // *inside* FixedUpdate — exactly when sync_transforms reads the parent
    // frame — and zero gravity means nothing else moves it.
    let parent = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 10.0, 0.0),
            RigidBody::Dynamic,
            Collider::Box {
                half_extents: Vec3::splat(0.5),
            },
            LinearVelocity(Vec3::new(1.0, 0.0, 0.0)),
        ))
        .id();

    // Child: a dynamic box parented under it, offset 2 m in local x so the
    // colliders never overlap (overlapping dynamic boxes would be pushed
    // apart by the contact solver — legitimate physics that would poison
    // the measurement). It spawns at world (2, 10, 0); no forces act on
    // it, so Jolt must keep it there while the parent drifts away.
    let child = app
        .world_mut()
        .spawn((
            Transform::from_xyz(2.0, 0.0, 0.0),
            ChildOf(parent),
            RigidBody::Dynamic,
            Collider::Box {
                half_extents: Vec3::splat(0.25),
            },
        ))
        .id();

    advance_time_safely(&mut app, Duration::from_secs(1));

    // Precondition: the parent must actually have drifted, or this test
    // proves nothing. The naive damping integral predicts 0.975 m; observed
    // ~0.87 m = 0.975 × (~57.6/64) — the manual clock's zero-tick first
    // update costs ~10% of the ticks. The gate only needs "the parent moved
    // meaningfully every tick".
    let parent_global = app.world().get::<GlobalTransform>(parent).unwrap();
    assert!(
        parent_global.translation().x > 0.8,
        "precondition failed: parent barely moved (x = {}); \
         the stale-parent bug cannot trigger without parent motion",
        parent_global.translation().x
    );

    // The claim: a force-free body holds its world position while its
    // parent moves. With the stale-parent-frame bug, each tick's local
    // conversion is wrong by the parent's per-tick motion and
    // apply_user_transforms feeds that error back into Jolt, so the child
    // is dragged along with the parent.
    let child_global = app.world().get::<GlobalTransform>(child).unwrap();
    assert!(
        child_global
            .translation()
            .distance(Vec3::new(2.0, 10.0, 0.0))
            < 1e-3,
        "child drifted with its parent: world position {} vs spawn (2, 10, 0)",
        child_global.translation()
    );
}

/// Test-only marker: a plain (non-physics) entity moved by user "script"
/// each frame, to exercise sync's plain-hierarchy ancestor walk.
#[derive(Component)]
struct MovingPlainParent;

/// Moves inside FixedUpdate (before Bolt's sets) so the motion lands in
/// the same tick sync composes against. Moving it in `Update` instead
/// leaves the child's ECS mirror one frame behind (sync composes the
/// local before Update moves the parent; propagation composes it after)
/// — constant one-step latency, not drift; physics stays correct.
fn move_plain_parent_left(
    time: Res<Time<Fixed>>,
    mut parents: Query<&mut Transform, With<MovingPlainParent>>,
) {
    for mut transform in &mut parents {
        transform.translation.x -= time.delta_secs();
    }
}

#[test]
fn test_child_of_rotated_static_parent_holds_world_position() {
    let mut app = physics_test_app();

    app.world_mut().insert_resource(Gravity(Vec3::ZERO));

    // Static parent box rotated +90° about Y: the child's local +X points
    // at world -Z. Every existing hierarchy test is axis-aligned; this one
    // exercises the full rotation round-trip — spawn local → world → Jolt →
    // sync's inverse-affine conversion → propagation → world again.
    let parent = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 10.0, 0.0)
                .with_rotation(Quat::from_rotation_y(std::f32::consts::FRAC_PI_2)),
            RigidBody::Static,
            Collider::Box {
                half_extents: Vec3::splat(0.5),
            },
        ))
        .id();

    let child = app
        .world_mut()
        .spawn((
            Transform::from_xyz(2.0, 0.0, 0.0),
            ChildOf(parent),
            RigidBody::Dynamic,
            Collider::Box {
                half_extents: Vec3::splat(0.25),
            },
        ))
        .id();

    advance_time_safely(&mut app, Duration::from_secs(1));

    // Precondition: the child spawned where the rotated frame puts it —
    // (0, 10, -2), not the naive (2, 10, 0).
    // Claim: it stays there (nothing acts on it; the parent never moves).
    let child_global = app.world().get::<GlobalTransform>(child).unwrap();
    assert!(
        child_global
            .translation()
            .distance(Vec3::new(0.0, 10.0, -2.0))
            < 1e-3,
        "child did not hold its rotated-frame world position: {}",
        child_global.translation()
    );
}

#[test]
fn test_child_of_moving_plain_parent_holds_world_position() {
    let mut app = physics_test_app();

    app.world_mut().insert_resource(Gravity(Vec3::ZERO));

    // A NON-physics parent (no RigidBody/Collider) moved by a script each
    // frame — the plain-hierarchy branch of compute_world_affine, which no
    // other pipeline test exercises. The child is a physics body: physics
    // owns its world position, so it must stay put while the parent recedes
    // and only its LOCAL spelling changes. The run is 1 s, so the child
    // also falls asleep at the 0.5 s mark mid-test — sleeping sync with a
    // moving parent frame, exercised together.
    let parent = app
        .world_mut()
        .spawn((Transform::from_xyz(0.0, 10.0, 0.0), MovingPlainParent))
        .id();

    let child = app
        .world_mut()
        .spawn((
            Transform::from_xyz(2.0, 0.0, 0.0),
            ChildOf(parent),
            RigidBody::Dynamic,
            Collider::Box {
                half_extents: Vec3::splat(0.25),
            },
        ))
        .id();

    // Warm-up BEFORE the script attaches: the child's authoritative world
    // pose is locked in when Bolt's spawn system first processes it. If the
    // parent's script has already run by then, the initial pose inherits
    // that moved frame (a one-tick-earlier initial condition — not drift,
    // not lag; physics holds whatever pose it spawned with forever). Pump
    // updates with the parent stationary so the locked pose is exactly
    // (2, 10, 0), then start the script. 200 ms guarantees at least one
    // fixed run even though a manual clock's very first update can run
    // zero.
    advance_time_safely(&mut app, Duration::from_millis(200));

    app.add_systems(
        FixedUpdate,
        move_plain_parent_left.before(bolt_bevy::systems::PhysicsSet::Spawn),
    );

    advance_time_safely(&mut app, Duration::from_secs(1));

    // Precondition: the plain parent actually receded. The warm-up above
    // absorbed the zero-tick first update, so the scripted second runs at
    // full tick count (~1 m); 0.8 stays a floor against jitter.
    let parent_x = app.world().get::<Transform>(parent).unwrap().translation.x;
    assert!(
        parent_x < -0.8,
        "precondition failed: plain parent barely moved (x = {parent_x})"
    );

    // The claim: the child's world position is physics-owned — held to the
    // millimeter for the full second while the plain parent recedes, the
    // ECS mirror tracking Jolt exactly (probe-verified: ecs_global ==
    // Jolt pose every update; only the local spelling changes).
    let child_global = app.world().get::<GlobalTransform>(child).unwrap();
    assert!(
        child_global
            .translation()
            .distance(Vec3::new(2.0, 10.0, 0.0))
            < 1e-3,
        "child drifted with its plain parent: {}",
        child_global.translation()
    );

    // …while its LOCAL spelling grew by the parent's travel (~3.0 = 2.0
    // local + 1.0 m of parent recession), proving the local re-expression
    // keeps running through the walk and through sleep.
    let child_local_x = app.world().get::<Transform>(child).unwrap().translation.x;
    let expected_local_x = 2.0 - parent_x;
    assert!(
        child_local_x > expected_local_x - 0.1,
        "child local not re-expressed against the receding parent: \
         {child_local_x} (expected ≈ {expected_local_x})"
    );
}

#[test]
fn test_child_of_rotated_moving_parent_holds_world_position() {
    let mut app = physics_test_app();

    app.world_mut().insert_resource(Gravity(Vec3::ZERO));

    // A dynamic parent rotated +90° about Y, drifting at 1 m/s in +x. The
    // child's local offset (2, 0, 0) therefore points at world −z, and the
    // parent's travel is perpendicular to that offset, so the colliders
    // never meet. Unlike the rotated-static variant, the parent frame
    // TRANSLATES every tick, forcing sync's world→local inverse-affine
    // write path through a rotation on every tick — the static variant
    // only ever exercises the computation (epsilon skips the writes).
    let parent = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 10.0, 0.0)
                .with_rotation(Quat::from_rotation_y(std::f32::consts::FRAC_PI_2)),
            RigidBody::Dynamic,
            Collider::Box {
                half_extents: Vec3::splat(0.5),
            },
            LinearVelocity(Vec3::new(1.0, 0.0, 0.0)),
        ))
        .id();

    let child = app
        .world_mut()
        .spawn((
            Transform::from_xyz(2.0, 0.0, 0.0),
            ChildOf(parent),
            RigidBody::Dynamic,
            Collider::Box {
                half_extents: Vec3::splat(0.25),
            },
        ))
        .id();

    advance_time_safely(&mut app, Duration::from_secs(1));

    // Precondition: the parent drifted meaningfully under its default
    // damping (observed ~0.87 m; naive integral says 0.975).
    let parent_global = app.world().get::<GlobalTransform>(parent).unwrap();
    assert!(
        parent_global.translation().x > 0.8,
        "precondition failed: parent barely moved (x = {})",
        parent_global.translation().x
    );

    // The claim: the child holds its rotated-frame spawn world position
    // (0, 10, -2) to the millimeter while the rotated frame translates.
    let child_global = app.world().get::<GlobalTransform>(child).unwrap();
    assert!(
        child_global
            .translation()
            .distance(Vec3::new(0.0, 10.0, -2.0))
            < 1e-3,
        "child did not hold its rotated-frame world position: {}",
        child_global.translation()
    );
}

#[test]
fn test_child_of_scaled_parent_holds_world_position() {
    let mut app = physics_test_app();

    app.world_mut().insert_resource(Gravity(Vec3::ZERO));

    // A parent with ECS scale (2, 1, 1): spawn bakes the scale into the
    // child's world offset (local (2, 0, 0) → world offset 4), and sync's
    // parent frame must carry the same scale or the local conversion
    // divides by the wrong frame and propagation diverges permanently.
    let parent = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 10.0, 0.0).with_scale(Vec3::new(2.0, 1.0, 1.0)),
            RigidBody::Static,
            Collider::Box {
                half_extents: Vec3::splat(0.25),
            },
        ))
        .id();

    let child = app
        .world_mut()
        .spawn((
            Transform::from_xyz(2.0, 0.0, 0.0),
            ChildOf(parent),
            RigidBody::Dynamic,
            Collider::Box {
                half_extents: Vec3::splat(0.25),
            },
        ))
        .id();

    advance_time_safely(&mut app, Duration::from_secs(1));

    // The claim: the child holds its scale-baked spawn world position
    // (4, 10, 0), and its LOCAL spelling stays at 2.0 — parent_scale ×
    // local == world offset, exactly what propagation recomputes.
    let child_global = app.world().get::<GlobalTransform>(child).unwrap();
    assert!(
        child_global
            .translation()
            .distance(Vec3::new(4.0, 10.0, 0.0))
            < 1e-3,
        "scaled-parent frame diverged: {}",
        child_global.translation()
    );
    let child_local_x = app.world().get::<Transform>(child).unwrap().translation.x;
    assert!(
        (child_local_x - 2.0).abs() < 1e-3,
        "local spelling wrong for scaled parent: {child_local_x}"
    );
}

/// Test-only marker: a kinematic platform moved by user "script" through
/// Transform writes — the authoring path #4's remaining item targets.
#[derive(Component)]
struct SlidingPlatform;

fn slide_platform(
    time: Res<Time<Fixed>>,
    mut platforms: Query<&mut Transform, With<SlidingPlatform>>,
) {
    for mut transform in &mut platforms {
        transform.translation.x += 1.5 * time.delta_secs();
    }
}

#[test]
fn test_transform_driven_kinematic_platform_carries_dynamic_rider() {
    let mut app = physics_test_app();

    // Gravity stays ON: it presses the crate into the platform, which is
    // what makes friction — the only thing that can carry it — act.
    //
    // The scene is deliberately HORIZONTAL. A rising platform can fake a
    // pass even with teleport semantics: each tick's teleport pushes the
    // platform into the crate and Jolt's depenetration shoves it upward —
    // carry by artifact. Sideways motion carries a rider ONLY through
    // friction, and friction only acts on velocity: a sideways teleport
    // exerts no horizontal force at all, so pre-fix the crate has no
    // mechanism to move. The crate is NOT parented — contact is the story.
    let platform = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 0.0, 0.0),
            RigidBody::Kinematic,
            Collider::Box {
                half_extents: Vec3::new(1.5, 0.1, 1.5),
            },
            SlidingPlatform,
        ))
        .id();
    let crate_entity = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 0.6, 0.0), // bottom face on the platform's top (0.1)
            RigidBody::Dynamic,
            Collider::Box {
                half_extents: Vec3::splat(0.5),
            },
        ))
        .id();

    // No warm-up: the claims are differential (crate vs platform), so a
    // one-tick initial shift from the spawn-frame subtlety is invisible.
    app.add_systems(
        FixedUpdate,
        slide_platform.before(bolt_bevy::systems::PhysicsSet::Spawn),
    );

    advance_time_safely(&mut app, Duration::from_secs(1));

    // Precondition: the platform actually slid. 1.5 m/s for ~57.6/64 ticks
    // ≈ 1.35 m; gate at 1.0.
    let platform_x = app
        .world()
        .get::<Transform>(platform)
        .unwrap()
        .translation
        .x;
    assert!(
        platform_x > 1.0,
        "precondition failed: platform barely moved (x = {platform_x})"
    );

    // The claim, with friction arithmetic derived first (Jolt default
    // μ = 0.2): crate accel = μg ≈ 1.96 m/s², reaches platform speed at
    // t ≈ 0.77 s after ≈ 0.57 m, then rides: ≈ 0.93 m after 1 s. The crate
    // legitimately lags the platform by ~0.4 m — friction-limited, correct
    // physics — so the gate sits between the two worlds: far above the
    // pre-fix crate at ~0 (no velocity, no friction, nothing to move it),
    // safely below 0.93.
    let crate_x = app
        .world()
        .get::<GlobalTransform>(crate_entity)
        .unwrap()
        .translation()
        .x;
    assert!(
        crate_x > 0.5,
        "crate was not carried: x = {crate_x} while platform reached {platform_x} \
         (pre-fix expectation ≈ 0.0 — the Coyote test)"
    );

    // Invariant: friction can accelerate but never let the crate lead.
    assert!(
        crate_x < platform_x,
        "crate leads the platform ({crate_x} > {platform_x})?"
    );

    // Second signal: sync writes Jolt's velocity back into the component —
    // a carried crate is moving with the platform, not standing still.
    let crate_velocity_x = app.world().get::<LinearVelocity>(crate_entity).unwrap().x;
    assert!(
        crate_velocity_x > 1.0,
        "crate not moving with the platform: velocity.x = {crate_velocity_x}"
    );
}

#[derive(Component)]
struct StoppingPlatform;

fn slide_then_stop(
    time: Res<Time<Fixed>>,
    mut elapsed: Local<f32>,
    mut platforms: Query<&mut Transform, With<StoppingPlatform>>,
) {
    // Scripted for the first half second of simulated time, then hands off.
    // `Local` accumulates fixed-tick time, so "stop" is in simulated time,
    // immune to wall-clock jitter.
    if *elapsed >= 0.5 {
        return;
    }
    *elapsed += time.delta_secs();
    for mut transform in &mut platforms {
        transform.translation.x += 1.5 * time.delta_secs();
    }
}

#[test]
fn test_transform_driven_platform_halts_when_script_stops() {
    let mut app = physics_test_app();

    let platform = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 0.0, 0.0),
            RigidBody::Kinematic,
            Collider::Box {
                half_extents: Vec3::new(1.5, 0.1, 1.5),
            },
            StoppingPlatform,
        ))
        .id();

    app.add_systems(
        FixedUpdate,
        slide_then_stop.before(bolt_bevy::systems::PhysicsSet::Spawn),
    );

    // One second of updates covers ~0.9 s of ticks; the script stops at its
    // own 0.5 s mark, strictly inside this window — so the first read is
    // already the parked position (stopping at 1.0 s instead would land the
    // halt early in the SECOND window and read legitimate motion as drift).
    advance_time_safely(&mut app, Duration::from_secs(1));

    let x_parked = app
        .world()
        .get::<Transform>(platform)
        .unwrap()
        .translation
        .x;
    let v_parked = app.world().get::<LinearVelocity>(platform).unwrap().x;
    assert!(
        x_parked > 0.5,
        "precondition: platform slid (x = {x_parked})"
    );
    assert!(
        v_parked.abs() < 0.05,
        "velocity must decay to ~0 once the script stops (got {v_parked})"
    );

    // A full second of silence: the platform must not drift (the per-tick
    // re-issue decays the velocity rather than letting it persist).
    advance_time_safely(&mut app, Duration::from_secs(1));

    let x_final = app
        .world()
        .get::<Transform>(platform)
        .unwrap()
        .translation
        .x;
    assert!(
        (x_final - x_parked).abs() < 0.05,
        "platform drifted after its script stopped: {x_final} vs {x_parked}"
    );
}
