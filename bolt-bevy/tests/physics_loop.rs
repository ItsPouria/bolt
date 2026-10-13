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
    // Ideal travel is -5 m/s * 0.5 s = -2.5 m; Jolt's default linear damping
    // (0.05/s) trims it slightly (~-2.47 m). Tolerance keeps the test
    // meaningful: a dropped velocity write leaves the body at y = 10.
    assert!(
        (transform.translation.y - 7.5).abs() < 0.1,
        "expected ~-2.5 m of travel from the velocity write, got y = {}",
        transform.translation.y
    );
    let velocity = app.world().get::<LinearVelocity>(entity).unwrap();
    assert!(
        (velocity.y - (-5.0)).abs() < 0.2,
        "velocity must be preserved up to linear damping"
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
