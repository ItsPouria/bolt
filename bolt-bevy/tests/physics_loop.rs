use bevy::prelude::*;
use bolt_bevy::prelude::*;
use std::time::Duration;

#[test]
fn test_gravity_pulls_dynamic_bodies() {
    let mut app = App::new();
    app.add_plugins(MinimalPlugins);
    app.add_plugins(BoltPlugin::default());

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

    app.world_mut()
        .insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(
            Duration::from_secs(1),
        ));

    app.update();

    let transform = app.world().get::<Transform>(entity).unwrap();

    assert!(
        transform.translation.y < 10.0,
        "Box did not fall! Y is still {}",
        transform.translation.y
    );
}

#[test]
fn test_fixed_timestep_prevents_micro_updates() {
    let mut app = App::new();
    app.add_plugins(MinimalPlugins);
    app.add_plugins(bolt_bevy::plugin::BoltPlugin::default());

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
    assert_eq!(
        transform.translation.y, start_y,
        "The box moved! Physics is running on variable Update instead of FixedUpdate!"
    );
}

#[test]
fn test_despawn_cleans_up_physics_body() {
    let mut app = App::new();
    app.add_plugins(MinimalPlugins);
    app.add_plugins(bolt_bevy::plugin::BoltPlugin::default());

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

    // 3. Verify JoltBody component exists and is live in Jolt
    let jolt_body = app
        .world()
        .get::<bolt_bevy::prelude::JoltBody>(entity)
        .copied()
        .expect("JoltBody was not attached to entity!");
    let body_id = jolt_body.0;

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
    let mut app = App::new();
    app.add_plugins(MinimalPlugins);
    app.add_plugins(TransformPlugin);
    app.add_plugins(bolt_bevy::plugin::BoltPlugin::default());

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

    let jolt_body = app
        .world()
        .get::<bolt_bevy::prelude::JoltBody>(child)
        .copied()
        .expect("JoltBody component missing on child");

    let physics_world = app.world().resource::<bolt_bevy::prelude::PhysicsWorld>();
    let (pos, _) = physics_world
        .get_transform(jolt_body.0)
        .expect("Body transform not found");

    // Child world pos should be (10.0, 25.0, 30.0), not local (0.0, 5.0, 0.0)
    assert_eq!(pos, Vec3::new(10.0, 25.0, 30.0));
}

#[test]
fn test_child_entity_dynamic_sync_transforms() {
    let mut app = App::new();
    app.add_plugins(MinimalPlugins);
    app.add_plugins(TransformPlugin);
    app.add_plugins(bolt_bevy::plugin::BoltPlugin::default());

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
    app.world_mut()
        .insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(
            Duration::from_secs(1),
        ));

    app.update();

    let child_transform = app.world().get::<Transform>(child).unwrap();

    // The child fell below the parent, so its local Y translation relative to parent must be negative
    assert!(
        child_transform.translation.y < 0.0,
        "Child local Y translation did not sync relative to parent! Y is {}",
        child_transform.translation.y
    );
}

