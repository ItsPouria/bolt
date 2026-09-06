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

    app.world_mut().insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(Duration::from_secs(1)));

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
    let entity = app.world_mut().spawn((
        Transform::from_xyz(0.0, start_y, 0.0),
        RigidBody::Dynamic,
        Collider::Box {
            half_extents: Vec3::splat(1.0),
        },
    )).id();

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
