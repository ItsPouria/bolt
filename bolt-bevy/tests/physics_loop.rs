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
        .resource_mut::<Time>()
        .advance_by(Duration::from_secs(1));

    app.update();

    let transform = app.world().get::<Transform>(entity).unwrap();

    assert!(
        transform.translation.y < 10.0,
        "Box did not fall! Y is still {}",
        transform.translation.y
    );
}
