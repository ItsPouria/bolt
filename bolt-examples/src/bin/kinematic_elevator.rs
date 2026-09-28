use bevy::prelude::*;
use bolt_bevy::prelude::*;
use bolt_examples::ExampleStatsPlugin;

#[derive(Component)]
struct Elevator;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(BoltPlugin::default())
        .add_plugins(ExampleStatsPlugin)
        .add_systems(Startup, setup)
        .add_systems(Update, move_elevator)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // 1. Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 8.0, 15.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // 2. Light
    commands.spawn((
        DirectionalLight {
            illuminance: 5000.0,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 4.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // 3. The Floor
    let floor_size = Vec3::new(10.0, 0.1, 10.0);
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::from_size(floor_size))),
        MeshMaterial3d(materials.add(Color::srgb(0.3, 0.5, 0.3))),
        Transform::from_xyz(0.0, -1.0, 0.0),
        RigidBody::Static,
        Collider::Box {
            half_extents: floor_size / 2.0,
        },
    ));

    // 4. The Elevator
    let elevator_size = Vec3::new(3.0, 0.2, 3.0);
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::from_size(elevator_size))),
        MeshMaterial3d(materials.add(Color::srgb(0.2, 0.2, 0.8))),
        Transform::from_xyz(0.0, 0.0, 0.0),
        RigidBody::Kinematic,
        Collider::Box {
            half_extents: elevator_size / 2.0,
        },
        LinearVelocity(Vec3::new(0.0, 2.0, 0.0)),
        Elevator,
    ));

    // 5. The Rider Crate
    let crate_size = Vec3::splat(1.0);
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::from_size(crate_size))),
        MeshMaterial3d(materials.add(Color::srgb(0.8, 0.8, 0.2))),
        Transform::from_xyz(0.0, 1.0, 0.0),
        RigidBody::Dynamic,
        Collider::Box {
            half_extents: crate_size / 2.0,
        },
    ));
}

fn move_elevator(mut query: Query<(&mut LinearVelocity, &Transform), With<Elevator>>) {
    for (mut velocity, transform) in query.iter_mut() {
        if transform.translation.y > 5.0 {
            velocity.y = -2.0;
        } else if transform.translation.y < 0.0 {
            velocity.y = 2.0;
        }
    }
}
