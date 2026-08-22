use bevy::prelude::*;
use bolt_bevy::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins) // Upgrade to a real window!
        .add_plugins(BoltPlugin::default())
        .add_systems(Startup, setup)
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

    // 2. Light (Directional is much easier to see than Point in Bevy 0.15+)
    commands.spawn((
        DirectionalLight {
            illuminance: 5000.0,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 4.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // 3. The Floor (Static)
    let floor_size = Vec3::new(10.0, 0.1, 10.0);
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::from_size(floor_size))),
        MeshMaterial3d(materials.add(Color::srgb(0.3, 0.5, 0.3))),
        Transform::from_xyz(0.0, -1.0, 0.0),
        RigidBody::Static,
        Collider::Box { half_extents: floor_size / 2.0 },
    ));

    // 4. The Falling Box (Dynamic)
    let box_size = Vec3::splat(1.0);
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::from_size(box_size))),
        MeshMaterial3d(materials.add(Color::srgb(0.8, 0.2, 0.2))),
        Transform::from_xyz(0.0, 5.0, 0.0),
        RigidBody::Dynamic,
        Collider::Box { half_extents: box_size / 2.0 },
    ));
}
