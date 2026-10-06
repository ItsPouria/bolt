use bevy::prelude::*;
use bolt_bevy::prelude::*;
use bolt_examples::ExampleStatsPlugin;

#[derive(Resource)]
struct SpawnTimer(Timer);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(BoltPlugin {
            config: bolt_bevy::config::PhysicsConfig {
                collision_steps: 4,
                ..Default::default()
            },
        })
        .add_plugins(ExampleStatsPlugin)
        .insert_resource(SpawnTimer(Timer::from_seconds(2.0, TimerMode::Repeating)))
        .add_systems(Startup, setup)
        .add_systems(Update, spawn_random_shapes)
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
        Collider::Box {
            half_extents: floor_size / 2.0,
        },
    ));

    // 4. The Initial Falling Sphere (Dynamic)
    let radius = 0.5;
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(radius))),
        MeshMaterial3d(materials.add(Color::srgb(0.8, 0.2, 0.2))),
        Transform::from_xyz(0.0, 5.0, 0.0),
        RigidBody::Dynamic,
        Collider::Sphere { radius },
        bolt_bevy::components::ContinuousCollision,
    ));
}

fn spawn_random_shapes(
    mut commands: Commands,
    time: Res<Time>,
    mut timer: ResMut<SpawnTimer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    if timer.0.tick(time.delta()).just_finished() {
        let x = (rand::random::<f32>() - 0.5) * 8.0;
        let z = (rand::random::<f32>() - 0.5) * 8.0;
        let y = rand::random::<f32>() * 5.0 + 5.0;

        let r = rand::random::<f32>();
        let g = rand::random::<f32>();
        let b = rand::random::<f32>();

        let shape_type = rand::random::<u32>() % 4;
        let transform = Transform::from_xyz(x, y, z);
        let material = MeshMaterial3d(materials.add(Color::srgb(r, g, b)));

        match shape_type {
            0 => {
                let box_size = Vec3::splat(1.0);
                commands.spawn((
                    Mesh3d(meshes.add(Cuboid::from_size(box_size))),
                    material,
                    transform,
                    RigidBody::Dynamic,
                    Collider::Box {
                        half_extents: box_size / 2.0,
                    },
                    bolt_bevy::components::ContinuousCollision,
                ));
            }
            1 => {
                let radius = 0.5;
                commands.spawn((
                    Mesh3d(meshes.add(Sphere::new(radius))),
                    material,
                    transform,
                    RigidBody::Dynamic,
                    Collider::Sphere { radius },
                    bolt_bevy::components::ContinuousCollision,
                ));
            }
            2 => {
                let radius = 0.5;
                let half_height = 0.5;
                commands.spawn((
                    Mesh3d(meshes.add(Capsule3d::new(radius, half_height * 2.0))),
                    material,
                    transform,
                    RigidBody::Dynamic,
                    Collider::Capsule {
                        half_height,
                        radius,
                    },
                    bolt_bevy::components::ContinuousCollision,
                ));
            }
            _ => {
                let radius = 0.5;
                let half_height = 0.5;
                commands.spawn((
                    Mesh3d(meshes.add(Cylinder::new(radius, half_height * 2.0))),
                    material,
                    transform,
                    RigidBody::Dynamic,
                    Collider::Cylinder {
                        half_height,
                        radius,
                    },
                    bolt_bevy::components::ContinuousCollision,
                ));
            }
        }
    }
}
