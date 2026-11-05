//! The push family's playable demo: a crate that jumps on command.
//!
//! Run with `--release` (debug Bevy is 10-50x slower and will lie about
//! performance). Space jumps — each press is one `ExternalImpulse` write,
//! and a fresh kick is a fresh write.

use bevy::prelude::*;
use bolt_bevy::prelude::*;
use bolt_examples::ExampleStatsPlugin;

/// The kick, in newton-seconds. The crate is a 1 m cube = 1 m³ = 1000 kg
/// at Jolt's default density (1000 kg/m³), so 4000 N·s launches it at
/// 4 m/s — roughly 0.8 m of jump height per press (v²/2g).
const JUMP_IMPULSE: f32 = 4000.0;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(BoltPlugin::default())
        .add_plugins(ExampleStatsPlugin)
        .add_systems(Startup, setup)
        .add_systems(Update, jump_on_space)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 6.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Light
    commands.spawn((
        DirectionalLight {
            illuminance: 5000.0,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 4.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Floor (Static)
    let floor_size = Vec3::new(20.0, 0.1, 20.0);
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::from_size(floor_size))),
        MeshMaterial3d(materials.add(Color::srgb(0.3, 0.3, 0.35))),
        Transform::from_xyz(0.0, -0.05, 0.0), // top face at y = 0
        RigidBody::Static,
        Collider::Box {
            half_extents: floor_size / 2.0,
        },
    ));

    // The jumping crate (Dynamic). The ExternalImpulse component is
    // present from spawn at zero — a zero impulse does nothing (it
    // doesn't even wake a sleeper), and the jump system writes into it
    // on demand.
    let crate_size = Vec3::splat(1.0);
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::from_size(crate_size))),
        MeshMaterial3d(materials.add(Color::srgb(0.8, 0.55, 0.2))),
        Transform::from_xyz(0.0, 0.5, 0.0), // resting on the floor
        RigidBody::Dynamic,
        Collider::Box {
            half_extents: crate_size / 2.0,
        },
        ExternalImpulse(Vec3::ZERO),
    ));
}

fn jump_on_space(keys: Res<ButtonInput<KeyCode>>, mut query: Query<&mut ExternalImpulse>) {
    // just_pressed, not pressed: holding space must not machine-gun —
    // one press, one kick, exactly the one-shot contract.
    if keys.just_pressed(KeyCode::Space) {
        for mut impulse in &mut query {
            impulse.0 = Vec3::new(0.0, JUMP_IMPULSE, 0.0);
        }
    }
}
