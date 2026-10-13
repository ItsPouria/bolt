//! Interactive raycast demo: a sphere resting on a plane fires a visible ray
//! down a corridor while two kinematic crates slide across it in opposition.
//!
//! Watch the HUD text: it names the crate the ray currently hits (and how far
//! away it is) or reports a miss while both crates sit at opposite extremes of
//! their lanes. Green ray with an impact marker = hit; gray ray = miss.
//!
//! The crates run at different depths and speeds from opposite starting
//! phases, so over time every case appears: only crate A in the corridor,
//! only crate B, both at once (the nearer one wins — closest-hit), and
//! neither (the miss the demo exists to show).
//!
//! Run with `--release` (debug Bevy is 10-50x slower):
//!   cargo run --release -p bolt-examples --bin raycast_demo

use bevy::gizmos::prelude::*;
use bevy::prelude::*;
use bolt_bevy::prelude::*;
use bolt_examples::ExampleStatsPlugin;

/// The dynamic sphere the ray fires from.
#[derive(Component)]
struct RaySource;

/// A sliding crate; the payload names it in the HUD.
#[derive(Component)]
struct Crate(&'static str);

/// Top-left HUD text reporting the latest ray result.
#[derive(Component)]
struct HudText;

/// Total ray length in meters. The direction vector's magnitude IS the ray's
/// maximum distance — that is the raycast convention this demo exercises.
const RAY_LENGTH: f32 = 30.0;
/// How far the crates slide left/right before reversing.
const X_BOUND: f32 = 8.0;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(BoltPlugin::default())
        .add_plugins(ExampleStatsPlugin)
        .add_systems(Startup, setup)
        .add_systems(Update, (move_crates, cast_and_draw_ray).chain())
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // 1. Camera: an elevated side view of the corridor, sphere near, crates far.
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(16.0, 9.0, -2.0).looking_at(Vec3::new(0.0, 0.5, 8.0), Vec3::Y),
    ));

    // 2. Light
    commands.spawn((
        DirectionalLight {
            illuminance: 5000.0,
            ..default()
        },
        Transform::from_xyz(4.0, 12.0, 4.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // 3. The plane: a wide, thin static box whose top surface sits at y = 0.
    let plane_size = Vec3::new(40.0, 0.1, 40.0);
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::from_size(plane_size))),
        MeshMaterial3d(materials.add(Color::srgb(0.25, 0.3, 0.28))),
        Transform::from_xyz(0.0, -0.05, 0.0),
        RigidBody::Static,
        Collider::Box {
            half_extents: plane_size / 2.0,
        },
    ));

    // 4. The ray source: a dynamic sphere resting on the plane.
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(0.5))),
        MeshMaterial3d(materials.add(Color::srgb(0.85, 0.75, 0.2))),
        Transform::from_xyz(0.0, 0.5, 0.0),
        RigidBody::Dynamic,
        Collider::Sphere { radius: 0.5 },
        RaySource,
    ));

    // 5. Two kinematic crates patrolling the corridor. Crate A is nearer
    //    (z = 6) and slower; crate B is farther (z = 10) and faster. They
    //    start at opposite extremes, so the demo opens on a miss.
    let crate_mesh = meshes.add(Cuboid::from_size(Vec3::splat(1.0)));
    commands.spawn((
        Mesh3d(crate_mesh.clone()),
        MeshMaterial3d(materials.add(Color::srgb(0.8, 0.4, 0.2))),
        Transform::from_xyz(-X_BOUND, 0.5, 6.0),
        RigidBody::Kinematic,
        Collider::Box {
            half_extents: Vec3::splat(0.5),
        },
        LinearVelocity(Vec3::new(2.0, 0.0, 0.0)),
        Crate("A"),
    ));
    commands.spawn((
        Mesh3d(crate_mesh),
        MeshMaterial3d(materials.add(Color::srgb(0.25, 0.5, 0.85))),
        Transform::from_xyz(X_BOUND, 0.5, 10.0),
        RigidBody::Kinematic,
        Collider::Box {
            half_extents: Vec3::splat(0.5),
        },
        LinearVelocity(Vec3::new(-3.0, 0.0, 0.0)),
        Crate("B"),
    ));

    // 6. HUD text, top-left.
    commands.spawn((
        Text::new("waiting for first cast…"),
        TextFont {
            font_size: FontSize::Px(22.0),
            ..default()
        },
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(12.0),
            ..default()
        },
        HudText,
    ));
}

/// Ping-pong the crates: reverse horizontal velocity at the lane bounds.
/// Kinematic bodies are driven by velocity writes, exactly like the elevator
/// demo — Bolt's change detection carries the writes into Jolt.
fn move_crates(mut crates: Query<(&mut LinearVelocity, &Transform), With<Crate>>) {
    for (mut velocity, transform) in &mut crates {
        if (transform.translation.x > X_BOUND && velocity.x > 0.0)
            || (transform.translation.x < -X_BOUND && velocity.x < 0.0)
        {
            velocity.x = -velocity.x;
        }
    }
}

/// Cast the ray every frame, draw it, and report who (if anyone) was struck.
fn cast_and_draw_ray(
    sphere: Query<&Transform, With<RaySource>>,
    crates: Query<&Crate>,
    physics: Res<PhysicsWorld>,
    mut gizmos: Gizmos,
    mut hud: Query<&mut Text, With<HudText>>,
) {
    let (Ok(sphere), Ok(mut hud)) = (sphere.single(), hud.single_mut()) else {
        return;
    };

    // Start beyond the sphere's surface with clearance: an origin exactly on
    // (or inside) the source body reports that body itself at fraction 0,
    // shadowing everything downrange. Engines normally exclude the shooter
    // with a body filter — deferred capability, tracked in issue #9 — so the
    // demo simply steps the origin past the source.
    let origin = sphere.translation + Vec3::new(0.0, 0.0, 0.75);
    let direction = Vec3::new(0.0, 0.0, RAY_LENGTH);

    let message = match physics.ray_cast(origin, direction) {
        Some(hit) => {
            // The payload is the struck entity's bits — the round trip that
            // turns a physics hit back into an ECS entity.
            let entity = Entity::from_bits(hit.user_data);
            let who = crates
                .get(entity)
                .map(|crate_marker| crate_marker.0)
                .unwrap_or("something (no Crate component)");
            let distance = hit.fraction * RAY_LENGTH;

            gizmos.line(origin, hit.hit_point, Color::srgb(0.3, 1.0, 0.4));
            gizmos.sphere(hit.hit_point, 0.12, Color::srgb(1.0, 0.45, 0.2));

            format!(
                "HIT   crate {who} (entity #{}) at {distance:.1} m",
                entity.index()
            )
        }
        None => {
            gizmos.line(
                origin,
                origin + direction,
                Color::srgba(0.65, 0.65, 0.65, 0.45),
            );
            "MISS  corridor clear (crates at opposite extremes)".to_string()
        }
    };

    hud.0 = message;
}
