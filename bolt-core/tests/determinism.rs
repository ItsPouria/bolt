//! Determinism probe (#7 item 5): two identically-constructed worlds,
//! stepped identically, must produce bit-identical transforms.
//!
//! Jolt spreads each `step()` across a job system, and floating-point
//! arithmetic is order-sensitive, so run-to-run divergence is at least
//! conceivable. The single-threaded test is the control; the 4-thread test
//! is the probe that actually exercises the job system. Comparisons are
//! bit-exact (`assert_eq!`) on purpose: an approximate-equality assertion
//! would forgive exactly the last-bit divergence this probe hunts for.

use bolt_core::shapes::ShapeSpec;
use bolt_core::world::{BodyHandle, BodySpawn, BodyType, PhysicsWorld, WorldConfig};
use glam::{Quat, Vec3};

/// Builds a fresh world with a crooked 6-box tower on a static floor,
/// a sphere incoming from the side, and a spinning capsule.
/// The single source of "identical": called twice, never duplicated.
fn build_world(num_threads: i32) -> (PhysicsWorld, Vec<BodyHandle>) {
    let mut world = PhysicsWorld::new(WorldConfig {
        num_threads,
        ..WorldConfig::default()
    });

    // The template: a dynamic 1 m box, zeroed out. Every body below
    // overrides a few fields and inherits the rest.
    let box_one = BodySpawn {
        shape: ShapeSpec::Box {
            half_extents: Vec3::splat(0.5),
        },
        body_type: BodyType::Dynamic,
        position: Vec3::new(0.0, 5.0, 0.0),
        rotation: Quat::IDENTITY,
        linear_velocity: Vec3::ZERO,
        angular_velocity: Vec3::ZERO,
        linear_damping: 0.05,
        angular_damping: 0.05,
        user_data: 7,
        use_ccd: false,
    };

    let mut handles = Vec::new();

    // Floor: static slab, top surface at y = 0.
    let handle = world
        .spawn_body(BodySpawn {
            shape: ShapeSpec::Box {
                half_extents: Vec3::new(10.0, 1.0, 10.0),
            },
            body_type: BodyType::Static,
            position: Vec3::new(0.0, -1.0, 0.0),
            user_data: 0,
            ..box_one.clone()
        })
        .expect("spawn must succeed");
    handles.push(handle);

    // Crooked 6-box tower: each box is 1 m tall, so box i's center sits at
    // y = 0.5 + i. The small x-offset per box makes the stack unstable —
    // a perfectly symmetric tower can mask divergence.
    for i in 0..6 {
        let handle = world
            .spawn_body(BodySpawn {
                position: Vec3::new(i as f32 * 0.05, 0.5 + i as f32, 0.0),
                user_data: 1 + i as u64,
                ..box_one.clone()
            })
            .expect("spawn must succeed");
        handles.push(handle);
    }

    // Sphere: fast lateral shot into the tower's mid-section.
    let handle = world
        .spawn_body(BodySpawn {
            shape: ShapeSpec::Sphere { radius: 0.5 },
            position: Vec3::new(-6.0, 3.0, 0.0),
            linear_velocity: Vec3::new(20.0, 0.0, 0.0),
            user_data: 10,
            ..box_one.clone()
        })
        .expect("spawn must succeed");
    handles.push(handle);

    // Capsule: dropped beside the wreckage, spinning.
    let handle = world
        .spawn_body(BodySpawn {
            shape: ShapeSpec::Capsule {
                half_height: 0.5,
                radius: 0.25,
            },
            position: Vec3::new(2.5, 5.0, 1.5),
            angular_velocity: Vec3::new(0.0, 0.0, 7.0),
            user_data: 11,
            ..box_one.clone()
        })
        .expect("spawn must succeed");
    handles.push(handle);

    (world, handles)
}

/// Steps both worlds side by side for 2 simulated seconds, then asserts
/// bit-exact transform agreement, body for body (spawn order pairs them).
fn step_and_compare(
    first_world: &mut PhysicsWorld,
    first_handles: &[BodyHandle],
    second_world: &mut PhysicsWorld,
    second_handles: &[BodyHandle],
) {
    for _ in 0..120 {
        first_world.step(1.0 / 60.0, 1);
        second_world.step(1.0 / 60.0, 1);
    }
    for (i, (first_handle, second_handle)) in first_handles.iter().zip(second_handles).enumerate() {
        let (first_position, first_rotation) = first_world
            .get_transform(*first_handle)
            .expect("body alive");
        let (second_position, second_rotation) = second_world
            .get_transform(*second_handle)
            .expect("body alive");
        assert_eq!(
            first_position, second_position,
            "body {i}: position diverged"
        );
        assert_eq!(
            first_rotation, second_rotation,
            "body {i}: rotation diverged"
        );
    }
}

#[test]
fn twin_worlds_agree_single_threaded() {
    let (mut first_world, first_handles) = build_world(1);
    let (mut second_world, second_handles) = build_world(1);
    step_and_compare(
        &mut first_world,
        &first_handles,
        &mut second_world,
        &second_handles,
    );
}

#[test]
fn twin_worlds_agree_multithreaded() {
    let (mut first_world, first_handles) = build_world(4);
    let (mut second_world, second_handles) = build_world(4);
    step_and_compare(
        &mut first_world,
        &first_handles,
        &mut second_world,
        &second_handles,
    );
}
