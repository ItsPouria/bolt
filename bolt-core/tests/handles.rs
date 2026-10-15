//! Public-boundary contract tests for body handles.
//!
//! Files in `tests/` compile as separate, external crates: they can only use
//! `bolt_core`'s public API, exactly like a stranger consuming it. Forging a
//! handle from raw bits is therefore unrepresentable here by construction —
//! these tests verify the runtime half of the contract: legitimately obtained
//! handles that go stale must be rejected by every API.

use bolt_core::shapes::ShapeSpec;
use bolt_core::world::{BodySpawn, BodyType, PhysicsWorld, WorldConfig};
use glam::{Quat, Vec3};

/// A minimal dynamic box with sensible defaults.
fn dynamic_box() -> BodySpawn {
    BodySpawn {
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
    }
}

#[test]
fn handles_are_valid_while_their_body_lives() {
    let mut world = PhysicsWorld::new(WorldConfig::default());
    let handle = world.spawn_body(dynamic_box()).expect("spawn must succeed");

    assert!(world.is_active(handle));
    assert!(world.get_transform(handle).is_some());
    assert!(world.get_linear_velocity(handle).is_some());
    assert!(world.get_angular_velocity(handle).is_some());
}

#[test]
fn stale_handles_are_rejected_after_destroy() {
    let mut world = PhysicsWorld::new(WorldConfig::default());
    let handle = world.spawn_body(dynamic_box()).expect("spawn must succeed");

    assert!(world.destroy_body(handle));

    // The handle is now stale: every API must reject or ignore it.
    assert!(world.get_transform(handle).is_none());
    assert!(world.get_linear_velocity(handle).is_none());
    assert!(world.get_angular_velocity(handle).is_none());
    assert!(!world.is_active(handle));
    world.set_linear_velocity(handle, Vec3::X); // must be a safe no-op
    world.set_angular_velocity(handle, Vec3::Y);
}

#[test]
fn double_destroy_reports_false() {
    let mut world = PhysicsWorld::new(WorldConfig::default());
    let handle = world.spawn_body(dynamic_box()).expect("spawn must succeed");

    assert!(world.destroy_body(handle));
    assert!(!world.destroy_body(handle));
}

#[test]
fn cross_world_handles_do_not_corrupt() {
    // Cross-world use is documented as safe-but-wrong: presenting world A's
    // handle to world B must not panic or corrupt memory, whatever it returns.
    let mut world_a = PhysicsWorld::new(WorldConfig::default());
    let mut world_b = PhysicsWorld::new(WorldConfig::default());

    let handle_a = world_a.spawn_body(dynamic_box()).expect("spawn in A");
    let handle_b = world_b.spawn_body(dynamic_box()).expect("spawn in B");

    // Identical fresh worlds hand out matching raw IDs for their first
    // bodies, so these calls operate on B's body — the documented
    // "wrong but safe" outcome. What must hold: no panic, no corruption,
    // and both worlds keep working through their own handles afterwards.
    let _ = world_b.get_transform(handle_a);
    let _ = world_b.is_active(handle_a);
    let _ = world_b.user_data(handle_a);
    world_b.set_linear_velocity(handle_a, Vec3::X);
    world_b.set_angular_velocity(handle_a, Vec3::Y);
    world_b.set_position_and_rotation(handle_a, Vec3::new(1.0, 2.0, 3.0), Quat::IDENTITY);

    assert!(world_b.get_transform(handle_b).is_some());
    assert_eq!(world_b.user_data(handle_b), Some(7));

    // Destroying via the foreign handle frees B's slot; B's own handle then
    // reports stale while world A remains untouched.
    assert!(world_b.destroy_body(handle_a));
    assert!(world_b.get_transform(handle_b).is_none());
    assert!(world_a.get_transform(handle_a).is_some());
}

#[test]
fn user_data_round_trips_through_public_api() {
    let mut world = PhysicsWorld::new(WorldConfig::default());
    let mut spawn = dynamic_box();
    spawn.user_data = 0xdead_beef;
    let handle = world.spawn_body(spawn).expect("spawn must succeed");

    assert_eq!(world.user_data(handle), Some(0xdead_beef));

    // The getter honors the handle lifecycle like every other API.
    assert!(world.destroy_body(handle));
    assert_eq!(world.user_data(handle), None);
}

#[test]
fn spawn_reports_none_when_the_world_is_at_capacity() {
    let config = WorldConfig {
        max_bodies: 2,
        ..Default::default()
    };
    let mut world = PhysicsWorld::new(config);

    let first = world
        .spawn_body(dynamic_box())
        .expect("first spawn must succeed");
    let second = world
        .spawn_body(dynamic_box())
        .expect("second spawn must succeed");

    // The documented capacity failure mode: None, not a panic.
    assert!(
        world.spawn_body(dynamic_box()).is_none(),
        "third spawn must report capacity exhaustion"
    );

    // Bodies spawned before exhaustion remain fully live.
    assert!(world.get_transform(first).is_some());
    assert!(world.get_transform(second).is_some());
}
