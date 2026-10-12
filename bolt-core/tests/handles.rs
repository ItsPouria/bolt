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
    let _handle_b = world_b.spawn_body(dynamic_box()).expect("spawn in B");

    let _ = world_b.get_transform(handle_a);
    let _ = world_b.is_active(handle_a);
    let _ = world_b.destroy_body(handle_a);
}
