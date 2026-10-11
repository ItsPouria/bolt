//! Headless physics benchmark: no window, no renderer, no Bevy — `bolt-core`
//! driven directly, proving the engine-agnostic crate works standalone.
//!
//! Usage: `cargo run --release -p bolt-examples --bin headless_bench -- [bodies] [substeps]`
//!
//! Defaults to 1000 dynamic bodies and 1 substep per step. To mirror the
//! hello_world example's configuration, run with `40 4`.

use bolt_core::shapes::ShapeSpec;
use bolt_core::world::{BodySpawn, BodyType, PhysicsWorld, WorldConfig};
use glam::{Quat, Vec3};

fn box_spawn(body_type: BodyType, half_extents: Vec3, position: Vec3) -> BodySpawn {
    BodySpawn {
        shape: ShapeSpec::Box { half_extents },
        body_type,
        position,
        rotation: Quat::IDENTITY,
        linear_velocity: Vec3::ZERO,
        angular_velocity: Vec3::ZERO,
        user_data: 0,
        use_ccd: true, // mirrors hello_world: ContinuousCollision on every body
    }
}

fn main() {
    let body_count: usize = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(1000);
    let substeps: i32 = std::env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(1);

    let mut world = PhysicsWorld::new(WorldConfig::default());

    // Static floor.
    world
        .spawn_body(box_spawn(
            BodyType::Static,
            Vec3::new(25.0, 0.5, 25.0),
            Vec3::new(0.0, -1.0, 0.0),
        ))
        .expect("failed to spawn floor");

    // Dynamic bodies in a 20x20 grid, stacked in layers so they collide,
    // settle, and (eventually) sleep — a realistic workload.
    for i in 0..body_count {
        let x = (i % 20) as f32 - 9.5;
        let z = (i / 20) as f32 - 9.5;
        let y = (i / 400) as f32 * 1.2 + 0.6;
        world
            .spawn_body(box_spawn(
                BodyType::Dynamic,
                Vec3::splat(0.5),
                Vec3::new(x, y, z),
            ))
            .expect("failed to spawn body");
    }

    // Warm-up: one second of simulation so the timed section covers the
    // interesting active-settling phase rather than pure free fall.
    for _ in 0..60 {
        world.step(1.0 / 60.0, substeps);
    }

    let steps = 600;
    let start = std::time::Instant::now();
    for _ in 0..steps {
        world.step(1.0 / 60.0, substeps);
    }
    let elapsed = start.elapsed();

    let per_step_us = elapsed.as_micros() as f64 / steps as f64;
    println!("bodies:            {} dynamic + 1 static", body_count);
    println!("substeps per step: {}", substeps);
    println!("steps:             {} @ 60 Hz", steps);
    println!("total:             {:.2?}", elapsed);
    println!("per step:          {:.1} µs", per_step_us);
    println!(
        "frame budget use:  {:.2}% of a 16.6 ms frame",
        per_step_us / 16_600.0 * 100.0
    );
}
