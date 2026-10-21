//! Issue #6: despawning a physics entity must not emit any log warnings.
//!
//! Bevy 0.20's `EntityCommands::remove` warns when its command applies to an
//! entity that no longer exists. The `Remove<RigidBody>`/`Remove<Collider>`
//! observers fire *mid-despawn* — while the entity still exists — and queue
//! removals that apply only after the entity is gone, so every despawn of a
//! physics entity used to log two spurious warnings (one per observer). The
//! fix routes those observers through `try_remove`.
//!
//! The warnings are emitted through the `log` facade (bevy_ecs'
//! `error::warn` command-error handler), which is silent in tests because
//! `MinimalPlugins` carries no `LogPlugin`. These tests therefore install a
//! capturing `log::Log` implementation and assert the captured buffer stays
//! empty — turning "no warnings" into a CI-enforced contract.

use std::cell::RefCell;
use std::sync::{Arc, Mutex, Once};

use bevy::prelude::*;
use bolt_bevy::prelude::*;

/// Installs the process-wide logger exactly once. `log::set_boxed_logger`
/// succeeds only on the first call, so `Once` makes the race between parallel
/// test threads harmless.
static INSTALL: Once = Once::new();

thread_local! {
    /// The `log` global logger is process-wide, but `cargo test` runs tests
    /// in parallel *threads* of one process. Isolation comes from dispatching
    /// captured records into whichever thread currently has a sink installed,
    /// so one test never sees another test's warnings.
    static SINK: RefCell<Option<Arc<Mutex<Vec<String>>>>> = const { RefCell::new(None) };
}

/// Captures `log` records at warn level and above into the current thread's sink.
struct Capture;

impl log::Log for Capture {
    fn enabled(&self, metadata: &log::Metadata) -> bool {
        metadata.level() <= log::Level::Warn
    }

    fn log(&self, record: &log::Record) {
        if !self.enabled(record.metadata()) {
            return;
        }
        SINK.with(|sink| {
            if let Some(sink) = sink.borrow().as_ref() {
                sink.lock()
                    .unwrap()
                    .push(format!("{}: {}", record.level(), record.args()));
            }
        });
    }

    fn flush(&self) {}
}

/// Runs `body` with warnings captured; returns everything logged at warn level
/// or above while it ran.
fn capture_warnings(body: impl FnOnce()) -> Vec<String> {
    INSTALL.call_once(|| {
        let _ = log::set_boxed_logger(Box::new(Capture));
        log::set_max_level(log::LevelFilter::Warn);
    });
    let sink = Arc::new(Mutex::new(Vec::new()));
    SINK.with(|slot| *slot.borrow_mut() = Some(sink.clone()));
    body();
    SINK.with(|slot| *slot.borrow_mut() = None);
    sink.lock().unwrap().clone()
}

fn physics_test_app() -> App {
    let mut app = App::new();
    app.add_plugins(MinimalPlugins);
    app.add_plugins(bevy::transform::TransformPlugin);
    app.add_plugins(bolt_bevy::plugin::BoltPlugin::default());
    app
}

fn spawn_dynamic_box(app: &mut App) -> Entity {
    app.world_mut()
        .spawn((
            Transform::from_xyz(0.0, 10.0, 0.0),
            RigidBody::Dynamic,
            Collider::Box {
                half_extents: Vec3::splat(0.5),
            },
        ))
        .id()
}

#[test]
fn despawn_emits_zero_warnings() {
    let warnings = capture_warnings(|| {
        let mut app = physics_test_app();
        let entity = spawn_dynamic_box(&mut app);

        // Spawn the body in Jolt, then despawn the entity. The observers fire
        // mid-despawn; their queued removals apply against an entity that no
        // longer exists.
        app.update();
        app.world_mut().despawn(entity);
        app.world_mut().flush();
    });

    assert!(
        warnings.is_empty(),
        "despawning a physics entity emitted warnings:\n{}",
        warnings.join("\n")
    );
}

#[test]
fn static_despawn_emits_zero_warnings() {
    let warnings = capture_warnings(|| {
        let mut app = physics_test_app();
        let entity = app
            .world_mut()
            .spawn((
                Transform::from_xyz(0.0, 0.0, 0.0),
                RigidBody::Static,
                Collider::Box {
                    half_extents: Vec3::splat(0.5),
                },
            ))
            .id();

        app.update();
        // Statics carry `StaticMarker`; its removal path must be equally quiet.
        app.world_mut().despawn(entity);
        app.world_mut().flush();
    });

    assert!(
        warnings.is_empty(),
        "despawning a static physics entity emitted warnings:\n{}",
        warnings.join("\n")
    );
}
