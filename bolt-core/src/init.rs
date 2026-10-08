//! One-time global initialization of the Jolt C++ runtime.

use joltc_sys::{JPC_FactoryInit, JPC_RegisterDefaultAllocator, JPC_RegisterTypes};
use std::sync::Once;

static JOLT_INIT: Once = Once::new();

/// Initializes the Jolt core (allocators, factory, types).
///
/// Idempotent: safe to call any number of times — only the first call does work.
/// Must run before any Jolt object is created.
// TODO: make `pub(crate)` once `CorePhysicsWorld::new` calls this internally.
pub fn ensure_jolt_initialized() {
    JOLT_INIT.call_once(|| {
        // SAFETY: Jolt's C-API global setup must run exactly once before any
        // other Jolt call. `Once` guarantees this is the one and only call.
        unsafe {
            JPC_RegisterDefaultAllocator();
            JPC_FactoryInit();
            JPC_RegisterTypes();
        }
    });
}
