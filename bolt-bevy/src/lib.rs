//! Bevy integration for the Jolt Physics engine.

use bevy::prelude::*;
use joltc_sys::*;
use rolt::{
    BroadPhaseLayer, BroadPhaseLayerInterface, ObjectLayer, ObjectLayerPairFilter,
    ObjectVsBroadPhaseLayerFilter, PhysicsSystem,
};

/// A simple broad phase layer implementation that puts all objects in a single layer.
pub struct SimpleBroadPhaseLayer;
impl BroadPhaseLayerInterface for SimpleBroadPhaseLayer {
    fn get_num_broad_phase_layers(&self) -> u32 {
        1
    }
    fn get_broad_phase_layer(&self, _layer: ObjectLayer) -> BroadPhaseLayer {
        BroadPhaseLayer::new(0)
    }
}

/// A default filter that allows all objects to collide with the broad phase.
pub struct SimpleObjectVsBroadPhaseLayerFilter;
impl ObjectVsBroadPhaseLayerFilter for SimpleObjectVsBroadPhaseLayerFilter {
    fn should_collide(&self, _layer1: ObjectLayer, _layer2: BroadPhaseLayer) -> bool {
        true
    }
}

/// A default filter that allows all objects to collide with each other.
pub struct SimpleObjectLayerPairFilter;
impl ObjectLayerPairFilter for SimpleObjectLayerPairFilter {
    fn should_collide(&self, _layer1: ObjectLayer, _layer2: ObjectLayer) -> bool {
        true
    }
}

/// The core Bevy resource representing the Jolt physics world.
///
/// This struct owns the Jolt `PhysicsSystem` as well as the temporary allocator
/// and job system required to step the simulation.
#[derive(Resource)]
pub struct PhysicsWorld {
    pub physics_system: PhysicsSystem,
    pub temp_allocator: *mut JPC_TempAllocatorImpl,
    pub job_system: *mut JPC_JobSystemThreadPool,
}

impl PhysicsWorld {
    /// Creates a new physics world with default settings.
    pub fn new() -> Self {
        // Initialize the Jolt core. This is required before any Jolt objects can be created.
        unsafe {
            JPC_RegisterDefaultAllocator();
            JPC_FactoryInit();
            JPC_RegisterTypes();
        }

        let mut physics_system = PhysicsSystem::new();
        physics_system.init(
            10240, // max_bodies
            0,     // num_body_mutexes (0 = default)
            65536, // max_body_pairs
            10240, // max_contact_constraints
            SimpleBroadPhaseLayer,
            SimpleObjectVsBroadPhaseLayerFilter,
            SimpleObjectLayerPairFilter,
        );

        let temp_allocator = unsafe { JPC_TempAllocatorImpl_new(10 * 1024 * 1024) }; // 10 MB
        let job_system = unsafe {
            JPC_JobSystemThreadPool_new3(
                JPC_MAX_PHYSICS_JOBS as u32,
                JPC_MAX_PHYSICS_BARRIERS as u32,
                2, // num_threads
            )
        };

        Self {
            physics_system,
            temp_allocator,
            job_system,
        }
    }
}

impl Default for PhysicsWorld {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for PhysicsWorld {
    fn drop(&mut self) {
        // Clean up C++ allocated memory to prevent leaks
        unsafe {
            JPC_JobSystemThreadPool_delete(self.job_system);
            JPC_TempAllocatorImpl_delete(self.temp_allocator);
        }
    }
}

// SAFETY: The Jolt `PhysicsSystem` is designed for multi-threaded access.
// Bevy's `ResMut` ensures we do not mutate the physics world from multiple
// systems simultaneously, making it safe to implement `Send` and `Sync`.
unsafe impl Send for PhysicsWorld {}
unsafe impl Sync for PhysicsWorld {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hello_world_physics() {
        let world = PhysicsWorld::new();

        let delta_time = 1.0 / 60.0; // 60 FPS
        let collision_steps = 1;

        unsafe {
            world.physics_system.update(
                delta_time,
                collision_steps,
                world.temp_allocator,
                world.job_system,
            );
        }
    }
}
