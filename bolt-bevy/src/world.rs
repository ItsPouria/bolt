use std::ptr::NonNull;

use bevy::ecs::resource::Resource;
use joltc_sys::{
    JPC_FactoryInit, JPC_JobSystemThreadPool, JPC_JobSystemThreadPool_delete,
    JPC_JobSystemThreadPool_new3, JPC_MAX_PHYSICS_BARRIERS, JPC_MAX_PHYSICS_JOBS,
    JPC_RegisterDefaultAllocator, JPC_RegisterTypes, JPC_TempAllocatorImpl,
    JPC_TempAllocatorImpl_delete, JPC_TempAllocatorImpl_new,
};
use rolt::PhysicsSystem;

use crate::layers::{
    SimpleBroadPhaseLayer, SimpleObjectLayerPairFilter, SimpleObjectVsBroadPhaseLayerFilter,
};

/// The core Bevy resource representing the Jolt physics world.
///
/// This struct owns the Jolt `PhysicsSystem` as well as the temporary allocator
/// and job system required to step the simulation.
#[derive(Resource)]
pub struct PhysicsWorld {
    pub physics_system: PhysicsSystem,
    pub temp_allocator: NonNull<JPC_TempAllocatorImpl>,
    pub job_system: NonNull<JPC_JobSystemThreadPool>,
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

        let temp_allocator_ptr = unsafe { JPC_TempAllocatorImpl_new(10 * 1024 * 1024) }; // 10 MB
        let temp_allocator = NonNull::new(temp_allocator_ptr)
            .expect("Failed to allocate Jolt TempAllocator: Out of memory");
        let job_system_ptr = unsafe {
            JPC_JobSystemThreadPool_new3(
                JPC_MAX_PHYSICS_JOBS as u32,
                JPC_MAX_PHYSICS_BARRIERS as u32,
                2, // num_threads
            )
        };
        let job_system =
            NonNull::new(job_system_ptr).expect("Failed to allocate Jolt JobSystem: Out of memory");

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
            JPC_JobSystemThreadPool_delete(self.job_system.as_ptr());
            JPC_TempAllocatorImpl_delete(self.temp_allocator.as_ptr());
        }
    }
}

// SAFETY: The Jolt `PhysicsSystem` is designed for multi-threaded access.
// Bevy's `ResMut` ensures we do not mutate the physics world from multiple
// systems simultaneously, making it safe to implement `Send` and `Sync`.
unsafe impl Send for PhysicsWorld {}
unsafe impl Sync for PhysicsWorld {}
