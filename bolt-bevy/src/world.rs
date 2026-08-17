use crate::config::PhysicsConfig;
use std::mem::ManuallyDrop;
use std::ptr::NonNull;

use bevy::ecs::resource::Resource;
use bevy::math::Vec3;
use joltc_sys::{
    JPC_FactoryInit, JPC_JobSystemThreadPool, JPC_JobSystemThreadPool_delete,
    JPC_JobSystemThreadPool_new3, JPC_MAX_PHYSICS_BARRIERS, JPC_MAX_PHYSICS_JOBS,
    JPC_PhysicsSystem_SetGravity, JPC_RegisterDefaultAllocator, JPC_RegisterTypes,
    JPC_TempAllocatorImpl, JPC_TempAllocatorImpl_delete, JPC_TempAllocatorImpl_new, JPC_Vec3,
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
    physics_system: ManuallyDrop<PhysicsSystem>,
    temp_allocator: NonNull<JPC_TempAllocatorImpl>,
    job_system: NonNull<JPC_JobSystemThreadPool>,
}

impl PhysicsWorld {
    /// Creates a new physics world with default settings.
    pub fn new(config: PhysicsConfig) -> Self {
        // Initialize the Jolt core. This is required before any Jolt objects can be created.
        unsafe {
            JPC_RegisterDefaultAllocator();
            JPC_FactoryInit();
            JPC_RegisterTypes();
        }

        let mut physics_system = PhysicsSystem::new();

        physics_system.init(
            config.max_bodies,
            0, // num_body_mutexes (0 = default) calculated automatically by Jolt
            config.max_body_pairs,
            config.max_contact_constraints,
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
                config.num_threads, // num_threads
            )
        };
        let job_system =
            NonNull::new(job_system_ptr).expect("Failed to allocate Jolt JobSystem: Out of memory");

        Self {
            physics_system: ManuallyDrop::new(physics_system),
            temp_allocator,
            job_system,
        }
    }

    pub fn physics_system(&self) -> &PhysicsSystem {
        &self.physics_system
    }

    pub fn step(&mut self, delta_time: f32, collision_steps: i32) {
        unsafe {
            self.physics_system.update(
                delta_time,
                collision_steps,
                self.temp_allocator.as_ptr(),
                self.job_system.as_ptr(),
            );
        }
    }

    pub fn set_gravity(&mut self, gravity: Vec3) {
        // SAFETY: `physics_system.raw()` returns a valid pointer to the initialized
        // JPC_PhysicsSystem. The JPC_Vec3 struct is correctly initialized with padding.
        unsafe {
            let raw_physics_system = self.physics_system.raw();
            let gravity_vec = JPC_Vec3 {
                x: gravity.x,
                y: gravity.y,
                z: gravity.z,
                _w: 0.0,
            };
            JPC_PhysicsSystem_SetGravity(raw_physics_system, gravity_vec);
        }
    }
}

impl Default for PhysicsWorld {
    fn default() -> Self {
        Self::new(PhysicsConfig::default())
    }
}

impl Drop for PhysicsWorld {
    fn drop(&mut self) {
        unsafe {
            ManuallyDrop::drop(&mut self.physics_system);
        }

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
