use crate::components::RigidBody;
use crate::config::PhysicsConfig;
use std::ptr::NonNull;
use std::{mem::ManuallyDrop, ptr};

use bevy::ecs::resource::Resource;
use bevy::math::Vec3;
use bevy::transform::components::Transform;
use joltc_sys::{
    JPC_BoxShapeSettings, JPC_BoxShapeSettings_Create, JPC_FactoryInit, JPC_JobSystemThreadPool,
    JPC_JobSystemThreadPool_delete, JPC_JobSystemThreadPool_new3, JPC_MAX_PHYSICS_BARRIERS,
    JPC_MAX_PHYSICS_JOBS, JPC_PhysicsSystem_SetGravity, JPC_RegisterDefaultAllocator,
    JPC_RegisterTypes, JPC_Shape, JPC_String, JPC_TempAllocatorImpl, JPC_TempAllocatorImpl_delete,
    JPC_TempAllocatorImpl_new, JPC_Vec3,
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

    pub fn spawn_box(
        &mut self,
        half_extents: Vec3,
        transform: &Transform,
        rigidbody: &RigidBody,
    ) -> Option<rolt::BodyId> {
        let shape_ptr = create_box_shape(half_extents)?;

        let motion_type = match rigidbody {
            RigidBody::Dynamic => joltc_sys::JPC_MOTION_TYPE_DYNAMIC,
            RigidBody::Static => joltc_sys::JPC_MOTION_TYPE_STATIC,
        };

        let position = joltc_sys::JPC_Vec3 {
            x: transform.translation.x,
            y: transform.translation.y,
            z: transform.translation.z,
            _w: 0.0,
        };

        let rotation = joltc_sys::JPC_Quat {
            x: transform.rotation.x,
            y: transform.rotation.y,
            z: transform.rotation.z,
            w: transform.rotation.w,
        };

        let object_layer = if motion_type == joltc_sys::JPC_MOTION_TYPE_STATIC { 0 } else { 1 };

        let settings = joltc_sys::JPC_BodyCreationSettings {
            Position: position,
            Rotation: rotation,
            MotionType: motion_type,
            ObjectLayer: object_layer,
            Shape: shape_ptr,
            ..Default::default()
        };

        let body_id = unsafe {
            let raw_physics_system = self.physics_system.raw();
            let body_interface = joltc_sys::JPC_PhysicsSystem_GetBodyInterface(raw_physics_system);
            
            joltc_sys::JPC_BodyInterface_CreateAndAddBody(
                body_interface,
                &settings,
                joltc_sys::JPC_ACTIVATION_ACTIVATE,
            )
        };

        Some(rolt::BodyId::new(body_id))
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

fn create_box_shape(half_extents: Vec3) -> Option<*mut JPC_Shape> {
    let mut shape: *mut JPC_Shape = ptr::null_mut();
    let mut err: *mut JPC_String = ptr::null_mut();

    let settings = JPC_BoxShapeSettings {
        HalfExtent: JPC_Vec3 {
            x: half_extents.x,
            y: half_extents.y,
            z: half_extents.z,
            _w: 0.0,
        },
        ..Default::default()
    };

    unsafe {
        if JPC_BoxShapeSettings_Create(&settings, &mut shape, &mut err) {
            Some(shape)
        } else {
            None
        }
    }
}

// SAFETY: The Jolt `PhysicsSystem` is designed for multi-threaded access.
// Bevy's `ResMut` ensures we do not mutate the physics world from multiple
// systems simultaneously, making it safe to implement `Send` and `Sync`.
unsafe impl Send for PhysicsWorld {}
unsafe impl Sync for PhysicsWorld {}
