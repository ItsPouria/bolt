use crate::components::RigidBody;
use crate::config::PhysicsConfig;
use std::ptr::NonNull;
use std::sync::Once;
use std::{mem::ManuallyDrop, ptr};

use bevy::ecs::entity::Entity;
use bevy::ecs::resource::Resource;
use bevy::ecs::world::{FromWorld, World};
use bevy::log::error;
use bevy::math::{Quat, Vec3};
use joltc_sys::{
    JPC_BoxShapeSettings, JPC_BoxShapeSettings_Create, JPC_FactoryInit, JPC_JobSystemThreadPool,
    JPC_JobSystemThreadPool_delete, JPC_JobSystemThreadPool_new3, JPC_MAX_PHYSICS_BARRIERS,
    JPC_MAX_PHYSICS_JOBS, JPC_PhysicsSystem_SetGravity, JPC_RegisterDefaultAllocator,
    JPC_RegisterTypes, JPC_Shape, JPC_String, JPC_TempAllocatorImpl, JPC_TempAllocatorImpl_delete,
    JPC_TempAllocatorImpl_new, JPC_Vec3,
};
use rolt::PhysicsSystem;

use crate::layers::{
    OBJECT_LAYER_DYNAMIC, OBJECT_LAYER_STATIC, SimpleBroadPhaseLayer, SimpleObjectLayerPairFilter,
    SimpleObjectVsBroadPhaseLayerFilter,
};

static JOLT_INIT: Once = Once::new();
/// Invalid Body Id constant.
pub const INVALID_BODY_ID: u32 = 0xffff_ffff;

/// The core Bevy resource representing the Jolt physics world.
///
/// This struct owns the Jolt `PhysicsSystem` as well as the temporary allocator
/// and job system required to step the simulation.
#[derive(Resource)]
/// Physicsworld struct.
pub struct PhysicsWorld {
    physics_system: ManuallyDrop<PhysicsSystem>,
    temp_allocator: NonNull<JPC_TempAllocatorImpl>,
    job_system: NonNull<JPC_JobSystemThreadPool>,
}

impl PhysicsWorld {
    /// Creates a new physics world with default settings.
    pub fn new(config: PhysicsConfig) -> Self {
        // Initialize the Jolt core. This is required before any Jolt objects can be created.
        JOLT_INIT.call_once(|| 
        // SAFETY: Initializing the Jolt C-API is safe to call exactly once globally.
        unsafe {
            JPC_RegisterDefaultAllocator();
            JPC_FactoryInit();
            JPC_RegisterTypes();
        });

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

        // SAFETY: 10MB is a valid size for the Jolt temp allocator.
        let temp_allocator_ptr = unsafe { JPC_TempAllocatorImpl_new(config.temp_allocator_size_mb * 1024 * 1024) }; // 10 MB
        let temp_allocator = NonNull::new(temp_allocator_ptr)
            .expect("Failed to allocate Jolt TempAllocator: Out of memory");
        // SAFETY: Thread counts and max jobs constants are valid parameters for Jolt.
        let job_system_ptr = unsafe {
            JPC_JobSystemThreadPool_new3(
                JPC_MAX_PHYSICS_JOBS as u32,
                JPC_MAX_PHYSICS_BARRIERS as u32,
                config.num_threads as i32,
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

    /// Physics System.
    pub fn physics_system(&self) -> &PhysicsSystem {
        &self.physics_system
    }

    /// Helper to get the Jolt BodyInterface.
    ///
    /// # Safety
    /// The caller must ensure that the `PhysicsSystem` is still valid.
    unsafe fn body_interface(&self) -> *mut joltc_sys::JPC_BodyInterface {
        unsafe { joltc_sys::JPC_PhysicsSystem_GetBodyInterface(self.physics_system.raw()) }
    }

    /// Spawn Box.
    pub fn spawn_box(
        &mut self,
        entity: Entity,
        half_extents: Vec3,
        transform: (Vec3, Quat),
        rigidbody: &RigidBody,
        linear_velocity: Vec3,
        angular_velocity: Vec3,
    ) -> Option<rolt::BodyId> {
        let (position, rotation) = transform;
        let shape_ptr = create_box_shape(half_extents)?;

        let motion_type = match rigidbody {
            RigidBody::Dynamic => joltc_sys::JPC_MOTION_TYPE_DYNAMIC,
            RigidBody::Static => joltc_sys::JPC_MOTION_TYPE_STATIC,
            RigidBody::Kinematic => joltc_sys::JPC_MOTION_TYPE_KINEMATIC,
        };

        let position = joltc_sys::JPC_Vec3 {
            x: position.x,
            y: position.y,
            z: position.z,
            _w: 0.0,
        };

        let rotation = joltc_sys::JPC_Quat {
            x: rotation.x,
            y: rotation.y,
            z: rotation.z,
            w: rotation.w,
        };

        let lin_vel = joltc_sys::JPC_Vec3 {
            x: linear_velocity.x,
            y: linear_velocity.y,
            z: linear_velocity.z,
            _w: 0.0,
        };

        let ang_vel = joltc_sys::JPC_Vec3 {
            x: angular_velocity.x,
            y: angular_velocity.y,
            z: angular_velocity.z,
            _w: 0.0,
        };

        let object_layer = match rigidbody {
            RigidBody::Static => OBJECT_LAYER_STATIC.raw(),
            RigidBody::Dynamic | RigidBody::Kinematic => OBJECT_LAYER_DYNAMIC.raw(),
        };

        let settings = joltc_sys::JPC_BodyCreationSettings {
            Position: position,
            Rotation: rotation,
            MotionType: motion_type,
            ObjectLayer: object_layer,
            Shape: shape_ptr,
            LinearVelocity: lin_vel,
            AngularVelocity: ang_vel,
            UserData: entity.to_bits(),
            ..Default::default()
        };

        // SAFETY: settings and shape_ptr are valid, and motion types match Jolt requirements.
        let body_id = unsafe {
            let body_interface = self.body_interface();

            let id = joltc_sys::JPC_BodyInterface_CreateAndAddBody(
                body_interface,
                &settings,
                joltc_sys::JPC_ACTIVATION_ACTIVATE,
            );

            joltc_sys::JPC_Shape_Release(shape_ptr);

            id
        };

        // Check for invalid body ID from Jolt (cInvalidBodyID = 0xFFFFFFFF)
        if body_id == INVALID_BODY_ID {
            error!("Failed to create Jolt body: body limit reached or invalid settings");
            return None;
        }
        Some(rolt::BodyId::new(body_id))
    }

    /// Get Transform.
    pub fn get_transform(&self, body_id: rolt::BodyId) -> Option<(Vec3, Quat)> {
        if body_id.raw() == INVALID_BODY_ID {
            return None;
        }
        // SAFETY: We verify the body ID is valid and added before querying its state.
        unsafe {
            let body_interface = self.body_interface();

            if !joltc_sys::JPC_BodyInterface_IsAdded(body_interface, body_id.raw()) {
                return None;
            }

            let pos = joltc_sys::JPC_BodyInterface_GetPosition(body_interface, body_id.raw());
            let rot = joltc_sys::JPC_BodyInterface_GetRotation(body_interface, body_id.raw());

            Some((
                Vec3::new(pos.x as f32, pos.y as f32, pos.z as f32),
                bevy::prelude::Quat::from_xyzw(rot.x, rot.y, rot.z, rot.w),
            ))
        }
    }

    /// Get Linear Velocity.
    pub fn get_linear_velocity(&self, body_id: rolt::BodyId) -> Option<Vec3> {
        if body_id.raw() == INVALID_BODY_ID {
            return None;
        }
        // SAFETY: We verify the body ID is valid and added before querying its state.
        unsafe {
            let body_interface = self.body_interface();

            if !joltc_sys::JPC_BodyInterface_IsAdded(body_interface, body_id.raw()) {
                return None;
            }

            let lin_vel =
                joltc_sys::JPC_BodyInterface_GetLinearVelocity(body_interface, body_id.raw());

            Some(Vec3::new(lin_vel.x, lin_vel.y, lin_vel.z))
        }
    }

    /// Get Angular Velocity.
    pub fn get_angular_velocity(&self, body_id: rolt::BodyId) -> Option<Vec3> {
        if body_id.raw() == INVALID_BODY_ID {
            return None;
        }
        // SAFETY: We verify the body ID is valid and added before querying its state.
        unsafe {
            let body_interface = self.body_interface();

            if !joltc_sys::JPC_BodyInterface_IsAdded(body_interface, body_id.raw()) {
                return None;
            }

            let ang_vel =
                joltc_sys::JPC_BodyInterface_GetAngularVelocity(body_interface, body_id.raw());

            Some(Vec3::new(ang_vel.x, ang_vel.y, ang_vel.z))
        }
    }

    /// Set Linear Velocity.
    pub fn set_linear_velocity(&mut self, body_id: rolt::BodyId, linear_velocity: Vec3) {
        if body_id.raw() == INVALID_BODY_ID {
            return;
        }
        // SAFETY: We verify the body ID is valid and added before mutating its state.
        unsafe {
            let body_interface = self.body_interface();

            if !joltc_sys::JPC_BodyInterface_IsAdded(body_interface, body_id.raw()) {
                return;
            }

            let jolt_vel = joltc_sys::JPC_Vec3 {
                x: linear_velocity.x,
                y: linear_velocity.y,
                z: linear_velocity.z,
                _w: 0.0,
            };

            joltc_sys::JPC_BodyInterface_SetLinearVelocity(body_interface, body_id.raw(), jolt_vel);
        }
    }

    /// Set Angular Velocity.
    pub fn set_angular_velocity(&mut self, body_id: rolt::BodyId, angular_velocity: Vec3) {
        if body_id.raw() == INVALID_BODY_ID {
            return;
        }
        // SAFETY: We verify the body ID is valid and added before mutating its state.
        unsafe {
            let body_interface = self.body_interface();

            if !joltc_sys::JPC_BodyInterface_IsAdded(body_interface, body_id.raw()) {
                return;
            }

            let jolt_vel = joltc_sys::JPC_Vec3 {
                x: angular_velocity.x,
                y: angular_velocity.y,
                z: angular_velocity.z,
                _w: 0.0,
            };

            joltc_sys::JPC_BodyInterface_SetAngularVelocity(
                body_interface,
                body_id.raw(),
                jolt_vel,
            );
        }
    }

    /// Step.
    pub fn step(&mut self, delta_time: f32, collision_steps: i32) {
        if delta_time <= 0.0 || delta_time.is_nan() {
            return;
        }

        // SAFETY: physics_system, allocator, and job_system pointers are valid for the lifetime of PhysicsWorld.
        unsafe {
            self.physics_system.update(
                delta_time,
                collision_steps.max(1),
                self.temp_allocator.as_ptr(),
                self.job_system.as_ptr(),
            );
        }
    }

    /// Set Gravity.
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

    /// Destroy Body.
    pub fn destroy_body(&mut self, body_id: rolt::BodyId) {
        if body_id.raw() == INVALID_BODY_ID {
            return;
        }

        // SAFETY: The body ID is valid, and we check if it is added before removing and destroying it.
        unsafe {
            let body_interface = self.body_interface();
            if joltc_sys::JPC_BodyInterface_IsAdded(body_interface, body_id.raw()) {
                joltc_sys::JPC_BodyInterface_RemoveBody(body_interface, body_id.raw());
            }
            joltc_sys::JPC_BodyInterface_DestroyBody(body_interface, body_id.raw());
        }
    }
}

impl FromWorld for PhysicsWorld {
    fn from_world(world: &mut World) -> Self {
       let config = world.get_resource::<PhysicsConfig>().cloned().unwrap_or_default();
       Self::new(config)
    }
}

impl Drop for PhysicsWorld {
    fn drop(&mut self) {
        // SAFETY: Drop ordering is critical. The `physics_system` MUST be dropped first,
        // as Jolt internally references the job_system and temp_allocator during its shutdown.
        unsafe {
            ManuallyDrop::drop(&mut self.physics_system);
        }

        // SAFETY: Now that the physics system is destroyed, it is safe to delete the allocator and job system.
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

    // SAFETY: FFI call to create a box shape with valid settings.
    unsafe {
        if JPC_BoxShapeSettings_Create(&settings, &mut shape, &mut err) {
            Some(shape)
        } else {
            // If creation fails, free the C++ error string allocated by Jolt
            if !err.is_null() {
                joltc_sys::JPC_String_delete(err);
            }
            None
        }
    }
}

// SAFETY: The Jolt `PhysicsSystem` is designed for multi-threaded access.
// The raw pointers (`temp_allocator`, `job_system`) are only accessed through 
// `&mut self` methods (single-writer safe), and no aliased mutable pointers exist.
unsafe impl Send for PhysicsWorld {}
// SAFETY: PhysicsWorld uses single-writer mutation and has no aliased pointers, making it safe to share references.
unsafe impl Sync for PhysicsWorld {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_physics_system_getter() {
        let physics_world = PhysicsWorld::default();
        let system = physics_world.physics_system();

        // Assert that the raw C++ pointer inside the system successfully initialized
        assert!(
            !system.raw().is_null(),
            "The C++ Jolt PhysicsSystem pointer was null!"
        );
    }

    #[test]
    fn test_spawn_static_box() {
        let mut physics_world = PhysicsWorld::default();
        let rigidbody = RigidBody::Static;
        let box_size = Vec3::splat(1.0);

        let result = physics_world.spawn_box(
            Entity::PLACEHOLDER,
            box_size,
            Vec3::ZERO,
            Quat::IDENTITY,
            &rigidbody,
            Vec3::ZERO,
            Vec3::ZERO,
        );

        assert!(result.is_some());
    }

    #[test]
    fn test_spawn_kinematic_box() {
        let mut physics_world = PhysicsWorld::default();
        let rigidbody = RigidBody::Kinematic;
        let box_size = Vec3::splat(1.0);

        let result = physics_world.spawn_box(
            Entity::PLACEHOLDER,
            box_size,
            Vec3::ZERO,
            Quat::IDENTITY,
            &rigidbody,
            Vec3::ZERO,
            Vec3::ZERO,
        );

        assert!(result.is_some());
    }

    #[test]
    fn test_create_box_shape_failure() {
        let mut physics_world = PhysicsWorld::default();
        let rigidbody = RigidBody::Static;
        let box_size = Vec3::splat(-1.0);

        let result = physics_world.spawn_box(
            Entity::PLACEHOLDER,
            box_size,
            Vec3::ZERO,
            Quat::IDENTITY,
            &rigidbody,
            Vec3::ZERO,
            Vec3::ZERO,
        );

        assert!(result.is_none());
    }

    #[test]
    fn test_get_transform_invalid_and_destroyed_body() {
        let mut physics_world = PhysicsWorld::default();

        // 1. Querying an invalid body ID returns None
        let invalid_id = rolt::BodyId::new(INVALID_BODY_ID);
        assert!(physics_world.get_transform(invalid_id).is_none());

        // 2. Querying an unallocated body ID returns None
        let fake_id = rolt::BodyId::new(9999);
        assert!(physics_world.get_transform(fake_id).is_none());

        // 3. Spawning a valid box returns Some(...)
        let body_id = physics_world
            .spawn_box(
                Entity::PLACEHOLDER,
                Vec3::splat(1.0),
                Vec3::new(1.0, 2.0, 3.0),
                Quat::IDENTITY,
                &RigidBody::Dynamic,
                Vec3::ZERO,
                Vec3::ZERO,
            )
            .expect("Failed to spawn box");
        assert!(physics_world.get_transform(body_id).is_some());

        // 4. Destroying the body causes get_transform to return None
        physics_world.destroy_body(body_id);
        assert!(physics_world.get_transform(body_id).is_none());
    }

    #[test]
    fn test_spawn_box_stores_entity_user_data() {
        let mut physics_world = PhysicsWorld::default();
        let test_entity = Entity::from_raw_u32(42).unwrap();

        let body_id = physics_world
            .spawn_box(
                test_entity,
                Vec3::splat(1.0),
                Vec3::ZERO,
                Quat::IDENTITY,
                &RigidBody::Dynamic,
                Vec3::ZERO,
                Vec3::ZERO,
            )
            .expect("Failed to spawn box");

        let user_data = physics_world
            .physics_system()
            .body_interface()
            .user_data(body_id);
        assert_eq!(Entity::from_bits(user_data), test_entity);
    }

    #[test]
    fn test_step_handles_invalid_delta_time() {
        let mut physics_world = PhysicsWorld::default();

        // Calling step with 0.0, negative dt, or NaN should safely return without panic
        physics_world.step(0.0, 1);
        physics_world.step(-1.0 / 60.0, 1);
        physics_world.step(f32::NAN, 1);
    }

    #[test]
    fn test_velocity_getters_and_setters() {
        let mut physics_world = PhysicsWorld::default();
        let rigidbody = RigidBody::Dynamic;
        let box_size = Vec3::splat(1.0);

        let body_id = physics_world
            .spawn_box(
                Entity::PLACEHOLDER,
                box_size,
                Vec3::ZERO,
                Quat::IDENTITY,
                &rigidbody,
                Vec3::ZERO,
                Vec3::ZERO,
            )
            .expect("Failed to spawn box");

        let target_lin_vel = Vec3::new(1.0, 2.0, 3.0);
        let target_ang_vel = Vec3::new(4.0, 5.0, 6.0);

        physics_world.set_linear_velocity(body_id, target_lin_vel);
        physics_world.set_angular_velocity(body_id, target_ang_vel);

        assert_eq!(
            physics_world.get_linear_velocity(body_id).unwrap(),
            target_lin_vel
        );
        assert_eq!(
            physics_world.get_angular_velocity(body_id).unwrap(),
            target_ang_vel
        );
    }
}
