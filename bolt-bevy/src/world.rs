use crate::components::{Collider, RigidBody};
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
/// Constant representing an invalid or unallocated Jolt body ID (0xFFFFFFFF).
pub const INVALID_BODY_ID: u32 = 0xffff_ffff;

/// The core Bevy resource representing the Jolt physics world.
///
/// This struct owns the Jolt `PhysicsSystem` as well as the temporary allocator
/// and job system required to step the simulation.
#[derive(Resource)]
pub struct PhysicsWorld {
    physics_system: ManuallyDrop<PhysicsSystem>,
    temp_allocator: NonNull<JPC_TempAllocatorImpl>,
    job_system: NonNull<JPC_JobSystemThreadPool>,
    body_registry: std::collections::HashMap<rolt::BodyId, Entity>,
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

        let temp_allocator_bytes = config
            .temp_allocator_size_mb
            .checked_mul(1024 * 1024)
            .expect("temp_allocator_size_mb in bytes overflows u32");

        // SAFETY: 10MB is a valid size for the Jolt temp allocator.
        let temp_allocator_ptr = unsafe { JPC_TempAllocatorImpl_new(temp_allocator_bytes) };
        let temp_allocator = NonNull::new(temp_allocator_ptr)
            .expect("Failed to allocate Jolt TempAllocator: Out of memory");
        // SAFETY: Thread counts and max jobs constants are valid parameters for Jolt.
        let job_system_ptr = unsafe {
            JPC_JobSystemThreadPool_new3(
                JPC_MAX_PHYSICS_JOBS as u32,
                JPC_MAX_PHYSICS_BARRIERS as u32,
                config.num_threads,
            )
        };
        let job_system =
            NonNull::new(job_system_ptr).expect("Failed to allocate Jolt JobSystem: Out of memory");

        Self {
            physics_system: ManuallyDrop::new(physics_system),
            temp_allocator,
            job_system,
            body_registry: std::collections::HashMap::new(),
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
        // SAFETY: The raw pointer from `physics_system` is valid.
        unsafe { joltc_sys::JPC_PhysicsSystem_GetBodyInterface(self.physics_system.raw()) }
    }

    /// Spawns a body into the Jolt physics world and returns its ID.
    #[allow(clippy::too_many_arguments)]
    pub fn spawn_body(
        &mut self,
        entity: Entity,
        collider: &Collider,
        scale: Vec3,
        transform: (Vec3, Quat),
        rigidbody: &RigidBody,
        linear_velocity: Vec3,
        angular_velocity: Vec3,
    ) -> Option<rolt::BodyId> {
        let (position, rotation) = transform;
        let shape_ptr = match collider {
            Collider::Box { half_extents } => create_box_shape(*half_extents * scale)?,
        };

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
        let id = rolt::BodyId::new(body_id);
        self.body_registry.insert(id, entity);
        Some(id)
    }

    /// Returns whether the specified body is active (awake) in the Jolt physics engine.
    pub fn is_active(&self, body_id: rolt::BodyId) -> bool {
        if body_id.raw() == INVALID_BODY_ID || !self.body_registry.contains_key(&body_id) {
            return false;
        }
        // SAFETY: The body ID is registered in body_registry and confirmed added before querying.
        unsafe {
            let body_interface = self.body_interface();
            if !joltc_sys::JPC_BodyInterface_IsAdded(body_interface, body_id.raw()) {
                return false;
            }
            joltc_sys::JPC_BodyInterface_IsActive(body_interface, body_id.raw())
        }
    }

    /// Teleports the body to a new position and rotation, forcefully waking it up.
    pub fn set_position_and_rotation(
        &mut self,
        body_id: rolt::BodyId,
        position: Vec3,
        rotation: Quat,
    ) {
        if body_id.raw() == INVALID_BODY_ID || !self.body_registry.contains_key(&body_id) {
            return;
        }
        // SAFETY: body_id is valid and registered.
        unsafe {
            let body_interface = self.body_interface();
            let pos = joltc_sys::JPC_Vec3 {
                x: position.x,
                y: position.y,
                z: position.z,
                _w: 0.0,
            };
            let rot = joltc_sys::JPC_Quat {
                x: rotation.x,
                y: rotation.y,
                z: rotation.z,
                w: rotation.w,
            };
            joltc_sys::JPC_BodyInterface_SetPositionAndRotation(
                body_interface,
                body_id.raw(),
                pos,
                rot,
                joltc_sys::JPC_ACTIVATION_ACTIVATE,
            );
        }
    }

    /// Retrieves the current position and rotation of the body from Jolt.
    pub fn get_transform(&self, body_id: rolt::BodyId) -> Option<(Vec3, Quat)> {
        if body_id.raw() == INVALID_BODY_ID || !self.body_registry.contains_key(&body_id) {
            return None;
        }
        // SAFETY: body_id is verified to be valid and allocated in this PhysicsWorld.
        unsafe {
            let body_interface = self.body_interface();

            if !joltc_sys::JPC_BodyInterface_IsAdded(body_interface, body_id.raw()) {
                return None;
            }

            let mut pos = joltc_sys::JPC_Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                _w: 0.0,
            };
            let mut rot = joltc_sys::JPC_Quat {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 0.0,
            };

            joltc_sys::JPC_BodyInterface_GetPositionAndRotation(
                body_interface,
                body_id.raw(),
                &mut pos,
                &mut rot,
            );

            Some((
                Vec3::new(pos.x, pos.y, pos.z),
                bevy::prelude::Quat::from_xyzw(rot.x, rot.y, rot.z, rot.w),
            ))
        }
    }
    /// Get Linear Velocity.
    pub fn get_linear_velocity(&self, body_id: rolt::BodyId) -> Option<Vec3> {
        if body_id.raw() == INVALID_BODY_ID || !self.body_registry.contains_key(&body_id) {
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
        if body_id.raw() == INVALID_BODY_ID || !self.body_registry.contains_key(&body_id) {
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
        if body_id.raw() == INVALID_BODY_ID || !self.body_registry.contains_key(&body_id) {
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
        if body_id.raw() == INVALID_BODY_ID || !self.body_registry.contains_key(&body_id) {
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

    /// Advances the physics simulation by `delta_time`, performing `collision_steps` substeps.
    pub fn step(&mut self, delta_time: f32, collision_steps: i32) {
        if delta_time <= 0.0 || !delta_time.is_finite() {
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

    /// Destroys a body in Jolt if it is registered to this PhysicsWorld.
    pub fn destroy_body(&mut self, body_id: rolt::BodyId) -> bool {
        if body_id.raw() == INVALID_BODY_ID {
            return false;
        }

        if self.body_registry.remove(&body_id).is_none() {
            return false;
        }

        // SAFETY: The body ID is valid because we only destroy it if it exists in our body_registry,
        // which guarantees it was allocated by this PhysicsWorld and has not yet been destroyed.
        unsafe {
            let body_interface = self.body_interface();
            if joltc_sys::JPC_BodyInterface_IsAdded(body_interface, body_id.raw()) {
                joltc_sys::JPC_BodyInterface_RemoveBody(body_interface, body_id.raw());
            }
            joltc_sys::JPC_BodyInterface_DestroyBody(body_interface, body_id.raw());
        }
        true
    }
}

impl FromWorld for PhysicsWorld {
    fn from_world(world: &mut World) -> Self {
        let config = world
            .get_resource::<PhysicsConfig>()
            .cloned()
            .unwrap_or_default();
        Self::new(config)
    }
}

impl Drop for PhysicsWorld {
    fn drop(&mut self) {
        //SAFETY: Clean up all live bodies before tearing down the physics system.
        unsafe {
            let body_interface = self.body_interface();
            for (body_id, _) in self.body_registry.drain() {
                if joltc_sys::JPC_BodyInterface_IsAdded(body_interface, body_id.raw()) {
                    joltc_sys::JPC_BodyInterface_RemoveBody(body_interface, body_id.raw());
                }
                joltc_sys::JPC_BodyInterface_DestroyBody(body_interface, body_id.raw());
            }
        }

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
    if !half_extents.is_finite() || half_extents.min_element() <= 0.0 {
        error!("Invalid box half_extents: {}", half_extents);
        return None;
    }

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
            let error_msg = extract_jolt_error(err);
            error!("Failed to create box shape: {error_msg}");
            None
        }
    }
}

// SAFETY: The Jolt `PhysicsSystem` is designed for multi-threaded access.
// SAFETY: The Jolt `PhysicsSystem` and `BodyInterface` are internally synchronized
// via mutexes (num_body_mutexes). We enforce that `PhysicsWorld` only exposes
// Jolt state mutation through `&mut self` (exclusive access), ensuring no
// data races can occur from the Rust side.
unsafe impl Send for PhysicsWorld {}
// SAFETY: See Send justification. Jolt's C++ locks make `&self` reads safe across threads.
unsafe impl Sync for PhysicsWorld {}

/// Safely extracts Jolt's FFI error string into a Rust String and deletes the C++ allocation.
///
/// # Safety
/// If `err` is non-null, it must point to a valid `JPC_String` allocated by Jolt.
unsafe fn extract_jolt_error(err: *mut joltc_sys::JPC_String) -> String {
    if err.is_null() {
        return "unknown Jolt error".to_string();
    }

    // SAFETY: The caller guarantees `err` is a valid `JPC_String`. We extract the string and delete the allocation.
    unsafe {
        let c_str = std::ffi::CStr::from_ptr(joltc_sys::JPC_String_c_str(err));
        let message = c_str.to_string_lossy().into_owned();
        joltc_sys::JPC_String_delete(err);
        message
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_physics_system_getter() {
        let physics_world = PhysicsWorld::new(PhysicsConfig::default());
        let system = physics_world.physics_system();

        // Assert that the raw C++ pointer inside the system successfully initialized
        assert!(
            !system.raw().is_null(),
            "The C++ Jolt PhysicsSystem pointer was null!"
        );
    }

    #[test]
    fn test_set_position_and_rotation_valid_body() {
        let mut physics_world = PhysicsWorld::new(PhysicsConfig::default());
        let body_id = physics_world
            .spawn_body(
                Entity::PLACEHOLDER,
                &crate::components::Collider::Box {
                    half_extents: Vec3::splat(1.0),
                },
                Vec3::ONE,
                (Vec3::ZERO, Quat::IDENTITY),
                &RigidBody::Dynamic,
                Vec3::ZERO,
                Vec3::ZERO,
            )
            .expect("Failed to spawn body");

        let new_pos = Vec3::new(10.0, 20.0, 30.0);
        let new_rot = Quat::from_rotation_x(std::f32::consts::PI / 2.0);

        physics_world.set_position_and_rotation(body_id, new_pos, new_rot);

        let (fetched_pos, fetched_rot) = physics_world.get_transform(body_id).unwrap();
        assert!(fetched_pos.distance(new_pos) < 1e-4);
        // Quat::angle_between requires normalized quats, and angle might be slightly off.
        // Dot product is safe.
        assert!((1.0 - fetched_rot.dot(new_rot).abs()) < 1e-4);
    }

    #[test]
    fn test_set_position_and_rotation_invalid_body_noop() {
        let mut physics_world = PhysicsWorld::new(PhysicsConfig::default());
        let invalid_id = rolt::BodyId::new(INVALID_BODY_ID);
        // Should not panic or crash
        physics_world.set_position_and_rotation(invalid_id, Vec3::ZERO, Quat::IDENTITY);

        let fake_id = rolt::BodyId::new(9999);
        physics_world.set_position_and_rotation(fake_id, Vec3::ZERO, Quat::IDENTITY);
    }

    #[test]
    fn test_spawn_static_box() {
        let mut physics_world = PhysicsWorld::new(PhysicsConfig::default());
        let rigidbody = RigidBody::Static;
        let box_size = Vec3::splat(1.0);

        let result = physics_world.spawn_body(
            Entity::PLACEHOLDER,
            &crate::components::Collider::Box {
                half_extents: box_size,
            },
            Vec3::ONE,
            (Vec3::ZERO, Quat::IDENTITY),
            &rigidbody,
            Vec3::ZERO,
            Vec3::ZERO,
        );

        assert!(result.is_some());
    }

    #[test]
    fn test_spawn_kinematic_box() {
        let mut physics_world = PhysicsWorld::new(PhysicsConfig::default());
        let rigidbody = RigidBody::Kinematic;
        let box_size = Vec3::splat(1.0);

        let result = physics_world.spawn_body(
            Entity::PLACEHOLDER,
            &crate::components::Collider::Box {
                half_extents: box_size,
            },
            Vec3::ONE,
            (Vec3::ZERO, Quat::IDENTITY),
            &rigidbody,
            Vec3::ZERO,
            Vec3::ZERO,
        );

        assert!(result.is_some());
    }

    #[test]
    fn test_create_box_shape_failure() {
        let mut physics_world = PhysicsWorld::new(PhysicsConfig::default());
        let rigidbody = RigidBody::Static;
        let box_size = Vec3::splat(-1.0);

        let result = physics_world.spawn_body(
            Entity::PLACEHOLDER,
            &crate::components::Collider::Box {
                half_extents: box_size,
            },
            Vec3::ONE,
            (Vec3::ZERO, Quat::IDENTITY),
            &rigidbody,
            Vec3::ZERO,
            Vec3::ZERO,
        );

        assert!(result.is_none());
    }

    #[test]
    fn test_get_transform_invalid_and_destroyed_body() {
        let mut physics_world = PhysicsWorld::new(PhysicsConfig::default());

        // 1. Querying an invalid body ID returns None
        let invalid_id = rolt::BodyId::new(INVALID_BODY_ID);
        assert!(physics_world.get_transform(invalid_id).is_none());

        // 2. Querying an unallocated body ID returns None
        let fake_id = rolt::BodyId::new(9999);
        assert!(physics_world.get_transform(fake_id).is_none());

        // 3. Spawning a valid box returns Some(...)
        let body_id = physics_world
            .spawn_body(
                Entity::PLACEHOLDER,
                &crate::components::Collider::Box {
                    half_extents: Vec3::splat(1.0),
                },
                Vec3::ONE,
                (Vec3::new(1.0, 2.0, 3.0), Quat::IDENTITY),
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
    fn test_spawn_body_stores_entity_user_data() {
        let mut physics_world = PhysicsWorld::new(PhysicsConfig::default());
        let test_entity = Entity::from_raw_u32(42).unwrap();

        let body_id = physics_world
            .spawn_body(
                test_entity,
                &crate::components::Collider::Box {
                    half_extents: Vec3::splat(1.0),
                },
                Vec3::ONE,
                (Vec3::ZERO, Quat::IDENTITY),
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
        let mut physics_world = PhysicsWorld::new(PhysicsConfig::default());

        // Calling step with 0.0, negative dt, or NaN should safely return without panic
        physics_world.step(0.0, 1);
        physics_world.step(-1.0 / 60.0, 1);
        physics_world.step(f32::NAN, 1);
    }

    #[test]
    fn test_velocity_getters_and_setters() {
        let mut physics_world = PhysicsWorld::new(PhysicsConfig::default());
        let rigidbody = RigidBody::Dynamic;
        let box_size = Vec3::splat(1.0);

        let body_id = physics_world
            .spawn_body(
                Entity::PLACEHOLDER,
                &crate::components::Collider::Box {
                    half_extents: box_size,
                },
                Vec3::ONE,
                (Vec3::ZERO, Quat::IDENTITY),
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

    #[test]
    fn test_is_active_lifecycle() {
        let mut physics_world = PhysicsWorld::new(PhysicsConfig::default());

        // 1. Invalid and unallocated IDs return false
        assert!(!physics_world.is_active(rolt::BodyId::new(INVALID_BODY_ID)));
        assert!(!physics_world.is_active(rolt::BodyId::new(9999)));

        // 2. Newly spawned dynamic box is active
        let body_id = physics_world
            .spawn_body(
                Entity::PLACEHOLDER,
                &crate::components::Collider::Box {
                    half_extents: Vec3::splat(1.0),
                },
                Vec3::ONE,
                (Vec3::ZERO, Quat::IDENTITY),
                &RigidBody::Dynamic,
                Vec3::ZERO,
                Vec3::ZERO,
            )
            .expect("Failed to spawn box");
        assert!(physics_world.is_active(body_id));

        // 3. Destroyed body returns false
        physics_world.destroy_body(body_id);
        assert!(!physics_world.is_active(body_id));
    }

    #[test]
    fn test_extract_jolt_error_handles_null_pointer_gracefully() {
        // SAFETY: Testing graceful null handling
        let message = unsafe { extract_jolt_error(std::ptr::null_mut()) };
        assert_eq!(message, "unknown Jolt error");
    }

    #[test]
    fn test_extract_jolt_error_extracts_and_frees_real_jolt_string() {
        let _world = PhysicsWorld::new(PhysicsConfig::default());
        let mut shape: *mut joltc_sys::JPC_Shape = std::ptr::null_mut();
        let mut err: *mut joltc_sys::JPC_String = std::ptr::null_mut();

        let settings = joltc_sys::JPC_BoxShapeSettings {
            HalfExtent: joltc_sys::JPC_Vec3 {
                x: -1.0,
                y: -1.0,
                z: -1.0,
                _w: 0.0,
            },
            ..Default::default()
        };

        // Trigger Jolt C++ to allocate an actual JPC_String error
        // SAFETY: `settings` is initialized, and `shape`/`err` are valid output pointers.
        let success =
            unsafe { joltc_sys::JPC_BoxShapeSettings_Create(&settings, &mut shape, &mut err) };
        assert!(!success, "Shape creation must fail for negative extents");
        assert!(
            !err.is_null(),
            "Jolt must allocate an error string on failure"
        );

        // SAFETY: err is a valid JPC_String allocated by Jolt.
        // Verifies conversion to Rust String AND that JPC_String_delete succeeds without crash.
        let error_message = unsafe { extract_jolt_error(err) };
        assert!(
            !error_message.is_empty(),
            "Error message should not be empty"
        );
        assert_ne!(
            error_message, "unknown Jolt error",
            "Should contain actual Jolt error details"
        );
    }

    #[test]
    fn test_create_box_shape_valid_extents_succeeds() {
        let _world = PhysicsWorld::new(PhysicsConfig::default());
        let shape = create_box_shape(Vec3::splat(1.0));
        assert!(shape.is_some(), "Valid half-extents must produce a shape");

        let shape_ptr = shape.unwrap();
        assert!(!shape_ptr.is_null());
        // Clean up the ref-counted shape
        // SAFETY: `shape_ptr` is a valid shape created by `create_box_shape`.
        unsafe { joltc_sys::JPC_Shape_Release(shape_ptr) };
    }

    #[test]
    fn test_create_box_shape_negative_extents_returns_none() {
        let _world = PhysicsWorld::new(PhysicsConfig::default());
        let shape = create_box_shape(Vec3::splat(-1.0));
        assert!(shape.is_none(), "Negative half-extents must return None");
    }

    #[test]
    fn test_create_box_shape_zero_extents_returns_none() {
        let _world = PhysicsWorld::new(PhysicsConfig::default());
        let shape = create_box_shape(Vec3::ZERO);
        assert!(shape.is_none(), "Zero half-extents must return None");
    }

    #[test]
    fn test_physics_world_drop_destroys_live_bodies_cleanly() {
        let mut physics_world = PhysicsWorld::new(PhysicsConfig::default());

        // Spawn multiple dynamic bodies
        for i in 0..10 {
            let entity = Entity::from_raw_u32(i + 1).unwrap();
            physics_world.spawn_body(
                entity,
                &crate::components::Collider::Box {
                    half_extents: Vec3::splat(1.0),
                },
                Vec3::ONE,
                (Vec3::new(i as f32, 0.0, 0.0), Quat::IDENTITY),
                &RigidBody::Dynamic,
                Vec3::ZERO,
                Vec3::ZERO,
            );
        }

        // Dropping physics_world with active bodies must not trigger JPH_ASSERT(mNumBodies == 0)
        drop(physics_world);
    }

    #[test]
    fn test_getters_return_none_for_unallocated_body_id() {
        let physics_world = PhysicsWorld::new(PhysicsConfig::default());
        let unallocated = rolt::BodyId::new(99_999);

        assert!(physics_world.get_transform(unallocated).is_none());
        assert!(physics_world.get_linear_velocity(unallocated).is_none());
        assert!(physics_world.get_angular_velocity(unallocated).is_none());
    }

    #[test]
    fn test_setters_noop_for_unallocated_body_id() {
        let mut physics_world = PhysicsWorld::new(PhysicsConfig::default());
        let unallocated = rolt::BodyId::new(99_999);

        // Mutating unallocated ID must safely no-op without memory corruption
        physics_world.set_linear_velocity(unallocated, Vec3::X);
        physics_world.set_angular_velocity(unallocated, Vec3::Y);
    }

    #[test]
    #[should_panic(expected = "temp_allocator_size_mb in bytes overflows u32")]
    fn test_temp_allocator_overflow_panics_safely() {
        let config = PhysicsConfig {
            temp_allocator_size_mb: u32::MAX,
            ..Default::default()
        };
        let _ = PhysicsWorld::new(config);
    }
}
