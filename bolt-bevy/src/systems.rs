use crate::components::{Collider, JoltBodyId, RigidBody};
use crate::world::PhysicsWorld;
use bevy::prelude::*;
use joltc_sys::*;

/// Detects new entities with RigidBody/Collider components and creates them in Jolt.
pub fn spawn_bodies(
    mut commands: Commands,
    world: ResMut<PhysicsWorld>,
    query: Query<(Entity, &Transform, &RigidBody, &Collider), Without<JoltBodyId>>,
) {
    let body_interface = world.physics_system().body_interface();

    for (entity, transform, rigid_body, collider) in query.iter() {
        let shape_ptr = match collider {
            Collider::Box { half_extents } => {
                let mut shape_settings: JPC_BoxShapeSettings = unsafe { std::mem::zeroed() };
                unsafe { JPC_BoxShapeSettings_default(&mut shape_settings) };

                shape_settings.HalfExtent = JPC_Vec3 {
                    x: half_extents.x,
                    y: half_extents.y,
                    z: half_extents.z,
                    _w: 0.0,
                };

                let mut shape_ptr: *mut JPC_Shape = std::ptr::null_mut();
                let mut error_ptr: *mut JPC_String = std::ptr::null_mut();

                let success = unsafe {
                    JPC_BoxShapeSettings_Create(&shape_settings, &mut shape_ptr, &mut error_ptr)
                };

                if !success || shape_ptr.is_null() {
                    panic!("Failed to create Jolt BoxShape");
                }
                shape_ptr
            }
        };

        let mut body_settings: JPC_BodyCreationSettings = unsafe { std::mem::zeroed() };
        unsafe { JPC_BodyCreationSettings_default(&mut body_settings) };

        body_settings.Position = JPC_RVec3 {
            x: transform.translation.x,
            y: transform.translation.y,
            z: transform.translation.z,
            _w: 0.0,
        };

        body_settings.Rotation = JPC_Quat {
            x: transform.rotation.x,
            y: transform.rotation.y,
            z: transform.rotation.z,
            w: transform.rotation.w,
        };

        body_settings.Shape = shape_ptr;
        body_settings.MotionType = match rigid_body {
            RigidBody::Dynamic => JPC_MOTION_TYPE_DYNAMIC,
            RigidBody::Static => JPC_MOTION_TYPE_STATIC,
        };
        body_settings.ObjectLayer = 0;

        let body = unsafe { body_interface.create_body(&body_settings) };

        let Some(body) = body else {
            bevy::log::error!(
                "Failed to create Jolt body for entity {:?}: Physics world may be full",
                entity
            );
            continue;
        };

        body_interface.add_body(body.id(), JPC_ACTIVATION_ACTIVATE);

        commands.entity(entity).insert(JoltBodyId(body.id()));
    }
}
