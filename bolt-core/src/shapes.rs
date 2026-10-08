//! Geometry factory for Jolt collision shapes.
//!
//! This module wraps the raw `joltc-sys` shape-creation FFI behind narrow,
//! validating constructors. Every function:
//!
//! 1. Rejects non-finite or non-positive dimensions up front (Jolt asserts on
//!    invalid input in debug builds, which would abort the process).
//! 2. Delegates to the corresponding `JPC_*ShapeSettings_Create` C call.
//! 3. Frees the error string Jolt may allocate, so no C++ memory leaks.
//! 4. Returns the raw, reference-counted shape pointer on success, or `None`
//!    if the shape could not be built.
//!
//! # Ownership
//!
//! On success, the returned [`JPC_Shape`] pointer carries a single reference.
//! The caller must either transfer ownership to a body (via
//! `JPC_BodyCreationSettings`, after which Jolt manages the reference) or
//! release it explicitly with `JPC_Shape_Release` to avoid leaking the C++
//! allocation.
//!
//! # Errors
//!
//! Failures are reported as `None` rather than a typed error: callers of this
//! engine-agnostic crate decide for themselves whether (and how) to log. Any
//! diagnostic detail from Jolt is consumed internally while freeing the error
//! string.

// TODO(phase-2): tighten these constructors to `pub(crate)` once
// `CorePhysicsWorld::spawn_body` lives in this crate and becomes the sole
// consumer of the factory.

use std::ptr;

use glam::Vec3;
use joltc_sys::{
    JPC_BoxShapeSettings, JPC_BoxShapeSettings_Create, JPC_CapsuleShapeSettings,
    JPC_CapsuleShapeSettings_Create, JPC_CylinderShapeSettings, JPC_CylinderShapeSettings_Create,
    JPC_Shape, JPC_SphereShapeSettings, JPC_SphereShapeSettings_Create, JPC_String, JPC_Vec3,
};

/// Creates a box (rectangular prism) shape from its half-extents.
///
/// `half_extents` are the distances from the box's center to its faces along
/// each local axis, in meters. Every component must be finite and strictly
/// positive.
///
/// A convex radius of 0.05 m is applied, rounding the box's edges slightly to
/// improve the stability of collision detection.
pub fn create_box_shape(half_extents: Vec3) -> Option<*mut JPC_Shape> {
    if !half_extents.is_finite() || half_extents.min_element() <= 0.0 {
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
        ConvexRadius: 0.05,
        ..Default::default()
    };

    // SAFETY: `settings` is fully initialized, and `shape`/`err` are valid
    // out-pointers. Jolt either populates `shape` or allocates an error
    // string into `err`, which we free below.
    unsafe {
        if JPC_BoxShapeSettings_Create(&settings, &mut shape, &mut err) {
            Some(shape)
        } else {
            extract_jolt_error(err);
            None
        }
    }
}

/// Creates a sphere shape.
///
/// `radius` is the distance from the center to the surface, in meters. It
/// must be finite and strictly positive.
pub fn create_sphere_shape(radius: f32) -> Option<*mut JPC_Shape> {
    if radius <= 0.0 || !radius.is_finite() {
        return None;
    }
    let mut shape: *mut JPC_Shape = ptr::null_mut();
    let mut err: *mut JPC_String = ptr::null_mut();
    let settings = JPC_SphereShapeSettings {
        Radius: radius,
        ..Default::default()
    };
    // SAFETY: `settings` is fully initialized, and `shape`/`err` are valid
    // out-pointers. The error string, if any, is freed before returning.
    unsafe {
        JPC_SphereShapeSettings_Create(&settings, &mut shape, &mut err);
        extract_jolt_error(err);
    }
    if shape.is_null() { None } else { Some(shape) }
}

/// Creates a capsule shape: a cylinder capped with two hemispheres, aligned
/// along the local Y axis.
///
/// `half_height` is half the height of the inner cylindrical portion
/// (excluding the hemispherical caps), and `radius` is shared by the cylinder
/// and the caps. Both are in meters. `radius` must be finite and strictly
/// positive; `half_height` must be finite and non-negative.
pub fn create_capsule_shape(half_height: f32, radius: f32) -> Option<*mut JPC_Shape> {
    if radius <= 0.0 || half_height < 0.0 || !radius.is_finite() || !half_height.is_finite() {
        return None;
    }
    let mut shape: *mut JPC_Shape = ptr::null_mut();
    let mut err: *mut JPC_String = ptr::null_mut();
    let settings = JPC_CapsuleShapeSettings {
        HalfHeightOfCylinder: half_height,
        Radius: radius,
        ..Default::default()
    };
    // SAFETY: `settings` is fully initialized, and `shape`/`err` are valid
    // out-pointers. The error string, if any, is freed before returning.
    unsafe {
        JPC_CapsuleShapeSettings_Create(&settings, &mut shape, &mut err);
        extract_jolt_error(err);
    }
    if shape.is_null() { None } else { Some(shape) }
}

/// Creates a cylinder shape with flat circular end caps, aligned along the
/// local Y axis.
///
/// `half_height` is the distance from the center to either flat cap, and
/// `radius` is the radius of the circular cross-section. Both are in meters
/// and must be finite and strictly positive.
///
/// A convex radius of 0.05 m is applied, rounding the cylinder's edges
/// slightly to improve the stability of collision detection.
pub fn create_cylinder_shape(half_height: f32, radius: f32) -> Option<*mut JPC_Shape> {
    if radius <= 0.0 || half_height <= 0.0 || !radius.is_finite() || !half_height.is_finite() {
        return None;
    }
    let mut shape: *mut JPC_Shape = ptr::null_mut();
    let mut err: *mut JPC_String = ptr::null_mut();
    let settings = JPC_CylinderShapeSettings {
        HalfHeight: half_height,
        Radius: radius,
        ConvexRadius: 0.05,
        ..Default::default()
    };
    // SAFETY: `settings` is fully initialized, and `shape`/`err` are valid
    // out-pointers. The error string, if any, is freed before returning.
    unsafe {
        JPC_CylinderShapeSettings_Create(&settings, &mut shape, &mut err);
        extract_jolt_error(err);
    }
    if shape.is_null() { None } else { Some(shape) }
}

/// Converts a Jolt FFI error string into a Rust `String` and frees the C++
/// allocation, returning `"unknown Jolt error"` when `err` is null.
///
/// # Safety
/// If `err` is non-null, it must point to a valid `JPC_String` allocated by
/// Jolt. The string is deleted here; the pointer must not be used afterwards.
unsafe fn extract_jolt_error(err: *mut JPC_String) -> String {
    if err.is_null() {
        return "unknown Jolt error".to_string();
    }

    // SAFETY: The caller guarantees `err` is a valid `JPC_String`. We copy the
    // bytes into an owned Rust String and then delete the allocation.
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
    use crate::init::ensure_jolt_initialized;
    use joltc_sys::JPC_Shape_Release;

    // --- Box Tests ---

    #[test]
    fn test_create_box_shape_valid_extents_succeeds() {
        ensure_jolt_initialized();
        let shape = create_box_shape(Vec3::splat(1.0));
        assert!(shape.is_some(), "Valid half-extents must produce a shape");

        let shape_ptr = shape.unwrap();
        assert!(!shape_ptr.is_null());
        // SAFETY: `shape_ptr` is a valid shape created by `create_box_shape`;
        // releasing it here drops the only reference.
        unsafe { JPC_Shape_Release(shape_ptr) };
    }

    #[test]
    fn test_create_box_shape_negative_extents_returns_none() {
        ensure_jolt_initialized();
        let shape = create_box_shape(Vec3::splat(-1.0));
        assert!(shape.is_none(), "Negative half-extents must return None");
    }

    #[test]
    fn test_create_box_shape_zero_extents_returns_none() {
        ensure_jolt_initialized();
        let shape = create_box_shape(Vec3::ZERO);
        assert!(shape.is_none(), "Zero half-extents must return None");
    }

    #[test]
    fn test_extract_jolt_error_handles_null_pointer_gracefully() {
        // SAFETY: A null pointer is explicitly supported by the contract.
        let message = unsafe { extract_jolt_error(std::ptr::null_mut()) };
        assert_eq!(message, "unknown Jolt error");
    }

    #[test]
    fn test_extract_jolt_error_extracts_and_frees_real_jolt_string() {
        ensure_jolt_initialized();
        let mut shape: *mut JPC_Shape = std::ptr::null_mut();
        let mut err: *mut JPC_String = std::ptr::null_mut();

        let settings = JPC_BoxShapeSettings {
            HalfExtent: JPC_Vec3 {
                x: -1.0,
                y: -1.0,
                z: -1.0,
                _w: 0.0,
            },
            ..Default::default()
        };

        // Trigger Jolt C++ to allocate an actual error string.
        // SAFETY: `settings` is initialized, and `shape`/`err` are valid
        // out-pointers.
        let success = unsafe { JPC_BoxShapeSettings_Create(&settings, &mut shape, &mut err) };
        assert!(!success, "Shape creation must fail for negative extents");
        assert!(
            !err.is_null(),
            "Jolt must allocate an error string on failure"
        );

        // SAFETY: `err` is a valid `JPC_String` allocated by Jolt. Verifies
        // conversion to a Rust String AND that `JPC_String_delete` succeeds
        // without crashing.
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

    // --- Sphere Tests ---

    #[test]
    fn test_create_sphere_shape_valid_radius_succeeds() {
        ensure_jolt_initialized();
        let shape = create_sphere_shape(1.0);
        assert!(shape.is_some(), "Valid radius must produce a shape");
        let shape_ptr = shape.unwrap();
        assert!(!shape_ptr.is_null());
        // SAFETY: `shape_ptr` is a valid shape created by this factory;
        // releasing it here drops the only reference.
        unsafe { JPC_Shape_Release(shape_ptr) };
    }

    #[test]
    fn test_create_sphere_shape_zero_radius_returns_none() {
        ensure_jolt_initialized();
        let shape = create_sphere_shape(0.0);
        assert!(shape.is_none(), "Zero radius must return None");
    }

    #[test]
    fn test_create_sphere_shape_negative_radius_returns_none() {
        ensure_jolt_initialized();
        let shape = create_sphere_shape(-1.0);
        assert!(shape.is_none(), "Negative radius must return None");
    }

    // --- Capsule Tests ---

    #[test]
    fn test_create_capsule_shape_valid_dimensions_succeeds() {
        ensure_jolt_initialized();
        let shape = create_capsule_shape(1.0, 0.5);
        assert!(shape.is_some(), "Valid dimensions must produce a shape");
        let shape_ptr = shape.unwrap();
        assert!(!shape_ptr.is_null());
        // SAFETY: `shape_ptr` is a valid shape created by this factory;
        // releasing it here drops the only reference.
        unsafe { JPC_Shape_Release(shape_ptr) };
    }

    #[test]
    fn test_create_capsule_shape_zero_radius_returns_none() {
        ensure_jolt_initialized();
        let shape = create_capsule_shape(1.0, 0.0);
        assert!(shape.is_none(), "Zero radius must return None");
    }

    #[test]
    fn test_create_capsule_shape_negative_height_returns_none() {
        ensure_jolt_initialized();
        let shape = create_capsule_shape(-1.0, 0.5);
        assert!(shape.is_none(), "Negative height must return None");
    }

    // --- Cylinder Tests ---

    #[test]
    fn test_create_cylinder_shape_valid_dimensions_succeeds() {
        ensure_jolt_initialized();
        let shape = create_cylinder_shape(1.0, 0.5);
        assert!(shape.is_some(), "Valid dimensions must produce a shape");
        let shape_ptr = shape.unwrap();
        assert!(!shape_ptr.is_null());
        // SAFETY: `shape_ptr` is a valid shape created by this factory;
        // releasing it here drops the only reference.
        unsafe { JPC_Shape_Release(shape_ptr) };
    }

    #[test]
    fn test_create_cylinder_shape_zero_radius_returns_none() {
        ensure_jolt_initialized();
        let shape = create_cylinder_shape(1.0, 0.0);
        assert!(shape.is_none(), "Zero radius must return None");
    }

    #[test]
    fn test_create_cylinder_shape_negative_height_returns_none() {
        ensure_jolt_initialized();
        let shape = create_cylinder_shape(-1.0, 0.5);
        assert!(shape.is_none(), "Negative height must return None");
    }
}
