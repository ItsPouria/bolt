use bevy::prelude::*;
use rolt::BodyId;

/// Defines the motion type and physical behavior of a body.
#[non_exhaustive]
#[derive(Component, Clone, Debug, Eq, PartialEq, Reflect)]
#[reflect(Component)]
#[require(Transform, LinearVelocity, AngularVelocity)]
pub enum RigidBody {
    /// A dynamic body affected by forces, gravity, and impulses (e.g. crates, debris).
    Dynamic,
    /// A static body that does not move and has infinite mass (e.g. terrain, floors, walls).
    Static,
    /// A kinematic body that is not affected by gravity or forces, but can push dynamic bodies.
    /// It is moved manually by setting its velocity or position.
    Kinematic,
}

/// The geometric collision shape attached to a rigid body.
///
/// All dimensions are in local space (meters) before the entity's `Transform::scale` is applied.
/// When spawned, Bolt will automatically scale these primitives based on the entity's global scale.
#[non_exhaustive]
#[derive(Component, Clone, Debug, PartialEq, Reflect)]
#[reflect(Component)]
pub enum Collider {
    /// A 3D box defined by its half-extents from the center.
    ///
    /// The total width, height, and depth of the box are exactly twice the half-extents.
    Box {
        /// The distance from the center of the box to its faces along the local X, Y, and Z axes (in meters).
        half_extents: Vec3,
    },
    /// A perfectly spherical collision volume.
    ///
    /// **Note on scaling:** True ellipsoids are not supported by the underlying physics solver.
    /// If a non-uniform scale is applied to the entity, the sphere will be scaled uniformly
    /// by the largest component of the scale vector (`scale.max_element()`).
    Sphere {
        /// The distance from the center to the surface of the sphere (in meters).
        radius: f32,
    },
    /// A 3D capsule (a cylinder capped with two hemispheres).
    ///
    /// The capsule is aligned perfectly along the local Y axis. The total height of the
    /// capsule is `(half_height * 2.0) + (radius * 2.0)`.
    ///
    /// **Note on scaling:** Non-uniform scaling on the X and Z axes will use the larger
    /// of the two values to scale the radius uniformly. The Y axis scales the height independently.
    Capsule {
        /// Half the height of the inner cylindrical portion of the capsule (in meters).
        /// Does not include the hemispherical end caps.
        half_height: f32,
        /// The radius of the central cylinder and the hemispherical end caps (in meters).
        radius: f32,
    },
    /// A 3D cylinder aligned along the local Y axis with flat circular end caps.
    ///
    /// **Note on scaling:** Non-uniform scaling on the X and Z axes will use the larger
    /// of the two values to scale the radius uniformly. The Y axis scales the height independently.
    Cylinder {
        /// The distance from the center of the cylinder to the flat top and bottom caps (in meters).
        half_height: f32,
        /// The radius of the circular cross-section (in meters).
        radius: f32,
    },
}

/// A component attached to Bevy entities that have a live Jolt Physics body.
#[derive(Component, Debug)]
#[component(immutable)]
pub struct JoltBody(pub(crate) BodyId);

impl JoltBody {
    /// Returns the internal Jolt `BodyId` for this entity.
    pub fn id(&self) -> BodyId {
        self.0
    }
}

/// The linear velocity of a rigid body in meters per second.
///
/// Modifying this component will update the body's velocity in the physics engine.
#[derive(Component, Debug, Clone, Copy, PartialEq, Default, Deref, DerefMut, Reflect)]
#[reflect(Component)]
pub struct LinearVelocity(pub Vec3);

/// The angular velocity of a rigid body in radians per second around the local X, Y, and Z axes.
///
/// Modifying this component will update the body's angular velocity in the physics engine.
#[derive(Component, Debug, Clone, Copy, PartialEq, Default, Deref, DerefMut, Reflect)]
#[reflect(Component)]
pub struct AngularVelocity(pub Vec3);

/// A marker component used to efficiently filter out static bodies from transform synchronization.
#[derive(Component, Debug, Clone, Copy, PartialEq, Default, Reflect)]
#[reflect(Component)]
pub struct StaticMarker;

/// Enables Continuous Collision Detection (CCD) for a dynamic rigid body.
///
/// Use this on fast-moving objects to prevent them from tunneling through walls
/// or visually penetrating the floor.
#[derive(Component, Debug, Clone, Copy, PartialEq, Default, Reflect)]
#[reflect(Component)]
pub struct ContinuousCollision;
