use bevy::prelude::*;
use rolt::BodyId;

/// Defines the motion type and physical behavior of a body.
#[non_exhaustive]
#[derive(Component, Clone, Debug, Eq, PartialEq, Reflect)]
#[reflect(Component)]
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
#[non_exhaustive]
#[derive(Component, Clone, Debug, PartialEq, Reflect)]
#[reflect(Component)]
pub enum Collider {
    /// A 3D box defined by its half-extents from the center.
    Box {
        /// Half-extents along the X, Y, and Z axes (width / 2, height / 2, depth / 2).
        half_extents: Vec3,
    },
}

/// A component attached to Bevy entities that have a live Jolt Physics body.
#[derive(Component, Clone, Copy, Debug, PartialEq, Eq)]
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
