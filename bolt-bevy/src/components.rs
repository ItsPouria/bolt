use bevy::prelude::*;
use rolt::BodyId;

/// Defines the motion type and physical behavior of a body.
#[derive(Component, Clone, Debug, PartialEq)]
pub enum RigidBody {
    /// A dynamic body affected by forces, gravity, and impulses (e.g. crates, debris).
    Dynamic,
    /// A static body that does not move and has infinite mass (e.g. terrain, floors, walls).
    Static,
}

/// The geometric collision shape attached to a rigid body.
#[derive(Component, Clone, Debug, PartialEq)]
pub enum Collider {
    /// A 3D box defined by its half-extents from the center.
    Box {
        /// Half-extents along the X, Y, and Z axes (width / 2, height / 2, depth / 2).
        half_extents: Vec3,
    },
}

/// A component attached to Bevy entities that have a live Jolt Physics body.
#[derive(Component, Clone, Copy, Debug, PartialEq, Eq)]
pub struct JoltBody(pub BodyId);
