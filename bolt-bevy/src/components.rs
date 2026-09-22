use bevy::prelude::*;
use rolt::BodyId;

#[derive(Component, Clone, Debug, PartialEq)]
pub enum RigidBody {
    Dynamic,
    Static,
}

#[derive(Component, Clone, Debug, PartialEq)]
pub enum Collider {
    Box { half_extents: Vec3 },
}

/// A component attached to Bevy entities that have a live Jolt Physics body.
#[derive(Component, Clone, Copy, Debug, PartialEq, Eq)]
pub struct JoltBody(pub BodyId);
