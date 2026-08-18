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

#[derive(Component, Debug, Clone, Copy)]
pub struct JoltBodyId(pub BodyId)
