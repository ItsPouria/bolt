use bevy::prelude::*;

#[derive(Component, Clone, Debug, PartialEq)]
pub enum RigidBody {
    Dynamic,
    Static,
}

#[derive(Component, Clone, Debug, PartialEq)]
pub enum Collider {
    Box { half_extents: Vec3 },
}
