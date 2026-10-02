use crate::world::PhysicsWorld;
use bevy::prelude::*;

#[derive(Resource, Debug, Clone, Deref, DerefMut)]
/// Gravity struct.
pub struct Gravity(pub Vec3);

impl Default for Gravity {
    fn default() -> Self {
        Self(Vec3::new(0.0, -9.81, 0.0))
    }
}

/// Apply Gravity.
pub fn apply_gravity(mut world: ResMut<PhysicsWorld>, gravity: Res<Gravity>) {
    if gravity.is_changed() {
        world.set_gravity(gravity.0);
    }
}
