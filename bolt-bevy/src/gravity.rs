use crate::world::PhysicsWorld;
use bevy::prelude::*;

#[derive(Resource, Debug, Clone, Deref, DerefMut, Reflect)]
#[reflect(Resource)]
/// Resource that defines the global gravity vector applied to all dynamic bodies.
pub struct Gravity(pub Vec3);

impl Default for Gravity {
    fn default() -> Self {
        Self(Vec3::new(0.0, -9.81, 0.0))
    }
}

/// System that applies the global gravity vector to all dynamic bodies each tick.
pub fn apply_gravity(mut world: ResMut<PhysicsWorld>, gravity: Res<Gravity>) {
    if gravity.is_changed() {
        world.set_gravity(gravity.0);
    }
}
