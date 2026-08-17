use bevy::ecs::change_detection::DetectChanges;
use bevy::ecs::resource::Resource;
use bevy::ecs::system::{Res, ResMut};
use bevy::math::Vec3;

use crate::world::PhysicsWorld;

#[derive(Resource, Debug, Clone)]
pub struct Gravity(pub Vec3);

impl Default for Gravity {
    fn default() -> Self {
        Self(Vec3::new(0.0, -9.81, 0.0))
    }
}

pub fn apply_gravity(mut world: ResMut<PhysicsWorld>, gravity: Res<Gravity>) {
    if gravity.is_changed() {
        world.set_gravity(gravity.0);
    }
}
