use bevy::ecs::change_detection::DetectChanges;
use bevy::ecs::resource::Resource;
use bevy::ecs::system::{Res, ResMut};
use bevy::math::Vec3;

use crate::world::PhysicsWorld;

#[derive(Resource, Debug, Clone, Default)]
pub struct Gravity(pub Vec3);

pub fn apply_gravity(mut world: ResMut<PhysicsWorld>, gravity: Res<Gravity>) {
    if gravity.is_changed() {
        world.set_gravity(gravity.0);
    }
}
