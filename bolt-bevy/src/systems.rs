use bevy::prelude::*;

use crate::prelude::{Collider, RigidBody};
use crate::registry::PhysicsRegistry;
use crate::world::PhysicsWorld;

pub fn spawn_physics_bodies(
    query: Query<(Entity, &Transform, &RigidBody, &Collider), Added<RigidBody>>,
    mut physics_world: ResMut<PhysicsWorld>,
    mut registry: ResMut<PhysicsRegistry>,
) {
    for (entity, transform, rigidbody, collider) in query.iter() {
        // We can cleanly match on any new collider shapes we add in the future!
        let body_id = match collider {
            Collider::Box { half_extents } => {
                physics_world.spawn_box(*half_extents, transform, rigidbody)
            }
        };

        if let Some(id) = body_id {
            registry.register(entity, id);
        } else {
            error!("Failed to spawn physics body for entity {:?}", entity);
        }
    }
}

pub fn sync_transforms(
    mut query: Query<(Entity, &mut Transform), With<RigidBody>>,
    physics_registry: Res<PhysicsRegistry>,
    physics_world: Res<PhysicsWorld>,
) {
    for (entity, mut transform) in query.iter_mut() {
        let Some(body_id) = physics_registry.get_body(entity) else {
            continue;
        };

        let Some((new_pos, new_rot)) = physics_world.get_transform(body_id) else {
            continue;
        };

        transform.translation = new_pos;
        transform.rotation = new_rot;
    }
}
