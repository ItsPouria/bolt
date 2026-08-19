use bevy::prelude::*;

use crate::prelude::{Collider, RigidBody};
use crate::world::PhysicsWorld;
use crate::registry::PhysicsRegistry;

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

        // If the engine successfully built the body, write it in our Phonebook!
        if let Some(id) = body_id {
            registry.register(entity, id);
        } else {
            error!("Failed to spawn physics body for entity {:?}", entity);
        }
    }
}
