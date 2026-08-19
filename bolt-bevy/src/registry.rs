use std::collections::HashMap;

use bevy::prelude::*;
use rolt::BodyId;

#[derive(Resource, Default)]
pub struct PhysicsRegistry {
    entity_to_body: HashMap<Entity, BodyId>,
    body_to_entity: HashMap<BodyId, Entity>,
}

impl PhysicsRegistry {
    pub fn register(&mut self, entity: Entity, body_id: BodyId) {
        self.entity_to_body.insert(entity, body_id);
        self.body_to_entity.insert(body_id, entity);
    }
}
