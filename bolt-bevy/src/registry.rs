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

    pub fn get_body(&self, entity: Entity) -> Option<BodyId> {
        self.entity_to_body.get(&entity).copied()
    }

    pub fn remove_body(&mut self, entity: Entity) -> Option<BodyId> {
        if let Some(body_id) = self.entity_to_body.remove(&entity) {
            self.body_to_entity.remove(&body_id);
            Some(body_id)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::prelude::Entity;

    #[test]
    fn test_registry_insert_and_get() {
        let mut registry = PhysicsRegistry::default();

        // 1. Create a fake Bevy Entity (Entity::from_raw(1))
        let bevy_entity = Entity::from_raw_u32(1).unwrap();
        // 2. Create a fake Jolt BodyId (BodyId::new(42))
        let jolt_body_id = BodyId::new(42);
        // 3. Register them!
        registry.register(bevy_entity, jolt_body_id);
        // 4. Assert that get_body returns Some(...)
        assert_eq!(registry.get_body(bevy_entity), Some(jolt_body_id));
        // 5. Assert that getting a fake Entity returns None
        let fake_entity = Entity::from_raw_u32(99).unwrap();
        assert_eq!(registry.get_body(fake_entity), None);
    }
}
