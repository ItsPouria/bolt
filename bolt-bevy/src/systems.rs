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

pub fn cleanup_despawned_physics_bodies(
    mut removed: RemovedComponents<RigidBody>,
    mut physics_world: ResMut<PhysicsWorld>,
    mut registry: ResMut<PhysicsRegistry>,
) {
    for entity in removed.read() {
        if let Some(body_id) = registry.remove_body(entity) {
            physics_world.destroy_body(body_id);
            info!(
                "Successfully cleaned up Jolt body for despawned entity {:?}",
                entity
            )
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plugin::BoltPlugin;

    #[test]
    fn test_spawn_physics_bodies_failure() {
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(BoltPlugin::default());

        let broken_entity = app
            .world_mut()
            .spawn((
                Transform::from_xyz(0.0, 10.0, 0.0),
                RigidBody::Dynamic,
                Collider::Box {
                    half_extents: Vec3::splat(-1.0),
                },
            ))
            .id();

        app.update();

        let registry = app.world().resource::<PhysicsRegistry>();
        assert!(registry.get_body(broken_entity).is_none());
    }
}
