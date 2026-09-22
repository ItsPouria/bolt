use bevy::prelude::*;

use crate::components::JoltBody;
use crate::prelude::{Collider, RigidBody};
use crate::world::PhysicsWorld;

pub fn spawn_physics_bodies(
    mut commands: Commands,
    query: Query<(Entity, &Transform, &RigidBody, &Collider), Added<RigidBody>>,
    mut physics_world: ResMut<PhysicsWorld>,
) {
    for (entity, transform, rigidbody, collider) in query.iter() {
        let body_id = match collider {
            Collider::Box { half_extents } => {
                physics_world.spawn_box(entity, *half_extents, transform, rigidbody)
            }
        };

        if let Some(id) = body_id {
            commands.entity(entity).insert(JoltBody(id));
        } else {
            error!("Failed to spawn physics body for entity {:?}", entity);
        }
    }
}

pub fn sync_transforms(
    mut query: Query<(&mut Transform, &JoltBody), With<RigidBody>>,
    physics_world: Res<PhysicsWorld>,
) {
    for (mut transform, body) in query.iter_mut() {
        if let Some((new_pos, new_rot)) = physics_world.get_transform(body.0) {
            transform.translation = new_pos;
            transform.rotation = new_rot;
        }
    }
}

pub fn cleanup_despawned_physics_bodies(
    event: On<Remove, JoltBody>,
    query: Query<&JoltBody>,
    mut physics_world: ResMut<PhysicsWorld>,
) {
    if let Ok(body) = query.get(event.entity) {
        physics_world.destroy_body(body.0);
        info!(
            "Successfully cleaned up Jolt body for despawned entity {:?}",
            event.entity
        );
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

        assert!(app.world().get::<JoltBody>(broken_entity).is_none());
    }
}
