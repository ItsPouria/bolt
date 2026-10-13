//! The Bevy resource wrapper around `bolt-core`'s physics world.
//!
//! This type is the ECS host: it implements `Resource` so Bevy can store
//! the world, converts Bevy component data into `bolt-core`'s engine-agnostic
//! vocabulary at the boundary, and delegates all simulation work to the
//! wrapped `bolt_core::world::PhysicsWorld`.

use bevy::ecs::entity::Entity;
use bevy::ecs::resource::Resource;
use bevy::ecs::world::{FromWorld, World};
use bevy::log::error;
use bevy::math::{Quat, Vec3};
use bolt_core::world::{
    BodyHandle, BodySpawn, BodyType, PhysicsWorld as CorePhysicsWorld, WorldConfig,
};

use crate::components::{Collider, RigidBody};
use crate::config::PhysicsConfig;

/// The Bevy resource hosting the Jolt physics world.
///
/// See the [crate-level docs](crate) and [`PhysicsWorld`](bolt_core::world::PhysicsWorld)
/// for the underlying engine-agnostic implementation.
#[derive(Resource)]
pub struct PhysicsWorld(pub(crate) CorePhysicsWorld);

impl From<&PhysicsConfig> for WorldConfig {
    fn from(config: &PhysicsConfig) -> Self {
        Self {
            max_bodies: config.max_bodies,
            max_body_pairs: config.max_body_pairs,
            max_contact_constraints: config.max_contact_constraints,
            num_threads: config.num_threads,
            temp_allocator_size_mb: config.temp_allocator_size_mb,
        }
    }
}

impl PhysicsWorld {
    /// Creates a new physics world resource from the given configuration.
    pub fn new(config: PhysicsConfig) -> Self {
        Self(CorePhysicsWorld::new(WorldConfig::from(&config)))
    }

    /// Spawns a Jolt body for the given entity and returns its ID.
    ///
    /// Returns `None` (and logs) if the collider's dimensions are invalid
    /// or the world is at body capacity.
    // Intentionally preserves the original 9-parameter signature so
    // `systems.rs` stays untouched mid-refactor; the ergonomic parameter
    // object (`BodySpawn`) lives in bolt-core. Revisit in the Phase 4 API pass.
    #[allow(clippy::too_many_arguments)]
    pub fn spawn_body(
        &mut self,
        entity: Entity,
        collider: &Collider,
        scale: Vec3,
        transform: (Vec3, Quat),
        rigidbody: &RigidBody,
        linear_velocity: Vec3,
        angular_velocity: Vec3,
        use_ccd: bool,
    ) -> Option<BodyHandle> {
        let (position, rotation) = transform;

        let body_type = match rigidbody {
            RigidBody::Dynamic => BodyType::Dynamic,
            RigidBody::Static => BodyType::Static,
            RigidBody::Kinematic => BodyType::Kinematic,
        };

        let spawn = BodySpawn {
            shape: collider.to_spec(scale),
            body_type,
            position,
            rotation,
            linear_velocity,
            angular_velocity,
            user_data: entity.to_bits(),
            use_ccd,
        };

        let result = self.0.spawn_body(spawn);
        if result.is_none() {
            error!(
                "Failed to spawn physics body for entity {:?}: invalid collider dimensions or world at capacity",
                entity
            );
        }
        result
    }

    /// Advances the simulation; see [`CorePhysicsWorld::step`].
    pub fn step(&mut self, delta_time: f32, collision_steps: i32) {
        self.0.step(delta_time, collision_steps);
    }

    /// Sets global gravity; see [`CorePhysicsWorld::set_gravity`].
    pub fn set_gravity(&mut self, gravity: Vec3) {
        self.0.set_gravity(gravity);
    }

    /// Returns whether the body is awake; see [`CorePhysicsWorld::is_active`].
    pub fn is_active(&self, handle: BodyHandle) -> bool {
        self.0.is_active(handle)
    }

    /// Teleports the body; see [`CorePhysicsWorld::set_position_and_rotation`].
    pub fn set_position_and_rotation(
        &mut self,
        handle: BodyHandle,
        position: Vec3,
        rotation: Quat,
    ) {
        self.0.set_position_and_rotation(handle, position, rotation);
    }

    /// Reads the body's transform; see [`CorePhysicsWorld::get_transform`].
    pub fn get_transform(&self, handle: BodyHandle) -> Option<(Vec3, Quat)> {
        self.0.get_transform(handle)
    }

    /// Reads the body's linear velocity, in m/s; see
    /// [`CorePhysicsWorld::get_linear_velocity`].
    pub fn get_linear_velocity(&self, handle: BodyHandle) -> Option<Vec3> {
        self.0.get_linear_velocity(handle)
    }

    /// Reads the body's angular velocity, in rad/s; see
    /// [`CorePhysicsWorld::get_angular_velocity`].
    pub fn get_angular_velocity(&self, handle: BodyHandle) -> Option<Vec3> {
        self.0.get_angular_velocity(handle)
    }

    /// Writes the body's linear velocity, in m/s; see
    /// [`CorePhysicsWorld::set_linear_velocity`].
    pub fn set_linear_velocity(&mut self, handle: BodyHandle, linear_velocity: Vec3) {
        self.0.set_linear_velocity(handle, linear_velocity);
    }

    /// Writes the body's angular velocity, in rad/s; see
    /// [`CorePhysicsWorld::set_angular_velocity`].
    pub fn set_angular_velocity(&mut self, handle: BodyHandle, angular_velocity: Vec3) {
        self.0.set_angular_velocity(handle, angular_velocity);
    }

    /// Reads the body's per-body payload (`entity.to_bits()` for entities
    /// spawned through bolt-bevy); see [`CorePhysicsWorld::user_data`].
    pub fn user_data(&self, handle: BodyHandle) -> Option<u64> {
        self.0.user_data(handle)
    }

    /// Destroys the body; see [`CorePhysicsWorld::destroy_body`].
    pub fn destroy_body(&mut self, handle: BodyHandle) -> bool {
        self.0.destroy_body(handle)
    }
}

impl FromWorld for PhysicsWorld {
    fn from_world(world: &mut World) -> Self {
        let config = world
            .get_resource::<PhysicsConfig>()
            .cloned()
            .unwrap_or_default();
        Self::new(config)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_physics_config_converts_to_world_config() {
        let config = PhysicsConfig {
            max_bodies: 100,
            max_body_pairs: 200,
            max_contact_constraints: 300,
            num_threads: 4,
            temp_allocator_size_mb: 5,
            collision_steps: 2,
        };
        let world_config = WorldConfig::from(&config);
        assert_eq!(world_config.max_bodies, 100);
        assert_eq!(world_config.max_body_pairs, 200);
        assert_eq!(world_config.max_contact_constraints, 300);
        assert_eq!(world_config.num_threads, 4);
        assert_eq!(world_config.temp_allocator_size_mb, 5);
    }

    #[test]
    fn test_wrapper_spawns_and_queries_body() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let body_id = world
            .spawn_body(
                Entity::PLACEHOLDER,
                &Collider::Box {
                    half_extents: Vec3::splat(1.0),
                },
                Vec3::ONE,
                (Vec3::new(1.0, 2.0, 3.0), Quat::IDENTITY),
                &RigidBody::Dynamic,
                Vec3::ZERO,
                Vec3::ZERO,
                false,
            )
            .expect("spawn via wrapper must succeed");

        let (position, _) = world.get_transform(body_id).unwrap();
        assert!(position.distance(Vec3::new(1.0, 2.0, 3.0)) < 1e-4);
    }
}
