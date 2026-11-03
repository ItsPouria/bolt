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
use bolt_core::world::{BodySpawn, BodyType, PhysicsWorld as CorePhysicsWorld, WorldConfig};

pub use bolt_core::world::{BodyHandle, RayHit};

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

/// The Bevy-side parameter object for [`PhysicsWorld::spawn_body`].
///
/// Carries everything the wrapper needs to convert one entity's components
/// into a [`BodySpawn`] for the engine-agnostic core: identity, geometry
/// inputs (collider in local space plus the world scale that bakes it),
/// world-space pose, initial motion, and per-body options.
#[derive(Debug, Clone, Copy)]
pub struct BodySpawnRequest<'a> {
    /// The entity the body is spawned for; stored as `entity.to_bits()` in
    /// the body's `user_data` so query hits map back to entities.
    pub entity: Entity,
    /// The collider to bake, in local space; final dimensions apply `scale`.
    pub collider: &'a Collider,
    /// The entity's world scale, baked into the collider's dimensions.
    pub scale: Vec3,
    /// The world-space position of the body's origin, in meters.
    pub position: Vec3,
    /// The world-space rotation of the body.
    pub rotation: Quat,
    /// The motion type the body is spawned with.
    pub rigidbody: &'a RigidBody,
    /// The initial linear velocity, in m/s.
    pub linear_velocity: Vec3,
    /// The initial angular velocity, in rad/s.
    pub angular_velocity: Vec3,
    /// The linear damping coefficient, in 1/s; see
    /// [`crate::components::LinearDamping`].
    pub linear_damping: f32,
    /// The angular damping coefficient, in 1/s; see
    /// [`crate::components::AngularDamping`].
    pub angular_damping: f32,
    /// Whether to enable continuous collision detection.
    pub use_ccd: bool,
}

impl PhysicsWorld {
    /// Creates a new physics world resource from the given configuration.
    pub fn new(config: PhysicsConfig) -> Self {
        Self(CorePhysicsWorld::new(WorldConfig::from(&config)))
    }

    /// Spawns a Jolt body for the given entity and returns its handle.
    ///
    /// Returns `None` (and logs) if the collider's dimensions are invalid,
    /// a damping value is non-finite or negative, the pose or velocity is
    /// invalid (non-finite or unnormalized rotation), or the world is at
    /// body capacity.
    pub fn spawn_body(&mut self, request: BodySpawnRequest<'_>) -> Option<BodyHandle> {
        let BodySpawnRequest {
            entity,
            collider,
            scale,
            position,
            rotation,
            rigidbody,
            linear_velocity,
            angular_velocity,
            linear_damping,
            angular_damping,
            use_ccd,
        } = request;

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
            linear_damping,
            angular_damping,
            user_data: entity.to_bits(),
            use_ccd,
        };

        let result = self.0.spawn_body(spawn);
        if result.is_none() {
            error!(
                "Failed to spawn physics body for entity {:?}: invalid parameters (geometry, pose, velocity, or damping) or world at capacity",
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
    #[must_use]
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

    /// Moves a kinematic body toward a target pose over `delta_time` by
    /// setting its velocity internally (delegates to bolt-core's
    /// [`PhysicsWorld::move_kinematic`](bolt_core::world::PhysicsWorld::move_kinematic)).
    ///
    /// The velocity persists until re-issued, so the per-tick
    /// [`move_transform_driven_kinematics`](crate::systems::move_transform_driven_kinematics)
    /// system calls this every fixed tick: once the user stops editing the
    /// `Transform`, target ≡ current and the velocity decays to zero.
    /// Ignored for stale handles, non-finite/unnormalized targets,
    /// non-positive `delta_time`, and non-kinematic bodies.
    pub fn move_kinematic(
        &mut self,
        handle: BodyHandle,
        target_position: Vec3,
        target_rotation: Quat,
        delta_time: f32,
    ) {
        self.0
            .move_kinematic(handle, target_position, target_rotation, delta_time);
    }

    /// Reads the body's transform; see [`CorePhysicsWorld::get_transform`].
    pub fn get_transform(&self, handle: BodyHandle) -> Option<(Vec3, Quat)> {
        self.0.get_transform(handle)
    }

    /// Reads the body's linear and angular velocities in a single lock acquisition;
    /// see [`CorePhysicsWorld::get_linear_and_angular_velocity`].
    pub fn get_linear_and_angular_velocity(&self, handle: BodyHandle) -> Option<(Vec3, Vec3)> {
        self.0.get_linear_and_angular_velocity(handle)
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

    /// Applies a one-shot impulse, in newton-seconds, to a dynamic body's
    /// center of mass — velocity changes immediately (`Δv = impulse / mass`),
    /// with nothing to re-issue; see [`CorePhysicsWorld::add_impulse`] for
    /// the full contract.
    pub fn add_impulse(&mut self, handle: BodyHandle, impulse: Vec3) {
        self.0.add_impulse(handle, impulse);
    }

    /// Adds a continuous linear force, in newtons, to a dynamic body's
    /// accumulator — consumed and cleared by the next [`step`](Self::step);
    /// see [`CorePhysicsWorld::add_force`] for the full contract.
    pub fn add_force(&mut self, handle: BodyHandle, force: Vec3) {
        self.0.add_force(handle, force);
    }

    /// Adds a continuous torque, in newton-meters, to a dynamic body's
    /// accumulator — same per-step contract as [`add_force`](Self::add_force);
    /// the world-space vector's direction is the spin axis. See
    /// [`CorePhysicsWorld::add_torque`] for the full contract.
    pub fn add_torque(&mut self, handle: BodyHandle, torque: Vec3) {
        self.0.add_torque(handle, torque);
    }

    /// Reads the body's per-body payload (`entity.to_bits()` for entities
    /// spawned through bolt-bevy); see [`CorePhysicsWorld::user_data`].
    pub fn user_data(&self, handle: BodyHandle) -> Option<u64> {
        self.0.user_data(handle)
    }

    /// Casts a ray into the simulation and returns the closest hit, if any;
    /// see [`CorePhysicsWorld::ray_cast`] for the full contract (notably:
    /// **the direction's magnitude is the ray's maximum distance**).
    ///
    /// Hits translate back to entities through the payload: entities spawned
    /// by bolt-bevy store `entity.to_bits()` in `user_data`, so
    /// `Entity::from_bits(hit.user_data)` recovers the hit entity.
    pub fn ray_cast(&self, origin: Vec3, direction: Vec3) -> Option<RayHit> {
        self.0.ray_cast(origin, direction)
    }

    /// Destroys the body; see [`CorePhysicsWorld::destroy_body`].
    pub fn destroy_body(&mut self, handle: BodyHandle) -> bool {
        self.0.destroy_body(handle)
    }
}

/// Picks up a [`Gravity`](crate::gravity::Gravity) resource if one exists at
/// construction time — including one the user inserted before adding the
/// plugin (the plugin initializes `Gravity` before `PhysicsWorld` for exactly
/// this reason). Gravity changes made afterwards are picked up by
/// [`apply_gravity`](crate::gravity::apply_gravity) every fixed tick instead.
impl FromWorld for PhysicsWorld {
    fn from_world(world: &mut World) -> Self {
        let config = world
            .get_resource::<PhysicsConfig>()
            .cloned()
            .unwrap_or_default();
        let mut physics_world = Self::new(config);
        if let Some(gravity) = world.get_resource::<crate::gravity::Gravity>() {
            physics_world.set_gravity(gravity.0);
        }
        physics_world
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
            .spawn_body(BodySpawnRequest {
                entity: Entity::PLACEHOLDER,
                collider: &Collider::Box {
                    half_extents: Vec3::splat(1.0),
                },
                scale: Vec3::ONE,
                position: Vec3::new(1.0, 2.0, 3.0),
                rotation: Quat::IDENTITY,
                rigidbody: &RigidBody::Dynamic,
                linear_velocity: Vec3::ZERO,
                angular_velocity: Vec3::ZERO,
                linear_damping: 0.05,
                angular_damping: 0.05,
                use_ccd: false,
            })
            .expect("spawn via wrapper must succeed");

        let (position, _) = world.get_transform(body_id).unwrap();
        assert!(position.distance(Vec3::new(1.0, 2.0, 3.0)) < 1e-4);
    }

    /// Spawns the wrapper twin of core's `box_spawn` mule: a dynamic 1 m
    /// half-extent box — 8 m³ at Jolt's default 1000 kg/m³ density means
    /// **8000 kg** and **16000/3 kg·m²** of inertia per axis, the numbers
    /// the analytic assertions below are built on (full narration in
    /// bolt-core's `world.rs` tests).
    fn spawn_dynamic_box(
        world: &mut PhysicsWorld,
        position: Vec3,
        linear_damping: f32,
        angular_damping: f32,
    ) -> BodyHandle {
        world
            .spawn_body(BodySpawnRequest {
                entity: Entity::PLACEHOLDER,
                collider: &Collider::Box {
                    half_extents: Vec3::splat(1.0),
                },
                scale: Vec3::ONE,
                position,
                rotation: Quat::IDENTITY,
                rigidbody: &RigidBody::Dynamic,
                linear_velocity: Vec3::ZERO,
                angular_velocity: Vec3::ZERO,
                linear_damping,
                angular_damping,
                use_ccd: false,
            })
            .expect("wrapper spawn must succeed")
    }

    #[test]
    fn test_wrapper_add_impulse_changes_velocity_immediately() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        world.set_gravity(Vec3::ZERO);
        let body = spawn_dynamic_box(&mut world, Vec3::ZERO, 0.05, 0.05);

        // 8000 N·s on the 8000 kg mule = Δv 1 m/s, readable before any
        // step: the wrapper must forward to core's impulse, not to a
        // step-cleared accumulator.
        world.add_impulse(body, Vec3::new(0.0, 8000.0, 0.0));
        let velocity = world.get_linear_velocity(body).unwrap();
        assert!(
            velocity.distance(Vec3::new(0.0, 1.0, 0.0)) < 1e-6,
            "wrapper impulse must land immediately, got {velocity:?}"
        );
    }

    #[test]
    fn test_wrapper_add_force_waits_for_step() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        world.set_gravity(Vec3::ZERO);
        // Zero damping keeps the analytic value exact.
        let body = spawn_dynamic_box(&mut world, Vec3::ZERO, 0.0, 0.05);

        world.add_force(body, Vec3::new(0.0, 8000.0, 0.0));
        let before = world.get_linear_velocity(body).unwrap();
        assert_eq!(
            before,
            Vec3::ZERO,
            "force must sit in the accumulator until a step consumes it"
        );

        // ΣF/m·dt = 1 m/s² × 1/60 s.
        world.step(1.0 / 60.0, 1);
        let after = world.get_linear_velocity(body).unwrap();
        assert!(
            (after.y - 1.0 / 60.0).abs() < 1e-5,
            "wrapper force must integrate to F/m·dt = {}, got {}",
            1.0 / 60.0,
            after.y
        );
    }

    #[test]
    fn test_wrapper_add_torque_spins_without_translating() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        world.set_gravity(Vec3::ZERO);
        let body = spawn_dynamic_box(&mut world, Vec3::ZERO, 0.05, 0.0);

        // 16000 N·m on 16000/3 kg·m² = 3 rad/s²; one 1/60 s step → 0.05.
        world.add_torque(body, Vec3::new(0.0, 16000.0, 0.0));
        world.step(1.0 / 60.0, 1);
        let angular = world.get_angular_velocity(body).unwrap();
        assert!(
            (angular.y - 0.05).abs() < 1e-4,
            "wrapper torque must spin at τ/I·dt = 0.05 rad/s, got {angular:?}"
        );

        // A pure torque rotates in place: the wrapper must not have
        // forwarded the vector to a linear entry point.
        let linear = world.get_linear_velocity(body).unwrap();
        assert!(
            linear.length() < 1e-6,
            "wrapper torque leaked into linear velocity: {linear:?}"
        );
    }
}
