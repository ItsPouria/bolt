use bevy::prelude::*;
use bolt_core::shapes::ShapeSpec;
use bolt_core::world::BodyHandle;

/// Defines the motion type and physical behavior of a body.
#[non_exhaustive]
#[derive(Component, Clone, Debug, Eq, PartialEq, Reflect)]
#[reflect(Component)]
#[require(
    Transform,
    LinearVelocity,
    AngularVelocity,
    LinearDamping,
    AngularDamping
)]
pub enum RigidBody {
    /// A dynamic body affected by forces, gravity, and impulses (e.g. crates, debris).
    Dynamic,
    /// A static body that does not move and has infinite mass (e.g. terrain, floors, walls).
    Static,
    /// A kinematic body that is not affected by gravity or forces, but can push dynamic bodies.
    /// It is moved manually by setting its velocity or position.
    Kinematic,
}

/// The geometric collision shape attached to a rigid body.
///
/// All dimensions are in local space (meters) before the entity's `Transform::scale` is applied.
/// When spawned, Bolt will automatically scale these primitives based on the entity's global scale.
#[non_exhaustive]
#[derive(Component, Clone, Debug, PartialEq, Reflect)]
#[reflect(Component)]
pub enum Collider {
    /// A 3D box defined by its half-extents from the center.
    ///
    /// The total width, height, and depth of the box are exactly twice the half-extents.
    Box {
        /// The distance from the center of the box to its faces along the local X, Y, and Z axes (in meters).
        half_extents: Vec3,
    },
    /// A perfectly spherical collision volume.
    ///
    /// **Note on scaling:** True ellipsoids are not supported by the underlying physics solver.
    /// If a non-uniform scale is applied to the entity, the sphere will be scaled uniformly
    /// by the largest component of the scale vector (`scale.max_element()`).
    Sphere {
        /// The distance from the center to the surface of the sphere (in meters).
        radius: f32,
    },
    /// A 3D capsule (a cylinder capped with two hemispheres).
    ///
    /// The capsule is aligned perfectly along the local Y axis. The total height of the
    /// capsule is `(half_height * 2.0) + (radius * 2.0)`.
    ///
    /// **Note on scaling:** Non-uniform scaling on the X and Z axes will use the larger
    /// of the two values to scale the radius uniformly. The Y axis scales the height independently.
    Capsule {
        /// Half the height of the inner cylindrical portion of the capsule (in meters).
        /// Does not include the hemispherical end caps.
        half_height: f32,
        /// The radius of the central cylinder and the hemispherical end caps (in meters).
        radius: f32,
    },
    /// A 3D cylinder aligned along the local Y axis with flat circular end caps.
    ///
    /// **Note on scaling:** Non-uniform scaling on the X and Z axes will use the larger
    /// of the two values to scale the radius uniformly. The Y axis scales the height independently.
    Cylinder {
        /// The distance from the center of the cylinder to the flat top and bottom caps (in meters).
        half_height: f32,
        /// The radius of the circular cross-section (in meters).
        radius: f32,
    },
}

impl Collider {
    /// Converts this collider into an engine-agnostic [`ShapeSpec`], applying
    /// the entity's world scale to bake final shape dimensions.
    ///
    /// Scaling policy (documented on each variant):
    /// - `Box` scales each axis independently.
    /// - `Sphere` scales uniformly by the largest scale component (true
    ///   ellipsoids are not supported by the solver).
    /// - `Capsule`/`Cylinder` scale height by `scale.y` and radius by the
    ///   larger of `scale.x`/`scale.z`.
    pub(crate) fn to_spec(&self, scale: Vec3) -> ShapeSpec {
        let scale = scale.abs();
        match self {
            Collider::Box { half_extents } => ShapeSpec::Box {
                half_extents: *half_extents * scale,
            },
            Collider::Sphere { radius } => ShapeSpec::Sphere {
                radius: radius * scale.max_element(),
            },
            Collider::Capsule {
                half_height,
                radius,
            } => ShapeSpec::Capsule {
                half_height: half_height * scale.y,
                radius: radius * scale.x.max(scale.z),
            },
            Collider::Cylinder {
                half_height,
                radius,
            } => ShapeSpec::Cylinder {
                half_height: half_height * scale.y,
                radius: radius * scale.x.max(scale.z),
            },
        }
    }
}

/// A component attached to Bevy entities that have a live Jolt Physics body.
#[derive(Component, Debug)]
#[component(immutable)]
pub struct JoltBody(pub(crate) BodyHandle);

impl JoltBody {
    /// Returns the [`BodyHandle`] linking this entity to its physics body.
    ///
    /// Handles are opaque and cannot be forged; the physics world rejects
    /// stale ones after their body is destroyed.
    pub fn handle(&self) -> BodyHandle {
        self.0
    }
}

/// The linear velocity of a rigid body in meters per second.
///
/// Modifying this component will update the body's velocity in the physics engine.
#[derive(Component, Debug, Clone, Copy, PartialEq, Default, Deref, DerefMut, Reflect)]
#[reflect(Component)]
pub struct LinearVelocity(pub Vec3);

/// The angular velocity of a rigid body in radians per second around the local X, Y, and Z axes.
///
/// Modifying this component will update the body's angular velocity in the physics engine.
#[derive(Component, Debug, Clone, Copy, PartialEq, Default, Deref, DerefMut, Reflect)]
#[reflect(Component)]
pub struct AngularVelocity(pub Vec3);

/// The linear (translational) damping of a rigid body, in inverse seconds.
///
/// Velocity decays exponentially as `v(t) = v₀·e^(−d·t)`: the default of
/// `0.05` (Jolt's own) retains ~97.5% of a body's speed after half a second,
/// while `0.0` leaves analytic gameplay code (projectile arcs, aiming,
/// replays) exact. Only meaningful for dynamic bodies.
///
/// Read once at spawn — unlike the velocity components, changing this
/// component after the body exists has no effect until the body is respawned
/// (runtime damping control is pending an upstream JoltC API). Values must
/// be finite and non-negative; the world rejects invalid ones at spawn.
#[derive(Component, Debug, Clone, Copy, PartialEq, Deref, DerefMut, Reflect)]
#[reflect(Component)]
pub struct LinearDamping(pub f32);

impl Default for LinearDamping {
    /// Jolt's own default, so bodies spawned without an explicit damping
    /// value keep the behavior they previously inherited silently.
    fn default() -> Self {
        Self(0.05)
    }
}

/// The angular (rotational) damping of a rigid body, in inverse seconds.
///
/// Governs spin decay with the same exponential law and rules as
/// [`LinearDamping`]: `ω(t) = ω₀·e^(−d·t)`, meaningful only for dynamic
/// bodies, read once at spawn, and finite and non-negative.
#[derive(Component, Debug, Clone, Copy, PartialEq, Deref, DerefMut, Reflect)]
#[reflect(Component)]
pub struct AngularDamping(pub f32);

impl Default for AngularDamping {
    /// Jolt's own default; see [`LinearDamping`]'s rationale.
    fn default() -> Self {
        Self(0.05)
    }
}

/// A marker component used to efficiently filter out static bodies from transform synchronization.
#[derive(Component, Debug, Clone, Copy, PartialEq, Default, Reflect)]
#[reflect(Component)]
pub struct StaticMarker;

/// Enables Continuous Collision Detection (CCD) for a dynamic rigid body.
///
/// Use this on fast-moving objects to prevent them from tunneling through walls
/// or visually penetrating the floor.
#[derive(Component, Debug, Clone, Copy, PartialEq, Default, Reflect)]
#[reflect(Component)]
pub struct ContinuousCollision;

/// Marks a kinematic body as driven by user `Transform` edits.
///
/// While marked, the entity's local `Transform` is **user-authoritative**:
/// `sync_transforms` deliberately does not mirror the physics state into
/// it (that mirror-write is what laundered the parent's motion out of the
/// drive target for parented bodies — the drive would chase its own tail
/// and preserve the status-quo world pose instead of tracking the parent
/// frame). The per-tick kinematic drive issues `move_kinematic` toward
/// parent frame × local, so propagation renders the truth regardless, and
/// each component keeps a single writer: the user writes the local of a
/// marked body, sync writes everything else.
///
/// Removed when the user writes a velocity component (the velocity path
/// takes over) or the body is respawned. Managed by Bolt. Manual
/// insertion: on a non-kinematic it is inert (the drive skips it and sync
/// keeps mirroring); on a kinematic it claims the drive immediately,
/// halting velocity-path motion at the body's current pose.
#[derive(Component, Debug, Clone, Copy, Default, Reflect)]
#[reflect(Component)]
pub struct TransformDriven;
