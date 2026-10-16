//! Core, engine-agnostic abstractions and math types for the Bolt physics engine.
//!
//! This crate holds the Jolt physics runtime: world lifecycle, body management,
//! collision layers, and shape construction. It has zero Bevy dependencies and
//! can be used standalone with any ECS or renderer.

#![deny(unsafe_op_in_unsafe_fn)]
#![warn(missing_docs)]
#![warn(clippy::undocumented_unsafe_blocks)]

mod init;

/// Collision layer definitions and broad-phase/object-layer pair filters.
pub mod layers;

/// Validating factory functions for Jolt collision shapes.
pub mod shapes;

/// The engine-agnostic Jolt physics world and its construction vocabulary.
pub mod world;
