//! Core, engine-agnostic abstractions and math types for the Bolt physics engine.
//!
//! This crate holds the Jolt physics runtime: world lifecycle, body management,
//! collision layers, and shape construction. It has zero Bevy dependencies and
//! can be used standalone with any ECS or renderer.

#![warn(missing_docs)]

/// One-time global initialization of the Jolt C++ runtime.
pub mod init;

/// Collision layer definitions and broad-phase/object-layer pair filters.
pub mod layers;

/// Validating factory functions for Jolt collision shapes.
pub mod shapes;

/// The engine-agnostic Jolt physics world and its construction vocabulary.
pub mod world;
