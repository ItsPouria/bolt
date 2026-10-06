//! Core, engine-agnostic abstractions and math types for the Bolt physics engine.
//!
//! This crate holds the Jolt physics runtime: world lifecycle, body management,
//! collision layers, and shape construction. It has zero Bevy dependencies and
//! can be used standalone with any ECS or renderer.

#![warn(missing_docs)]

pub mod layers;
