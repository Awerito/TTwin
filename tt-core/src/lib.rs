//! # TT Core
//!
//! A physics engine for realistic table tennis simulation.
//!
//! ## Architecture
//!
//! - `types`: Core data structures (Vec3, states, material properties)
//! - `integrator`: Numerical integration (Velocity Verlet)
//! - `forces`: Physical forces (gravity, drag, Magnus effect)
//! - `collision`: Detection and resolution with Hertzian contact model
//! - `materials`: YAML-based material configuration loader
//! - `simulation`: Main orchestrator

pub mod collision;
pub mod forces;
pub mod integrator;
pub mod materials;
pub mod types;

// These modules will be implemented incrementally
// pub mod simulation;
