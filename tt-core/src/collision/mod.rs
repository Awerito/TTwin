//! Collision detection and resolution for table tennis physics.
//!
//! This module handles:
//! - **Detection**: Finding when and where collisions occur (swept sphere)
//! - **Resolution**: Computing post-collision velocities and spin (next module)
//!
//! ## Swept Sphere Algorithm
//!
//! Instead of checking if the ball overlaps a surface (which fails at high speeds),
//! we check if the ball's trajectory intersects the surface during a timestep.
//!
//! ```text
//! Time t=0         Time t=dt
//!    ●────────────────●
//!    Ball            Ball
//!    start           end
//!         \    ↓
//!          \   Surface
//! ══════════╳═══════════
//!           └─ Collision point
//! ```
//!
//! This detects collisions regardless of ball speed or timestep size.

pub mod detection;
pub mod resolution;

pub use detection::*;
pub use resolution::*;
