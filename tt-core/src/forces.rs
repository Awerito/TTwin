//! Physical forces acting on the ball.
//!
//! This module implements the `ForceModel` trait for realistic table tennis physics:
//!
//! - **Gravity**: Constant downward acceleration
//! - **Drag**: Air resistance opposing motion, proportional to v²
//! - **Magnus**: Lift force from spin, causes curved trajectories
//!
//! ## The Magnus Effect
//!
//! When a spinning ball moves through air, it creates pressure differences:
//! - The side spinning with the airflow has lower pressure
//! - The side spinning against has higher pressure
//! - This creates a force perpendicular to both velocity and spin axis
//!
//! ```text
//! Topspin (spin.x > 0):
//!     ↓ Magnus force pushes ball DOWN
//!     Ball dives faster than gravity alone
//!
//! Backspin (spin.x < 0):
//!     ↑ Magnus force pushes ball UP
//!     Ball "floats" and travels further
//! ```

use crate::integrator::ForceModel;
use crate::types::{constants, BallProperties, BallState, Vec3};

/// Complete force model for table tennis physics.
///
/// Combines gravity, aerodynamic drag, and Magnus effect.
pub struct TableTennisForces {
    /// Air density in kg/m³ (default: sea level at 20°C)
    pub air_density: f64,

    /// Gravity vector (default: -9.81 in Y)
    pub gravity: Vec3,

    /// Enable/disable individual forces (useful for testing)
    pub enable_gravity: bool,
    pub enable_drag: bool,
    pub enable_magnus: bool,
}

impl Default for TableTennisForces {
    fn default() -> Self {
        Self {
            air_density: constants::AIR_DENSITY,
            gravity: Vec3::new(0.0, -constants::GRAVITY, 0.0),
            enable_gravity: true,
            enable_drag: true,
            enable_magnus: true,
        }
    }
}

impl TableTennisForces {
    /// Create a new force model with default parameters.
    pub fn new() -> Self {
        Self::default()
    }

    /// Create a force model with only gravity (for testing).
    pub fn gravity_only() -> Self {
        Self {
            enable_drag: false,
            enable_magnus: false,
            ..Self::default()
        }
    }

    /// Calculate drag force on the ball.
    ///
    /// Drag equation: F_drag = -0.5 * ρ * Cd * A * |v|² * v̂
    ///
    /// Where:
    /// - ρ = air density
    /// - Cd = drag coefficient
    /// - A = cross-sectional area (πr²)
    /// - v = velocity vector
    /// - v̂ = velocity unit vector (direction)
    fn drag_acceleration(&self, state: &BallState, props: &BallProperties) -> Vec3 {
        let speed_sq = state.vel.magnitude_squared();
        if speed_sq < constants::EPSILON {
            return Vec3::ZERO;
        }

        let area = std::f64::consts::PI * props.radius * props.radius;

        // Drag force magnitude: 0.5 * ρ * Cd * A * v²
        let drag_force = 0.5 * self.air_density * props.drag_coefficient * area * speed_sq;

        // Acceleration = Force / mass, opposite to velocity direction
        let drag_acc = drag_force / props.mass;

        // Return acceleration opposing velocity
        state.vel.normalized() * (-drag_acc)
    }

    /// Calculate Magnus force on the ball.
    ///
    /// Magnus equation: F_magnus = 0.5 * ρ * CL * A * |v|² * (ω̂ × v̂)
    ///
    /// The force is perpendicular to both spin axis and velocity.
    /// Direction follows right-hand rule: spin × velocity.
    ///
    /// For table tennis:
    /// - Topspin (ω around +X axis) + forward velocity (+X) → downward force
    /// - Backspin (ω around -X axis) + forward velocity (+X) → upward force
    fn magnus_acceleration(&self, state: &BallState, props: &BallProperties) -> Vec3 {
        let speed_sq = state.vel.magnitude_squared();
        let spin_mag = state.spin.magnitude();

        if speed_sq < constants::EPSILON || spin_mag < constants::EPSILON {
            return Vec3::ZERO;
        }

        let speed = speed_sq.sqrt();
        let area = std::f64::consts::PI * props.radius * props.radius;

        // Magnus force direction: velocity × spin (right-hand rule)
        // For topspin (+Z spin) moving forward (+X vel): X × Z = -Y (downward) ✓
        let magnus_direction = state.vel.cross(&state.spin).normalized();

        // Magnus force magnitude
        // The lift coefficient scales with spin rate relative to ball speed
        // Using a simplified model: CL depends on spin parameter S = (ω * r) / v
        let spin_parameter = (spin_mag * props.radius) / speed;

        // Effective lift coefficient (empirical approximation)
        // At high spin parameters, lift saturates around 0.4-0.5
        let effective_cl = props.lift_coefficient * spin_parameter.min(1.0);

        let magnus_force = 0.5 * self.air_density * effective_cl * area * speed_sq;
        let magnus_acc = magnus_force / props.mass;

        magnus_direction * magnus_acc
    }

    /// Calculate spin decay due to air resistance.
    ///
    /// Spin decays as the ball interacts with air. This is a simplified model
    /// where decay rate is proportional to spin magnitude.
    fn spin_decay(&self, state: &BallState, props: &BallProperties) -> Vec3 {
        // More sophisticated model: decay depends on air viscosity and ball surface
        // For now, use empirical decay constant
        // Real balls lose about 20-30% spin per second at high speeds

        let speed = state.vel.magnitude();

        // Decay is faster at higher speeds (more air interaction)
        let base_decay_rate = 0.05; // 1/s at rest
        let speed_factor = 1.0 + speed * 0.02; // Increases with speed

        // Also proportional to ball surface area / mass ratio
        let surface_factor =
            4.0 * std::f64::consts::PI * props.radius * props.radius / props.mass;

        let decay_rate = base_decay_rate * speed_factor * surface_factor * 0.001;

        state.spin * (-decay_rate)
    }
}

impl ForceModel for TableTennisForces {
    fn linear_acceleration(&self, state: &BallState, props: &BallProperties) -> Vec3 {
        let mut acc = Vec3::ZERO;

        if self.enable_gravity {
            acc += self.gravity;
        }

        if self.enable_drag {
            acc += self.drag_acceleration(state, props);
        }

        if self.enable_magnus {
            acc += self.magnus_acceleration(state, props);
        }

        acc
    }

    fn angular_acceleration(&self, state: &BallState, props: &BallProperties) -> Vec3 {
        self.spin_decay(state, props)
    }
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::integrator::VelocityVerlet;

    #[test]
    fn test_gravity_only() {
        let forces = TableTennisForces::gravity_only();
        let props = BallProperties::default();
        let state = BallState::at_rest(Vec3::ZERO);

        let acc = forces.linear_acceleration(&state, &props);

        assert!((acc.x).abs() < constants::EPSILON);
        assert!((acc.y + constants::GRAVITY).abs() < constants::EPSILON);
        assert!((acc.z).abs() < constants::EPSILON);
    }

    #[test]
    fn test_drag_opposes_motion() {
        let forces = TableTennisForces {
            enable_gravity: false,
            enable_magnus: false,
            ..TableTennisForces::default()
        };
        let props = BallProperties::default();

        // Ball moving in +X direction
        let state = BallState::new(Vec3::ZERO, Vec3::new(10.0, 0.0, 0.0), Vec3::ZERO);

        let acc = forces.linear_acceleration(&state, &props);

        // Drag should be in -X direction
        assert!(acc.x < 0.0, "Drag should oppose motion, got ax={}", acc.x);
        assert!(acc.y.abs() < constants::EPSILON);
        assert!(acc.z.abs() < constants::EPSILON);
    }

    #[test]
    fn test_drag_increases_with_speed() {
        let forces = TableTennisForces {
            enable_gravity: false,
            enable_magnus: false,
            ..TableTennisForces::default()
        };
        let props = BallProperties::default();

        let slow = BallState::new(Vec3::ZERO, Vec3::new(5.0, 0.0, 0.0), Vec3::ZERO);
        let fast = BallState::new(Vec3::ZERO, Vec3::new(20.0, 0.0, 0.0), Vec3::ZERO);

        let acc_slow = forces.linear_acceleration(&slow, &props);
        let acc_fast = forces.linear_acceleration(&fast, &props);

        // Drag ∝ v², so 4x speed should give ~16x drag
        assert!(
            acc_fast.x.abs() > acc_slow.x.abs() * 10.0,
            "Fast drag should be >> slow drag"
        );
    }

    #[test]
    fn test_topspin_curves_down() {
        let forces = TableTennisForces {
            enable_gravity: false,
            enable_drag: false,
            ..TableTennisForces::default()
        };
        let props = BallProperties::default();

        // Ball moving forward (+X) with topspin (rotation around +Z axis)
        // Right-hand rule: +Z × +X = -Y (downward)
        let state = BallState::new(
            Vec3::ZERO,
            Vec3::new(10.0, 0.0, 0.0),  // Forward velocity
            Vec3::new(0.0, 0.0, 100.0), // Topspin (around Z axis)
        );

        let acc = forces.linear_acceleration(&state, &props);

        // Magnus should push ball down
        assert!(
            acc.y < 0.0,
            "Topspin should curve down, got ay={}",
            acc.y
        );
    }

    #[test]
    fn test_backspin_curves_up() {
        let forces = TableTennisForces {
            enable_gravity: false,
            enable_drag: false,
            ..TableTennisForces::default()
        };
        let props = BallProperties::default();

        // Ball moving forward (+X) with backspin (rotation around -Z axis)
        // Right-hand rule: -Z × +X = +Y (upward)
        let state = BallState::new(
            Vec3::ZERO,
            Vec3::new(10.0, 0.0, 0.0),   // Forward velocity
            Vec3::new(0.0, 0.0, -100.0), // Backspin (around -Z axis)
        );

        let acc = forces.linear_acceleration(&state, &props);

        // Magnus should push ball up
        assert!(acc.y > 0.0, "Backspin should curve up, got ay={}", acc.y);
    }

    #[test]
    fn test_sidespin_curves_sideways() {
        let forces = TableTennisForces {
            enable_gravity: false,
            enable_drag: false,
            ..TableTennisForces::default()
        };
        let props = BallProperties::default();

        // Ball moving forward (+X) with sidespin (rotation around Y axis)
        // Right-hand rule: +Y × +X = -Z (curves right from ball's perspective)
        let state = BallState::new(
            Vec3::ZERO,
            Vec3::new(10.0, 0.0, 0.0),  // Forward velocity
            Vec3::new(0.0, 100.0, 0.0), // Sidespin (around Y axis)
        );

        let acc = forces.linear_acceleration(&state, &props);

        // Magnus should push ball sideways
        assert!(
            acc.z.abs() > constants::EPSILON,
            "Sidespin should curve sideways"
        );
    }

    #[test]
    fn test_trajectory_with_topspin() {
        // Simulate a ball with topspin - should drop faster than gravity alone
        let forces_with_spin = TableTennisForces::default();
        let forces_no_spin = TableTennisForces::gravity_only();
        let props = BallProperties::default();

        let with_spin = BallState::new(
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(15.0, 5.0, 0.0),   // Forward and upward
            Vec3::new(0.0, 0.0, 300.0),  // Heavy topspin
        );

        let no_spin = BallState::new(
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(15.0, 5.0, 0.0),
            Vec3::ZERO,
        );

        let dt = 0.001;
        let steps = 200; // 0.2 seconds

        let mut state_spin = with_spin;
        let mut state_nospin = no_spin;

        for _ in 0..steps {
            state_spin = VelocityVerlet::step(&state_spin, &props, &forces_with_spin, dt).state;
            state_nospin = VelocityVerlet::step(&state_nospin, &props, &forces_no_spin, dt).state;
        }

        // Ball with topspin should be lower (dropped more)
        assert!(
            state_spin.pos.y < state_nospin.pos.y,
            "Topspin ball should be lower: spin_y={}, nospin_y={}",
            state_spin.pos.y,
            state_nospin.pos.y
        );
    }

    #[test]
    fn test_realistic_serve_trajectory() {
        // Simulate a typical serve: ~15 m/s, moderate topspin
        let forces = TableTennisForces::default();
        let props = BallProperties::default();

        let serve = BallState::new(
            Vec3::new(-1.0, 0.9, 0.0),  // Start behind table, above net height
            Vec3::new(12.0, 2.0, 0.0),  // Forward and slightly up
            Vec3::new(0.0, 0.0, 150.0), // Moderate topspin (~1500 RPM)
        );

        let dt = 0.0001; // 0.1ms for accuracy
        let mut state = serve;
        let mut max_height = state.pos.y;
        let mut time = 0.0;

        // Simulate for 1 second
        while time < 1.0 && state.pos.y > 0.0 {
            state = VelocityVerlet::step(&state, &props, &forces, dt).state;
            max_height = max_height.max(state.pos.y);
            time += dt;
        }

        // Ball should have traveled forward
        assert!(state.pos.x > 0.0, "Ball should move forward");

        // Max height should be reasonable (not too high due to drag)
        assert!(
            max_height < 2.0,
            "Max height should be < 2m, got {}",
            max_height
        );

        // Should have some forward progress
        assert!(
            state.pos.x > 1.0,
            "Should travel at least 1m forward, got {}",
            state.pos.x
        );
    }
}
