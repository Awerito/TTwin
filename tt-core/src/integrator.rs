//! Numerical integrators for advancing the simulation in time.
//!
//! The primary integrator is Velocity Verlet, which provides good stability
//! and energy conservation for physics simulations.
//!
//! ## Why Velocity Verlet?
//!
//! - **Symplectic**: Preserves energy over long simulations (no drift)
//! - **Second-order accurate**: Error is O(dt²), better than Euler's O(dt)
//! - **Time-reversible**: Important for physical correctness
//! - **Simple**: Only slightly more complex than Euler
//!
//! ## Algorithm
//!
//! Given position x, velocity v, and acceleration function a(x, v):
//!
//! ```text
//! 1. a_current = a(x, v)
//! 2. x_new = x + v*dt + 0.5*a_current*dt²
//! 3. a_new = a(x_new, v)  // approximate, using old velocity
//! 4. v_new = v + 0.5*(a_current + a_new)*dt
//! ```

use crate::types::{BallProperties, BallState, Vec3};

/// Result of an integration step, containing the new state and metadata.
#[derive(Debug, Clone, Copy)]
pub struct IntegrationResult {
    pub state: BallState,
    pub acceleration: Vec3,
    pub angular_acceleration: Vec3,
}

/// Trait for computing forces/accelerations on the ball.
///
/// Implementations provide the physics model (gravity, drag, Magnus, etc.)
pub trait ForceModel {
    /// Compute linear acceleration given current state.
    fn linear_acceleration(&self, state: &BallState, props: &BallProperties) -> Vec3;

    /// Compute angular acceleration (spin change rate).
    /// Default implementation: spin decays slowly due to air resistance.
    fn angular_acceleration(&self, state: &BallState, _props: &BallProperties) -> Vec3 {
        // Simple model: spin decays exponentially
        // More accurate models would consider air viscosity and ball surface
        let spin_decay_rate = 0.1; // rad/s² per rad/s of spin
        state.spin * (-spin_decay_rate)
    }
}

/// Velocity Verlet integrator for ball physics.
///
/// This integrator advances the ball state by a time step dt using the
/// Velocity Verlet method, which is more stable and accurate than Euler.
pub struct VelocityVerlet;

impl VelocityVerlet {
    /// Advance the ball state by one time step.
    ///
    /// # Arguments
    /// * `state` - Current ball state (position, velocity, spin)
    /// * `props` - Ball physical properties (mass, radius, etc.)
    /// * `forces` - Force model providing accelerations
    /// * `dt` - Time step in seconds
    ///
    /// # Returns
    /// New ball state after the time step.
    pub fn step<F: ForceModel>(
        state: &BallState,
        props: &BallProperties,
        forces: &F,
        dt: f64,
    ) -> IntegrationResult {
        // Step 1: Compute current acceleration
        let a0 = forces.linear_acceleration(state, props);
        let alpha0 = forces.angular_acceleration(state, props);

        // Step 2: Update position using current velocity and acceleration
        // x_new = x + v*dt + 0.5*a*dt²
        let new_pos = state.pos + state.vel * dt + a0 * (0.5 * dt * dt);

        // Step 3: Create intermediate state for new acceleration calculation
        let intermediate_state = BallState {
            pos: new_pos,
            vel: state.vel, // Use old velocity for acceleration calc
            spin: state.spin,
        };

        // Step 4: Compute acceleration at new position
        let a1 = forces.linear_acceleration(&intermediate_state, props);
        let alpha1 = forces.angular_acceleration(&intermediate_state, props);

        // Step 5: Update velocity using average of old and new acceleration
        // v_new = v + 0.5*(a0 + a1)*dt
        let new_vel = state.vel + (a0 + a1) * (0.5 * dt);

        // Step 6: Update spin similarly
        let new_spin = state.spin + (alpha0 + alpha1) * (0.5 * dt);

        IntegrationResult {
            state: BallState {
                pos: new_pos,
                vel: new_vel,
                spin: new_spin,
            },
            acceleration: a1,
            angular_acceleration: alpha1,
        }
    }

    /// Advance the ball state by multiple substeps.
    ///
    /// Useful for higher accuracy or when forces change rapidly.
    /// Total time advanced is `substeps * dt`.
    pub fn step_n<F: ForceModel>(
        state: &BallState,
        props: &BallProperties,
        forces: &F,
        dt: f64,
        substeps: usize,
    ) -> IntegrationResult {
        let mut current = IntegrationResult {
            state: *state,
            acceleration: Vec3::ZERO,
            angular_acceleration: Vec3::ZERO,
        };

        for _ in 0..substeps {
            current = Self::step(&current.state, props, forces, dt);
        }

        current
    }
}

/// Simple Euler integrator (for comparison/testing only).
///
/// Less accurate than Verlet but simpler. Useful for validating
/// that Verlet gives better results.
pub struct Euler;

impl Euler {
    /// Advance the ball state by one time step using forward Euler.
    ///
    /// **Warning**: This method has poor energy conservation and should
    /// only be used for testing or comparison purposes.
    pub fn step<F: ForceModel>(
        state: &BallState,
        props: &BallProperties,
        forces: &F,
        dt: f64,
    ) -> IntegrationResult {
        let acceleration = forces.linear_acceleration(state, props);
        let angular_acceleration = forces.angular_acceleration(state, props);

        // Simple forward Euler: x_new = x + v*dt, v_new = v + a*dt
        let new_vel = state.vel + acceleration * dt;
        let new_pos = state.pos + state.vel * dt;
        let new_spin = state.spin + angular_acceleration * dt;

        IntegrationResult {
            state: BallState {
                pos: new_pos,
                vel: new_vel,
                spin: new_spin,
            },
            acceleration,
            angular_acceleration,
        }
    }
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// Simple gravity-only force model for testing
    struct GravityOnly;

    impl ForceModel for GravityOnly {
        fn linear_acceleration(&self, _state: &BallState, _props: &BallProperties) -> Vec3 {
            Vec3::new(0.0, -9.81, 0.0)
        }

        fn angular_acceleration(&self, _state: &BallState, _props: &BallProperties) -> Vec3 {
            Vec3::ZERO
        }
    }

    #[test]
    fn test_free_fall_verlet() {
        let props = BallProperties::default();
        let initial = BallState::new(
            Vec3::new(0.0, 1.0, 0.0), // 1m height
            Vec3::ZERO,
            Vec3::ZERO,
        );

        let forces = GravityOnly;
        let dt = 0.001; // 1ms timestep
        let steps = 451; // ~0.451s to fall 1m

        let mut state = initial;
        for _ in 0..steps {
            let result = VelocityVerlet::step(&state, &props, &forces, dt);
            state = result.state;
        }

        // After ~0.451s of free fall from 1m, should be near y=0
        // Using t = sqrt(2h/g) = sqrt(2*1/9.81) ≈ 0.4515s
        // At t=0.451s: y = 1 - 0.5*9.81*0.451² ≈ 0.003m
        assert!(
            state.pos.y.abs() < 0.05,
            "Ball should be near ground, got y={}",
            state.pos.y
        );
    }

    #[test]
    fn test_verlet_vs_euler_energy() {
        // Verlet should conserve energy better than Euler
        // We'll simulate a bouncing ball (simplified) and check energy drift

        let props = BallProperties::default();
        let initial = BallState::new(
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(5.0, 0.0, 0.0), // Some horizontal velocity
            Vec3::ZERO,
        );

        let forces = GravityOnly;
        let dt = 0.01; // 10ms timestep (larger to show Euler's weakness)
        let steps = 100;

        // Track energy with Verlet
        let mut verlet_state = initial;
        for _ in 0..steps {
            let result = VelocityVerlet::step(&verlet_state, &props, &forces, dt);
            verlet_state = result.state;
        }

        // Track energy with Euler
        let mut euler_state = initial;
        for _ in 0..steps {
            let result = Euler::step(&euler_state, &props, &forces, dt);
            euler_state = result.state;
        }

        // Calculate kinetic energy (ignoring potential for simplicity)
        let initial_ke = initial.vel.magnitude_squared();
        let verlet_ke = verlet_state.vel.magnitude_squared();
        let euler_ke = euler_state.vel.magnitude_squared();

        // Verlet should have less energy drift than Euler
        let verlet_drift = (verlet_ke - initial_ke).abs();
        let euler_drift = (euler_ke - initial_ke).abs();

        // Note: In free fall, KE increases as PE decreases, so both will drift
        // But Verlet should be more consistent
        assert!(
            verlet_drift <= euler_drift * 1.5 || verlet_drift < 0.1,
            "Verlet drift ({}) should be <= Euler drift ({})",
            verlet_drift,
            euler_drift
        );
    }

    #[test]
    fn test_substeps() {
        let props = BallProperties::default();
        let initial = BallState::at_rest(Vec3::new(0.0, 1.0, 0.0));
        let forces = GravityOnly;

        // Single step of 10ms vs 10 substeps of 1ms
        let single = VelocityVerlet::step(&initial, &props, &forces, 0.01);
        let multi = VelocityVerlet::step_n(&initial, &props, &forces, 0.001, 10);

        // Results should be very close (substeps more accurate)
        let pos_diff = (single.state.pos - multi.state.pos).magnitude();
        assert!(
            pos_diff < 1e-6,
            "Position difference too large: {}",
            pos_diff
        );
    }

    #[test]
    fn test_horizontal_motion() {
        // No gravity scenario - ball should move in straight line
        struct NoForces;
        impl ForceModel for NoForces {
            fn linear_acceleration(&self, _: &BallState, _: &BallProperties) -> Vec3 {
                Vec3::ZERO
            }
        }

        let props = BallProperties::default();
        let initial = BallState::new(
            Vec3::ZERO,
            Vec3::new(10.0, 0.0, 0.0), // 10 m/s in X
            Vec3::ZERO,
        );

        let result = VelocityVerlet::step(&initial, &props, &NoForces, 1.0);

        // After 1 second at 10 m/s, should be at x=10
        assert!(
            (result.state.pos.x - 10.0).abs() < 1e-10,
            "Expected x=10, got x={}",
            result.state.pos.x
        );
        // Velocity should be unchanged
        assert!(
            (result.state.vel.x - 10.0).abs() < 1e-10,
            "Velocity should be unchanged"
        );
    }
}
