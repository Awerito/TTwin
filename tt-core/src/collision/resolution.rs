//! Collision resolution for table tennis physics.
//!
//! Computes post-collision velocity and spin based on:
//! - Coefficient of restitution (bounciness)
//! - Friction (spin transfer)
//! - Surface properties (rubber grip, table friction)
//!
//! ## Model Assumptions
//!
//! - **Paddle as infinite mass**: The paddle doesn't move from ball impact.
//!   Valid because m_ball (~2.7g) << m_paddle+arm (~500g+).
//! - **Instantaneous collision**: We don't simulate the deformation over time,
//!   but use coefficients that capture the integrated effect.
//!
//! ## Spin Transfer Model
//!
//! When ball hits surface with spin:
//! 1. Normal component: reflects with energy loss (COR)
//! 2. Tangential component: friction affects spin
//!
//! ```text
//! Incoming ball with topspin hitting paddle:
//!
//!     ↓ velocity      ↻ spin (topspin)
//!     ●───────→
//!        ══════════ paddle surface
//!
//! The spin creates tangential velocity at contact point.
//! Friction opposes this, transferring momentum to/from spin.
//! ```

use crate::types::{
    constants, BallProperties, BallState, CollisionInfo, CollisionTarget, PaddleState,
    RubberProperties, SurfaceProperties, Vec3,
};

/// Collision resolver for table tennis.
pub struct CollisionResolver;

impl CollisionResolver {
    /// Resolve a collision and return the new ball state.
    ///
    /// # Arguments
    /// * `ball` - Ball state at moment of collision
    /// * `ball_props` - Ball physical properties
    /// * `collision` - Collision information (point, normal, etc.)
    /// * `paddles` - Paddle states (for getting rubber properties)
    /// * `rubber_props` - Rubber properties indexed by paddle
    /// * `surface_props` - Table surface properties
    ///
    /// # Returns
    /// New ball state after collision response.
    pub fn resolve(
        ball: &BallState,
        ball_props: &BallProperties,
        collision: &CollisionInfo,
        paddles: &[PaddleState],
        rubber_props: &[RubberProperties],
        surface_props: &SurfaceProperties,
    ) -> BallState {
        match collision.target {
            CollisionTarget::Table => {
                Self::resolve_table_collision(ball, ball_props, collision, surface_props)
            }
            CollisionTarget::Paddle(idx) => {
                let paddle = &paddles[idx];
                let rubber = &rubber_props[idx];
                Self::resolve_paddle_collision(ball, ball_props, collision, paddle, rubber)
            }
            CollisionTarget::Net => Self::resolve_net_collision(ball, collision),
            CollisionTarget::None => *ball,
        }
    }

    /// Resolve collision with table surface.
    fn resolve_table_collision(
        ball: &BallState,
        ball_props: &BallProperties,
        collision: &CollisionInfo,
        surface: &SurfaceProperties,
    ) -> BallState {
        // Use ball's restitution against table (characteristic of ball+table combo)
        let cor = ball_props.restitution_table.min(surface.restitution);

        // Decompose velocity into normal and tangential components
        let normal = collision.normal;
        let vel_normal = normal * ball.vel.dot(&normal);
        let vel_tangent = ball.vel - vel_normal;

        // Reflect normal component with energy loss
        let new_vel_normal = vel_normal * (-cor);

        // Tangential velocity at contact point due to spin
        // v_contact = v_center + ω × r (where r points from center to contact)
        let r_to_contact = normal * (-ball_props.radius);
        let spin_surface_vel = ball.spin.cross(&r_to_contact);

        // Total tangential velocity at contact
        let contact_tangent_vel = vel_tangent + spin_surface_vel;
        let tangent_speed = contact_tangent_vel.magnitude();

        // Apply friction to tangential motion
        let (new_vel_tangent, new_spin) = if tangent_speed > constants::EPSILON {
            // Friction force opposes tangential motion
            // For table: moderate friction, some spin preserved
            let friction_factor = surface.friction;

            // Friction reduces tangential velocity
            let tangent_reduction = friction_factor * 0.3; // Empirical factor
            let new_tangent = vel_tangent * (1.0 - tangent_reduction);

            // Spin is affected by friction at contact
            // Topspin hitting table: friction opposes forward spin
            // This reduces spin but also affects tangent velocity
            let spin_decay = 0.8; // Table doesn't grip as much as rubber
            let new_spin = ball.spin * spin_decay;

            (new_tangent, new_spin)
        } else {
            (vel_tangent, ball.spin)
        };

        BallState {
            pos: collision.point + normal * ball_props.radius,
            vel: new_vel_normal + new_vel_tangent,
            spin: new_spin,
        }
    }

    /// Resolve collision with paddle.
    fn resolve_paddle_collision(
        ball: &BallState,
        ball_props: &BallProperties,
        collision: &CollisionInfo,
        paddle: &PaddleState,
        rubber: &RubberProperties,
    ) -> BallState {
        let normal = collision.normal;
        let radius = ball_props.radius;

        // Relative velocity (ball velocity minus paddle velocity)
        let rel_vel = ball.vel - paddle.vel;

        // Decompose into normal and tangential
        let vel_normal_mag = rel_vel.dot(&normal);
        let vel_normal = normal * vel_normal_mag;
        let vel_tangent = rel_vel - vel_normal;

        // === Normal component: bounce with restitution ===
        let new_vel_normal = vel_normal * (-rubber.restitution);

        // === Tangential component: spin transfer ===

        // Contact point velocity from ball spin
        let r_to_contact = normal * (-radius);
        let spin_surface_vel = ball.spin.cross(&r_to_contact);

        // Total slip velocity at contact
        let slip_vel = vel_tangent + spin_surface_vel;
        let slip_speed = slip_vel.magnitude();

        let (new_vel_tangent, new_spin) = if slip_speed > constants::EPSILON {
            let slip_dir = slip_vel.normalized();

            // Friction impulse magnitude
            // Limited by Coulomb friction: |F_friction| <= μ * |F_normal|
            let normal_impulse = vel_normal_mag.abs() * (1.0 + rubber.restitution);
            let max_friction_impulse = rubber.friction_dynamic * normal_impulse;

            // Calculate impulse needed to stop slip
            // This involves both linear and angular momentum
            let ball_mass = ball_props.mass;
            let inertia = (2.0 / 3.0) * ball_mass * radius * radius; // Hollow sphere

            // Effective mass for tangential impulse
            let eff_mass = 1.0 / (1.0 / ball_mass + radius * radius / inertia);
            let required_impulse = slip_speed * eff_mass;

            // Apply friction impulse (capped by Coulomb limit)
            let friction_impulse = required_impulse.min(max_friction_impulse);
            let impulse_vec = slip_dir * (-friction_impulse);

            // Update tangential velocity
            let delta_vel_tangent = impulse_vec / ball_mass;
            let new_tangent = vel_tangent + delta_vel_tangent;

            // Update spin from friction torque
            // Torque = r × F, angular impulse = r × impulse
            let angular_impulse = r_to_contact.cross(&impulse_vec);
            let delta_spin = angular_impulse / inertia;
            let spin_from_friction = ball.spin + delta_spin;

            // === Spin transfer from paddle motion ===
            // Moving paddle imparts spin to ball
            let paddle_tangent_vel = paddle.vel - normal * paddle.vel.dot(&normal);
            let paddle_spin_contribution = if paddle_tangent_vel.magnitude() > constants::EPSILON {
                // Paddle brushing motion creates spin
                // Direction: perpendicular to paddle motion and normal
                let brush_axis = normal.cross(&paddle_tangent_vel.normalized());
                let brush_magnitude =
                    paddle_tangent_vel.magnitude() * rubber.spin_transfer_rate / radius;
                brush_axis * brush_magnitude
            } else {
                Vec3::ZERO
            };

            // === Incoming spin handling ===
            // Tacky rubber "grabs" incoming spin and can reverse it
            let spin_reversal = ball.spin * (-rubber.spin_reversal_factor);

            // Combine spin contributions
            let combined_spin = spin_from_friction * 0.5
                + paddle_spin_contribution * rubber.spin_transfer_rate
                + spin_reversal * 0.3;

            (new_tangent, combined_spin)
        } else {
            // No slip - rolling contact
            (vel_tangent, ball.spin * 0.9)
        };

        // Add back paddle velocity (we computed in relative frame)
        let final_vel = new_vel_normal + new_vel_tangent + paddle.vel;

        BallState {
            pos: collision.point + normal * radius,
            vel: final_vel,
            spin: new_spin,
        }
    }

    /// Resolve collision with net.
    ///
    /// Net collision is mostly inelastic - ball loses most energy.
    fn resolve_net_collision(ball: &BallState, collision: &CollisionInfo) -> BallState {
        let normal = collision.normal;

        // Net is very soft/inelastic
        let cor = 0.2;

        // Decompose velocity
        let vel_normal = normal * ball.vel.dot(&normal);
        let vel_tangent = ball.vel - vel_normal;

        // Net absorbs most normal energy, keeps some tangential
        let new_vel = vel_normal * (-cor) + vel_tangent * 0.5;

        // Spin mostly killed by net contact
        let new_spin = ball.spin * 0.3;

        BallState {
            pos: ball.pos, // Keep current position (collision point handled elsewhere)
            vel: new_vel,
            spin: new_spin,
        }
    }
}

/// Simplified resolver that uses default properties.
///
/// Useful for testing or when you don't need material-specific behavior.
pub struct SimpleResolver;

impl SimpleResolver {
    /// Resolve with default material properties.
    pub fn resolve(
        ball: &BallState,
        ball_props: &BallProperties,
        collision: &CollisionInfo,
    ) -> BallState {
        match collision.target {
            CollisionTarget::Table => {
                let surface = SurfaceProperties::default();
                CollisionResolver::resolve_table_collision(ball, ball_props, collision, &surface)
            }
            CollisionTarget::Paddle(_) => {
                // Use default rubber for simple resolution
                let rubber = RubberProperties::default();
                let paddle = PaddleState::new(
                    collision.point,
                    collision.normal,
                    Vec3::ZERO,
                    "default".to_string(),
                );
                CollisionResolver::resolve_paddle_collision(
                    ball, ball_props, collision, &paddle, &rubber,
                )
            }
            CollisionTarget::Net => CollisionResolver::resolve_net_collision(ball, collision),
            CollisionTarget::None => *ball,
        }
    }
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn test_ball_props() -> BallProperties {
        BallProperties::default()
    }

    fn default_surface() -> SurfaceProperties {
        SurfaceProperties::default()
    }

    fn default_rubber() -> RubberProperties {
        RubberProperties::default()
    }

    #[test]
    fn test_table_bounce_height() {
        // ITTF spec: ball dropped from 30cm should bounce ~23cm
        // COR = sqrt(23/30) ≈ 0.876, but ball+table combined ≈ 0.77
        let props = test_ball_props();
        let surface = default_surface();

        // Ball falling straight down
        let ball = BallState::new(Vec3::ZERO, Vec3::new(0.0, -2.0, 0.0), Vec3::ZERO);

        let collision = CollisionInfo {
            target: CollisionTarget::Table,
            time: 0.0,
            point: Vec3::new(0.0, 0.0, 0.0),
            normal: Vec3::new(0.0, 1.0, 0.0),
            penetration: 0.0,
        };

        let result =
            CollisionResolver::resolve_table_collision(&ball, &props, &collision, &surface);

        // Should bounce upward
        assert!(result.vel.y > 0.0, "Ball should bounce up");

        // Velocity should be reduced by COR
        let expected_vy = 2.0 * props.restitution_table;
        assert!(
            (result.vel.y - expected_vy).abs() < 0.1,
            "Bounce velocity should be ~{}, got {}",
            expected_vy,
            result.vel.y
        );
    }

    #[test]
    fn test_table_topspin_effect() {
        let props = test_ball_props();
        let surface = default_surface();

        // Ball with topspin hitting table
        let ball = BallState::new(
            Vec3::ZERO,
            Vec3::new(5.0, -2.0, 0.0),   // Forward and down
            Vec3::new(0.0, 0.0, 200.0),  // Topspin
        );

        let collision = CollisionInfo {
            target: CollisionTarget::Table,
            time: 0.0,
            point: Vec3::ZERO,
            normal: Vec3::new(0.0, 1.0, 0.0),
            penetration: 0.0,
        };

        let result =
            CollisionResolver::resolve_table_collision(&ball, &props, &collision, &surface);

        // Topspin should accelerate ball forward on bounce
        // (friction with spinning ball pushes it forward)
        assert!(
            result.vel.x >= ball.vel.x * 0.8,
            "Topspin should maintain/increase forward speed"
        );
    }

    #[test]
    fn test_paddle_bounce() {
        let props = test_ball_props();
        let rubber = default_rubber();

        // Ball hitting paddle straight on
        let ball = BallState::new(
            Vec3::new(-0.1, 0.0, 0.0),
            Vec3::new(10.0, 0.0, 0.0), // Moving right
            Vec3::ZERO,
        );

        let paddle = PaddleState::new(
            Vec3::ZERO,
            Vec3::new(-1.0, 0.0, 0.0), // Facing left (toward ball)
            Vec3::ZERO,                // Stationary
            "test".to_string(),
        );

        let collision = CollisionInfo {
            target: CollisionTarget::Paddle(0),
            time: 0.0,
            point: Vec3::new(-props.radius, 0.0, 0.0),
            normal: Vec3::new(-1.0, 0.0, 0.0),
            penetration: 0.0,
        };

        let result =
            CollisionResolver::resolve_paddle_collision(&ball, &props, &collision, &paddle, &rubber);

        // Ball should bounce back
        assert!(
            result.vel.x < 0.0,
            "Ball should bounce back, got vx={}",
            result.vel.x
        );

        // Speed should be reduced by COR
        assert!(
            result.vel.x.abs() < ball.vel.x.abs(),
            "Speed should decrease after bounce"
        );
    }

    #[test]
    fn test_paddle_spin_generation() {
        let props = test_ball_props();
        let rubber = default_rubber();

        // Ball hitting paddle, paddle moving upward (brushing for topspin)
        let ball = BallState::new(
            Vec3::new(-0.1, 0.0, 0.0),
            Vec3::new(5.0, 0.0, 0.0),
            Vec3::ZERO, // No initial spin
        );

        let paddle = PaddleState::new(
            Vec3::ZERO,
            Vec3::new(-1.0, 0.0, 0.0),
            Vec3::new(0.0, 5.0, 0.0), // Moving upward (brush stroke)
            "test".to_string(),
        );

        let collision = CollisionInfo {
            target: CollisionTarget::Paddle(0),
            time: 0.0,
            point: Vec3::new(-props.radius, 0.0, 0.0),
            normal: Vec3::new(-1.0, 0.0, 0.0),
            penetration: 0.0,
        };

        let result =
            CollisionResolver::resolve_paddle_collision(&ball, &props, &collision, &paddle, &rubber);

        // Should generate spin from brush motion
        let spin_magnitude = result.spin.magnitude();
        assert!(
            spin_magnitude > 1.0,
            "Should generate spin from brush, got |ω|={}",
            spin_magnitude
        );
    }

    #[test]
    fn test_net_collision_absorbs_energy() {
        let ball = BallState::new(
            Vec3::ZERO,
            Vec3::new(10.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 100.0),
        );

        let collision = CollisionInfo {
            target: CollisionTarget::Net,
            time: 0.0,
            point: Vec3::ZERO,
            normal: Vec3::new(-1.0, 0.0, 0.0),
            penetration: 0.0,
        };

        let result = CollisionResolver::resolve_net_collision(&ball, &collision);

        // Net should absorb most energy
        let speed_before = ball.vel.magnitude();
        let speed_after = result.vel.magnitude();
        assert!(
            speed_after < speed_before * 0.5,
            "Net should absorb most energy"
        );

        // Spin should be mostly killed
        assert!(
            result.spin.magnitude() < ball.spin.magnitude() * 0.5,
            "Net should reduce spin"
        );
    }

    #[test]
    fn test_simple_resolver() {
        let props = test_ball_props();
        let ball = BallState::new(Vec3::ZERO, Vec3::new(0.0, -5.0, 0.0), Vec3::ZERO);

        let collision = CollisionInfo {
            target: CollisionTarget::Table,
            time: 0.0,
            point: Vec3::ZERO,
            normal: Vec3::new(0.0, 1.0, 0.0),
            penetration: 0.0,
        };

        let result = SimpleResolver::resolve(&ball, &props, &collision);

        // Should bounce
        assert!(result.vel.y > 0.0);
    }

    #[test]
    fn test_angled_paddle_changes_direction() {
        let props = test_ball_props();
        let rubber = default_rubber();

        // Ball coming straight, paddle angled 45 degrees
        let ball = BallState::new(
            Vec3::new(-0.1, 0.0, 0.0),
            Vec3::new(10.0, 0.0, 0.0),
            Vec3::ZERO,
        );

        // Paddle normal pointing up-left (45 degrees)
        let normal = Vec3::new(-1.0, 1.0, 0.0).normalized();
        let paddle = PaddleState::new(Vec3::ZERO, normal, Vec3::ZERO, "test".to_string());

        let collision = CollisionInfo {
            target: CollisionTarget::Paddle(0),
            time: 0.0,
            point: Vec3::ZERO,
            normal,
            penetration: 0.0,
        };

        let result =
            CollisionResolver::resolve_paddle_collision(&ball, &props, &collision, &paddle, &rubber);

        // Ball should have upward component after bounce
        assert!(
            result.vel.y > 0.0,
            "Angled paddle should deflect ball upward, got vy={}",
            result.vel.y
        );
    }
}
