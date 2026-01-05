//! Swept sphere collision detection.
//!
//! Detects collisions between a moving sphere (ball) and static surfaces
//! (table, paddles, net) using continuous collision detection.

use crate::types::{
    constants, BallProperties, BallState, CollisionInfo, CollisionTarget, PaddleState, TableState,
    Vec3,
};

/// Configuration for collision detection.
#[derive(Debug, Clone)]
pub struct CollisionConfig {
    /// Paddle radius in meters (standard paddle is ~0.085m radius)
    pub paddle_radius: f64,
}

impl Default for CollisionConfig {
    fn default() -> Self {
        Self {
            paddle_radius: 0.085,
        }
    }
}

/// Collision detector for table tennis simulation.
pub struct CollisionDetector {
    pub config: CollisionConfig,
}

impl Default for CollisionDetector {
    fn default() -> Self {
        Self::new()
    }
}

impl CollisionDetector {
    pub fn new() -> Self {
        Self {
            config: CollisionConfig::default(),
        }
    }

    pub fn with_config(config: CollisionConfig) -> Self {
        Self { config }
    }

    /// Detect the first collision during a timestep.
    ///
    /// # Arguments
    /// * `ball` - Current ball state
    /// * `ball_props` - Ball physical properties
    /// * `table` - Table state
    /// * `paddles` - List of paddle states
    /// * `dt` - Timestep duration
    ///
    /// # Returns
    /// The earliest collision if any occurs, or None.
    pub fn detect(
        &self,
        ball: &BallState,
        ball_props: &BallProperties,
        table: &TableState,
        paddles: &[PaddleState],
        dt: f64,
    ) -> Option<CollisionInfo> {
        let mut earliest: Option<CollisionInfo> = None;

        // Check table collision
        if let Some(info) = self.detect_table_collision(ball, ball_props, table, dt) {
            earliest = Some(info);
        }

        // Check net collision
        if let Some(info) = self.detect_net_collision(ball, ball_props, table, dt) {
            if earliest.is_none() || info.time < earliest.as_ref().unwrap().time {
                earliest = Some(info);
            }
        }

        // Check paddle collisions
        for (i, paddle) in paddles.iter().enumerate() {
            if let Some(mut info) = self.detect_paddle_collision(ball, ball_props, paddle, dt) {
                info.target = CollisionTarget::Paddle(i);
                if earliest.is_none() || info.time < earliest.as_ref().unwrap().time {
                    earliest = Some(info);
                }
            }
        }

        earliest
    }

    /// Detect collision with the table surface.
    ///
    /// The table is modeled as a horizontal plane at y = table.surface_y().
    /// Collision occurs when ball center reaches distance ball_radius from surface.
    fn detect_table_collision(
        &self,
        ball: &BallState,
        ball_props: &BallProperties,
        table: &TableState,
        dt: f64,
    ) -> Option<CollisionInfo> {
        let surface_y = table.surface_y();
        let radius = ball_props.radius;

        // Ball center Y where collision occurs
        let collision_y = surface_y + radius;

        // Only check if ball is moving downward toward table
        if ball.vel.y >= 0.0 {
            return None;
        }

        // Current distance above collision plane
        let current_dist = ball.pos.y - collision_y;

        // If already below collision plane, something went wrong
        if current_dist < -constants::EPSILON {
            // Already penetrating - return immediate collision
            return Some(CollisionInfo {
                target: CollisionTarget::Table,
                time: 0.0,
                point: Vec3::new(ball.pos.x, surface_y, ball.pos.z),
                normal: Vec3::new(0.0, 1.0, 0.0),
                penetration: -current_dist,
            });
        }

        // Time to reach collision plane: t = distance / speed
        let t_hit = current_dist / (-ball.vel.y);

        // Check if collision happens within this timestep
        if t_hit < 0.0 || t_hit > dt {
            return None;
        }

        // Calculate collision point
        let hit_x = ball.pos.x + ball.vel.x * t_hit;
        let hit_z = ball.pos.z + ball.vel.z * t_hit;

        // Check if collision is within table bounds
        if !table.is_over_table(hit_x, hit_z) {
            return None;
        }

        Some(CollisionInfo {
            target: CollisionTarget::Table,
            time: t_hit,
            point: Vec3::new(hit_x, surface_y, hit_z),
            normal: Vec3::new(0.0, 1.0, 0.0), // Table normal points up
            penetration: 0.0,
        })
    }

    /// Detect collision with the net.
    ///
    /// The net is a vertical plane at x = table.pos.x (center of table).
    /// It extends from table surface to table.surface_y() + table.net_height.
    fn detect_net_collision(
        &self,
        ball: &BallState,
        ball_props: &BallProperties,
        table: &TableState,
        dt: f64,
    ) -> Option<CollisionInfo> {
        let net_x = table.pos.x;
        let radius = ball_props.radius;

        // Determine which side of net the ball is on
        let ball_side = (ball.pos.x - net_x).signum();

        // Only check if ball is moving toward net
        if ball_side * ball.vel.x >= 0.0 {
            return None; // Moving away from net
        }

        // Distance to net plane (accounting for ball radius)
        let collision_x = net_x + ball_side * radius;
        let current_dist = (ball.pos.x - collision_x).abs();

        // Time to reach net
        let t_hit = current_dist / ball.vel.x.abs();

        if t_hit < 0.0 || t_hit > dt {
            return None;
        }

        // Calculate collision point
        let hit_y = ball.pos.y + ball.vel.y * t_hit;
        let hit_z = ball.pos.z + ball.vel.z * t_hit;

        // Check if within net bounds
        let net_bottom = table.surface_y();
        let net_top = net_bottom + table.net_height;

        // Ball must hit the net (between table surface and net top)
        if hit_y - radius > net_top || hit_y + radius < net_bottom {
            return None; // Ball goes over or under
        }

        // Check Z bounds (within table width)
        if hit_z.abs() > table.half_width {
            return None; // Ball goes around the net
        }

        Some(CollisionInfo {
            target: CollisionTarget::Net,
            time: t_hit,
            point: Vec3::new(net_x, hit_y, hit_z),
            normal: Vec3::new(-ball_side, 0.0, 0.0), // Normal points toward ball
            penetration: 0.0,
        })
    }

    /// Detect collision with a paddle.
    ///
    /// The paddle is modeled as a circular disk with:
    /// - Center at paddle.pos
    /// - Normal direction paddle.normal
    /// - Radius from config.paddle_radius
    fn detect_paddle_collision(
        &self,
        ball: &BallState,
        ball_props: &BallProperties,
        paddle: &PaddleState,
        dt: f64,
    ) -> Option<CollisionInfo> {
        let radius = ball_props.radius;
        let paddle_radius = self.config.paddle_radius;

        // Paddle plane equation: normal · (p - paddle.pos) = 0
        // Ball collides when center is at distance `radius` from plane

        // Vector from paddle center to ball
        let to_ball = ball.pos - paddle.pos;

        // Distance from ball center to paddle plane (signed)
        let dist_to_plane = to_ball.dot(&paddle.normal);

        // Ball is on negative side if dist < 0
        let ball_side = dist_to_plane.signum();

        // Collision plane is at distance `radius` from paddle plane
        let collision_dist = ball_side * radius;

        // Current distance to collision plane
        let current_dist = dist_to_plane - collision_dist;

        // Velocity component toward paddle plane
        let vel_toward_plane = -ball.vel.dot(&paddle.normal) * ball_side;

        // Only check if moving toward paddle
        if vel_toward_plane <= 0.0 {
            return None;
        }

        // Time to reach collision plane
        let t_hit = current_dist.abs() / vel_toward_plane;

        if t_hit < 0.0 || t_hit > dt {
            return None;
        }

        // Calculate ball position at collision time
        let ball_at_hit = ball.pos + ball.vel * t_hit;

        // Project onto paddle plane to get collision point
        let to_ball_at_hit = ball_at_hit - paddle.pos;
        let dist_at_hit = to_ball_at_hit.dot(&paddle.normal);
        let collision_point = ball_at_hit - paddle.normal * dist_at_hit;

        // Check if collision point is within paddle radius
        let offset_from_center = collision_point - paddle.pos;
        let dist_from_center = offset_from_center.magnitude();

        if dist_from_center > paddle_radius {
            return None; // Missed the paddle
        }

        Some(CollisionInfo {
            target: CollisionTarget::Paddle(0), // Index set by caller
            time: t_hit,
            point: collision_point,
            normal: paddle.normal * ball_side, // Normal toward ball
            penetration: 0.0,
        })
    }

    /// Advance ball to collision point and return remaining time.
    ///
    /// This is used to handle the collision:
    /// 1. Move ball to collision point
    /// 2. Apply collision response (in resolution module)
    /// 3. Continue simulation with remaining time
    pub fn advance_to_collision(ball: &BallState, collision: &CollisionInfo) -> BallState {
        BallState {
            pos: ball.pos + ball.vel * collision.time,
            vel: ball.vel,
            spin: ball.spin,
        }
    }
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn test_ball() -> BallProperties {
        BallProperties::default()
    }

    fn test_table() -> TableState {
        TableState::ittf_regulation()
    }

    #[test]
    fn test_table_collision_falling_ball() {
        let detector = CollisionDetector::new();
        let table = test_table();
        let props = test_ball();

        // Ball falling toward table
        let ball = BallState::new(
            Vec3::new(0.0, table.surface_y() + 0.1, 0.0), // 10cm above table
            Vec3::new(0.0, -2.0, 0.0),                    // Falling at 2 m/s
            Vec3::ZERO,
        );

        let collision = detector.detect_table_collision(&ball, &props, &table, 1.0);

        assert!(collision.is_some(), "Should detect table collision");
        let info = collision.unwrap();

        // Time to fall 0.1m - radius at 2 m/s
        let expected_dist = 0.1 - props.radius;
        let expected_time = expected_dist / 2.0;

        assert!(
            (info.time - expected_time).abs() < 0.001,
            "Time should be ~{}, got {}",
            expected_time,
            info.time
        );
        assert_eq!(info.target, CollisionTarget::Table);
        assert!((info.normal.y - 1.0).abs() < constants::EPSILON);
    }

    #[test]
    fn test_no_collision_rising_ball() {
        let detector = CollisionDetector::new();
        let table = test_table();
        let props = test_ball();

        // Ball moving upward
        let ball = BallState::new(
            Vec3::new(0.0, table.surface_y() + 0.1, 0.0),
            Vec3::new(0.0, 2.0, 0.0), // Rising
            Vec3::ZERO,
        );

        let collision = detector.detect_table_collision(&ball, &props, &table, 1.0);
        assert!(collision.is_none(), "Should not detect collision for rising ball");
    }

    #[test]
    fn test_no_collision_outside_table() {
        let detector = CollisionDetector::new();
        let table = test_table();
        let props = test_ball();

        // Ball falling outside table bounds
        let ball = BallState::new(
            Vec3::new(5.0, table.surface_y() + 0.1, 0.0), // Way outside table
            Vec3::new(0.0, -2.0, 0.0),
            Vec3::ZERO,
        );

        let collision = detector.detect_table_collision(&ball, &props, &table, 1.0);
        assert!(collision.is_none(), "Should not collide outside table");
    }

    #[test]
    fn test_net_collision() {
        let detector = CollisionDetector::new();
        let table = test_table();
        let props = test_ball();

        // Ball moving toward net at net height
        let ball = BallState::new(
            Vec3::new(-0.5, table.surface_y() + 0.1, 0.0), // Left of net
            Vec3::new(5.0, 0.0, 0.0),                      // Moving right toward net
            Vec3::ZERO,
        );

        let collision = detector.detect_net_collision(&ball, &props, &table, 1.0);

        assert!(collision.is_some(), "Should detect net collision");
        let info = collision.unwrap();
        assert_eq!(info.target, CollisionTarget::Net);
    }

    #[test]
    fn test_ball_over_net() {
        let detector = CollisionDetector::new();
        let table = test_table();
        let props = test_ball();

        // Ball moving over net (high enough to clear)
        let ball = BallState::new(
            Vec3::new(-0.5, table.surface_y() + table.net_height + 0.1, 0.0),
            Vec3::new(5.0, 0.0, 0.0),
            Vec3::ZERO,
        );

        let collision = detector.detect_net_collision(&ball, &props, &table, 1.0);
        assert!(collision.is_none(), "Ball should clear the net");
    }

    #[test]
    fn test_paddle_collision_direct_hit() {
        let detector = CollisionDetector::new();
        let props = test_ball();

        // Paddle facing the ball
        let paddle = PaddleState::new(
            Vec3::new(0.0, 1.0, 0.0),  // Paddle position
            Vec3::new(-1.0, 0.0, 0.0), // Facing negative X
            Vec3::ZERO,
            "test".to_string(),
        );

        // Ball moving toward paddle
        let ball = BallState::new(
            Vec3::new(-0.5, 1.0, 0.0), // Left of paddle
            Vec3::new(5.0, 0.0, 0.0),  // Moving right toward paddle
            Vec3::ZERO,
        );

        let collision = detector.detect_paddle_collision(&ball, &props, &paddle, 1.0);

        assert!(collision.is_some(), "Should detect paddle collision");
    }

    #[test]
    fn test_paddle_collision_miss() {
        let detector = CollisionDetector::new();
        let props = test_ball();

        let paddle = PaddleState::new(
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(-1.0, 0.0, 0.0),
            Vec3::ZERO,
            "test".to_string(),
        );

        // Ball moving past paddle (too high)
        let ball = BallState::new(
            Vec3::new(-0.5, 1.5, 0.0), // Above paddle center by more than paddle radius
            Vec3::new(5.0, 0.0, 0.0),
            Vec3::ZERO,
        );

        let collision = detector.detect_paddle_collision(&ball, &props, &paddle, 1.0);
        assert!(collision.is_none(), "Should miss the paddle");
    }

    #[test]
    fn test_high_speed_no_tunneling() {
        let detector = CollisionDetector::new();
        let table = test_table();
        let props = test_ball();

        // Ball moving VERY fast (100 m/s) - would tunnel with discrete detection
        let ball = BallState::new(
            Vec3::new(0.0, table.surface_y() + 0.5, 0.0),
            Vec3::new(0.0, -100.0, 0.0), // 100 m/s downward!
            Vec3::ZERO,
        );

        // Large timestep that would cause tunneling
        let dt = 0.1; // Ball would move 10m in this time

        let collision = detector.detect_table_collision(&ball, &props, &table, dt);

        assert!(collision.is_some(), "Should detect collision even at high speed");
        let info = collision.unwrap();

        // Collision should happen very quickly
        assert!(
            info.time < 0.01,
            "Collision should be detected early, got t={}",
            info.time
        );
    }

    #[test]
    fn test_detect_earliest_collision() {
        let detector = CollisionDetector::new();
        let table = test_table();
        let props = test_ball();

        // Ball that would hit both net and table - net should be first
        let ball = BallState::new(
            Vec3::new(-0.3, table.surface_y() + 0.1, 0.0), // Near net, low
            Vec3::new(2.0, -0.5, 0.0),                     // Moving toward net and down
            Vec3::ZERO,
        );

        let paddles: Vec<PaddleState> = vec![];
        let collision = detector.detect(&ball, &props, &table, &paddles, 1.0);

        assert!(collision.is_some());
        // Should hit net first since it's closer
        assert_eq!(collision.unwrap().target, CollisionTarget::Net);
    }

    #[test]
    fn test_advance_to_collision() {
        let ball = BallState::new(
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(10.0, -5.0, 0.0),
            Vec3::new(0.0, 0.0, 100.0),
        );

        let collision = CollisionInfo {
            target: CollisionTarget::Table,
            time: 0.1,
            point: Vec3::new(1.0, 0.5, 0.0),
            normal: Vec3::new(0.0, 1.0, 0.0),
            penetration: 0.0,
        };

        let advanced = CollisionDetector::advance_to_collision(&ball, &collision);

        // Position should be ball.pos + ball.vel * 0.1
        assert!((advanced.pos.x - 1.0).abs() < constants::EPSILON);
        assert!((advanced.pos.y - 0.5).abs() < constants::EPSILON);

        // Velocity and spin unchanged
        assert_eq!(advanced.vel, ball.vel);
        assert_eq!(advanced.spin, ball.spin);
    }
}
