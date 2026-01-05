//! Core types for the physics simulation.
//!
//! All units are SI:
//! - Position: meters (m)
//! - Velocity: meters per second (m/s)
//! - Angular velocity (spin): radians per second (rad/s)
//! - Mass: kilograms (kg)
//! - Force: Newtons (N)

use serde::{Deserialize, Serialize};
use std::ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign};

// =============================================================================
// Vec3 - 3D Vector
// =============================================================================

/// A 3D vector used for positions, velocities, forces, and spin.
///
/// Coordinate system:
/// - X: horizontal, along the table length (positive toward opponent)
/// - Y: vertical (positive upward)
/// - Z: horizontal, along the table width (positive to the right)
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const ZERO: Vec3 = Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// Squared magnitude (avoids sqrt for comparisons)
    pub fn magnitude_squared(&self) -> f64 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    /// Magnitude (length) of the vector
    pub fn magnitude(&self) -> f64 {
        self.magnitude_squared().sqrt()
    }

    /// Returns a unit vector in the same direction, or zero if magnitude is zero
    pub fn normalized(&self) -> Self {
        let mag = self.magnitude();
        if mag < 1e-10 {
            Self::ZERO
        } else {
            *self / mag
        }
    }

    /// Dot product
    pub fn dot(&self, other: &Self) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    /// Cross product
    pub fn cross(&self, other: &Self) -> Self {
        Self {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }

    /// Component-wise multiplication
    pub fn component_mul(&self, other: &Self) -> Self {
        Self {
            x: self.x * other.x,
            y: self.y * other.y,
            z: self.z * other.z,
        }
    }

    /// Linear interpolation between two vectors
    pub fn lerp(&self, other: &Self, t: f64) -> Self {
        *self + (*other - *self) * t
    }

    /// Reflect vector around a normal
    pub fn reflect(&self, normal: &Self) -> Self {
        *self - *normal * 2.0 * self.dot(normal)
    }

    /// Project this vector onto another vector
    pub fn project_onto(&self, other: &Self) -> Self {
        let other_mag_sq = other.magnitude_squared();
        if other_mag_sq < 1e-10 {
            Self::ZERO
        } else {
            *other * (self.dot(other) / other_mag_sq)
        }
    }
}

// Operator overloads for Vec3
impl Add for Vec3 {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}

impl AddAssign for Vec3 {
    fn add_assign(&mut self, other: Self) {
        self.x += other.x;
        self.y += other.y;
        self.z += other.z;
    }
}

impl Sub for Vec3 {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }
}

impl SubAssign for Vec3 {
    fn sub_assign(&mut self, other: Self) {
        self.x -= other.x;
        self.y -= other.y;
        self.z -= other.z;
    }
}

impl Mul<f64> for Vec3 {
    type Output = Self;
    fn mul(self, scalar: f64) -> Self {
        Self {
            x: self.x * scalar,
            y: self.y * scalar,
            z: self.z * scalar,
        }
    }
}

impl Div<f64> for Vec3 {
    type Output = Self;
    fn div(self, scalar: f64) -> Self {
        Self {
            x: self.x / scalar,
            y: self.y / scalar,
            z: self.z / scalar,
        }
    }
}

impl Neg for Vec3 {
    type Output = Self;
    fn neg(self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }
}

impl Default for Vec3 {
    fn default() -> Self {
        Self::ZERO
    }
}

// =============================================================================
// Ball State
// =============================================================================

/// Complete state of the ball at a given instant.
///
/// The spin vector encodes both the axis and magnitude of rotation:
/// - Direction: axis of rotation (right-hand rule)
/// - Magnitude: angular velocity in rad/s
///
/// Examples:
/// - Topspin: spin.x > 0 (rotates forward)
/// - Backspin: spin.x < 0 (rotates backward)
/// - Sidespin: spin.y != 0 (rotates around vertical axis)
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BallState {
    pub pos: Vec3,
    pub vel: Vec3,
    pub spin: Vec3,
}

impl BallState {
    pub fn new(pos: Vec3, vel: Vec3, spin: Vec3) -> Self {
        Self { pos, vel, spin }
    }

    /// Ball at rest at a given position
    pub fn at_rest(pos: Vec3) -> Self {
        Self {
            pos,
            vel: Vec3::ZERO,
            spin: Vec3::ZERO,
        }
    }

    /// Kinetic energy (translational + rotational)
    pub fn kinetic_energy(&self, ball_props: &BallProperties) -> f64 {
        let translational = 0.5 * ball_props.mass * self.vel.magnitude_squared();
        // Moment of inertia for hollow sphere: I = (2/3) * m * r^2
        let inertia = (2.0 / 3.0) * ball_props.mass * ball_props.radius.powi(2);
        let rotational = 0.5 * inertia * self.spin.magnitude_squared();
        translational + rotational
    }
}

impl Default for BallState {
    fn default() -> Self {
        Self::at_rest(Vec3::ZERO)
    }
}

// =============================================================================
// Paddle State
// =============================================================================

/// State of a paddle at a given instant.
///
/// The paddle is modeled as a flat circular surface with:
/// - Position at the center of the hitting surface
/// - Normal vector pointing outward from the rubber
/// - Velocity of the paddle's movement (for spin transfer)
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PaddleState {
    pub pos: Vec3,
    pub normal: Vec3,
    pub vel: Vec3,
    pub rubber_id: String,
}

impl PaddleState {
    pub fn new(pos: Vec3, normal: Vec3, vel: Vec3, rubber_id: String) -> Self {
        Self {
            pos,
            normal: normal.normalized(),
            vel,
            rubber_id,
        }
    }
}

// =============================================================================
// Table State
// =============================================================================

/// The table tennis table.
///
/// ITTF regulation dimensions:
/// - Length: 2.74m (half_length = 1.37m)
/// - Width: 1.525m (half_width = 0.7625m)
/// - Height: 0.76m
/// - Net height: 0.1525m (15.25cm)
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TableState {
    pub pos: Vec3,
    pub half_length: f64,
    pub half_width: f64,
    pub height: f64,
    pub net_height: f64,
    pub surface_id: String,
}

impl TableState {
    /// Creates an ITTF regulation table centered at the origin
    pub fn ittf_regulation() -> Self {
        Self {
            pos: Vec3::ZERO,
            half_length: 1.37,
            half_width: 0.7625,
            height: 0.76,
            net_height: 0.1525,
            surface_id: "ittf_table".to_string(),
        }
    }

    /// Returns the Y coordinate of the table surface
    pub fn surface_y(&self) -> f64 {
        self.pos.y + self.height
    }

    /// Check if a point (x, z) is within the table bounds
    pub fn is_over_table(&self, x: f64, z: f64) -> bool {
        let dx = (x - self.pos.x).abs();
        let dz = (z - self.pos.z).abs();
        dx <= self.half_length && dz <= self.half_width
    }
}

impl Default for TableState {
    fn default() -> Self {
        Self::ittf_regulation()
    }
}

// =============================================================================
// Material Properties
// =============================================================================

/// Physical properties of a ball.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BallProperties {
    pub name: String,
    pub mass: f64,
    pub radius: f64,
    pub restitution_table: f64,
    pub drag_coefficient: f64,
    pub lift_coefficient: f64,
}

impl BallProperties {
    // TODO: remove when MaterialLoader is implemented - use YAML instead
    /// DHS D40+ 3-star ball (common competition ball)
    pub fn dhs_d40() -> Self {
        Self {
            name: "DHS D40+ 3-star".to_string(),
            mass: 0.00267,      // 2.67g in kg
            radius: 0.020,      // 40mm diameter = 20mm radius
            restitution_table: 0.77,
            drag_coefficient: 0.4,
            lift_coefficient: 0.35,
        }
    }
}

impl Default for BallProperties {
    fn default() -> Self {
        Self::dhs_d40()
    }
}

/// Physical properties of a rubber.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RubberProperties {
    pub name: String,
    pub manufacturer: String,
    pub ittf_approved: bool,

    // Contact properties
    pub restitution: f64,
    pub friction_static: f64,
    pub friction_dynamic: f64,

    // Hertzian contact model
    pub stiffness: f64,
    pub damping: f64,

    // Spin characteristics
    pub spin_transfer_rate: f64,
    pub spin_reversal_factor: f64,

    // Metadata (for reference, not used in physics)
    pub speed_rating: f64,
    pub spin_rating: f64,
    pub hardness_sponge: f64,
    pub thickness_mm: f64,
}

impl RubberProperties {
    // TODO: remove when MaterialLoader is implemented - use YAML instead
    /// Butterfly Tenergy 05 - popular tensor rubber
    pub fn tenergy_05() -> Self {
        Self {
            name: "Tenergy 05".to_string(),
            manufacturer: "Butterfly".to_string(),
            ittf_approved: true,
            restitution: 0.92,
            friction_static: 0.9,
            friction_dynamic: 0.75,
            stiffness: 45000.0,
            damping: 0.08,
            spin_transfer_rate: 0.85,
            spin_reversal_factor: 0.6,
            speed_rating: 13.0,
            spin_rating: 11.5,
            hardness_sponge: 36.0,
            thickness_mm: 2.1,
        }
    }
}

impl Default for RubberProperties {
    fn default() -> Self {
        Self::tenergy_05()
    }
}

/// Physical properties of a table surface.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SurfaceProperties {
    pub name: String,
    pub restitution: f64,
    pub friction: f64,
}

impl SurfaceProperties {
    // TODO: remove when MaterialLoader is implemented - use YAML instead
    /// ITTF regulation table surface
    pub fn ittf_standard() -> Self {
        Self {
            name: "ITTF Standard".to_string(),
            restitution: 0.77,
            friction: 0.5,
        }
    }
}

impl Default for SurfaceProperties {
    fn default() -> Self {
        Self::ittf_standard()
    }
}

// =============================================================================
// Simulation State
// =============================================================================

/// Complete state of the simulation at a given instant.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SimulationState {
    pub time: f64,
    pub ball: BallState,
    pub paddles: Vec<PaddleState>,
    pub table: TableState,
}

impl SimulationState {
    pub fn new(ball: BallState, paddles: Vec<PaddleState>, table: TableState) -> Self {
        Self {
            time: 0.0,
            ball,
            paddles,
            table,
        }
    }
}

// =============================================================================
// Collision Types
// =============================================================================

/// Result of a collision detection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CollisionTarget {
    Table,
    Paddle(usize),
    Net,
    None,
}

/// Detailed collision information.
#[derive(Debug, Clone, Copy)]
pub struct CollisionInfo {
    pub target: CollisionTarget,
    pub time: f64,
    pub point: Vec3,
    pub normal: Vec3,
    pub penetration: f64,
}

// =============================================================================
// Physical Constants
// =============================================================================

/// Physical constants used in the simulation.
pub mod constants {
    /// Gravitational acceleration (m/s²)
    pub const GRAVITY: f64 = 9.81;

    /// Air density at sea level, 20°C (kg/m³)
    pub const AIR_DENSITY: f64 = 1.204;

    /// Small value for floating-point comparisons
    pub const EPSILON: f64 = 1e-10;
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_vec3_operations() {
        let a = Vec3::new(1.0, 2.0, 3.0);
        let b = Vec3::new(4.0, 5.0, 6.0);

        assert_eq!(a + b, Vec3::new(5.0, 7.0, 9.0));
        assert_eq!(a - b, Vec3::new(-3.0, -3.0, -3.0));
        assert_eq!(a * 2.0, Vec3::new(2.0, 4.0, 6.0));
        assert_eq!(a.dot(&b), 32.0); // 1*4 + 2*5 + 3*6 = 32
    }

    #[test]
    fn test_vec3_cross_product() {
        let x = Vec3::new(1.0, 0.0, 0.0);
        let y = Vec3::new(0.0, 1.0, 0.0);
        let z = x.cross(&y);
        assert!((z.x).abs() < 1e-10);
        assert!((z.y).abs() < 1e-10);
        assert!((z.z - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_vec3_magnitude() {
        let v = Vec3::new(3.0, 4.0, 0.0);
        assert!((v.magnitude() - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_vec3_normalized() {
        let v = Vec3::new(3.0, 4.0, 0.0);
        let n = v.normalized();
        assert!((n.magnitude() - 1.0).abs() < 1e-10);
        assert!((n.x - 0.6).abs() < 1e-10);
        assert!((n.y - 0.8).abs() < 1e-10);
    }

    #[test]
    fn test_table_bounds() {
        let table = TableState::ittf_regulation();
        assert!(table.is_over_table(0.0, 0.0));
        assert!(table.is_over_table(1.0, 0.5));
        assert!(!table.is_over_table(2.0, 0.0)); // beyond table length
    }

    #[test]
    fn test_ball_kinetic_energy() {
        let ball_props = BallProperties::dhs_d40();
        let ball = BallState::new(
            Vec3::ZERO,
            Vec3::new(10.0, 0.0, 0.0), // 10 m/s
            Vec3::ZERO,
        );
        let energy = ball.kinetic_energy(&ball_props);
        // KE = 0.5 * 0.00267 * 100 = 0.1335 J (translational only)
        assert!((energy - 0.1335).abs() < 0.001);
    }
}
