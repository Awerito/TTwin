//! Python bindings for tt-core table tennis physics engine.
//!
//! Provides a simple Python API:
//!
//! ```python
//! from tt_physics import Simulation, Vec3
//!
//! sim = Simulation()
//! sim.set_ball_position(0.0, 0.9, 0.0)
//! sim.set_ball_velocity(5.0, 2.0, 0.0)
//!
//! for _ in range(100):
//!     sim.step(0.001)
//!     pos = sim.ball_position()
//!     print(f"Ball at ({pos.x}, {pos.y}, {pos.z})")
//! ```

use pyo3::prelude::*;

use tt_core::collision::{CollisionDetector, SimpleResolver};
use tt_core::forces::TableTennisForces;
use tt_core::integrator::VelocityVerlet;
use tt_core::types::{
    BallProperties, BallState, CollisionTarget, PaddleState, RubberProperties, SurfaceProperties,
    TableState, Vec3 as CoreVec3,
};

/// 3D vector for positions, velocities, etc.
#[pyclass]
#[derive(Clone, Copy)]
pub struct Vec3 {
    #[pyo3(get, set)]
    pub x: f64,
    #[pyo3(get, set)]
    pub y: f64,
    #[pyo3(get, set)]
    pub z: f64,
}

#[pymethods]
impl Vec3 {
    #[new]
    fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    fn __repr__(&self) -> String {
        format!("Vec3({:.4}, {:.4}, {:.4})", self.x, self.y, self.z)
    }

    fn magnitude(&self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    fn to_tuple(&self) -> (f64, f64, f64) {
        (self.x, self.y, self.z)
    }
}

impl From<CoreVec3> for Vec3 {
    fn from(v: CoreVec3) -> Self {
        Self {
            x: v.x,
            y: v.y,
            z: v.z,
        }
    }
}

impl From<Vec3> for CoreVec3 {
    fn from(v: Vec3) -> Self {
        CoreVec3::new(v.x, v.y, v.z)
    }
}

/// Main simulation class.
///
/// Handles physics stepping, collision detection, and state management.
#[pyclass]
pub struct Simulation {
    ball: BallState,
    ball_props: BallProperties,
    table: TableState,
    paddles: Vec<PaddleState>,
    rubber_props: Vec<RubberProperties>,
    #[allow(dead_code)] // Will be used when full material loading is implemented
    surface_props: SurfaceProperties,
    forces: TableTennisForces,
    detector: CollisionDetector,
    time: f64,
    last_collision: Option<String>,
}

#[pymethods]
impl Simulation {
    /// Create a new simulation with default settings.
    #[new]
    fn new() -> Self {
        Self {
            ball: BallState::at_rest(CoreVec3::new(0.0, 0.9, 0.0)),
            ball_props: BallProperties::default(),
            table: TableState::ittf_regulation(),
            paddles: vec![],
            rubber_props: vec![],
            surface_props: SurfaceProperties::default(),
            forces: TableTennisForces::default(),
            detector: CollisionDetector::default(),
            time: 0.0,
            last_collision: None,
        }
    }

    /// Current simulation time in seconds.
    #[getter]
    fn time(&self) -> f64 {
        self.time
    }

    /// Get ball position as Vec3.
    fn ball_position(&self) -> Vec3 {
        self.ball.pos.into()
    }

    /// Get ball velocity as Vec3.
    fn ball_velocity(&self) -> Vec3 {
        self.ball.vel.into()
    }

    /// Get ball spin as Vec3 (rad/s).
    fn ball_spin(&self) -> Vec3 {
        self.ball.spin.into()
    }

    /// Get ball speed in m/s.
    fn ball_speed(&self) -> f64 {
        self.ball.vel.magnitude()
    }

    /// Get ball spin magnitude in rad/s.
    fn ball_spin_magnitude(&self) -> f64 {
        self.ball.spin.magnitude()
    }

    /// Get ball spin in RPM (more intuitive).
    fn ball_spin_rpm(&self) -> f64 {
        self.ball.spin.magnitude() * 60.0 / (2.0 * std::f64::consts::PI)
    }

    /// Set ball position.
    fn set_ball_position(&mut self, x: f64, y: f64, z: f64) {
        self.ball.pos = CoreVec3::new(x, y, z);
    }

    /// Set ball velocity.
    fn set_ball_velocity(&mut self, vx: f64, vy: f64, vz: f64) {
        self.ball.vel = CoreVec3::new(vx, vy, vz);
    }

    /// Set ball spin in rad/s.
    fn set_ball_spin(&mut self, wx: f64, wy: f64, wz: f64) {
        self.ball.spin = CoreVec3::new(wx, wy, wz);
    }

    /// Set ball spin from RPM (more intuitive).
    fn set_ball_spin_rpm(&mut self, wx_rpm: f64, wy_rpm: f64, wz_rpm: f64) {
        let factor = 2.0 * std::f64::consts::PI / 60.0;
        self.ball.spin = CoreVec3::new(wx_rpm * factor, wy_rpm * factor, wz_rpm * factor);
    }

    /// Add a paddle to the simulation.
    ///
    /// Returns the paddle index.
    fn add_paddle(
        &mut self,
        x: f64,
        y: f64,
        z: f64,
        normal_x: f64,
        normal_y: f64,
        normal_z: f64,
    ) -> usize {
        let paddle = PaddleState::new(
            CoreVec3::new(x, y, z),
            CoreVec3::new(normal_x, normal_y, normal_z),
            CoreVec3::ZERO,
            "default".to_string(),
        );
        self.paddles.push(paddle);
        self.rubber_props.push(RubberProperties::default());
        self.paddles.len() - 1
    }

    /// Update paddle position and orientation.
    fn set_paddle(
        &mut self,
        index: usize,
        x: f64,
        y: f64,
        z: f64,
        normal_x: f64,
        normal_y: f64,
        normal_z: f64,
        vel_x: f64,
        vel_y: f64,
        vel_z: f64,
    ) {
        if index < self.paddles.len() {
            self.paddles[index].pos = CoreVec3::new(x, y, z);
            self.paddles[index].normal = CoreVec3::new(normal_x, normal_y, normal_z).normalized();
            self.paddles[index].vel = CoreVec3::new(vel_x, vel_y, vel_z);
        }
    }

    /// Get table surface Y coordinate.
    fn table_surface_y(&self) -> f64 {
        self.table.surface_y()
    }

    /// Get table dimensions as (half_length, half_width, height, net_height).
    fn table_dimensions(&self) -> (f64, f64, f64, f64) {
        (
            self.table.half_length,
            self.table.half_width,
            self.table.height,
            self.table.net_height,
        )
    }

    /// Check if ball is over the table.
    fn ball_over_table(&self) -> bool {
        self.table.is_over_table(self.ball.pos.x, self.ball.pos.z)
    }

    /// Get last collision type (if any): "table", "net", "paddle:N", or None.
    fn last_collision(&self) -> Option<String> {
        self.last_collision.clone()
    }

    /// Reset the simulation.
    fn reset(&mut self) {
        self.ball = BallState::at_rest(CoreVec3::new(0.0, 0.9, 0.0));
        self.time = 0.0;
        self.last_collision = None;
    }

    /// Advance simulation by dt seconds.
    ///
    /// Handles physics integration and collision detection/resolution.
    fn step(&mut self, dt: f64) {
        self.last_collision = None;

        // Check for collisions
        if let Some(collision) = self.detector.detect(
            &self.ball,
            &self.ball_props,
            &self.table,
            &self.paddles,
            dt,
        ) {
            // Advance to collision point
            self.ball = CollisionDetector::advance_to_collision(&self.ball, &collision);
            self.time += collision.time;

            // Resolve collision
            self.ball = SimpleResolver::resolve(&self.ball, &self.ball_props, &collision);

            // Record collision type
            self.last_collision = Some(match collision.target {
                CollisionTarget::Table => "table".to_string(),
                CollisionTarget::Net => "net".to_string(),
                CollisionTarget::Paddle(i) => format!("paddle:{}", i),
                CollisionTarget::None => "none".to_string(),
            });

            // Continue with remaining time
            let remaining = dt - collision.time;
            if remaining > 1e-6 {
                let result =
                    VelocityVerlet::step(&self.ball, &self.ball_props, &self.forces, remaining);
                self.ball = result.state;
                self.time += remaining;
            }
        } else {
            // No collision - normal physics step
            let result = VelocityVerlet::step(&self.ball, &self.ball_props, &self.forces, dt);
            self.ball = result.state;
            self.time += dt;
        }
    }

    /// Run multiple steps at once (more efficient).
    fn step_n(&mut self, dt: f64, steps: usize) {
        for _ in 0..steps {
            self.step(dt);
        }
    }

    /// Get current state as dict for easy inspection.
    fn state_dict(&self) -> PyResult<PyObject> {
        Python::with_gil(|py| {
            let dict = pyo3::types::PyDict::new(py);
            dict.set_item("time", self.time)?;
            dict.set_item("ball_x", self.ball.pos.x)?;
            dict.set_item("ball_y", self.ball.pos.y)?;
            dict.set_item("ball_z", self.ball.pos.z)?;
            dict.set_item("ball_vx", self.ball.vel.x)?;
            dict.set_item("ball_vy", self.ball.vel.y)?;
            dict.set_item("ball_vz", self.ball.vel.z)?;
            dict.set_item("ball_speed", self.ball.vel.magnitude())?;
            dict.set_item("ball_spin_rpm", self.ball_spin_rpm())?;
            Ok(dict.into())
        })
    }
}

/// Python module definition.
#[pymodule]
fn tt_physics(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<Vec3>()?;
    m.add_class::<Simulation>()?;
    Ok(())
}
