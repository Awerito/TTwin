#!/usr/bin/env python3
"""
3D visualization of table tennis physics using Ursina.

Controls:
    Mouse:
        Right-click + drag: Rotate camera
        Scroll: Zoom in/out
        Middle-click + drag: Pan

    Keyboard:
        SPACE - Launch ball
        R - Reset simulation
        T - Cycle spin type
        S - Flip sidespin direction

        UP/DOWN - Adjust vertical launch angle
        LEFT/RIGHT - Adjust launch speed
        A/D - Adjust horizontal angle

        1-5 - Time scale (1=0.1x, 2=0.25x, 3=0.5x, 4=1x, 5=2x)
        P - Pause/unpause

        C - Reset camera to default position
        ESC - Quit
"""

import math
import sys

from ursina import *

from tt_physics import Simulation


# Helper: Entity with unlit=True by default (lighting broken on AMD)
def E(**kwargs):
    kwargs.setdefault("unlit", True)
    return Entity(**kwargs)


# =============================================================================
# Constants
# =============================================================================

# Time scales available
TIME_SCALES = {
    "1": 0.1,
    "2": 0.25,
    "3": 0.5,
    "4": 1.0,
    "5": 2.0,
}

# Physics
PHYSICS_DT = 0.0001
BASE_SUBSTEPS = 166  # ~60fps worth of substeps

# Spin types
SPIN_TOPSPIN = 0
SPIN_SIDESPIN_RIGHT = 1
SPIN_BACKSPIN = 2
SPIN_SIDESPIN_LEFT = 3
SPIN_NONE = 4
SPIN_NAMES = ["Topspin", "Sidespin→", "Backspin", "←Sidespin", "No spin"]
SPIN_COLORS = [
    color.red,
    color.magenta,
    color.blue,
    color.cyan,
    color.gray,
]


# =============================================================================
# Game state
# =============================================================================


class GameState:
    """Holds all simulation state."""

    def __init__(self):
        self.sim = Simulation()
        self.paused = False
        self.time_scale = 1.0

        # Launch parameters
        self.launch_speed = 8.0
        self.launch_angle_v = 10.0
        self.launch_angle_h = 0.0
        self.spin_type = SPIN_TOPSPIN
        self.spin_rpm = 3000.0

        # Trajectory history
        self.trajectory: list[Vec3] = []
        self.max_trajectory_points = 300

    def launch_ball(self):
        """Launch ball with current parameters."""
        self.trajectory.clear()

        start_x = -1.2
        start_y = self.sim.table_surface_y() + 0.15
        start_z = 0.0

        angle_v_rad = math.radians(self.launch_angle_v)
        angle_h_rad = math.radians(self.launch_angle_h)

        v_horizontal = self.launch_speed * math.cos(angle_v_rad)
        vx = v_horizontal * math.cos(angle_h_rad)
        vz = v_horizontal * math.sin(angle_h_rad)
        vy = self.launch_speed * math.sin(angle_v_rad)

        spin_x, spin_y, spin_z = 0.0, 0.0, 0.0
        if self.spin_type == SPIN_TOPSPIN:
            spin_z = self.spin_rpm
        elif self.spin_type == SPIN_BACKSPIN:
            spin_z = -self.spin_rpm
        elif self.spin_type == SPIN_SIDESPIN_RIGHT:
            spin_y = self.spin_rpm
        elif self.spin_type == SPIN_SIDESPIN_LEFT:
            spin_y = -self.spin_rpm

        self.sim.reset()
        self.sim.set_ball_position(start_x, start_y, start_z)
        self.sim.set_ball_velocity(vx, vy, vz)
        self.sim.set_ball_spin_rpm(spin_x, spin_y, spin_z)

    def step(self, dt: float):
        """Advance physics."""
        if self.paused:
            return

        effective_dt = PHYSICS_DT * self.time_scale
        substeps = int(BASE_SUBSTEPS * self.time_scale)
        substeps = max(1, substeps)

        for _ in range(substeps):
            self.sim.step(PHYSICS_DT)

        # Record trajectory
        pos = self.sim.ball_position()
        self.trajectory.append(Vec3(pos.x, pos.y, pos.z))
        if len(self.trajectory) > self.max_trajectory_points:
            self.trajectory.pop(0)


# =============================================================================
# Paddle Physics - Real collision detection and response
# =============================================================================


class PaddlePhysics:
    """
    Real paddle collision physics.

    Detects ball-paddle collision and applies realistic bounce based on:
    - Paddle position and angle
    - Paddle swing velocity
    - Ball incoming velocity and spin
    """

    # Paddle dimensions (meters)
    PADDLE_WIDTH = 0.15  # 15cm wide
    PADDLE_HEIGHT = 0.17  # 17cm tall
    PADDLE_THICKNESS = 0.01  # 1cm thick

    # Physics coefficients
    RESTITUTION = 0.85  # Bounce coefficient
    FRICTION = 0.6  # Surface friction for spin

    def __init__(self, paddle: Entity, side: str, sim: Simulation):
        self.paddle = paddle
        self.side = side  # "left" or "right"
        self.sim = sim

        half_length, half_width, height, net_height = sim.table_dimensions()
        self.half_length = half_length
        self.half_width = half_width
        self.surface_y = sim.table_surface_y()

        # Paddle swing state
        self.is_swinging = False
        self.swing_velocity = Vec3(0, 0, 0)  # Paddle velocity during swing
        self.target_z = 0.0
        self.target_y = self.surface_y + 0.15

        # Movement parameters
        self.move_speed = 4.0  # m/s lateral movement
        self.swing_speed = 8.0  # m/s forward swing

    def predict_intercept_z(self) -> float:
        """Predict where ball will arrive at paddle's x plane."""
        pos = self.sim.ball_position()
        vel = self.sim.ball_velocity()

        # Our x position
        paddle_x = self.paddle.x

        # Check if ball is coming towards us
        if self.side == "left" and vel.x >= 0:
            return self.paddle.z
        if self.side == "right" and vel.x <= 0:
            return self.paddle.z

        # Time to reach our x
        dx = paddle_x - pos.x
        if abs(vel.x) < 0.1:
            return pos.z

        t = dx / vel.x
        if t < 0 or t > 2.0:  # More than 2 seconds away, don't bother
            return self.paddle.z

        # Predicted z position (simple linear, ignoring gravity effect on z)
        predicted_z = pos.z + vel.z * t

        # Clamp to reachable range
        return max(-self.half_width, min(self.half_width, predicted_z))

    def update(self, dt: float):
        """Update paddle position and check for collision."""
        pos = self.sim.ball_position()
        vel = self.sim.ball_velocity()

        # Predict where to move
        self.target_z = self.predict_intercept_z()

        # Move paddle towards target (smooth movement)
        current_z = self.paddle.z
        diff_z = self.target_z - current_z
        max_move = self.move_speed * dt
        if abs(diff_z) > max_move:
            move_z = max_move if diff_z > 0 else -max_move
        else:
            move_z = diff_z
        self.paddle.z = current_z + move_z

        # Check for collision
        if self._check_collision():
            self._apply_paddle_hit()
            return True

        return False

    def _check_collision(self) -> bool:
        """Check if ball is colliding with paddle."""
        ball_pos = self.sim.ball_position()
        ball_vel = self.sim.ball_velocity()
        paddle_pos = self.paddle.position

        # Ball must be moving towards paddle
        if self.side == "left" and ball_vel.x >= 0:
            return False
        if self.side == "right" and ball_vel.x <= 0:
            return False

        # Check x distance (paddle faces ball)
        dx = abs(ball_pos.x - paddle_pos.x)
        if dx > 0.08:  # Ball not close enough in x
            return False

        # Check y bounds (paddle height)
        dy = abs(ball_pos.y - paddle_pos.y)
        if dy > self.PADDLE_HEIGHT / 2 + 0.02:  # Ball radius
            return False

        # Check z bounds (paddle width)
        dz = abs(ball_pos.z - paddle_pos.z)
        if dz > self.PADDLE_WIDTH / 2 + 0.02:
            return False

        return True

    def _apply_paddle_hit(self):
        """Apply realistic paddle hit physics."""
        ball_vel = self.sim.ball_velocity()
        ball_speed = self.sim.ball_speed()

        # Paddle normal direction (facing opponent)
        if self.side == "left":
            normal_x = 1.0  # Facing right
            swing_dir = 1.0
        else:
            normal_x = -1.0  # Facing left
            swing_dir = -1.0

        # Paddle angle affects the hit
        # For topspin: paddle tilted forward, brushing up on ball
        paddle_angle = math.radians(15)  # 15 degrees closed (forward tilt)

        # Swing velocity (paddle moving forward and slightly up for topspin)
        swing_speed = self.swing_speed
        swing_vx = swing_dir * swing_speed * math.cos(paddle_angle)
        swing_vy = swing_speed * 0.3  # Upward brush for topspin

        # Reflect ball velocity off paddle
        # New velocity = reflection + paddle contribution
        incoming_vx = ball_vel.x
        incoming_vy = ball_vel.y
        incoming_vz = ball_vel.z

        # Simple reflection with paddle angle
        # The paddle imparts its velocity to the ball
        new_vx = swing_vx * 0.8 + (-incoming_vx * self.RESTITUTION * 0.2)
        new_vy = incoming_vy * self.RESTITUTION * 0.5 + swing_vy
        new_vz = incoming_vz * 0.7  # Reduce lateral speed slightly

        # Add slight angle towards diagonal (topspin forehand goes cross-court)
        cross_court_z = 0.15 * swing_dir  # Slight cross-court tendency
        new_vz += cross_court_z * swing_speed

        # Ensure minimum forward speed
        min_forward_speed = 4.0
        if abs(new_vx) < min_forward_speed:
            new_vx = swing_dir * min_forward_speed

        self.sim.set_ball_velocity(new_vx, new_vy, new_vz)

        # Apply topspin from brushing action
        # Positive Z spin for ball going +X (right), negative for -X
        topspin_rpm = 2500 * swing_dir
        self.sim.set_ball_spin_rpm(0, 0, topspin_rpm)


class RallyController:
    """Controls rally with real paddle physics."""

    def __init__(self, sim: Simulation, paddle_left: Entity, paddle_right: Entity):
        self.sim = sim
        self.paddle_left_entity = paddle_left
        self.paddle_right_entity = paddle_right

        half_length, half_width, height, net_height = sim.table_dimensions()
        self.half_length = half_length
        self.surface_y = sim.table_surface_y()

        # Create paddle physics controllers
        self.paddle_left = PaddlePhysics(paddle_left, "left", sim)
        self.paddle_right = PaddlePhysics(paddle_right, "right", sim)

        self.hit_count = 0
        self.last_hitter = None

    def start_rally(self):
        """Start rally with serve from left side."""
        self.sim.reset()
        self.hit_count = 0
        self.last_hitter = None

        # Ball starts at left side, above table
        start_x = -self.half_length - 0.15
        start_y = self.surface_y + 0.25
        start_z = -0.3  # Slightly to forehand side

        self.sim.set_ball_position(start_x, start_y, start_z)

        # Initial serve: forward with topspin
        serve_speed = 6.0
        serve_angle = math.radians(10)  # Slight upward angle
        vx = serve_speed * math.cos(serve_angle)
        vy = serve_speed * math.sin(serve_angle)
        vz = 0.3  # Slight cross-court

        self.sim.set_ball_velocity(vx, vy, vz)
        self.sim.set_ball_spin_rpm(0, 0, 2000)  # Topspin

        self.hit_count = 1
        self.last_hitter = "left"

    def update(self, dt: float):
        """Update paddle AI and check for hits."""
        # Update both paddles
        if self.paddle_left.update(dt):
            if self.last_hitter != "left":
                self.hit_count += 1
                self.last_hitter = "left"

        if self.paddle_right.update(dt):
            if self.last_hitter != "right":
                self.hit_count += 1
                self.last_hitter = "right"


# =============================================================================
# 3D Entities
# =============================================================================


def create_table(sim: Simulation) -> Entity:
    """Create the table tennis table."""
    half_length, half_width, height, net_height = sim.table_dimensions()

    # Table top - blue
    table = E(
        model="cube",
        color=color.blue,
        scale=(half_length * 2, 0.03, half_width * 2),
        position=(0, height, 0),
    )

    # White lines
    E(model="cube", color=color.white, scale=(0.02, 0.031, half_width * 2),
      position=(-half_length, height, 0))
    E(model="cube", color=color.white, scale=(0.02, 0.031, half_width * 2),
      position=(half_length, height, 0))
    E(model="cube", color=color.white, scale=(half_length * 2, 0.031, 0.02),
      position=(0, height, -half_width))
    E(model="cube", color=color.white, scale=(half_length * 2, 0.031, 0.02),
      position=(0, height, half_width))
    E(model="cube", color=color.white, scale=(half_length * 2, 0.031, 0.01),
      position=(0, height, 0))

    # Table legs
    for dx, dz in [(-1, -1), (-1, 1), (1, -1), (1, 1)]:
        E(model="cube", color=color.dark_gray, scale=(0.05, height, 0.05),
          position=((half_length - 0.1) * dx, height / 2, (half_width - 0.1) * dz))

    return table


def create_net(sim: Simulation) -> Entity:
    """Create the net."""
    half_length, half_width, height, net_height = sim.table_dimensions()
    surface_y = sim.table_surface_y()

    net = E(model="cube", color=color.light_gray,
            scale=(0.01, net_height, half_width * 2 + 0.1),
            position=(0, surface_y + net_height / 2, 0))
    # Net posts (use cube instead of cylinder - cylinder not available)
    E(model="cube", color=color.gray, scale=(0.02, net_height + 0.02, 0.02),
      position=(0, surface_y + net_height / 2, -half_width - 0.05))
    E(model="cube", color=color.gray, scale=(0.02, net_height + 0.02, 0.02),
      position=(0, surface_y + net_height / 2, half_width + 0.05))
    return net


def create_ball() -> Entity:
    """Create the ball - white."""
    return E(model="sphere", color=color.white, scale=0.04)


def create_paddle() -> Entity:
    """Create a paddle entity."""
    paddle = E(model="cube", color=color.brown, scale=(0.15, 0.17, 0.008))
    E(model="cube", color=color.red, scale=(0.14, 0.16, 0.002),
      position=(0, 0, 0.005), parent=paddle)
    E(model="cube", color=color.black, scale=(0.14, 0.16, 0.002),
      position=(0, 0, -0.005), parent=paddle)
    E(model="cube", color=color.orange, scale=(0.03, 0.1, 0.02),
      position=(0, -0.12, 0), parent=paddle)
    return paddle


def create_floor() -> Entity:
    """Create floor plane."""
    return E(model="plane", color=color.gray, scale=(10, 1, 10),
             position=(0, -0.01, 0))


# =============================================================================
# Trajectory visualization
# =============================================================================


class TrajectoryRenderer(Entity):
    """Renders the ball trajectory as a line."""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.lines: list[Entity] = []
        self.max_lines = 300

    def update_trajectory(self, points: list[Vec3]):
        """Update the trajectory visualization."""
        # Remove old lines
        for line in self.lines:
            destroy(line)
        self.lines.clear()

        if len(points) < 2:
            return

        # Create new line segments
        for i in range(len(points) - 1):
            alpha = i / len(points)
            segment = E(
                model=Mesh(vertices=[points[i], points[i + 1]], mode="line"),
                color=color.cyan,
            )
            self.lines.append(segment)


# =============================================================================
# UI
# =============================================================================


def create_ui(state: GameState) -> dict:
    """Create UI elements."""
    ui = {}

    # Text elements - repositioned to bottom-left
    ui["time_text"] = Text(
        parent=camera.ui,
        text="",
        position=(-0.65, -0.28),
        scale=0.9,
        color=color.white,
    )

    ui["speed_text"] = Text(
        parent=camera.ui,
        text="",
        position=(-0.65, -0.32),
        scale=0.8,
        color=color.white,
    )

    ui["spin_text"] = Text(
        parent=camera.ui,
        text="",
        position=(-0.65, -0.36),
        scale=0.8,
        color=color.white,
    )

    ui["launch_text"] = Text(
        parent=camera.ui,
        text="",
        position=(-0.65, -0.40),
        scale=0.7,
        color=color.light_gray,
    )

    ui["time_scale_text"] = Text(
        parent=camera.ui,
        text="",
        position=(-0.65, -0.44),
        scale=0.8,
        color=color.yellow,
    )

    ui["controls_text"] = Text(
        parent=camera.ui,
        text="SPACE:Rally  R:Reset  1-5:Speed",
        position=(-0.65, -0.48),
        scale=0.6,
        color=color.gray,
    )

    ui["collision_text"] = Text(
        parent=camera.ui,
        text="",
        position=(0.5, 0.45),
        scale=1.2,
        color=color.yellow,
    )

    ui["hit_count_text"] = Text(
        parent=camera.ui,
        text="",
        position=(0, 0.45),
        scale=1.5,
        color=color.green,
    )

    return ui


def update_ui(ui: dict, state: GameState):
    """Update UI text."""
    sim = state.sim
    pos = sim.ball_position()

    ui["time_text"].text = f"Time: {sim.time:.3f}s"
    ui["speed_text"].text = (
        f"Speed: {sim.ball_speed():.1f} m/s | Spin: {sim.ball_spin_rpm():.0f} RPM"
    )

    spin_name = SPIN_NAMES[state.spin_type]
    ui["spin_text"].text = f"Type: {spin_name}"

    ui["launch_text"].text = (
        f"Launch: {state.launch_speed:.0f}m/s "
        f"V:{state.launch_angle_v:.0f}° H:{state.launch_angle_h:.0f}°"
    )

    paused_str = " [PAUSED]" if state.paused else ""
    ui["time_scale_text"].text = f"Time: {state.time_scale:.2f}x{paused_str}"

    collision = sim.last_collision()
    if collision:
        ui["collision_text"].text = f"HIT: {collision.upper()}"
        ui["collision_text"].color = (
            color.yellow if collision == "table" else color.red
        )
    else:
        ui["collision_text"].text = ""


# =============================================================================
# Main
# =============================================================================


# =============================================================================
# Global state (needed for Ursina's update pattern)
# =============================================================================

app = None
state = None
rally = None
ball = None
trajectory_renderer = None
ui = None
spin_indicator = None


def main():
    global app, state, rally, ball, trajectory_renderer, ui, spin_indicator

    app = Ursina(
        title="Table Tennis Physics - 3D View",
        borderless=False,
        fullscreen=False,
        size=(1400, 900),
    )

    # Game state
    state = GameState()

    # Create scene
    floor = create_floor()
    table = create_table(state.sim)
    net = create_net(state.sim)
    ball = create_ball()

    # Paddles (positioned at ends of table)
    half_length, half_width, height, net_height = state.sim.table_dimensions()
    surface_y = state.sim.table_surface_y()

    paddle_left = create_paddle()
    paddle_left.position = (-half_length - 0.3, surface_y + 0.15, 0)
    paddle_left.rotation = (0, 90, 15)  # Angled slightly

    paddle_right = create_paddle()
    paddle_right.position = (half_length + 0.3, surface_y + 0.15, 0)
    paddle_right.rotation = (0, -90, -15)

    # Rally controller with real paddle physics
    rally = RallyController(state.sim, paddle_left, paddle_right)
    rally.start_rally()  # Start automatically

    # Trajectory renderer
    trajectory_renderer = TrajectoryRenderer()

    # EditorCamera - orbit camera with mouse controls
    # Right-click + drag: rotate, Scroll: zoom, Middle-click: pan
    ec = EditorCamera()
    ec.position = Vec3(0, surface_y + 0.3, 0)  # Pivot point
    ec.rotation = Vec3(30, -45, 0)  # Look down at 30°, rotated 45°
    camera.z = -4  # Pull camera back (zoom out)

    # UI
    ui = create_ui(state)

    # Background
    window.color = color.black

    # Spin indicator on ball
    spin_indicator = E(model="cube", color=color.red, scale=(0.002, 0.05, 0.002), parent=ball)

    app.run()


def update():
    """Called every frame by Ursina."""
    if state is None:
        return

    # Physics step
    state.step(time.dt)

    # Update rally (paddle AI + collision detection)
    if not state.paused:
        rally.update(time.dt * state.time_scale)

    # Update ball position
    pos = state.sim.ball_position()
    ball.position = (pos.x, pos.y, pos.z)

    # Update spin indicator rotation
    spin = state.sim.ball_spin()
    if state.sim.ball_spin_rpm() > 100:
        spin_indicator.visible = True
        spin_indicator.color = SPIN_COLORS[state.spin_type]
        # Rotate based on spin axis
        spin_indicator.rotation = (
            math.degrees(math.atan2(spin.z, spin.y)),
            math.degrees(math.atan2(spin.x, spin.z)),
            0,
        )
    else:
        spin_indicator.visible = False

    # Update trajectory
    trajectory_renderer.update_trajectory(state.trajectory)

    # Update UI
    update_ui(ui, state)
    ui["hit_count_text"].text = f"Hits: {rally.hit_count}"


def input(key):
    """Handle keyboard input."""
    if state is None:
        return

    if key == "space" or key == "e":
        rally.start_rally()
        state.trajectory.clear()
    elif key == "r":
        state.sim.reset()
        state.trajectory.clear()
    elif key == "t":
        state.spin_type = (state.spin_type + 1) % 5
    elif key == "s":
        if state.spin_type == SPIN_SIDESPIN_RIGHT:
            state.spin_type = SPIN_SIDESPIN_LEFT
        elif state.spin_type == SPIN_SIDESPIN_LEFT:
            state.spin_type = SPIN_SIDESPIN_RIGHT
    elif key == "p":
        state.paused = not state.paused
    elif key == "up arrow":
        state.launch_angle_v = min(state.launch_angle_v + 5, 45)
    elif key == "down arrow":
        state.launch_angle_v = max(state.launch_angle_v - 5, -15)
    elif key == "right arrow":
        state.launch_speed = min(state.launch_speed + 1, 25)
    elif key == "left arrow":
        state.launch_speed = max(state.launch_speed - 1, 2)
    elif key == "d":
        state.launch_angle_h = min(state.launch_angle_h + 5, 45)
    elif key == "a":
        state.launch_angle_h = max(state.launch_angle_h - 5, -45)
    elif key in TIME_SCALES:
        state.time_scale = TIME_SCALES[key]
    elif key == "escape":
        application.quit()


if __name__ == "__main__":
    main()
