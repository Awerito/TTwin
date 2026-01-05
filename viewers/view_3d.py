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
# Scripted Rally - Forehand topspin diagonal
# =============================================================================


class ScriptedRally:
    """Simple scripted rally: forehand topspin diagonal back and forth."""

    def __init__(self, sim: Simulation, paddle_left: Entity, paddle_right: Entity):
        self.sim = sim
        self.paddle_left = paddle_left
        self.paddle_right = paddle_right

        half_length, half_width, height, net_height = sim.table_dimensions()
        self.half_length = half_length
        self.half_width = half_width
        self.surface_y = sim.table_surface_y()

        # Hit zones (x position where we detect and apply hit)
        self.hit_x_left = -half_length - 0.2
        self.hit_x_right = half_length + 0.2

        # Rally parameters
        self.hit_speed = 8.0  # m/s
        self.hit_angle_v = 12.0  # degrees up
        self.topspin_rpm = 3000.0

        # Diagonal positions (forehand corners)
        self.z_left = -half_width * 0.6  # Left player forehand side
        self.z_right = half_width * 0.6  # Right player forehand side

        # Track state
        self.last_hit_side = None
        self.hit_count = 0

    def start_rally(self):
        """Start a rally from the left side."""
        self.sim.reset()
        self.hit_count = 0
        self.last_hit_side = None

        # Position paddles at diagonal corners
        self.paddle_left.z = self.z_left
        self.paddle_right.z = self.z_right

        # Ball starts at left paddle, launch towards right diagonal
        # Use explicit values: table surface ~0.76m, start 0.2m above
        start_x = -self.half_length - 0.1  # Just past left edge
        start_y = 0.96  # ~0.76 + 0.2 above table
        start_z = self.z_left

        self.sim.set_ball_position(start_x, start_y, start_z)
        self._hit_towards("right")
        self.last_hit_side = "left"
        self.hit_count = 1

    def _hit_towards(self, target_side: str):
        """Apply a forehand topspin hit towards target side."""
        # Target position
        target_z = self.z_right if target_side == "right" else self.z_left
        target_x = self.hit_x_right if target_side == "right" else self.hit_x_left

        pos = self.sim.ball_position()

        # Calculate direction
        dx = target_x - pos.x
        dz = target_z - pos.z
        dist_xz = math.sqrt(dx * dx + dz * dz)

        # Velocity components
        angle_v_rad = math.radians(self.hit_angle_v)
        v_horizontal = self.hit_speed * math.cos(angle_v_rad)
        vy = self.hit_speed * math.sin(angle_v_rad)

        # Direction in XZ plane
        vx = v_horizontal * (dx / dist_xz)
        vz = v_horizontal * (dz / dist_xz)

        self.sim.set_ball_velocity(vx, vy, vz)

        # Topspin (positive Z spin for ball moving in +X)
        spin_direction = 1 if dx > 0 else -1
        self.sim.set_ball_spin_rpm(0, 0, self.topspin_rpm * spin_direction)

    def update(self):
        """Check for hits and apply them."""
        pos = self.sim.ball_position()
        vel = self.sim.ball_velocity()

        # Check if ball reached right side (moving right)
        if vel.x > 0 and pos.x > self.hit_x_right and self.last_hit_side != "right":
            # Right paddle hits back to left
            self.paddle_right.z = pos.z  # Move paddle to ball
            self._hit_towards("left")
            self.last_hit_side = "right"
            self.hit_count += 1

        # Check if ball reached left side (moving left)
        elif vel.x < 0 and pos.x < self.hit_x_left and self.last_hit_side != "left":
            # Left paddle hits back to right
            self.paddle_left.z = pos.z  # Move paddle to ball
            self._hit_towards("right")
            self.last_hit_side = "left"
            self.hit_count += 1


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

    # Scripted rally controller
    rally = ScriptedRally(state.sim, paddle_left, paddle_right)
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

    # Update rally (check for hits)
    if not state.paused:
        rally.update()

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
