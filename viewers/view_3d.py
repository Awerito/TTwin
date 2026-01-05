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
    E(model="cylinder", color=color.gray, scale=(0.015, net_height + 0.02, 0.015),
      position=(0, surface_y + net_height / 2, -half_width - 0.05))
    E(model="cylinder", color=color.gray, scale=(0.015, net_height + 0.02, 0.015),
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
        position=(-0.82, -0.28),
        scale=0.9,
        color=color.white,
    )

    ui["speed_text"] = Text(
        parent=camera.ui,
        text="",
        position=(-0.82, -0.32),
        scale=0.8,
        color=color.white,
    )

    ui["spin_text"] = Text(
        parent=camera.ui,
        text="",
        position=(-0.82, -0.36),
        scale=0.8,
        color=color.white,
    )

    ui["launch_text"] = Text(
        parent=camera.ui,
        text="",
        position=(-0.82, -0.40),
        scale=0.7,
        color=color.light_gray,
    )

    ui["time_scale_text"] = Text(
        parent=camera.ui,
        text="",
        position=(-0.82, -0.44),
        scale=0.8,
        color=color.yellow,
    )

    ui["controls_text"] = Text(
        parent=camera.ui,
        text="SPACE:Launch R:Reset T:Spin 1-5:Speed",
        position=(-0.82, -0.48),
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


def main():
    app = Ursina(
        title="Table Tennis Physics - 3D View",
        borderless=False,
        fullscreen=False,
        size=(1400, 900),
    )

    # Game state
    state = GameState()
    state.launch_ball()

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

    def update():
        # Physics step
        state.step(time.dt)

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

    def input(key):
        if key == "space":
            state.launch_ball()
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
        elif key == "c":
            # Reset camera
            ec.rotation = Vec3(30, -45, 0)
            camera.z = -4
        elif key in TIME_SCALES:
            state.time_scale = TIME_SCALES[key]
        elif key == "escape":
            application.quit()

    app.update = update
    app.input = input

    app.run()


if __name__ == "__main__":
    main()
