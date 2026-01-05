#!/usr/bin/env python3
"""
Dual view (Side + Top) visualization of table tennis physics.

Shows both X-Y plane (side view) and X-Z plane (top view) to visualize
all 3 dimensions including sidespin curves.

Controls:
    SPACE - Launch ball with current parameters
    R     - Reset simulation
    T     - Cycle spin type: topspin → sidespin → backspin → no spin
    S     - Toggle sidespin direction (left/right)
    UP/DOWN - Adjust launch angle (vertical)
    LEFT/RIGHT - Adjust launch speed
    A/D   - Adjust horizontal angle (for cross-table shots)
    ESC/Q - Quit
    P     - Pause/unpause
"""

import math
import sys

import pygame

from tt_physics import Simulation, Vec3


# =============================================================================
# Constants
# =============================================================================

# Window layout
WINDOW_WIDTH = 1400
WINDOW_HEIGHT = 800
SIDE_VIEW_WIDTH = 900
TOP_VIEW_WIDTH = WINDOW_WIDTH - SIDE_VIEW_WIDTH
VIEW_HEIGHT = 650
INFO_HEIGHT = WINDOW_HEIGHT - VIEW_HEIGHT
FPS = 60

# Physics
PHYSICS_DT = 0.0001  # 0.1ms timestep for accuracy
PHYSICS_SUBSTEPS = int((1 / FPS) / PHYSICS_DT)  # Steps per frame

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GRAY = (128, 128, 128)
DARK_GRAY = (64, 64, 64)
GREEN = (34, 139, 34)
DARK_GREEN = (0, 100, 0)
ORANGE = (255, 165, 0)
RED = (255, 80, 80)
BLUE = (100, 149, 237)
YELLOW = (255, 255, 0)
CYAN = (0, 255, 255)
MAGENTA = (255, 0, 255)

# Scale: pixels per meter
SIDE_SCALE = 200  # Side view scale
TOP_SCALE = 180  # Top view scale (slightly smaller to fit width)

# Spin types
SPIN_TOPSPIN = 0
SPIN_SIDESPIN_RIGHT = 1
SPIN_BACKSPIN = 2
SPIN_SIDESPIN_LEFT = 3
SPIN_NONE = 4
SPIN_NAMES = ["Topspin", "Sidespin (R)", "Backspin", "Sidespin (L)", "No spin"]


# =============================================================================
# View classes for coordinate management
# =============================================================================


class SideView:
    """Side view showing X-Y plane (looking from +Z toward -Z)."""

    def __init__(self, x: int, y: int, width: int, height: int, scale: float):
        self.rect = pygame.Rect(x, y, width, height)
        self.scale = scale
        # Origin at center-bottom of view
        self.origin_x = x + width // 2
        self.origin_y = y + height - 80

    def world_to_screen(self, x: float, y: float) -> tuple[int, int]:
        """Convert world X-Y to screen coordinates."""
        screen_x = int(self.origin_x + x * self.scale)
        screen_y = int(self.origin_y - y * self.scale)
        return screen_x, screen_y

    def meters_to_pixels(self, m: float) -> int:
        """Convert meters to pixels."""
        return int(m * self.scale)


class TopView:
    """Top view showing X-Z plane (looking from +Y down toward -Y)."""

    def __init__(self, x: int, y: int, width: int, height: int, scale: float):
        self.rect = pygame.Rect(x, y, width, height)
        self.scale = scale
        # Origin at center of view
        self.origin_x = x + width // 2
        self.origin_z = y + height // 2

    def world_to_screen(self, x: float, z: float) -> tuple[int, int]:
        """Convert world X-Z to screen coordinates."""
        screen_x = int(self.origin_x + x * self.scale)
        # Z positive is to the right in world, but we want it going down on screen
        # for intuitive "looking down at table" view
        screen_z = int(self.origin_z - z * self.scale)
        return screen_x, screen_z

    def meters_to_pixels(self, m: float) -> int:
        """Convert meters to pixels."""
        return int(m * self.scale)


# =============================================================================
# Drawing functions - Side View
# =============================================================================


def draw_side_view_table(screen: pygame.Surface, view: SideView, sim: Simulation):
    """Draw table from side (X-Y plane)."""
    half_length, half_width, height, net_height = sim.table_dimensions()
    surface_y = sim.table_surface_y()

    # Table surface
    left = view.world_to_screen(-half_length, surface_y)
    right = view.world_to_screen(half_length, surface_y)
    pygame.draw.line(screen, GREEN, left, right, 4)

    # Table legs
    leg_left = view.world_to_screen(-half_length + 0.1, 0)
    leg_right = view.world_to_screen(half_length - 0.1, 0)
    pygame.draw.line(screen, DARK_GRAY, left, leg_left, 3)
    pygame.draw.line(screen, DARK_GRAY, right, leg_right, 3)

    # Net
    net_bottom = view.world_to_screen(0, surface_y)
    net_top = view.world_to_screen(0, surface_y + net_height)
    pygame.draw.line(screen, WHITE, net_bottom, net_top, 2)
    pygame.draw.circle(screen, WHITE, net_top, 4)

    # Ground reference
    ground_left = view.world_to_screen(-2.5, 0)
    ground_right = view.world_to_screen(2.5, 0)
    pygame.draw.line(screen, DARK_GRAY, ground_left, ground_right, 1)


def draw_side_view_ball(screen: pygame.Surface, view: SideView, sim: Simulation):
    """Draw ball in side view."""
    pos = sim.ball_position()
    screen_pos = view.world_to_screen(pos.x, pos.y)
    radius = max(view.meters_to_pixels(0.02), 5)

    pygame.draw.circle(screen, ORANGE, screen_pos, radius)
    pygame.draw.circle(screen, WHITE, screen_pos, radius, 1)

    # Spin indicator for topspin/backspin (Z component)
    spin = sim.ball_spin()
    if abs(spin.z) > 10:  # More than ~100 RPM
        # Arrow indicating rotation direction
        indicator_len = radius + 4
        # Topspin (positive Z) rotates clockwise when viewed from right
        direction = 1 if spin.z > 0 else -1
        # Draw curved arrow
        for i in range(3):
            angle = direction * (i * 0.3 - 0.3)
            x1 = screen_pos[0] + int(indicator_len * math.cos(angle))
            y1 = screen_pos[1] - int(indicator_len * math.sin(angle))
            angle2 = direction * ((i + 1) * 0.3 - 0.3)
            x2 = screen_pos[0] + int(indicator_len * math.cos(angle2))
            y2 = screen_pos[1] - int(indicator_len * math.sin(angle2))
            pygame.draw.line(screen, RED, (x1, y1), (x2, y2), 2)


def draw_side_view_velocity(screen: pygame.Surface, view: SideView, sim: Simulation):
    """Draw velocity vector in side view."""
    pos = sim.ball_position()
    vel = sim.ball_velocity()
    screen_pos = view.world_to_screen(pos.x, pos.y)

    vel_scale = 8
    end_x = screen_pos[0] + int(vel.x * vel_scale)
    end_y = screen_pos[1] - int(vel.y * vel_scale)

    pygame.draw.line(screen, BLUE, screen_pos, (end_x, end_y), 2)

    # Arrow head
    if vel.x != 0 or vel.y != 0:
        angle = math.atan2(-vel.y, vel.x)
        arrow_len = 6
        arrow_angle = 0.5
        ax1 = end_x - int(arrow_len * math.cos(angle - arrow_angle))
        ay1 = end_y + int(arrow_len * math.sin(angle - arrow_angle))
        ax2 = end_x - int(arrow_len * math.cos(angle + arrow_angle))
        ay2 = end_y + int(arrow_len * math.sin(angle + arrow_angle))
        pygame.draw.line(screen, BLUE, (end_x, end_y), (ax1, ay1), 2)
        pygame.draw.line(screen, BLUE, (end_x, end_y), (ax2, ay2), 2)


def draw_side_view_trajectory(
    screen: pygame.Surface, view: SideView, trajectory: list[tuple[float, float, float]]
):
    """Draw trajectory in side view (X-Y projection)."""
    if len(trajectory) < 2:
        return

    points = [view.world_to_screen(x, y) for x, y, z in trajectory]
    for i in range(len(points) - 1):
        alpha = int(255 * (i / len(points)))
        color = (alpha // 2, alpha // 2, alpha)
        pygame.draw.line(screen, color, points[i], points[i + 1], 1)


# =============================================================================
# Drawing functions - Top View
# =============================================================================


def draw_top_view_table(screen: pygame.Surface, view: TopView, sim: Simulation):
    """Draw table from top (X-Z plane)."""
    half_length, half_width, height, net_height = sim.table_dimensions()

    # Table outline
    corners = [
        view.world_to_screen(-half_length, -half_width),
        view.world_to_screen(half_length, -half_width),
        view.world_to_screen(half_length, half_width),
        view.world_to_screen(-half_length, half_width),
    ]
    pygame.draw.polygon(screen, DARK_GREEN, corners)
    pygame.draw.polygon(screen, WHITE, corners, 2)

    # Center line
    center_near = view.world_to_screen(0, -half_width)
    center_far = view.world_to_screen(0, half_width)
    pygame.draw.line(screen, WHITE, center_near, center_far, 1)

    # Net (as a line across the table)
    net_left = view.world_to_screen(0, -half_width - 0.1)
    net_right = view.world_to_screen(0, half_width + 0.1)
    pygame.draw.line(screen, GRAY, net_left, net_right, 3)

    # Side line (center longitudinal)
    side_near = view.world_to_screen(-half_length, 0)
    side_far = view.world_to_screen(half_length, 0)
    pygame.draw.line(screen, WHITE, side_near, side_far, 1)


def draw_top_view_ball(screen: pygame.Surface, view: TopView, sim: Simulation):
    """Draw ball in top view with height indicator."""
    pos = sim.ball_position()
    screen_pos = view.world_to_screen(pos.x, pos.z)
    radius = max(view.meters_to_pixels(0.02), 5)

    # Ball shadow (darker, slightly offset based on height)
    shadow_offset = int(pos.y * 5)  # Higher ball = more offset
    shadow_pos = (screen_pos[0] + shadow_offset, screen_pos[1] + shadow_offset)
    pygame.draw.circle(screen, DARK_GRAY, shadow_pos, radius)

    # Ball (color intensity based on height - brighter when higher)
    height_factor = min(1.0, max(0.3, pos.y / 1.5))
    ball_color = (
        int(255 * height_factor),
        int(165 * height_factor),
        0,
    )
    pygame.draw.circle(screen, ball_color, screen_pos, radius)
    pygame.draw.circle(screen, WHITE, screen_pos, radius, 1)

    # Sidespin indicator (Y component of spin)
    spin = sim.ball_spin()
    if abs(spin.y) > 10:
        indicator_len = radius + 4
        # Sidespin causes rotation around Y axis
        direction = 1 if spin.y > 0 else -1
        # Draw rotation indicator
        for i in range(3):
            angle = direction * (i * 0.4) + math.pi / 2
            x1 = screen_pos[0] + int(indicator_len * math.cos(angle))
            y1 = screen_pos[1] + int(indicator_len * math.sin(angle))
            angle2 = direction * ((i + 1) * 0.4) + math.pi / 2
            x2 = screen_pos[0] + int(indicator_len * math.cos(angle2))
            y2 = screen_pos[1] + int(indicator_len * math.sin(angle2))
            pygame.draw.line(screen, MAGENTA, (x1, y1), (x2, y2), 2)


def draw_top_view_velocity(screen: pygame.Surface, view: TopView, sim: Simulation):
    """Draw velocity vector in top view (X-Z components)."""
    pos = sim.ball_position()
    vel = sim.ball_velocity()
    screen_pos = view.world_to_screen(pos.x, pos.z)

    vel_scale = 8
    end_x = screen_pos[0] + int(vel.x * vel_scale)
    end_z = screen_pos[1] - int(vel.z * vel_scale)

    pygame.draw.line(screen, CYAN, screen_pos, (end_x, end_z), 2)

    # Arrow head
    if vel.x != 0 or vel.z != 0:
        angle = math.atan2(-vel.z, vel.x)
        arrow_len = 6
        arrow_angle = 0.5
        ax1 = end_x - int(arrow_len * math.cos(angle - arrow_angle))
        ay1 = end_z + int(arrow_len * math.sin(angle - arrow_angle))
        ax2 = end_x - int(arrow_len * math.cos(angle + arrow_angle))
        ay2 = end_z + int(arrow_len * math.sin(angle + arrow_angle))
        pygame.draw.line(screen, CYAN, (end_x, end_z), (ax1, ay1), 2)
        pygame.draw.line(screen, CYAN, (end_x, end_z), (ax2, ay2), 2)


def draw_top_view_trajectory(
    screen: pygame.Surface, view: TopView, trajectory: list[tuple[float, float, float]]
):
    """Draw trajectory in top view (X-Z projection)."""
    if len(trajectory) < 2:
        return

    points = [view.world_to_screen(x, z) for x, y, z in trajectory]
    for i in range(len(points) - 1):
        alpha = int(255 * (i / len(points)))
        color = (0, alpha, alpha)  # Cyan fade
        pygame.draw.line(screen, color, points[i], points[i + 1], 1)


# =============================================================================
# Info panel
# =============================================================================


def draw_info(
    screen: pygame.Surface,
    sim: Simulation,
    font: pygame.font.Font,
    launch_speed: float,
    launch_angle_v: float,
    launch_angle_h: float,
    spin_type: int,
    spin_rpm: float,
    paused: bool,
):
    """Draw information panel at bottom."""
    info_rect = pygame.Rect(0, VIEW_HEIGHT, WINDOW_WIDTH, INFO_HEIGHT)
    pygame.draw.rect(screen, (20, 20, 30), info_rect)
    pygame.draw.line(screen, GRAY, (0, VIEW_HEIGHT), (WINDOW_WIDTH, VIEW_HEIGHT), 1)

    pos = sim.ball_position()
    vel = sim.ball_velocity()
    spin = sim.ball_spin()

    # Left column - Ball state
    col1_x = 20
    lines1 = [
        f"Time: {sim.time:.3f}s",
        f"Position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})",
        f"Speed: {sim.ball_speed():.1f} m/s ({sim.ball_speed() * 3.6:.1f} km/h)",
        f"Spin: {sim.ball_spin_rpm():.0f} RPM",
    ]

    # Middle column - Launch params
    col2_x = 400
    lines2 = [
        f"Launch speed: {launch_speed:.1f} m/s (←→)",
        f"Vertical angle: {launch_angle_v:.0f}° (↑↓)",
        f"Horizontal angle: {launch_angle_h:.0f}° (A/D)",
        f"Spin: {SPIN_NAMES[spin_type]} ({spin_rpm:.0f} RPM) (T/S)",
    ]

    # Right column - Controls
    col3_x = 850
    lines3 = [
        "SPACE: Launch | R: Reset | P: Pause",
        "T: Cycle spin type | S: Flip sidespin",
        f"{'[PAUSED]' if paused else ''}",
    ]

    y_start = VIEW_HEIGHT + 15
    for i, line in enumerate(lines1):
        text = font.render(line, True, WHITE)
        screen.blit(text, (col1_x, y_start + i * 24))

    for i, line in enumerate(lines2):
        text = font.render(line, True, WHITE)
        screen.blit(text, (col2_x, y_start + i * 24))

    for i, line in enumerate(lines3):
        color = YELLOW if "[PAUSED]" in line else WHITE
        text = font.render(line, True, color)
        screen.blit(text, (col3_x, y_start + i * 24))

    # Collision indicator
    collision = sim.last_collision()
    if collision:
        color = YELLOW if collision == "table" else RED
        text = font.render(f"COLLISION: {collision}", True, color)
        screen.blit(text, (col3_x, y_start + 80))


def draw_view_labels(screen: pygame.Surface, font: pygame.font.Font):
    """Draw labels for each view."""
    # Side view label
    label = font.render("SIDE VIEW (X-Y)", True, WHITE)
    screen.blit(label, (10, 10))

    # Top view label
    label = font.render("TOP VIEW (X-Z)", True, WHITE)
    screen.blit(label, (SIDE_VIEW_WIDTH + 10, 10))

    # Divider line
    pygame.draw.line(
        screen, GRAY, (SIDE_VIEW_WIDTH, 0), (SIDE_VIEW_WIDTH, VIEW_HEIGHT), 2
    )


# =============================================================================
# Main
# =============================================================================


def main():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("Table Tennis Physics - Dual View (Side + Top)")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("monospace", 16)

    # Create views
    side_view = SideView(0, 0, SIDE_VIEW_WIDTH, VIEW_HEIGHT, SIDE_SCALE)
    top_view = TopView(SIDE_VIEW_WIDTH, 0, TOP_VIEW_WIDTH, VIEW_HEIGHT, TOP_SCALE)

    sim = Simulation()
    trajectory: list[tuple[float, float, float]] = []  # Store x, y, z
    running = True
    paused = False

    # Launch parameters
    launch_speed = 8.0
    launch_angle_v = 10.0  # Vertical angle (degrees above horizontal)
    launch_angle_h = 0.0  # Horizontal angle (degrees, 0 = straight, + = right)
    spin_type = SPIN_TOPSPIN
    spin_rpm = 3000.0

    def launch_ball():
        """Launch a ball with current parameters."""
        nonlocal trajectory
        trajectory = []

        # Start position: left side of table, above surface
        start_x = -1.2
        start_y = sim.table_surface_y() + 0.15
        start_z = 0.0

        # Velocity from angles and speed
        angle_v_rad = math.radians(launch_angle_v)
        angle_h_rad = math.radians(launch_angle_h)

        # Horizontal component
        v_horizontal = launch_speed * math.cos(angle_v_rad)
        vx = v_horizontal * math.cos(angle_h_rad)
        vz = v_horizontal * math.sin(angle_h_rad)
        vy = launch_speed * math.sin(angle_v_rad)

        # Spin based on type
        # Coordinate system: X toward opponent, Y up, Z right
        # Topspin: rotation around Z axis (positive = forward rotation)
        # Sidespin: rotation around Y axis
        # Backspin: rotation around Z axis (negative)
        spin_x, spin_y, spin_z = 0.0, 0.0, 0.0
        if spin_type == SPIN_TOPSPIN:
            spin_z = spin_rpm
        elif spin_type == SPIN_BACKSPIN:
            spin_z = -spin_rpm
        elif spin_type == SPIN_SIDESPIN_RIGHT:
            spin_y = spin_rpm  # Curves right
        elif spin_type == SPIN_SIDESPIN_LEFT:
            spin_y = -spin_rpm  # Curves left

        sim.reset()
        sim.set_ball_position(start_x, start_y, start_z)
        sim.set_ball_velocity(vx, vy, vz)
        sim.set_ball_spin_rpm(spin_x, spin_y, spin_z)

    # Initial launch
    launch_ball()

    while running:
        # Event handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    running = False
                elif event.key == pygame.K_SPACE:
                    launch_ball()
                elif event.key == pygame.K_r:
                    sim.reset()
                    trajectory = []
                elif event.key == pygame.K_t:
                    spin_type = (spin_type + 1) % 5
                elif event.key == pygame.K_s:
                    # Flip sidespin direction
                    if spin_type == SPIN_SIDESPIN_RIGHT:
                        spin_type = SPIN_SIDESPIN_LEFT
                    elif spin_type == SPIN_SIDESPIN_LEFT:
                        spin_type = SPIN_SIDESPIN_RIGHT
                elif event.key == pygame.K_p:
                    paused = not paused
                elif event.key == pygame.K_UP:
                    launch_angle_v = min(launch_angle_v + 5, 45)
                elif event.key == pygame.K_DOWN:
                    launch_angle_v = max(launch_angle_v - 5, -15)
                elif event.key == pygame.K_RIGHT:
                    launch_speed = min(launch_speed + 1, 25)
                elif event.key == pygame.K_LEFT:
                    launch_speed = max(launch_speed - 1, 2)
                elif event.key == pygame.K_d:
                    launch_angle_h = min(launch_angle_h + 5, 45)
                elif event.key == pygame.K_a:
                    launch_angle_h = max(launch_angle_h - 5, -45)

        # Physics update
        if not paused:
            for _ in range(PHYSICS_SUBSTEPS):
                sim.step(PHYSICS_DT)

            # Record trajectory (x, y, z)
            pos = sim.ball_position()
            trajectory.append((pos.x, pos.y, pos.z))
            if len(trajectory) > 500:
                trajectory.pop(0)

        # Drawing
        screen.fill(BLACK)

        # Side view
        draw_side_view_table(screen, side_view, sim)
        draw_side_view_trajectory(screen, side_view, trajectory)
        draw_side_view_ball(screen, side_view, sim)
        draw_side_view_velocity(screen, side_view, sim)

        # Top view
        draw_top_view_table(screen, top_view, sim)
        draw_top_view_trajectory(screen, top_view, trajectory)
        draw_top_view_ball(screen, top_view, sim)
        draw_top_view_velocity(screen, top_view, sim)

        # UI
        draw_view_labels(screen, font)
        draw_info(
            screen,
            sim,
            font,
            launch_speed,
            launch_angle_v,
            launch_angle_h,
            spin_type,
            spin_rpm,
            paused,
        )

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
