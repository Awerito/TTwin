#!/usr/bin/env python3
"""
Side view (X-Y plane) visualization of table tennis physics.

Controls:
    SPACE - Launch ball with random parameters
    R     - Reset simulation
    T     - Toggle topspin/backspin
    UP/DOWN - Adjust launch angle
    LEFT/RIGHT - Adjust launch speed
    ESC/Q - Quit
"""

import math
import random
import sys

import pygame

from tt_physics import Simulation, Vec3


# =============================================================================
# Constants
# =============================================================================

# Window
WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 700
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

# Scale: pixels per meter
SCALE = 250
ORIGIN_X = WINDOW_WIDTH // 2  # Center of table at screen center
ORIGIN_Y = WINDOW_HEIGHT - 100  # Ground level near bottom


# =============================================================================
# Coordinate conversion
# =============================================================================


def world_to_screen(x: float, y: float) -> tuple[int, int]:
    """Convert world coordinates (meters) to screen coordinates (pixels)."""
    screen_x = int(ORIGIN_X + x * SCALE)
    screen_y = int(ORIGIN_Y - y * SCALE)
    return screen_x, screen_y


def meters_to_pixels(m: float) -> int:
    """Convert a distance in meters to pixels."""
    return int(m * SCALE)


# =============================================================================
# Drawing functions
# =============================================================================


def draw_table(screen: pygame.Surface, sim: Simulation):
    """Draw the table and net."""
    half_length, half_width, height, net_height = sim.table_dimensions()
    surface_y = sim.table_surface_y()

    # Table surface
    left = world_to_screen(-half_length, surface_y)
    right = world_to_screen(half_length, surface_y)
    pygame.draw.line(screen, GREEN, left, right, 4)

    # Table legs
    leg_left = world_to_screen(-half_length + 0.1, 0)
    leg_right = world_to_screen(half_length - 0.1, 0)
    pygame.draw.line(screen, DARK_GRAY, left, leg_left, 3)
    pygame.draw.line(screen, DARK_GRAY, right, leg_right, 3)

    # Net
    net_bottom = world_to_screen(0, surface_y)
    net_top = world_to_screen(0, surface_y + net_height)
    pygame.draw.line(screen, WHITE, net_bottom, net_top, 2)

    # Net post indicators
    pygame.draw.circle(screen, WHITE, net_top, 4)


def draw_ball(screen: pygame.Surface, sim: Simulation):
    """Draw the ball with spin indicator."""
    pos = sim.ball_position()
    screen_pos = world_to_screen(pos.x, pos.y)
    radius = meters_to_pixels(0.02)  # 20mm radius

    # Ball body
    pygame.draw.circle(screen, ORANGE, screen_pos, max(radius, 6))
    pygame.draw.circle(screen, WHITE, screen_pos, max(radius, 6), 1)

    # Spin indicator (line showing rotation direction)
    spin_rpm = sim.ball_spin_rpm()
    if spin_rpm > 100:
        spin = sim.ball_spin()
        # For side view, Z component of spin causes visible rotation
        angle = math.atan2(spin.z, spin.x) if (spin.x != 0 or spin.z != 0) else 0
        indicator_len = min(radius * 2, meters_to_pixels(0.03))
        end_x = screen_pos[0] + int(indicator_len * math.cos(angle))
        end_y = screen_pos[1] - int(indicator_len * math.sin(angle))
        pygame.draw.line(screen, RED, screen_pos, (end_x, end_y), 2)


def draw_velocity_vector(screen: pygame.Surface, sim: Simulation):
    """Draw velocity vector from ball."""
    pos = sim.ball_position()
    vel = sim.ball_velocity()
    screen_pos = world_to_screen(pos.x, pos.y)

    # Scale velocity for visualization (1 m/s = 10 pixels)
    vel_scale = 10
    end_x = screen_pos[0] + int(vel.x * vel_scale)
    end_y = screen_pos[1] - int(vel.y * vel_scale)

    pygame.draw.line(screen, BLUE, screen_pos, (end_x, end_y), 2)

    # Arrow head
    if vel.x != 0 or vel.y != 0:
        angle = math.atan2(-vel.y, vel.x)
        arrow_len = 8
        arrow_angle = 0.5
        ax1 = end_x - int(arrow_len * math.cos(angle - arrow_angle))
        ay1 = end_y + int(arrow_len * math.sin(angle - arrow_angle))
        ax2 = end_x - int(arrow_len * math.cos(angle + arrow_angle))
        ay2 = end_y + int(arrow_len * math.sin(angle + arrow_angle))
        pygame.draw.line(screen, BLUE, (end_x, end_y), (ax1, ay1), 2)
        pygame.draw.line(screen, BLUE, (end_x, end_y), (ax2, ay2), 2)


def draw_trajectory(screen: pygame.Surface, trajectory: list[tuple[float, float]]):
    """Draw trajectory trail."""
    if len(trajectory) < 2:
        return

    points = [world_to_screen(x, y) for x, y in trajectory]

    # Fade older points
    for i in range(len(points) - 1):
        alpha = int(255 * (i / len(points)))
        color = (alpha // 2, alpha // 2, alpha)
        pygame.draw.line(screen, color, points[i], points[i + 1], 1)


def draw_info(
    screen: pygame.Surface,
    sim: Simulation,
    font: pygame.font.Font,
    launch_speed: float,
    launch_angle: float,
    use_topspin: bool,
):
    """Draw information overlay."""
    pos = sim.ball_position()
    vel = sim.ball_velocity()

    lines = [
        f"Time: {sim.time:.3f}s",
        f"Position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})",
        f"Speed: {sim.ball_speed():.1f} m/s ({sim.ball_speed() * 3.6:.1f} km/h)",
        f"Spin: {sim.ball_spin_rpm():.0f} RPM",
        "",
        f"Launch speed: {launch_speed:.1f} m/s (←→)",
        f"Launch angle: {launch_angle:.0f}° (↑↓)",
        f"Spin: {'Topspin' if use_topspin else 'Backspin'} (T)",
        "",
        "SPACE: Launch | R: Reset | ESC: Quit",
    ]

    y = 10
    for line in lines:
        if line:
            text = font.render(line, True, WHITE)
            screen.blit(text, (10, y))
        y += 22

    # Collision indicator
    collision = sim.last_collision()
    if collision:
        color = YELLOW if collision == "table" else RED
        text = font.render(f"COLLISION: {collision}", True, color)
        screen.blit(text, (WINDOW_WIDTH - 200, 10))


def draw_ground(screen: pygame.Surface):
    """Draw ground reference line."""
    left = world_to_screen(-3, 0)
    right = world_to_screen(3, 0)
    pygame.draw.line(screen, DARK_GRAY, left, right, 1)


# =============================================================================
# Main
# =============================================================================


def main():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("Table Tennis Physics - Side View")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("monospace", 16)

    sim = Simulation()
    trajectory: list[tuple[float, float]] = []
    running = True
    paused = False

    # Launch parameters
    launch_speed = 8.0
    launch_angle = 15.0  # degrees above horizontal
    use_topspin = True
    spin_rpm = 3000.0

    def launch_ball():
        """Launch a ball with current parameters."""
        nonlocal trajectory
        trajectory = []

        # Start position: left side of table, above surface
        start_x = -1.2
        start_y = sim.table_surface_y() + 0.15

        # Velocity from angle and speed
        angle_rad = math.radians(launch_angle)
        vx = launch_speed * math.cos(angle_rad)
        vy = launch_speed * math.sin(angle_rad)

        # Spin: topspin is positive Z rotation for ball moving in +X
        spin_z = spin_rpm if use_topspin else -spin_rpm

        sim.reset()
        sim.set_ball_position(start_x, start_y, 0)
        sim.set_ball_velocity(vx, vy, 0)
        sim.set_ball_spin_rpm(0, 0, spin_z)

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
                    use_topspin = not use_topspin
                elif event.key == pygame.K_p:
                    paused = not paused
                elif event.key == pygame.K_UP:
                    launch_angle = min(launch_angle + 5, 45)
                elif event.key == pygame.K_DOWN:
                    launch_angle = max(launch_angle - 5, -10)
                elif event.key == pygame.K_RIGHT:
                    launch_speed = min(launch_speed + 1, 20)
                elif event.key == pygame.K_LEFT:
                    launch_speed = max(launch_speed - 1, 2)

        # Physics update
        if not paused:
            for _ in range(PHYSICS_SUBSTEPS):
                sim.step(PHYSICS_DT)

            # Record trajectory
            pos = sim.ball_position()
            trajectory.append((pos.x, pos.y))
            if len(trajectory) > 500:
                trajectory.pop(0)

        # Drawing
        screen.fill(BLACK)
        draw_ground(screen)
        draw_table(screen, sim)
        draw_trajectory(screen, trajectory)
        draw_ball(screen, sim)
        draw_velocity_vector(screen, sim)
        draw_info(screen, sim, font, launch_speed, launch_angle, use_topspin)

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
