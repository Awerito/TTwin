#!/usr/bin/env python3
"""
OAK-D 3D Pose Playback - Visualize Processed Recordings

PURPOSE: Play back recorded RGB with 3D pose overlay.
         Requires poses_3d.json from process_recording.py.

Input: Recording folder with poses_3d.json
Output: OpenCV window with skeleton visualization

Visual Indicators:
- Keypoint color: red (close) → blue (far) based on depth
- Distance overlay: average torso distance in meters
- Depth labels on shoulders, hips, ankles

Controls:
    SPACE       - Play/pause
    LEFT/RIGHT  - Previous/next frame
    +/-         - Speed up/down playback
    q           - Quit

Usage:
    python oak/playback_3d.py recordings/20260115_225627

Workflow:
    1. oak/capture_raw.py      → Record RGB + Depth
    2. oak/process_recording.py → Extract 3D poses
    3. oak/playback_3d.py       → Visualize results (this script)
"""

import json
import sys
from pathlib import Path

import cv2
import numpy as np


CONNECTIONS = [
    (11, 12), (11, 13), (13, 15), (12, 14), (14, 16),
    (11, 23), (12, 24), (23, 24),
    (23, 25), (25, 27), (24, 26), (26, 28),
    (27, 29), (27, 31), (28, 30), (28, 32),
]

JOINT_NAMES = {
    11: "L_shldr", 12: "R_shldr",
    13: "L_elbow", 14: "R_elbow",
    15: "L_wrist", 16: "R_wrist",
    23: "L_hip", 24: "R_hip",
    25: "L_knee", 26: "R_knee",
    27: "L_ankle", 28: "R_ankle",
}


def main():
    if len(sys.argv) < 2:
        print("Usage: python playback_3d.py <recording_dir>")
        sys.exit(1)

    recording_dir = Path(sys.argv[1])

    # Load data
    with open(recording_dir / "poses_3d.json") as f:
        data = json.load(f)

    with open(recording_dir / "intrinsics.json") as f:
        intrinsics = json.load(f)

    with open(recording_dir / "metadata.json") as f:
        metadata = json.load(f)

    poses = data["poses"]
    num_frames = len(poses)
    fx, fy = intrinsics["fx"], intrinsics["fy"]
    cx, cy = intrinsics["cx"], intrinsics["cy"]
    width, height = intrinsics["width"], intrinsics["height"]
    recorded_fps = metadata.get("avg_fps", 15)

    print(f"Recording: {recording_dir.name}")
    print(f"Frames: {num_frames}")
    print(f"Recorded at: {recorded_fps:.1f} fps")
    print()
    print("Controls: SPACE=play/pause, LEFT/RIGHT=step, +/-=speed, q=quit")

    current_frame = 0
    playing = False
    playback_delay = int(1000 / recorded_fps)  # Match recorded FPS

    while True:
        frame_id = f"{current_frame:06d}"
        rgb_path = recording_dir / "rgb" / f"{frame_id}.jpg"

        if not rgb_path.exists():
            break

        rgb = cv2.imread(str(rgb_path))
        pose_data = poses[current_frame]

        # Draw skeleton if we have landmarks
        if pose_data["landmarks_3d"]:
            lms = pose_data["landmarks_3d"]

            # Build pixel coordinates from 3D
            pixels = {}
            for lm in lms:
                idx = lm["id"]
                if lm["x"] is not None:
                    # Back-project 3D to 2D
                    x_m, y_m, z_m = lm["x"], lm["y"], lm["z"]
                    px = int(x_m * fx / z_m + cx)
                    py = int(y_m * fy / z_m + cy)
                    pixels[idx] = (px, py, z_m)

            # Draw connections
            for i, j in CONNECTIONS:
                if i in pixels and j in pixels:
                    p1 = pixels[i][:2]
                    p2 = pixels[j][:2]
                    cv2.line(rgb, p1, p2, (0, 255, 0), 2)

            # Draw keypoints with depth labels
            for idx, (px, py, z_m) in pixels.items():
                if idx < 11:
                    continue

                # Color by depth (closer=red, farther=blue)
                depth_norm = np.clip((z_m - 0.5) / 3.0, 0, 1)
                color = (
                    int(255 * depth_norm),      # B
                    int(100),                    # G
                    int(255 * (1 - depth_norm))  # R
                )

                cv2.circle(rgb, (px, py), 6, color, -1)
                cv2.circle(rgb, (px, py), 6, (255, 255, 255), 1)

                # Show depth for key joints
                if idx in [11, 12, 23, 24, 27, 28]:
                    cv2.putText(
                        rgb, f"{z_m:.2f}m",
                        (px + 8, py - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1
                    )

        # Status overlay
        status = "PLAYING" if playing else "PAUSED"
        current_fps = 1000 / playback_delay
        cv2.putText(
            rgb, f"[{status}] Frame {current_frame}/{num_frames-1} ({current_fps:.0f}fps)",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2
        )

        # Show 3D stats for current frame
        if pose_data["landmarks_3d"]:
            lms = pose_data["landmarks_3d"]
            # Get torso center (avg of shoulders and hips)
            torso_z = []
            for lm in lms:
                if lm["id"] in [11, 12, 23, 24] and lm["z"] is not None:
                    torso_z.append(lm["z"])
            if torso_z:
                avg_z = sum(torso_z) / len(torso_z)
                cv2.putText(
                    rgb, f"Distance: {avg_z:.2f}m",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2
                )

        cv2.imshow("3D Pose Playback", rgb)

        # Handle input
        wait_time = playback_delay if playing else 0
        key = cv2.waitKey(wait_time) & 0xFF

        if key == ord("q"):
            break
        elif key == ord(" "):
            playing = not playing
        elif key == 81 or key == 2:  # LEFT
            current_frame = max(0, current_frame - 1)
            playing = False
        elif key == 83 or key == 3:  # RIGHT
            current_frame = min(num_frames - 1, current_frame + 1)
            playing = False
        elif key == ord("+") or key == ord("="):
            playback_delay = max(10, playback_delay - 10)
        elif key == ord("-"):
            playback_delay = min(200, playback_delay + 10)

        # Auto advance if playing
        if playing:
            current_frame += 1
            if current_frame >= num_frames:
                current_frame = 0

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
