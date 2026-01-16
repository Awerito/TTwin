#!/usr/bin/env python3
"""
OAK-D Recording Processor - Offline 3D Pose Extraction

PURPOSE: Process recorded RGB+Depth with MediaPipe pose detection.
         Outputs 3D landmarks using depth + camera intrinsics.

Input: Recording from oak_capture_raw.py
Output: poses_3d.json with 33 landmarks per frame in camera coordinates (meters)

3D Coordinate System:
- X: right (positive) / left (negative)
- Y: down (positive) / up (negative)
- Z: forward (distance from camera)

Formula (pixel + depth → 3D):
    X = (px - cx) * Z / fx
    Y = (py - cy) * Z / fy
    Z = depth_mm / 1000

Performance: ~24 fps on CPU (XNNPACK)

Usage:
    python oak/process_recording.py recordings/20260115_225627

Output file: recordings/<session>/poses_3d.json
"""

import json
import sys
import time
import urllib.request
from pathlib import Path

import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks import python
from mediapipe.tasks.python import vision


MODELS_DIR = Path(__file__).parent / "models"
MODEL_URL = "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_full/float16/1/pose_landmarker_full.task"
MODEL_FILE = MODELS_DIR / "pose_landmarker_full.task"


def download_model():
    if MODEL_FILE.exists():
        return
    MODEL_FILE.parent.mkdir(parents=True, exist_ok=True)
    print("Downloading model...")
    urllib.request.urlretrieve(MODEL_URL, MODEL_FILE)


def main():
    if len(sys.argv) < 2:
        print("Usage: python process_recording.py <recording_dir>")
        print("Example: python process_recording.py recordings/20260115_225627")
        sys.exit(1)

    recording_dir = Path(sys.argv[1])
    if not recording_dir.exists():
        print(f"Error: {recording_dir} not found")
        sys.exit(1)

    # Load metadata
    with open(recording_dir / "metadata.json") as f:
        metadata = json.load(f)

    with open(recording_dir / "intrinsics.json") as f:
        intrinsics = json.load(f)

    num_frames = metadata["frames"]
    fx, fy = intrinsics["fx"], intrinsics["fy"]
    cx, cy = intrinsics["cx"], intrinsics["cy"]

    print(f"Recording: {recording_dir.name}")
    print(f"Frames: {num_frames}")
    print(f"Intrinsics: fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}")
    print()

    download_model()

    # MediaPipe in IMAGE mode (single frames)
    pose_options = vision.PoseLandmarkerOptions(
        base_options=python.BaseOptions(model_asset_path=str(MODEL_FILE)),
        running_mode=vision.RunningMode.IMAGE,
        num_poses=1,
        min_pose_detection_confidence=0.5,
    )
    pose_landmarker = vision.PoseLandmarker.create_from_options(pose_options)

    # Output file
    output_file = recording_dir / "poses_3d.json"
    all_poses = []

    print(f"Processing {num_frames} frames...")
    start_time = time.time()

    for frame_idx in range(num_frames):
        frame_id = f"{frame_idx:06d}"
        rgb_path = recording_dir / "rgb" / f"{frame_id}.jpg"
        depth_path = recording_dir / "depth" / f"{frame_id}.png"

        if not rgb_path.exists():
            continue

        # Load frames
        rgb = cv2.imread(str(rgb_path))
        depth = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
        h, w = rgb.shape[:2]

        # Run MediaPipe
        mp_image = mp.Image(
            image_format=mp.ImageFormat.SRGB,
            data=cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB),
        )
        result = pose_landmarker.detect(mp_image)

        frame_data = {"frame": frame_idx, "landmarks_3d": None}

        if result.pose_landmarks:
            landmarks = result.pose_landmarks[0]
            landmarks_3d = []

            for i, lm in enumerate(landmarks):
                px, py = int(lm.x * w), int(lm.y * h)

                # Get depth
                z_mm = 0
                if 0 <= px < w and 0 <= py < h:
                    z_mm = depth[py, px]
                z_m = z_mm / 1000.0

                # Convert to 3D camera coordinates
                if 0.3 < z_m < 5.0:
                    x_m = (px - cx) * z_m / fx
                    y_m = (py - cy) * z_m / fy
                    landmarks_3d.append({
                        "id": i,
                        "x": round(x_m, 4),
                        "y": round(y_m, 4),
                        "z": round(z_m, 4),
                        "visibility": round(lm.visibility, 3),
                    })
                else:
                    landmarks_3d.append({
                        "id": i,
                        "x": None,
                        "y": None,
                        "z": None,
                        "visibility": round(lm.visibility, 3),
                    })

            frame_data["landmarks_3d"] = landmarks_3d

        all_poses.append(frame_data)

        # Progress
        if (frame_idx + 1) % 100 == 0 or frame_idx == num_frames - 1:
            elapsed = time.time() - start_time
            fps = (frame_idx + 1) / elapsed
            print(f"  {frame_idx + 1}/{num_frames} ({fps:.1f} fps)")

    # Save results
    with open(output_file, "w") as f:
        json.dump({
            "recording": recording_dir.name,
            "num_frames": num_frames,
            "intrinsics": intrinsics,
            "poses": all_poses,
        }, f)

    elapsed = time.time() - start_time
    print()
    print(f"Done in {elapsed:.1f}s ({num_frames/elapsed:.1f} fps)")
    print(f"Output: {output_file}")


if __name__ == "__main__":
    main()
