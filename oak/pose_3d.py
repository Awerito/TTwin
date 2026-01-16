#!/usr/bin/env python3
"""
OAK-D Pose 3D - DEMO/VISUALIZATION ONLY

PURPOSE: Real-time visualization demo showing pose + depth overlay.
         NOT for recording - inference runs every frame.

Based on: oak/pose.py (the working baseline)

Features:
- RGB 640x480 @ ~30fps
- Stereo depth aligned to RGB (non-blocking)
- IR dot projector ON (0.5 intensity)
- MediaPipe pose detection (full model)
- Depth sampling at keypoints with color coding:
  - Green: valid depth (0.3m - 5.0m)
  - Red: invalid depth
  - Yellow: depth not yet available

Limitations:
- Lower resolution (640x480) for real-time performance
- Depth is cached (may be 1-2 frames behind RGB)
- NOT suitable for recording - use oak_capture_raw.py instead

Controls:
- q: Quit

For RECORDING without inference, use: oak/capture_raw.py
"""

import sys
import time
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from config import CAMERA_IP

import cv2
import depthai as dai
import mediapipe as mp
import numpy as np
from mediapipe.tasks import python
from mediapipe.tasks.python import vision


MODELS_DIR = Path(__file__).parent / "models"
MODEL_PATH = MODELS_DIR / "pose_landmarker_full.task"
MODEL_URL = "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_full/float16/1/pose_landmarker_full.task"

# Same resolution as working baseline
WIDTH = 640
HEIGHT = 480


def download_model():
    if MODEL_PATH.exists():
        return
    MODEL_PATH.parent.mkdir(parents=True, exist_ok=True)
    print("Downloading model...")
    urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)


def main():
    download_model()

    print(f"Connecting to {CAMERA_IP}...")
    device_info = dai.DeviceInfo(CAMERA_IP)

    with dai.Pipeline() as pipeline:
        device = pipeline.getDefaultDevice()
        print(f"Connected: {device.getDeviceName()}")

        # RGB camera
        cam_rgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)

        # Stereo cameras
        cam_left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        cam_right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

        # Stereo depth
        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)

        # Link stereo
        cam_left.requestOutput((WIDTH, HEIGHT)).link(stereo.left)
        cam_right.requestOutput((WIDTH, HEIGHT)).link(stereo.right)

        # RGB output
        rgb_out = cam_rgb.requestOutput((WIDTH, HEIGHT), dai.ImgFrame.Type.BGR888i)

        # Align depth to RGB
        rgb_out.link(stereo.inputAlignTo)

        # Queues
        q_rgb = rgb_out.createOutputQueue(maxSize=1, blocking=False)
        q_depth = stereo.depth.createOutputQueue(maxSize=1, blocking=False)

        # IR projector ON
        device.setIrLaserDotProjectorIntensity(0.5)
        print("IR Dot Projector: ON")

        # Intrinsics for 3D
        calib = device.readCalibration()
        intrinsics = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, WIDTH, HEIGHT)
        fx, fy = intrinsics[0][0], intrinsics[1][1]
        cx, cy = intrinsics[0][2], intrinsics[1][2]
        print(f"Intrinsics: fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}")

        # MediaPipe
        pose_options = vision.PoseLandmarkerOptions(
            base_options=python.BaseOptions(model_asset_path=str(MODEL_PATH)),
            running_mode=vision.RunningMode.VIDEO,
            num_poses=1,
            min_pose_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        pose = vision.PoseLandmarker.create_from_options(pose_options)

        pipeline.start()
        print("Running... Press 'q' to quit")

        connections = [
            (11, 12), (11, 13), (13, 15), (12, 14), (14, 16),
            (11, 23), (12, 24), (23, 24),
            (23, 25), (25, 27), (24, 26), (26, 28),
        ]

        frame_ts = 0
        fps_counter = 0
        fps_time = time.time()
        fps_display = 0
        inference_ms = 0
        depth = None  # Cache last depth frame

        while True:
            in_rgb = q_rgb.tryGet()
            in_depth = q_depth.tryGet()

            # Always update depth cache when available
            if in_depth is not None:
                depth = in_depth.getFrame()

            # Only need RGB to proceed (depth is optional per frame)
            if in_rgb is None:
                time.sleep(0.001)
                continue

            # Count real frames
            fps_counter += 1
            now = time.time()
            if now - fps_time >= 1.0:
                fps_display = fps_counter
                fps_counter = 0
                fps_time = now

            frame_ts += 33
            rgb = in_rgb.getCvFrame()
            h, w = rgb.shape[:2]
            has_depth = depth is not None

            # MediaPipe
            t0 = time.time()
            mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB))
            result = pose.detect_for_video(mp_img, frame_ts)
            inference_ms = (time.time() - t0) * 1000

            # Draw
            if result.pose_landmarks:
                lms = result.pose_landmarks[0]

                # Connections
                for i, j in connections:
                    if i < len(lms) and j < len(lms):
                        p1 = (int(lms[i].x * w), int(lms[i].y * h))
                        p2 = (int(lms[j].x * w), int(lms[j].y * h))
                        cv2.line(rgb, p1, p2, (0, 255, 0), 2)

                # Keypoints with depth
                for i, lm in enumerate(lms):
                    if i < 11:
                        continue
                    px, py = int(lm.x * w), int(lm.y * h)

                    # Sample depth (if available)
                    z_m = 0
                    valid = False
                    if has_depth and 0 <= px < w and 0 <= py < h:
                        z_mm = depth[py, px]
                        z_m = z_mm / 1000.0
                        valid = 0.3 < z_m < 5.0

                    # Color by validity (yellow if no depth data)
                    if not has_depth:
                        color = (0, 255, 255)  # Yellow - no depth
                    elif valid:
                        color = (0, 255, 0)  # Green - valid depth
                    else:
                        color = (0, 0, 255)  # Red - invalid depth
                    cv2.circle(rgb, (px, py), 5, color, -1)

                    # Show depth for key joints
                    if i in [11, 12, 23, 24, 27, 28] and valid:
                        cv2.putText(rgb, f"{z_m:.2f}m", (px + 5, py - 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

            # Info
            cv2.putText(rgb, f"FPS: {fps_display}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(rgb, f"Inference: {inference_ms:.0f}ms", (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(rgb, f"Res: {w}x{h}", (10, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            depth_status = "Depth: OK" if has_depth else "Depth: waiting..."
            cv2.putText(rgb, f"IR: ON | {depth_status}", (10, 95),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            cv2.imshow("RGB + Pose 3D", rgb)

            # Depth colormap (only if depth available)
            if has_depth:
                depth_color = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET
                )
                cv2.imshow("Depth", depth_color)

            if cv2.waitKey(1) == ord('q'):
                break

        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
