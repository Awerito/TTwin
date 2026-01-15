#!/usr/bin/env python3
"""
OAK-D Pose Estimation with Tracking

Uses OAK-D as camera, runs YOLOv8 Pose + ByteTrack on CPU.
Each person gets a persistent ID that survives occlusions.

Performance: ~32 FPS with ~30ms latency on Ryzen 7 5700X.

Usage:
    python oak/pose.py
"""

import time

import cv2
import depthai as dai
from ultralytics import YOLO

from config import CAMERA_IP

# COCO pose skeleton connections (17 keypoints)
SKELETON = [
    (0, 1), (0, 2), (1, 3), (2, 4),  # Head
    (5, 6), (5, 7), (7, 9), (6, 8), (8, 10),  # Arms
    (5, 11), (6, 12), (11, 12),  # Torso
    (11, 13), (13, 15), (12, 14), (14, 16),  # Legs
]


def main():
    print(f"Connecting to OAK-D at {CAMERA_IP}...")

    device_info = dai.DeviceInfo(CAMERA_IP)

    try:
        device = dai.Device(device_info)
        pipeline = dai.Pipeline(device)

        print(f"Connected to {device.getDeviceName()}")

        # RGB Camera
        cam = pipeline.create(dai.node.Camera).build(
            boardSocket=dai.CameraBoardSocket.CAM_A
        )
        rgb_out = cam.requestOutput((640, 480), dai.ImgFrame.Type.BGR888p, fps=50)

        # Output queue - latest frame only (drop old frames)
        q_rgb = rgb_out.createOutputQueue(maxSize=1, blocking=False)

        print("Loading YOLOv8 Pose model...")
        model = YOLO("yolov8n-pose.pt")
        print("Model loaded!")

        print()
        print("Controls:")
        print("  q/ESC - Quit")
        print("  s     - Toggle smoothing")
        print("  +/-   - Adjust smoothing (0.0-0.9)")
        print()

        cv2.namedWindow("OAK-D Pose", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("OAK-D Pose", 1280, 960)

        pipeline.start()

        # Metrics
        fps_counter = 0
        fps_start = time.time()
        fps_display = 0.0
        inference_ms = 0.0

        # Smoothing (EMA filter)
        prev_keypoints = {}
        smoothing_alpha = 0.4
        smoothing_enabled = True

        while pipeline.isRunning():
            in_rgb = q_rgb.tryGet()

            if in_rgb is not None:
                frame = in_rgb.getCvFrame()
                now = time.time()

                # Inference + tracking
                t0 = time.perf_counter()
                results = model.track(frame, verbose=False, persist=True)
                inference_ms = (time.perf_counter() - t0) * 1000

                # FPS
                fps_counter += 1
                if now - fps_start >= 1.0:
                    fps_display = fps_counter / (now - fps_start)
                    fps_counter = 0
                    fps_start = now

                # Process detections
                num_persons = 0
                for result in results:
                    if result.keypoints is None:
                        continue

                    keypoints_data = result.keypoints.data.cpu().numpy()
                    track_ids = None
                    if result.boxes.id is not None:
                        track_ids = result.boxes.id.cpu().numpy().astype(int)

                    for idx, kps in enumerate(keypoints_data):
                        person_id = track_ids[idx] if track_ids is not None else idx
                        num_persons += 1

                        kp_coords = []
                        for kp_idx, kp in enumerate(kps):
                            x, y, conf = int(kp[0]), int(kp[1]), kp[2]

                            # Smoothing
                            if smoothing_enabled and person_id in prev_keypoints:
                                prev = prev_keypoints[person_id]
                                if kp_idx < len(prev) and prev[kp_idx][2] > 0.3:
                                    x = int(
                                        smoothing_alpha * prev[kp_idx][0]
                                        + (1 - smoothing_alpha) * x
                                    )
                                    y = int(
                                        smoothing_alpha * prev[kp_idx][1]
                                        + (1 - smoothing_alpha) * y
                                    )

                            kp_coords.append((x, y, conf))
                            if conf > 0.3:
                                cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)

                        prev_keypoints[person_id] = kp_coords

                        # Skeleton
                        for i, j in SKELETON:
                            if i < len(kp_coords) and j < len(kp_coords):
                                if kp_coords[i][2] > 0.3 and kp_coords[j][2] > 0.3:
                                    cv2.line(
                                        frame,
                                        (kp_coords[i][0], kp_coords[i][1]),
                                        (kp_coords[j][0], kp_coords[j][1]),
                                        (255, 0, 0),
                                        2,
                                    )

                        # Person ID label
                        if kp_coords[0][2] > 0.3:
                            cv2.putText(
                                frame,
                                f"ID:{person_id}",
                                (kp_coords[0][0] + 10, kp_coords[0][1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.6,
                                (0, 255, 0),
                                2,
                            )

                # Overlay
                cv2.putText(
                    frame, f"FPS: {fps_display:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
                )
                cv2.putText(
                    frame, f"Persons: {num_persons}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
                )
                cv2.putText(
                    frame, f"Inference: {inference_ms:.1f}ms", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
                )
                smooth_text = f"Smooth: {smoothing_alpha:.1f}" if smoothing_enabled else "Smooth: OFF"
                cv2.putText(
                    frame, smooth_text, (10, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (255, 255, 0) if smoothing_enabled else (128, 128, 128), 2
                )

                cv2.imshow("OAK-D Pose", frame)

            key = cv2.waitKey(1)
            if key == ord("q") or key == 27:
                pipeline.stop()
                break
            elif key == ord("s"):
                smoothing_enabled = not smoothing_enabled
                print(f"Smoothing: {'ON' if smoothing_enabled else 'OFF'}")
            elif key in (ord("+"), ord("=")):
                smoothing_alpha = min(0.9, smoothing_alpha + 0.1)
                print(f"Smoothing: {smoothing_alpha:.1f}")
            elif key == ord("-"):
                smoothing_alpha = max(0.0, smoothing_alpha - 0.1)
                print(f"Smoothing: {smoothing_alpha:.1f}")

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    exit(main())
