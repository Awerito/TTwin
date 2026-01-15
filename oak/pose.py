#!/usr/bin/env python3
"""
OAK-D Pose Estimation with Feet Detail

MediaPipe Pose: 33 keypoints including 6 foot points:
- Ankles (27, 28)
- Heels (29, 30)
- Toes (31, 32)

Features:
- EMA smoothing to reduce jitter
- Simple tracking with persistent IDs
- Model selection: lite, full, heavy

Performance: ~30 FPS, ~20ms inference with "full" model on Ryzen 7 5700X

Usage:
    python oak/pose.py [--model lite|full|heavy]
"""

import argparse
import time
import urllib.request
from pathlib import Path

import cv2
import depthai as dai
import mediapipe as mp
import numpy as np
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

from config import CAMERA_IP


MODELS_DIR = Path(__file__).parent / "models"

POSE_MODELS = {
    "lite": {
        "url": "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_lite/float16/1/pose_landmarker_lite.task",
        "file": "pose_landmarker_lite.task",
    },
    "full": {
        "url": "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_full/float16/1/pose_landmarker_full.task",
        "file": "pose_landmarker_full.task",
    },
    "heavy": {
        "url": "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_heavy/float16/1/pose_landmarker_heavy.task",
        "file": "pose_landmarker_heavy.task",
    },
}


def download_model(url: str, path: Path) -> None:
    if path.exists():
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    print(f"Downloading {path.name}...")
    urllib.request.urlretrieve(url, path)
    print(f"  Downloaded to {path}")


class PoseTracker:
    """Pose tracker with smoothing and persistent IDs."""

    CONNECTIONS = [
        (11, 12), (11, 13), (13, 15), (12, 14), (14, 16),  # Arms
        (11, 23), (12, 24), (23, 24),  # Torso
        (23, 25), (25, 27), (24, 26), (26, 28),  # Legs
        (27, 29), (27, 31), (29, 31),  # Left foot
        (28, 30), (28, 32), (30, 32),  # Right foot
    ]
    FEET_INDICES = {27, 28, 29, 30, 31, 32}
    WRIST_INDICES = {15, 16}

    def __init__(self):
        self.tracks = {}
        self.next_id = 1
        self.smoothing_alpha = 0.4
        self.smoothing_enabled = True

    def _get_centroid(self, landmarks, w, h):
        if len(landmarks) > 24:
            x = (landmarks[23].x + landmarks[24].x) / 2 * w
            y = (landmarks[23].y + landmarks[24].y) / 2 * h
            return (x, y)
        return None

    def _match_to_track(self, centroid, threshold=100):
        best_id = None
        best_dist = threshold
        for track_id, track in self.tracks.items():
            if "centroid" in track:
                dist = np.sqrt(
                    (centroid[0] - track["centroid"][0]) ** 2
                    + (centroid[1] - track["centroid"][1]) ** 2
                )
                if dist < best_dist:
                    best_dist = dist
                    best_id = track_id
        return best_id

    def update(self, pose_landmarks_list, w, h):
        now = time.time()
        results = []

        for landmarks in pose_landmarks_list:
            centroid = self._get_centroid(landmarks, w, h)
            if centroid is None:
                continue

            track_id = self._match_to_track(centroid)
            if track_id is None:
                track_id = self.next_id
                self.next_id += 1
                self.tracks[track_id] = {}

            smoothed = []
            prev = self.tracks[track_id].get("landmarks", None)

            for i, lm in enumerate(landmarks):
                x, y = lm.x * w, lm.y * h
                vis = lm.visibility if hasattr(lm, "visibility") else 1.0

                if self.smoothing_enabled and prev and i < len(prev):
                    x = self.smoothing_alpha * prev[i][0] + (1 - self.smoothing_alpha) * x
                    y = self.smoothing_alpha * prev[i][1] + (1 - self.smoothing_alpha) * y

                smoothed.append((x, y, vis))

            self.tracks[track_id]["landmarks"] = smoothed
            self.tracks[track_id]["centroid"] = centroid
            self.tracks[track_id]["last_seen"] = now
            results.append((track_id, smoothed))

        # Remove old tracks
        to_remove = [
            tid for tid, t in self.tracks.items()
            if now - t.get("last_seen", 0) > 1.0
        ]
        for tid in to_remove:
            del self.tracks[tid]

        return results

    def draw(self, frame, track_id, landmarks):
        for i, j in self.CONNECTIONS:
            if i < len(landmarks) and j < len(landmarks):
                if landmarks[i][2] > 0.5 and landmarks[j][2] > 0.5:
                    is_foot = i in self.FEET_INDICES or j in self.FEET_INDICES
                    color = (0, 255, 255) if is_foot else (0, 200, 0)
                    thickness = 3 if is_foot else 2
                    pt1 = (int(landmarks[i][0]), int(landmarks[i][1]))
                    pt2 = (int(landmarks[j][0]), int(landmarks[j][1]))
                    cv2.line(frame, pt1, pt2, color, thickness)

        for idx, (x, y, vis) in enumerate(landmarks):
            if vis > 0.5:
                pt = (int(x), int(y))
                if idx in self.FEET_INDICES:
                    cv2.circle(frame, pt, 8, (0, 255, 255), -1)
                    cv2.circle(frame, pt, 8, (0, 0, 0), 2)
                elif idx in self.WRIST_INDICES:
                    cv2.circle(frame, pt, 7, (255, 0, 255), -1)
                elif idx >= 11:
                    cv2.circle(frame, pt, 5, (0, 255, 0), -1)

        if len(landmarks) > 0 and landmarks[0][2] > 0.5:
            pt = (int(landmarks[0][0]) + 10, int(landmarks[0][1]) - 10)
            cv2.putText(frame, f"ID:{track_id}", pt,
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)


def main():
    parser = argparse.ArgumentParser(description="OAK-D Pose Estimation")
    parser.add_argument(
        "--model", choices=["lite", "full", "heavy"], default="full",
        help="Model complexity (default: full)",
    )
    args = parser.parse_args()

    model_info = POSE_MODELS[args.model]
    model_path = MODELS_DIR / model_info["file"]
    download_model(model_info["url"], model_path)

    print(f"Model: {args.model}")
    print(f"Connecting to OAK-D at {CAMERA_IP}...")

    device_info = dai.DeviceInfo(CAMERA_IP)

    try:
        device = dai.Device(device_info)
        pipeline = dai.Pipeline(device)
        print(f"Connected to {device.getDeviceName()}")

        cam = pipeline.create(dai.node.Camera).build(
            boardSocket=dai.CameraBoardSocket.CAM_A
        )
        rgb_out = cam.requestOutput((640, 480), dai.ImgFrame.Type.BGR888p, fps=30)
        q_rgb = rgb_out.createOutputQueue(maxSize=1, blocking=False)

        print(f"Loading MediaPipe Pose ({args.model})...")
        pose_options = vision.PoseLandmarkerOptions(
            base_options=python.BaseOptions(model_asset_path=str(model_path)),
            running_mode=vision.RunningMode.VIDEO,
            num_poses=2,
            min_pose_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        pose_landmarker = vision.PoseLandmarker.create_from_options(pose_options)
        print("Model loaded!")

        tracker = PoseTracker()

        print()
        print("Controls:")
        print("  q/ESC - Quit")
        print("  s     - Toggle smoothing")
        print("  +/-   - Adjust smoothing (0.0-0.9)")
        print()
        print("Colors: Yellow=Feet | Magenta=Wrists | Green=Body")
        print()

        cv2.namedWindow("OAK-D Pose", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("OAK-D Pose", 1280, 960)

        pipeline.start()

        fps_counter = 0
        fps_start = time.time()
        fps_display = 0.0
        inference_ms = 0.0
        frame_timestamp = 0

        while pipeline.isRunning():
            in_rgb = q_rgb.tryGet()

            if in_rgb is not None:
                frame = in_rgb.getCvFrame()
                h, w = frame.shape[:2]
                now = time.time()
                frame_timestamp += 33

                mp_image = mp.Image(
                    image_format=mp.ImageFormat.SRGB,
                    data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB),
                )

                t0 = time.perf_counter()
                pose_result = pose_landmarker.detect_for_video(mp_image, frame_timestamp)
                inference_ms = (time.perf_counter() - t0) * 1000

                fps_counter += 1
                if now - fps_start >= 1.0:
                    fps_display = fps_counter / (now - fps_start)
                    fps_counter = 0
                    fps_start = now

                tracks = []
                if pose_result.pose_landmarks:
                    tracks = tracker.update(pose_result.pose_landmarks, w, h)

                for track_id, landmarks in tracks:
                    tracker.draw(frame, track_id, landmarks)

                cv2.putText(
                    frame, f"FPS: {fps_display:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2,
                )
                cv2.putText(
                    frame, f"Inference: {inference_ms:.1f}ms", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2,
                )
                cv2.putText(
                    frame, f"Poses: {len(tracks)}", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2,
                )
                smooth_text = (
                    f"Smooth: {tracker.smoothing_alpha:.1f}"
                    if tracker.smoothing_enabled else "Smooth: OFF"
                )
                cv2.putText(
                    frame, smooth_text, (10, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 0) if tracker.smoothing_enabled else (128, 128, 128), 2,
                )

                cv2.imshow("OAK-D Pose", frame)

            key = cv2.waitKey(1)
            if key == ord("q") or key == 27:
                pipeline.stop()
                break
            elif key == ord("s"):
                tracker.smoothing_enabled = not tracker.smoothing_enabled
                print(f"Smoothing: {'ON' if tracker.smoothing_enabled else 'OFF'}")
            elif key in (ord("+"), ord("=")):
                tracker.smoothing_alpha = min(0.9, tracker.smoothing_alpha + 0.1)
                print(f"Smoothing: {tracker.smoothing_alpha:.1f}")
            elif key == ord("-"):
                tracker.smoothing_alpha = max(0.0, tracker.smoothing_alpha - 0.1)
                print(f"Smoothing: {tracker.smoothing_alpha:.1f}")

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
