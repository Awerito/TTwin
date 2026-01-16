#!/usr/bin/env python3
"""
OAK-D Raw Capture - Maximum Quality Recording

PURPOSE: Record raw RGB + Depth at maximum quality for offline processing.
         NO inference during capture - just pure data.

Output Structure:
    recordings/<timestamp>/
    ├── rgb/           # JPEG frames (95% quality)
    │   ├── 000000.jpg
    │   └── ...
    ├── depth/         # 16-bit PNG (lossless, millimeters)
    │   ├── 000000.png
    │   └── ...
    ├── timestamps.csv # Frame timestamps for sync
    ├── intrinsics.json # Camera calibration data
    └── metadata.json   # Recording session info

Camera Settings:
- RGB: 1280x800 (tested maximum with depth)
- Depth: 1280x800 aligned to RGB
- IR Dot Projector: ON (0.5 intensity)
- Stereo: Left-right check, subpixel enabled

Controls:
    r - Start/stop recording
    q - Quit

Usage:
    python oak/capture_raw.py
    python oak/capture_raw.py --output recordings/my_session
"""

import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path
from queue import Queue, Empty
from threading import Thread, Event

sys.path.insert(0, str(Path(__file__).parent))
from config import CAMERA_IP

import cv2
import depthai as dai
import numpy as np


# =============================================================================
# RECORDING SETTINGS - Maximum tested resolution with depth
# =============================================================================
RGB_WIDTH = 1280
RGB_HEIGHT = 800

# Preview for validation (small, just to see what's happening)
PREVIEW_SCALE = 0.5


class DiskWriter:
    """Threaded disk writer to avoid blocking capture."""

    def __init__(self, output_dir: Path):
        self.output_dir = output_dir
        self.rgb_dir = output_dir / "rgb"
        self.depth_dir = output_dir / "depth"
        self.queue = Queue(maxsize=120)  # ~2 seconds buffer at 60fps
        self.stop_event = Event()
        self.thread = None
        self.frames_written = 0
        self.frames_dropped = 0

    def start(self):
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.rgb_dir.mkdir(exist_ok=True)
        self.depth_dir.mkdir(exist_ok=True)
        self.stop_event.clear()
        self.frames_written = 0
        self.frames_dropped = 0
        self.thread = Thread(target=self._writer_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=5.0)

    def enqueue(self, frame_id: int, rgb: np.ndarray, depth: np.ndarray):
        """Add frame to write queue. Returns True if queued, False if dropped."""
        try:
            self.queue.put_nowait((frame_id, rgb, depth))
            return True
        except:
            self.frames_dropped += 1
            return False

    def _writer_loop(self):
        while not self.stop_event.is_set() or not self.queue.empty():
            try:
                frame_id, rgb, depth = self.queue.get(timeout=0.1)

                frame_str = f"{frame_id:06d}"

                # Save RGB as JPEG (fast, good quality)
                cv2.imwrite(
                    str(self.rgb_dir / f"{frame_str}.jpg"),
                    rgb,
                    [cv2.IMWRITE_JPEG_QUALITY, 95],
                )

                # Save depth as 16-bit PNG (lossless)
                cv2.imwrite(str(self.depth_dir / f"{frame_str}.png"), depth)

                self.frames_written += 1

            except Empty:
                continue


def main():
    parser = argparse.ArgumentParser(description="OAK-D Raw Capture")
    parser.add_argument(
        "--output",
        "-o",
        type=str,
        default=None,
        help="Output directory (default: recordings/<timestamp>)",
    )
    args = parser.parse_args()

    # Output directory
    if args.output:
        output_dir = Path(args.output)
    else:
        output_dir = Path(f"recordings/{datetime.now().strftime('%Y%m%d_%H%M%S')}")

    print(f"Output: {output_dir}")
    print(f"Connecting to {CAMERA_IP}...")

    device_info = dai.DeviceInfo(CAMERA_IP)

    with dai.Pipeline() as pipeline:
        device = pipeline.getDefaultDevice()
        print(f"Connected: {device.getDeviceName()}")

        # =============================================================
        # CAMERAS
        # =============================================================
        cam_rgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        cam_left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        cam_right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

        # =============================================================
        # STEREO DEPTH
        # =============================================================
        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
        stereo.setRectifyEdgeFillColor(0)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)

        # Link stereo cameras
        cam_left.requestOutput((RGB_WIDTH, RGB_HEIGHT)).link(stereo.left)
        cam_right.requestOutput((RGB_WIDTH, RGB_HEIGHT)).link(stereo.right)

        # =============================================================
        # OUTPUTS
        # =============================================================
        rgb_out = cam_rgb.requestOutput((RGB_WIDTH, RGB_HEIGHT), dai.ImgFrame.Type.BGR888i)

        # Align depth to RGB
        rgb_out.link(stereo.inputAlignTo)

        # Queues - larger for recording
        q_rgb = rgb_out.createOutputQueue(maxSize=4, blocking=False)
        q_depth = stereo.depth.createOutputQueue(maxSize=4, blocking=False)

        # =============================================================
        # IR PROJECTOR
        # =============================================================
        device.setIrLaserDotProjectorIntensity(0.5)
        print("IR Dot Projector: ON (0.5)")

        # =============================================================
        # CAMERA INTRINSICS
        # =============================================================
        calib = device.readCalibration()
        intrinsics = calib.getCameraIntrinsics(
            dai.CameraBoardSocket.CAM_A, RGB_WIDTH, RGB_HEIGHT
        )
        fx, fy = intrinsics[0][0], intrinsics[1][1]
        cx, cy = intrinsics[0][2], intrinsics[1][2]
        print(f"Intrinsics: fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}")

        # =============================================================
        # START PIPELINE
        # =============================================================
        pipeline.start()

        print()
        print("=" * 60)
        print(f"Resolution: {RGB_WIDTH}x{RGB_HEIGHT}")
        print(f"Depth: Aligned to RGB, subpixel ON")
        print(f"IR: ON")
        print("=" * 60)
        print()
        print("Controls:")
        print("  r - Start/stop recording")
        print("  q - Quit")
        print()

        # State
        writer = DiskWriter(output_dir)
        recording = False
        frame_count = 0
        rec_start_time = None
        timestamps = []

        # FPS tracking
        fps_counter = 0
        fps_time = time.time()
        fps_display = 0

        # Depth cache (non-blocking)
        depth_frame = None

        while True:
            # FPS
            fps_counter += 1
            now = time.time()
            if now - fps_time >= 1.0:
                fps_display = fps_counter
                fps_counter = 0
                fps_time = now

            # Get frames
            in_rgb = q_rgb.tryGet()
            in_depth = q_depth.tryGet()

            # Cache depth when available
            if in_depth is not None:
                depth_frame = in_depth.getFrame()

            # Need RGB to continue
            if in_rgb is None:
                time.sleep(0.001)
                continue

            rgb_frame = in_rgb.getCvFrame()
            ts = time.time()

            # Record if active and depth is available
            if recording and depth_frame is not None:
                queued = writer.enqueue(frame_count, rgb_frame, depth_frame)
                if queued:
                    timestamps.append((frame_count, ts))
                    frame_count += 1

            # Preview (downscaled)
            preview_w = int(RGB_WIDTH * PREVIEW_SCALE)
            preview_h = int(RGB_HEIGHT * PREVIEW_SCALE)
            preview = cv2.resize(rgb_frame, (preview_w, preview_h))

            # Status overlay
            if recording:
                elapsed = ts - rec_start_time
                status = f"[REC] {frame_count} frames | {elapsed:.1f}s"
                status_color = (0, 0, 255)
            else:
                status = "[STANDBY] Press 'r' to record"
                status_color = (200, 200, 200)

            cv2.putText(
                preview, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2
            )
            cv2.putText(
                preview,
                f"FPS: {fps_display} | Res: {RGB_WIDTH}x{RGB_HEIGHT}",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
            )

            if recording:
                dropped = writer.frames_dropped
                written = writer.frames_written
                queued = writer.queue.qsize()
                cv2.putText(
                    preview,
                    f"Written: {written} | Queue: {queued} | Dropped: {dropped}",
                    (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 255),
                    1,
                )

            cv2.imshow("Preview (raw capture)", preview)

            # Depth preview
            if depth_frame is not None:
                depth_preview = cv2.resize(depth_frame, (preview_w, preview_h))
                depth_color = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_preview, alpha=0.03), cv2.COLORMAP_JET
                )
                cv2.imshow("Depth", depth_color)

            # Keyboard
            key = cv2.waitKey(1)
            if key == ord("q"):
                break
            elif key == ord("r"):
                if recording:
                    # Stop recording
                    recording = False
                    writer.stop()

                    elapsed = time.time() - rec_start_time
                    avg_fps = frame_count / elapsed if elapsed > 0 else 0

                    # Save timestamps
                    ts_file = output_dir / "timestamps.csv"
                    with open(ts_file, "w") as f:
                        f.write("frame_id,timestamp\n")
                        for fid, t in timestamps:
                            f.write(f"{fid},{t:.6f}\n")

                    # Save intrinsics
                    intrinsics_file = output_dir / "intrinsics.json"
                    with open(intrinsics_file, "w") as f:
                        json.dump(
                            {
                                "fx": fx,
                                "fy": fy,
                                "cx": cx,
                                "cy": cy,
                                "width": RGB_WIDTH,
                                "height": RGB_HEIGHT,
                            },
                            f,
                            indent=2,
                        )

                    # Save metadata
                    metadata_file = output_dir / "metadata.json"
                    with open(metadata_file, "w") as f:
                        json.dump(
                            {
                                "frames": frame_count,
                                "elapsed_seconds": elapsed,
                                "avg_fps": avg_fps,
                                "frames_dropped": writer.frames_dropped,
                                "rgb_resolution": [RGB_WIDTH, RGB_HEIGHT],
                                "depth_resolution": [RGB_WIDTH, RGB_HEIGHT],
                                "ir_projector": 0.5,
                                "stereo_subpixel": True,
                                "timestamp": datetime.now().isoformat(),
                            },
                            f,
                            indent=2,
                        )

                    print()
                    print("=" * 60)
                    print(f"Recording stopped")
                    print(f"Frames: {frame_count}")
                    print(f"Duration: {elapsed:.1f}s")
                    print(f"Avg FPS: {avg_fps:.1f}")
                    print(f"Dropped: {writer.frames_dropped}")
                    print(f"Output: {output_dir}")
                    print("=" * 60)
                    print()

                else:
                    # Start recording
                    recording = True
                    frame_count = 0
                    timestamps = []
                    rec_start_time = time.time()
                    writer = DiskWriter(output_dir)
                    writer.start()
                    print(f"Recording to {output_dir}...")

        # Cleanup
        if recording:
            writer.stop()
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
