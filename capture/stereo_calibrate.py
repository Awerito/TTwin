#!/usr/bin/env python3
"""
Stereo Camera Calibration Capture

Captures synchronized frames from stereo cameras for calibration.
Detects checkerboard pattern and provides visual feedback.

Companion app: https://github.com/Awerito/StereoVideoCamera

================================================================================
SETUP
================================================================================

1. Install dependencies:
    pip install opencv-python av numpy

2. Enable USB Debugging on phone (once per device):
    - Settings > About phone > Tap "MIUI version" 7 times (enables Developer options)
    - Settings > Additional settings > Developer options
    - Enable "USB debugging"
    - Enable "USB debugging (Security settings)" <- IMPORTANT for Xiaomi

3. Connect phone via USB:
    adb devices                 # Should show "unauthorized"
    -> Phone shows popup "Allow USB debugging?" -> tap ALLOW (check "Always allow")
    adb devices                 # Now shows "device" instead of "unauthorized"
    adb forward tcp:9556 tcp:9556 && adb forward tcp:9557 tcp:9557

   Or via WiFi (if USB not possible):
    # First with USB connected and authorized:
    adb tcpip 5555
    adb shell ip addr show wlan0 | grep "inet "   # Get phone IP (e.g. 192.168.1.47)

    # Disconnect USB, then:
    adb connect 192.168.1.47:5555                  # Use your phone's IP
    adb devices                                     # Should show "device"
    adb forward tcp:9556 tcp:9556 && adb forward tcp:9557 tcp:9557

4. Generate checkerboard pattern:
    - Go to https://calib.io/pages/camera-calibration-pattern-generator
    - Select: Checkerboard, 9 columns, 6 rows (inner corners)
    - Download or display FULLSCREEN on a FLAT TV/monitor
    - Measure ONE square with a ruler (in cm) -> use with --square-size

================================================================================
USAGE
================================================================================

    python stereo_calibrate.py --square-size 4.9

    (4.9 cm is the square size on a 55" TV at fullscreen)

================================================================================
CALIBRATION TIPS
================================================================================

- Capture 15-20 image pairs minimum
- Move pattern to ALL areas of the image (corners, edges, center)
- Tilt the pattern at different angles (not just straight-on)
- Vary the distance from camera (close, medium, far)
- Auto-capture is ON by default (2 sec cooldown between captures)

================================================================================
CONTROLS
================================================================================

    SPACE - Capture current frames (only if checkerboard detected in both)
    a     - Toggle AUTO-CAPTURE mode (captures when pattern detected)
    c     - Capture anyway (even without detection)
    r     - Run calibration with captured images
    q/ESC - Quit

================================================================================
OUTPUT
================================================================================

    capture/calibration_images/cam0_XXX.png   - Left camera frames
    capture/calibration_images/cam1_XXX.png   - Right camera frames
    capture/calibration_images/stereo_calibration.npz  - Calibration result
"""

import argparse
import socket
import struct
import threading
import subprocess
import sys
import time
from pathlib import Path
from queue import Queue, Empty

import cv2
import numpy as np

try:
    import av
except ImportError:
    print("ERROR: PyAV not installed. Run: pip install av")
    sys.exit(1)


CAM0_PORT = 9556
CAM1_PORT = 9557
HOST = "127.0.0.1"

# Calibration output directory
CALIB_DIR = Path(__file__).parent / "calibration_images"


def setup_adb_forward():
    """Setup adb port forwarding."""
    print("Setting up adb forward...")
    try:
        subprocess.run(
            ["adb", "forward", f"tcp:{CAM0_PORT}", f"tcp:{CAM0_PORT}"],
            check=True,
            capture_output=True,
        )
        subprocess.run(
            ["adb", "forward", f"tcp:{CAM1_PORT}", f"tcp:{CAM1_PORT}"],
            check=True,
            capture_output=True,
        )
        print(f"  Port {CAM0_PORT} -> {CAM0_PORT}")
        print(f"  Port {CAM1_PORT} -> {CAM1_PORT}")
        return True
    except subprocess.CalledProcessError as e:
        print(f"ERROR: adb forward failed: {e}")
        return False
    except FileNotFoundError:
        print("ERROR: adb not found. Is Android SDK installed?")
        return False


class H264Decoder:
    """Decodes h264 NAL units to frames using PyAV."""

    def __init__(self):
        self.codec = av.CodecContext.create("h264", "r")
        self.codec.thread_type = "AUTO"

    def decode(self, nal_data: bytes) -> np.ndarray | None:
        """Decode a single NAL unit, return BGR frame or None."""
        try:
            packet = av.Packet(nal_data)
            frames = self.codec.decode(packet)
            for frame in frames:
                img = frame.to_ndarray(format="bgr24")
                return img
        except Exception:
            pass
        return None


class CameraReceiver(threading.Thread):
    """Receives h264 stream from one camera."""

    def __init__(self, name: str, port: int, frame_queue: Queue):
        super().__init__(daemon=True)
        self.name = name
        self.port = port
        self.frame_queue = frame_queue
        self.running = True
        self.connected = False
        self.decoder = H264Decoder()
        self.frame_count = 0

    def run(self):
        while self.running:
            try:
                print(f"[{self.name}] Connecting to {HOST}:{self.port}...")
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.connect((HOST, self.port))
                sock.settimeout(5.0)
                self.connected = True
                print(f"[{self.name}] Connected!")

                while self.running:
                    size_data = self._recv_exact(sock, 4)
                    if not size_data:
                        break
                    frame_size = struct.unpack(">I", size_data)[0]

                    if frame_size > 10_000_000:
                        print(f"[{self.name}] Invalid frame size: {frame_size}")
                        break

                    frame_data = self._recv_exact(sock, frame_size)
                    if not frame_data:
                        break

                    frame = self.decoder.decode(frame_data)
                    if frame is not None:
                        self.frame_count += 1
                        try:
                            self.frame_queue.put_nowait((self.name, frame))
                        except:
                            pass

            except socket.timeout:
                continue
            except Exception as e:
                print(f"[{self.name}] Error: {e}")
                self.connected = False
                time.sleep(1)

    def _recv_exact(self, sock: socket.socket, size: int) -> bytes | None:
        """Receive exactly size bytes."""
        data = b""
        while len(data) < size:
            try:
                chunk = sock.recv(size - len(data))
                if not chunk:
                    return None
                data += chunk
            except socket.timeout:
                if not self.running:
                    return None
        return data

    def stop(self):
        self.running = False


class CheckerboardDetector:
    """Detects checkerboard pattern in images."""

    def __init__(self, rows: int, cols: int):
        # rows/cols are INNER CORNERS, not squares
        self.pattern_size = (cols, rows)
        self.criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001,
        )

    def detect(self, img: np.ndarray) -> tuple[bool, np.ndarray | None]:
        """
        Detect checkerboard in image.
        Returns (found, corners) where corners is refined if found.
        """
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(
            gray,
            self.pattern_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH
            + cv2.CALIB_CB_NORMALIZE_IMAGE
            + cv2.CALIB_CB_FAST_CHECK,
        )

        if found:
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)

        return found, corners

    def draw(self, img: np.ndarray, found: bool, corners: np.ndarray | None):
        """Draw detected pattern on image."""
        if corners is not None:
            cv2.drawChessboardCorners(img, self.pattern_size, corners, found)
        return img


def run_calibration(rows: int, cols: int, square_size: float):
    """Run stereo calibration on captured images."""
    print("\n" + "=" * 50)
    print("Running Stereo Calibration")
    print("=" * 50)

    # Find captured images
    cam0_images = sorted(CALIB_DIR.glob("cam0_*.png"))
    cam1_images = sorted(CALIB_DIR.glob("cam1_*.png"))

    if len(cam0_images) != len(cam1_images):
        print(
            f"ERROR: Mismatched image counts: {len(cam0_images)} vs {len(cam1_images)}"
        )
        return

    if len(cam0_images) < 10:
        print(
            f"WARNING: Only {len(cam0_images)} image pairs. Recommend at least 15-20."
        )

    print(f"Found {len(cam0_images)} image pairs")

    # Prepare object points (same for all images)
    pattern_size = (cols, rows)
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_size  # Scale by actual square size in cm

    obj_points = []  # 3D points in real world
    img_points_0 = []  # 2D points in cam0
    img_points_1 = []  # 2D points in cam1

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    img_size = None

    print("Detecting corners in all images...")
    for i, (path0, path1) in enumerate(zip(cam0_images, cam1_images)):
        img0 = cv2.imread(str(path0))
        img1 = cv2.imread(str(path1))

        if img_size is None:
            img_size = (img0.shape[1], img0.shape[0])

        gray0 = cv2.cvtColor(img0, cv2.COLOR_BGR2GRAY)
        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)

        found0, corners0 = cv2.findChessboardCorners(gray0, pattern_size, None)
        found1, corners1 = cv2.findChessboardCorners(gray1, pattern_size, None)

        if found0 and found1:
            corners0 = cv2.cornerSubPix(gray0, corners0, (11, 11), (-1, -1), criteria)
            corners1 = cv2.cornerSubPix(gray1, corners1, (11, 11), (-1, -1), criteria)

            obj_points.append(objp)
            img_points_0.append(corners0)
            img_points_1.append(corners1)
            print(f"  [{i+1}/{len(cam0_images)}] OK")
        else:
            print(f"  [{i+1}/{len(cam0_images)}] SKIP (pattern not found)")

    if len(obj_points) < 5:
        print(f"ERROR: Only {len(obj_points)} valid pairs. Need at least 5.")
        return

    print(f"\nCalibrating with {len(obj_points)} valid image pairs...")

    # Calibrate each camera individually first
    print("  Calibrating CAM0...")
    ret0, K0, D0, rvecs0, tvecs0 = cv2.calibrateCamera(
        obj_points, img_points_0, img_size, None, None
    )
    print(f"    RMS error: {ret0:.4f}")

    print("  Calibrating CAM1...")
    ret1, K1, D1, rvecs1, tvecs1 = cv2.calibrateCamera(
        obj_points, img_points_1, img_size, None, None
    )
    print(f"    RMS error: {ret1:.4f}")

    # Stereo calibration
    print("  Running stereo calibration...")
    flags = cv2.CALIB_FIX_INTRINSIC
    ret_stereo, K0, D0, K1, D1, R, T, E, F = cv2.stereoCalibrate(
        obj_points,
        img_points_0,
        img_points_1,
        K0,
        D0,
        K1,
        D1,
        img_size,
        flags=flags,
        criteria=criteria,
    )
    print(f"    Stereo RMS error: {ret_stereo:.4f}")

    # Compute rectification transforms
    print("  Computing rectification...")
    R0, R1, P0, P1, Q, roi0, roi1 = cv2.stereoRectify(
        K0, D0, K1, D1, img_size, R, T, alpha=0
    )

    # Save calibration
    calib_file = CALIB_DIR / "stereo_calibration.npz"
    np.savez(
        calib_file,
        K0=K0,
        D0=D0,
        K1=K1,
        D1=D1,
        R=R,
        T=T,
        E=E,
        F=F,
        R0=R0,
        R1=R1,
        P0=P0,
        P1=P1,
        Q=Q,
        img_size=img_size,
        square_size=square_size,
    )
    print(f"\nCalibration saved to: {calib_file}")

    # Print summary
    baseline = np.linalg.norm(T)
    print("\n" + "=" * 50)
    print("CALIBRATION RESULTS")
    print("=" * 50)
    print(f"Image size: {img_size[0]}x{img_size[1]}")
    print(f"Stereo RMS error: {ret_stereo:.4f} pixels")
    print(f"Baseline: {baseline:.2f} cm")
    print(f"\nCAM0 Intrinsics (K0):")
    print(f"  fx = {K0[0,0]:.2f}, fy = {K0[1,1]:.2f}")
    print(f"  cx = {K0[0,2]:.2f}, cy = {K0[1,2]:.2f}")
    print(f"\nCAM1 Intrinsics (K1):")
    print(f"  fx = {K1[0,0]:.2f}, fy = {K1[1,1]:.2f}")
    print(f"  cx = {K1[0,2]:.2f}, cy = {K1[1,2]:.2f}")
    print(f"\nTranslation (T): [{T[0,0]:.2f}, {T[1,0]:.2f}, {T[2,0]:.2f}] cm")


def main():
    parser = argparse.ArgumentParser(description="Stereo camera calibration capture")
    parser.add_argument(
        "--rows", type=int, default=6, help="Inner corner rows (default: 6)"
    )
    parser.add_argument(
        "--cols", type=int, default=9, help="Inner corner cols (default: 9)"
    )
    parser.add_argument(
        "--square-size",
        type=float,
        default=2.5,
        help="Square size in cm (default: 2.5)",
    )
    args = parser.parse_args()

    print("=" * 50)
    print("Stereo Camera Calibration Capture")
    print("=" * 50)
    print(f"Pattern: {args.cols}x{args.rows} inner corners")
    print(f"Square size: {args.square_size} cm")

    # Create output directory
    CALIB_DIR.mkdir(exist_ok=True)
    print(f"Output: {CALIB_DIR}")

    # Setup adb forward
    if not setup_adb_forward():
        return

    print("\nOn your phone:")
    print("  1. Open StereoVideoCamera app")
    print("  2. Press the STREAM button (blue)")
    print("\nControls:")
    print("  SPACE - Capture (if pattern detected in both)")
    print("  c     - Force capture")
    print("  r     - Run calibration")
    print("  q/ESC - Quit")
    print("\nWaiting for connection...\n")

    # Frame queues
    frame_queue = Queue(maxsize=10)

    # Start receivers
    cam0 = CameraReceiver("CAM0", CAM0_PORT, frame_queue)
    cam1 = CameraReceiver("CAM1", CAM1_PORT, frame_queue)
    cam0.start()
    cam1.start()

    # Checkerboard detector
    detector = CheckerboardDetector(args.rows, args.cols)

    # Display window
    cv2.namedWindow("Stereo Calibration", cv2.WINDOW_NORMAL)

    frame0 = None
    frame1 = None
    corners0 = None
    corners1 = None
    found0 = False
    found1 = False
    capture_count = len(list(CALIB_DIR.glob("cam0_*.png")))

    detect_every_n = 10  # Only detect every N frames to avoid blocking
    frame_counter = 0
    auto_capture = True  # Auto-capture when pattern detected in both
    last_capture_time = 0
    auto_capture_cooldown = 2.0  # Seconds between auto-captures

    try:
        while True:
            # Get frames from queue (consume all, keep last)
            got_new_frame = False
            try:
                while True:
                    name, frame = frame_queue.get_nowait()
                    if name == "CAM0":
                        frame0 = frame.copy()
                    else:
                        frame1 = frame.copy()
                    got_new_frame = True
            except Empty:
                pass

            # Only run detection every N frames to avoid blocking display
            frame_counter += 1
            if got_new_frame and frame_counter >= detect_every_n:
                frame_counter = 0
                if frame0 is not None:
                    found0, corners0 = detector.detect(frame0)
                if frame1 is not None:
                    found1, corners1 = detector.detect(frame1)

            # Auto-capture when both patterns detected
            current_time = time.time()
            if (
                auto_capture
                and found0
                and found1
                and frame0 is not None
                and frame1 is not None
                and (current_time - last_capture_time) > auto_capture_cooldown
            ):
                cv2.imwrite(str(CALIB_DIR / f"cam0_{capture_count:03d}.png"), frame0)
                cv2.imwrite(str(CALIB_DIR / f"cam1_{capture_count:03d}.png"), frame1)
                print(f"[AUTO] Captured pair {capture_count}")
                capture_count += 1
                last_capture_time = current_time

            # Display
            if frame0 is not None or frame1 is not None:
                h = 540
                w = 960

                if frame0 is not None:
                    f0 = frame0.copy()
                    detector.draw(f0, found0, corners0)
                    # Status indicator
                    color = (0, 255, 0) if found0 else (0, 0, 255)
                    cv2.circle(f0, (30, 30), 15, color, -1)
                    cv2.putText(
                        f0,
                        "CAM0",
                        (50, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (255, 255, 255),
                        2,
                    )
                    f0 = cv2.resize(f0, (w, h))
                else:
                    f0 = np.zeros((h, w, 3), dtype=np.uint8)

                if frame1 is not None:
                    f1 = frame1.copy()
                    detector.draw(f1, found1, corners1)
                    color = (0, 255, 0) if found1 else (0, 0, 255)
                    cv2.circle(f1, (30, 30), 15, color, -1)
                    cv2.putText(
                        f1,
                        "CAM1",
                        (50, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (255, 255, 255),
                        2,
                    )
                    f1 = cv2.resize(f1, (w, h))
                else:
                    f1 = np.zeros((h, w, 3), dtype=np.uint8)

                combined = np.hstack([f0, f1])

                # Info bar
                status = "READY" if (found0 and found1) else "Detecting..."
                auto_str = "AUTO ON" if auto_capture else "AUTO OFF"
                cv2.putText(
                    combined,
                    f"Captures: {capture_count} | {status} | {auto_str} | a=toggle auto, r=calibrate, q=quit",
                    (10, h - 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2,
                )

                cv2.imshow("Stereo Calibration", combined)

            key = cv2.waitKey(1) & 0xFF

            # SPACE - capture if pattern detected
            if key == ord(" "):
                if found0 and found1 and frame0 is not None and frame1 is not None:
                    cv2.imwrite(
                        str(CALIB_DIR / f"cam0_{capture_count:03d}.png"), frame0
                    )
                    cv2.imwrite(
                        str(CALIB_DIR / f"cam1_{capture_count:03d}.png"), frame1
                    )
                    print(f"Captured pair {capture_count}")
                    capture_count += 1
                else:
                    print("Pattern not detected in both cameras!")

            # a - toggle auto capture
            elif key == ord("a"):
                auto_capture = not auto_capture
                print(f"Auto-capture: {'ON' if auto_capture else 'OFF'}")

            # c - force capture
            elif key == ord("c"):
                if frame0 is not None and frame1 is not None:
                    cv2.imwrite(
                        str(CALIB_DIR / f"cam0_{capture_count:03d}.png"), frame0
                    )
                    cv2.imwrite(
                        str(CALIB_DIR / f"cam1_{capture_count:03d}.png"), frame1
                    )
                    print(f"Force captured pair {capture_count}")
                    capture_count += 1

            # r - run calibration
            elif key == ord("r"):
                if capture_count >= 5:
                    run_calibration(args.rows, args.cols, args.square_size)
                else:
                    print(f"Need at least 5 captures (have {capture_count})")

            # q/ESC - quit
            elif key == ord("q") or key == 27:
                break

            time.sleep(0.001)

    except KeyboardInterrupt:
        pass
    finally:
        print("\nShutting down...")
        cam0.stop()
        cam1.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
