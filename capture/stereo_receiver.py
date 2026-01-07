#!/usr/bin/env python3
"""
Stereo Camera Receiver

Receives h264 streams from Android app via USB (adb forward).
Decodes and displays both camera feeds.

Usage:
    python stereo_receiver.py

Prerequisites:
    pip install opencv-python av numpy
    adb forward tcp:9556 tcp:9556 && adb forward tcp:9557 tcp:9557
"""

import socket
import struct
import threading
import subprocess
import sys
import time
from queue import Queue, Empty

import cv2
import numpy as np

# Try to import av for h264 decoding
try:
    import av
except ImportError:
    print("ERROR: PyAV not installed. Run: pip install av")
    sys.exit(1)


CAM0_PORT = 9556
CAM1_PORT = 9557
HOST = "127.0.0.1"


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
                # Convert to numpy BGR
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
                    # Read frame size (4 bytes, big-endian int)
                    size_data = self._recv_exact(sock, 4)
                    if not size_data:
                        break
                    frame_size = struct.unpack(">I", size_data)[0]

                    if frame_size > 10_000_000:  # Sanity check
                        print(f"[{self.name}] Invalid frame size: {frame_size}")
                        break

                    # Read frame data
                    frame_data = self._recv_exact(sock, frame_size)
                    if not frame_data:
                        break

                    # Decode
                    frame = self.decoder.decode(frame_data)
                    if frame is not None:
                        self.frame_count += 1
                        # Put in queue, drop old frames if full
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


def main():
    print("=" * 50)
    print("Stereo Camera Receiver")
    print("=" * 50)

    # Setup adb forward
    if not setup_adb_forward():
        return

    print("\nOn your phone:")
    print("  1. Open StereoVideoCamera app")
    print("  2. Press the STREAM button (blue)")
    print("\nWaiting for connection...\n")

    # Frame queues
    frame_queue = Queue(maxsize=10)

    # Start receivers
    cam0 = CameraReceiver("CAM0", CAM0_PORT, frame_queue)
    cam1 = CameraReceiver("CAM1", CAM1_PORT, frame_queue)
    cam0.start()
    cam1.start()

    # Display window
    cv2.namedWindow("Stereo Camera", cv2.WINDOW_NORMAL)

    frame0 = None
    frame1 = None
    fps_time = time.time()
    fps_count = 0

    try:
        while True:
            # Get frames from queue
            try:
                while True:
                    name, frame = frame_queue.get_nowait()
                    if name == "CAM0":
                        frame0 = frame
                    else:
                        frame1 = frame
                    fps_count += 1
            except Empty:
                pass

            # Display side-by-side
            if frame0 is not None or frame1 is not None:
                h = 540
                w = 960

                if frame0 is not None:
                    f0 = cv2.resize(frame0, (w, h))
                else:
                    f0 = np.zeros((h, w, 3), dtype=np.uint8)

                if frame1 is not None:
                    f1 = cv2.resize(frame1, (w, h))
                else:
                    f1 = np.zeros((h, w, 3), dtype=np.uint8)

                combined = np.hstack([f0, f1])

                # FPS counter
                if time.time() - fps_time > 1.0:
                    fps = fps_count / (time.time() - fps_time)
                    cv2.setWindowTitle(
                        "Stereo Camera", f"Stereo Camera - {fps:.1f} FPS"
                    )
                    fps_time = time.time()
                    fps_count = 0

                cv2.imshow("Stereo Camera", combined)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
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
