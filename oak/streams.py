#!/usr/bin/env python3
"""
OAK-D Pro W PoE - RGB + Depth Stream Viewer

Displays RGB and depth streams from the OAK-D camera.

Setup (if camera has static IP in different subnet):
    sudo ip addr add 192.168.18.1/24 dev enp9s0

Usage:
    python oak/streams.py [--ip 192.168.18.103]

Controls:
    q / ESC - Quit
"""

import argparse
import cv2
import depthai as dai
import numpy as np

from config import CAMERA_IP


def colorize_depth(depth_frame, max_depth_mm=10000):
    """Convert depth frame to colorized visualization."""
    depth_normalized = np.clip(depth_frame / max_depth_mm * 255, 0, 255).astype(
        np.uint8
    )
    depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
    depth_colored[depth_frame == 0] = [0, 0, 0]
    return depth_colored


def main():
    parser = argparse.ArgumentParser(description="OAK-D RGB + Depth viewer")
    parser.add_argument(
        "--ip",
        default=CAMERA_IP,
        help=f"Camera IP address (default from config: {CAMERA_IP})",
    )
    args = parser.parse_args()

    print(f"Connecting to OAK-D at {args.ip}...")

    device_info = dai.DeviceInfo(args.ip)

    try:
        # Connect to device first, then create pipeline with it (v3 API)
        device = dai.Device(device_info)
        pipeline = dai.Pipeline(device)

        print(f"✓ Connected to {device.getDeviceName()}")

        # RGB Camera
        cam_rgb = pipeline.create(dai.node.Camera).build(
            boardSocket=dai.CameraBoardSocket.CAM_A
        )
        rgb_out = cam_rgb.requestOutput(
            size=(960, 540), type=dai.ImgFrame.Type.BGR888p, fps=30
        )

        # Stereo depth (auto-creates mono cameras)
        stereo = pipeline.create(dai.node.StereoDepth).build(
            autoCreateCameras=True, size=(640, 400), fps=30
        )
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)
        # Output size must be multiple of 16
        stereo.setOutputSize(640, 400)

        # Create output queues
        q_rgb = rgb_out.createOutputQueue()
        q_depth = stereo.depth.createOutputQueue()

        print("Starting streams...")
        print("Press 'q' or ESC to quit")
        print()

        pipeline.start()

        while pipeline.isRunning():
            in_rgb = q_rgb.tryGet()
            in_depth = q_depth.tryGet()

            if in_rgb is not None:
                frame_rgb = in_rgb.getCvFrame()
                cv2.imshow("RGB", frame_rgb)

            if in_depth is not None:
                depth_frame = in_depth.getFrame()
                depth_colored = colorize_depth(depth_frame)
                cv2.imshow("Depth", depth_colored)

            key = cv2.waitKey(1)
            if key == ord("q") or key == 27:
                pipeline.stop()
                break

    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback

        traceback.print_exc()
        return 1
    finally:
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    exit(main())
