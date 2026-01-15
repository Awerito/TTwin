#!/usr/bin/env python3
"""
OAK-D Pro W PoE - Connection Test

Tests connection to the OAK-D camera and prints device info.

Setup (if camera has static IP in different subnet):
    sudo ip addr add 192.168.18.1/24 dev enp9s0

Usage:
    python oak/connect.py [--ip 192.168.18.103]
"""

import argparse
import depthai as dai

from config import CAMERA_IP


def main():
    parser = argparse.ArgumentParser(description="Test OAK-D connection")
    parser.add_argument(
        "--ip",
        default=CAMERA_IP,
        help=f"Camera IP address (default from config: {CAMERA_IP})",
    )
    args = parser.parse_args()

    print(f"Connecting to OAK-D at {args.ip}...")
    print()

    device_info = dai.DeviceInfo(args.ip)

    try:
        with dai.Device(device_info) as device:
            print("✓ CONNECTED!")
            print()
            print(f"  DeviceId:   {device.getDeviceId()}")
            print(f"  Device:     {device.getDeviceName()}")
            print(f"  Cameras:    {device.getCameraSensorNames()}")
            print()

            # Read calibration
            calib = device.readCalibration()
            if calib:
                print("  Calibration: Available (factory calibrated)")
                # Get intrinsics for left mono camera
                intrinsics = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B)
                print(f"  Left cam fx: {intrinsics[0][0]:.1f} px")
                print(f"  Left cam fy: {intrinsics[1][1]:.1f} px")
                # Get baseline
                baseline = calib.getBaselineDistance(
                    dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C
                )
                print(f"  Baseline:    {baseline:.2f} cm")
            else:
                print("  Calibration: Not found")

    except Exception as e:
        print(f"✗ Connection failed: {e}")
        print()
        print("Troubleshooting:")
        print("  1. Check camera LED is on (power from PoE switch)")
        print("  2. Ping the camera: ping 192.168.18.103")
        print("  3. If ping fails, add IP route:")
        print("     sudo ip addr add 192.168.18.1/24 dev enp9s0")
        return 1

    return 0


if __name__ == "__main__":
    exit(main())
