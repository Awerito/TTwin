#!/usr/bin/env python3
# WARN: script not tested
"""
OAK-D Discovery & Network Setup Tool

Finds OAK cameras on the network and helps configure their IP address.

Usage:
    python oak/discover.py              # Scan and show found cameras
    python oak/discover.py --setup      # Interactive IP configuration

How OAK PoE cameras are discovered:
    1. depthai sends UDP broadcast on port 11491
    2. Cameras respond with their IP and device info
    3. If camera is on different subnet, broadcast won't reach it

If camera is not found:
    1. Check router's DHCP/LAN list for unknown devices
    2. Add secondary IP to your interface to reach that subnet
    3. Run this script again
"""

import argparse
import subprocess
import sys

import depthai as dai

from config import CAMERA_IP


def scan_network_for_devices():
    """Scan for OAK devices using depthai's built-in discovery."""
    print("Scanning for OAK devices...")
    print("(This uses UDP broadcast - camera must be on same subnet)\n")

    devices = dai.Device.getAllAvailableDevices()

    if not devices:
        print("No devices found via depthai discovery.\n")
        return []

    print(f"Found {len(devices)} device(s):\n")
    for d in devices:
        print(f"  MxId: {d.getMxId()}")
        print(f"  State: {d.state.name}")
        print(f"  Protocol: {d.protocol.name}")
        print()

    return devices


def try_direct_connection(ip: str) -> bool:
    """Try connecting directly to a known IP."""
    print(f"Trying direct connection to {ip}...")

    # First try ping
    result = subprocess.run(
        ["ping", "-c", "1", "-W", "1", ip],
        capture_output=True,
        text=True,
    )

    if result.returncode != 0:
        print(f"  ✗ Cannot ping {ip}")
        return False

    print(f"  ✓ Ping successful")

    # Try depthai connection
    try:
        device_info = dai.DeviceInfo(ip)
        with dai.Device(device_info) as device:
            print(f"  ✓ depthai connection successful")
            print(f"    Device: {device.getDeviceName()}")
            print(f"    ID: {device.getDeviceId()}")
            return True
    except Exception as e:
        print(f"  ✗ depthai connection failed: {e}")
        return False


def get_local_subnets():
    """Get list of subnets the PC can reach."""
    result = subprocess.run(
        ["ip", "addr", "show"],
        capture_output=True,
        text=True,
    )

    subnets = []
    for line in result.stdout.split("\n"):
        if "inet " in line and "127.0.0.1" not in line:
            parts = line.strip().split()
            for i, p in enumerate(parts):
                if p == "inet":
                    ip_cidr = parts[i + 1]
                    subnets.append(ip_cidr)
    return subnets


def add_secondary_ip(interface: str, ip_cidr: str):
    """Add a secondary IP to an interface."""
    print(f"\nAdding {ip_cidr} to {interface}...")
    print(f"Run: sudo ip addr add {ip_cidr} dev {interface}")

    result = subprocess.run(
        ["sudo", "ip", "addr", "add", ip_cidr, "dev", interface],
        capture_output=True,
        text=True,
    )

    if result.returncode == 0:
        print("  ✓ Success")
        return True
    elif "RTNETLINK answers: File exists" in result.stderr:
        print("  ✓ Already configured")
        return True
    else:
        print(f"  ✗ Failed: {result.stderr}")
        return False


def interactive_setup():
    """Interactive setup to find and configure camera."""
    print("=" * 60)
    print("OAK-D Network Setup Wizard")
    print("=" * 60)
    print()

    # Step 1: Check current subnets
    print("Step 1: Your current network configuration")
    print("-" * 40)
    subnets = get_local_subnets()
    for s in subnets:
        print(f"  {s}")
    print()

    # Step 2: Try depthai discovery
    print("Step 2: Searching for OAK cameras")
    print("-" * 40)
    devices = scan_network_for_devices()

    if devices:
        print("Camera found! You're all set.")
        return

    # Step 3: Ask for camera IP
    print("Step 3: Manual configuration")
    print("-" * 40)
    print("Camera not found via broadcast. This usually means it's on a")
    print("different subnet than your PC.")
    print()
    print("To find your camera's IP:")
    print("  1. Check your router's admin page (DHCP leases / LAN devices)")
    print("  2. Look for a device with MAC starting with common OAK prefixes")
    print("  3. Or try factory fallback IP: 169.254.1.222")
    print()

    camera_ip = input(f"Enter camera IP (or press Enter to try {CAMERA_IP}): ").strip()
    if not camera_ip:
        camera_ip = CAMERA_IP

    # Step 4: Check if we can reach that subnet
    camera_subnet = ".".join(camera_ip.split(".")[:3])
    pc_subnets = [".".join(s.split("/")[0].split(".")[:3]) for s in subnets]

    if camera_subnet not in pc_subnets:
        print(f"\nCamera subnet ({camera_subnet}.x) is different from your PC's subnets.")
        print("Adding a secondary IP to reach the camera...")

        # Find the main ethernet interface
        result = subprocess.run(
            ["ip", "route", "get", "8.8.8.8"],
            capture_output=True,
            text=True,
        )
        interface = "enp9s0"  # default
        for part in result.stdout.split():
            if part.startswith("en") or part.startswith("eth"):
                interface = part
                break

        secondary_ip = f"{camera_subnet}.1/24"
        add_secondary_ip(interface, secondary_ip)

    # Step 5: Try connection
    print()
    print("Step 4: Testing connection")
    print("-" * 40)
    if try_direct_connection(camera_ip):
        print()
        print("=" * 60)
        print("SUCCESS! Camera is accessible.")
        print("=" * 60)
        print()
        print("To use the camera:")
        print(f"  python oak/streams.py --ip {camera_ip}")
        print()
        print("Note: The secondary IP is temporary. After reboot, run:")
        print(f"  sudo ip addr add {camera_subnet}.1/24 dev {interface}")
        print("Or run this script again with --setup")
    else:
        print()
        print("=" * 60)
        print("FAILED: Could not connect to camera.")
        print("=" * 60)
        print()
        print("Troubleshooting:")
        print("  1. Verify camera has power (check LEDs)")
        print("  2. Verify ethernet cable is connected")
        print("  3. Check the IP address is correct")
        print("  4. Try a different IP")


def main():
    parser = argparse.ArgumentParser(
        description="Discover OAK cameras and configure network"
    )
    parser.add_argument(
        "--setup",
        action="store_true",
        help="Run interactive setup wizard",
    )
    parser.add_argument(
        "--ip",
        help="Try connecting to specific IP",
    )
    args = parser.parse_args()

    if args.setup:
        interactive_setup()
    elif args.ip:
        try_direct_connection(args.ip)
    else:
        # Default: just scan
        devices = scan_network_for_devices()
        if not devices:
            print("Tip: If camera is on different subnet, run with --setup")
            print("     or specify IP directly with --ip <CAMERA_IP>")


if __name__ == "__main__":
    main()
