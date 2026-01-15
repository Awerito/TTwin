"""
OAK-D Configuration

Stores camera IP and other settings. Edit CAMERA_IP below or use --ip flag.

To find your camera's IP:
  1. Check router admin page (DHCP leases / LAN devices)
  2. Or run: python oak/discover.py --setup
  3. Factory fallback IP is 169.254.1.222 (if no DHCP)
"""

# Camera IP address - CHANGE THIS to your camera's IP
CAMERA_IP = "CHANGE_ME"  # e.g., "192.168.1.100" or "169.254.1.222"
