# OAK-D Pro W PoE Integration

Stereo depth camera integration for the TTideas table tennis simulator.

## Hardware

**Camera:** OAK-D Pro W PoE (Luxonis)
- Stereo baseline: 7.5 cm
- RGB sensor: IMX378 (12MP)
- Mono sensors: OV9282 x2 (stereo pair)
- FOV: 150° stereo, 120° color (Wide variant)
- IR dot projector for active stereo
- Connection: PoE (Power over Ethernet)
- Factory calibrated

## Network Setup

The camera may be pre-configured with a static IP in a different subnet than your PC:

```
Camera IP: <CAMERA_IP>       # Find this (see below)
PC IP:     <YOUR_PC_IP>      # Your network (e.g., 192.168.1.x)
```

### Physical Connection

```
[Router/Internet] ──► [Switch PoE port 5 (uplink)]
                              │
                    ┌─────────┴─────────┐
                    │                   │
            [PC port 1-4]        [OAK-D port 1-4]
            (gets data only)     (gets PoE power + data)
```

**Hardware used:**
- Tenda 5-port Gigabit Switch with 4-port PoE
- Standard Cat5e/Cat6 ethernet cables

### Finding the Camera IP

OAK PoE cameras by default use **DHCP** (get IP from router). If DHCP fails, they fall back to **169.254.1.222**.

If someone configured a static IP (like in this case), you need to find it:

**Method 1: Check Router Admin Page**
1. Open your router's admin page (usually http://192.168.1.1)
2. Look for "LAN Devices", "DHCP Leases", or "Connected Devices"
3. Find the device by MAC address (OAK devices often show no hostname)
4. Note the IP address

**Method 2: Network Scan (if on same subnet)**
```bash
# Scan local network
nmap -sn 192.168.1.0/24

# Or use depthai discovery
python -c "import depthai as dai; print(dai.Device.getAllAvailableDevices())"
```

**Method 3: Try Common IPs**
```bash
# Factory fallback IP (when no DHCP available)
ping 169.254.1.222
```

### Connecting to Different Subnet

If the camera is on a different subnet than your PC, add a secondary IP:

```bash
# Find your ethernet interface name
ip link show  # Look for enp*, eth*, etc.

# Add secondary IP in camera's subnet
# Example: if camera is 192.168.18.103, use 192.168.18.1
sudo ip addr add <CAMERA_SUBNET>.1/24 dev <YOUR_INTERFACE>

# Verify
ip addr show <YOUR_INTERFACE> | grep inet

# Test connectivity
ping <CAMERA_IP>
```

**Note:** This is temporary. After reboot you need to run the command again, or:
1. Reconfigure the camera's IP to your subnet (risk of soft-brick if done wrong)
2. Add persistent secondary IP via NetworkManager

## Quick Start

```bash
# 1. Find camera IP (see "Finding the Camera IP" above)

# 2. Edit config.py with your camera's IP
nano oak/config.py

# 3. Add secondary IP if camera is on different subnet
sudo ip addr add <CAMERA_SUBNET>.1/24 dev <YOUR_INTERFACE>

# 4. View streams
python oak/streams.py
```

## Configuration

Edit `oak/config.py` to set your camera's IP:

```python
CAMERA_IP = "<YOUR_CAMERA_IP>"  # Change to your camera's IP
```

All scripts read from this config. You can also override with `--ip`:
```bash
python oak/streams.py --ip <CAMERA_IP>
```

## Scripts

### discover.py - Discovery & Setup Wizard

Finds cameras and helps configure network access:

```bash
# Scan for cameras (only works if same subnet)
python oak/discover.py

# Interactive setup wizard (recommended for first time)
python oak/discover.py --setup

# Test specific IP
python oak/discover.py --ip <CAMERA_IP>
```

The wizard will:
1. Show your current network configuration
2. Scan for OAK cameras via UDP broadcast
3. If not found, guide you to add a secondary IP
4. Test the connection

### connect.py - Connection Test

Tests connection and displays device info:

```bash
python oak/connect.py [--ip <CAMERA_IP>]
```

Output:
```
✓ CONNECTED!
  Device:     OAK-D-PRO-W-POE
  Cameras:    CAM_A: IMX378, CAM_B: OV9282, CAM_C: OV9282
  Calibration: Available (factory calibrated)
```

### streams.py - RGB + Depth Viewer

Live viewer for RGB and depth streams:

```bash
python oak/streams.py [--ip <CAMERA_IP>]
```

**Controls:**
- `q` or `ESC` - Quit

**Depth colormap:**
- Black = no data (too close/far)
- Blue = close (~20cm)
- Green/Yellow = medium distance
- Red = far

## depthai v3 API Notes

This project uses **depthai 3.2.1** which has a significantly different API from v2:

### Pipeline Creation
```python
# v3: Device first, then Pipeline with device
device = dai.Device(dai.DeviceInfo(ip))
pipeline = dai.Pipeline(device)
```

### Camera Node
```python
# v3: Use Camera.build() instead of setBoardSocket()
cam = pipeline.create(dai.node.Camera).build(
    boardSocket=dai.CameraBoardSocket.CAM_A
)

# Request output with specific size/format
output = cam.requestOutput(
    size=(960, 540),
    type=dai.ImgFrame.Type.BGR888p,
    fps=30
)
```

### Stereo Depth
```python
# v3: Auto-create mono cameras
stereo = pipeline.create(dai.node.StereoDepth).build(
    autoCreateCameras=True,
    size=(640, 400),
    fps=30
)

# Output size must be multiple of 16
stereo.setOutputSize(640, 400)
```

### Output Queues
```python
# v3: Create queue directly from output
queue = output.createOutputQueue()

# No more XLinkOut nodes needed
```

### Main Loop
```python
pipeline.start()
while pipeline.isRunning():
    frame = queue.tryGet()
    if frame:
        cv_frame = frame.getCvFrame()
```

## Troubleshooting

### Camera not detected
1. Check LEDs on camera (power + data should be on)
2. Verify PoE switch is providing power
3. Add secondary IP: `sudo ip addr add <CAMERA_SUBNET>.1/24 dev <YOUR_INTERFACE>`
4. Test ping: `ping <CAMERA_IP>`

### "Width must be multiple of 16" error
When aligning depth to RGB, the output size must be multiple of 16:
```python
stereo.setOutputSize(640, 400)  # Both divisible by 16
```

### Connection drops / X_LINK_ERROR
- Check ethernet cable quality
- Reduce resolution or FPS
- Ensure stable PoE power delivery

## Camera Specifications

| Spec | Value |
|------|-------|
| Baseline | 7.5 cm |
| RGB Resolution | Up to 4056x3040 (12MP) |
| Mono Resolution | 1280x800 (native), 640x400 (binned) |
| Depth Range | ~20cm - 10m+ |
| FPS | Up to 60fps (resolution dependent) |
| FOV (stereo) | 150° DFOV |
| FOV (color) | 120° DFOV |
| IR Illumination | Dot projector + flood |
| Power | PoE 802.3af (up to 7.5W) |
| Enclosure | IP65 rated |

## Dependencies

```
depthai>=3.0.0
opencv-python
numpy
```
