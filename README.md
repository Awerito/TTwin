# 🏓 TTwin - Table Tennis Digital Twin

A physics-accurate table tennis simulator built with Rust and Python. The name
**TTwin** combines **TT** (Table Tennis) + **Twin** (Digital Twin) - a virtual
replica of real-world table tennis physics.

![3D View](https://img.shields.io/badge/3D-Ursina-blue)
![Physics](https://img.shields.io/badge/Physics-Rust-orange)
![Python](https://img.shields.io/badge/Bindings-Python%203.12-green)

## Features

- **Real Physics Engine** - Written in Rust for performance
- **Magnus Effect** - Spin actually curves the ball
- **Topspin/Backspin/Sidespin** - Different spins behave differently
- **Table & Net Collision** - Accurate bounce physics
- **Multiple Viewers** - 2D (pygame) and 3D (Ursina) visualization
- **Stereo Camera Capture** - 3D tracking via dual-camera phone setup
- **Slow Motion** - See the physics in detail

## System Overview

```
┌─────────────────────────┐         ┌─────────────────────────┐
│   Depth Camera          │   PoE   │        TTwin            │
│   (OAK-D Pro W PoE)     │ ──────► │   (Physics Simulator)   │
│                         │         │                         │
│ • Stereo depth          │         │ • 3D body tracking      │
│ • RGB 12MP @ 30fps      │         │ • Physics simulation    │
│ • Factory calibrated    │         │ • Real-time rendering   │
└─────────────────────────┘         └─────────────────────────┘
        Luxonis                            PC (Linux)
```

📷 **Primary Camera:** OAK-D Pro W PoE (see [oak/README.md](oak/README.md))

📱 **Alternative:** [StereoVideoCamera](https://github.com/Awerito/StereoVideoCamera)
Android app for dual-camera capture (requires manual calibration)

## Quick Start

```bash
# Clone and setup
git clone https://github.com/Awerito/TTwin
cd TTwin
python -m venv env
source env/bin/activate
pip install -r requirements.txt

# Build Rust physics engine
cd tt-core
maturin develop
cd ..

# Run 3D viewer
python viewers/view_3d.py

# Or 2D side view
python viewers/side_view.py
```

## Viewers

### 3D View (`viewers/view_3d.py`)

Full 3D visualization with orbit camera.

**Controls:**
| Key | Action |
|-----|--------|
| `SPACE` | Start/restart rally |
| `R` | Reset simulation |
| `P` | Pause/unpause |
| `1-5` | Time scale (0.1x → 2x) |
| `ESC` | Quit |

**Mouse:**
| Action | Effect |
|--------|--------|
| Right-click + drag | Rotate camera |
| Scroll | Zoom in/out |
| Middle-click + drag | Pan |

### 2D Side View (`viewers/side_view.py`)

Classic side view for analyzing trajectories.

**Controls:**
| Key | Action |
|-----|--------|
| `SPACE` | Launch ball |
| `R` | Reset |
| `T` | Toggle topspin/backspin |
| `↑/↓` | Adjust launch angle |
| `←/→` | Adjust launch speed |

## OAK-D Camera - BlazePose On-Device

Real-time 3D pose tracking using [BlazePose](https://github.com/geaxgx/depthai_blazepose) running **on the Myriad X VPU** (not on CPU).

### Setup

```bash
# 1. Connect camera to PoE switch
# 2. Find camera IP (check router or use discover script)
# 3. Edit oak/config.py with your camera's IP
# 4. If camera is on different subnet, add secondary IP:
sudo ip addr add <CAMERA_SUBNET>.1/24 dev <YOUR_INTERFACE>

# 5. Test connection
python oak/connect.py

# 6. Run pose tracking
python oak/blazepose_live.py
```

### Performance

| Model | FPS | FPS with 3D |
|-------|-----|-------------|
| Lite  | 26  | 22          |
| Full  | 20  | 18          |
| Heavy | 8   | 7           |

### How it works

```
┌─────────────────────────────────────────────────────┐
│                   OAK-D Pro W PoE                   │
│                                                     │
│  RGB Camera ──► BlazePose ──► Landmarks (33 pts)   │
│                 (on VPU)                            │
│  Stereo Cams ──► Depth ──────► 3D reference point  │
│                                                     │
└──────────────────────┬──────────────────────────────┘
                       │ ~3KB/frame (just keypoints)
                       ▼
                      PC (visualization only)
```

All inference runs on-device. PC only receives landmarks, not raw frames.

### Credits

Based on [depthai_blazepose](https://github.com/geaxgx/depthai_blazepose) by geaxgx.

## Phone Stereo Capture (Alternative)

The `capture/` module handles stereo video from a dual-camera phone for 3D tracking.

### Setup

1. Install [StereoVideoCamera](https://github.com/Awerito/StereoVideoCamera) on
   your Android phone
2. Connect via USB or WiFi (ADB)
3. Forward ports: `adb forward tcp:9556 tcp:9556 && adb forward tcp:9557
   tcp:9557`

### Usage

```bash
# View stereo feed
python capture/stereo_receiver.py

# Calibrate cameras (for 3D reconstruction)
python capture/stereo_calibrate.py --square-size 4.9
```

See [CLAUDE.md](CLAUDE.md) for detailed calibration instructions.

## Physics Engine

The Rust physics engine (`tt-core`) simulates:

### Ball Dynamics
```
Position: x, y, z (meters)
Velocity: vx, vy, vz (m/s)
Spin: ωx, ωy, ωz (RPM)
```

### Forces Applied
- **Gravity**: -9.81 m/s²
- **Air Resistance**: Proportional to velocity²
- **Magnus Force**: F = Cl × (ω × v)
  - Topspin → ball dips faster
  - Backspin → ball floats longer
  - Sidespin → ball curves left/right

### Collision Physics
- **Table bounce**: Spin transfers to linear velocity
- **Net collision**: Detected and reported
- **Spin decay**: Friction reduces spin on bounce

## Python API

```python
from tt_physics import Simulation

# Create simulation
sim = Simulation()

# Set ball state
sim.set_ball_position(-1.0, 1.0, 0.0)  # x, y, z in meters
sim.set_ball_velocity(5.0, 2.0, 0.0)   # vx, vy, vz in m/s
sim.set_ball_spin_rpm(0, 0, 3000)      # topspin: positive Z

# Run simulation
dt = 0.0001  # 0.1ms timestep for accuracy
for _ in range(10000):
    sim.step(dt)

    # Get state
    pos = sim.ball_position()
    vel = sim.ball_velocity()
    speed = sim.ball_speed()
    spin_rpm = sim.ball_spin_rpm()

    # Check collisions
    collision = sim.last_collision()  # "table", "net", or None

    print(f"Ball at ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")

# Table info
half_len, half_wid, height, net_h = sim.table_dimensions()
surface_y = sim.table_surface_y()  # ~0.76m
```

## Project Structure

```
TTwin/
├── tt-core/                 # Rust physics engine
│   ├── Cargo.toml
│   └── src/
│       └── lib.rs           # Physics implementation
├── tt_physics/              # Python package (built by maturin)
├── viewers/
│   ├── view_3d.py          # 3D Ursina viewer
│   ├── side_view.py        # 2D pygame side view
│   └── dual_view.py        # 2D pygame dual view
├── oak/                     # OAK-D Pro W PoE + BlazePose on-device
│   ├── blazepose_live.py   # Main entry point
│   ├── BlazeposeDepthaiEdge.py # On-device tracker
│   ├── config.py           # Camera IP configuration
│   ├── connect.py          # Connection test
│   ├── models/             # Neural network blobs for VPU
│   └── README.md           # Setup documentation
├── capture/                 # Phone stereo capture (alternative)
│   ├── stereo_receiver.py  # View stereo feed from phone
│   └── stereo_calibrate.py # Camera calibration
├── recordings/              # Captured sessions (gitignored)
├── requirements.txt
├── CLAUDE.md               # Development notes
└── README.md
```

## Spin Types Explained

| Spin | Axis | Effect |
|------|------|--------|
| **Topspin** | +Z (right-hand rule) | Ball dips, bounces forward |
| **Backspin** | -Z | Ball floats, bounces back |
| **Sidespin Right** | +Y | Curves right |
| **Sidespin Left** | -Y | Curves left |

```python
# Topspin (ball traveling +X direction)
sim.set_ball_spin_rpm(0, 0, 3000)

# Backspin
sim.set_ball_spin_rpm(0, 0, -3000)

# Sidespin (curves right)
sim.set_ball_spin_rpm(0, 3000, 0)
```

## Development

### Building the Rust Engine

```bash
cd tt-core
maturin develop        # Debug build
maturin develop -r     # Release build (faster)
```

### Running Tests

```bash
# Test physics
python viewers/side_view.py

# Test OAK-D camera
python oak/connect.py
```

## Known Issues

- AMD GPUs require `unlit=True` on all Ursina entities
- Rally mode needs tuning (paddle collision WIP)

## Roadmap

- [x] Rust physics engine
- [x] Python bindings
- [x] 2D viewers (side, dual)
- [x] 3D viewer
- [x] Spin physics (Magnus effect)
- [x] Phone stereo camera capture
- [x] OAK-D Pro W PoE integration
- [x] 3D body tracking (BlazePose on-device, ~18fps)
- [ ] 3D ball tracking
- [ ] Paddle collision physics
- [ ] Continuous rally
- [ ] Player control
- [ ] Scoring system

## Related Projects

- [StereoVideoCamera](https://github.com/Awerito/StereoVideoCamera) - Android
app for dual-camera stereo capture

## License

MIT

---

*Built with Rust 🦀 + Python 🐍 + Ursina 🎮*
