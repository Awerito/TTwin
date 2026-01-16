#!/usr/bin/env python3
"""
BlazePose Live - On-device pose estimation with OAK-D

Runs BlazePose on the Myriad X VPU for real-time 3D pose tracking.
Uses Edge mode for maximum performance (~15-18 fps with 3D).

Based on: https://github.com/geaxgx/depthai_blazepose

Usage:
    python oak/blazepose_live.py

Controls:
    q - Quit
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from BlazeposeDepthaiEdge import BlazeposeDepthai
from BlazeposeRenderer import BlazeposeRenderer


def main():
    # Initialize tracker: edge mode, full model, 3D coords
    tracker = BlazeposeDepthai(
        input_src="rgb",
        lm_model="full",
        xyz=True,
        internal_fps=18,
        stats=True,
    )

    renderer = BlazeposeRenderer(tracker, show_3d=None, output=None)

    print("BlazePose Live - On-device inference")
    print("Press 'q' to quit")
    print()

    while True:
        frame, body = tracker.next_frame()
        if frame is None:
            break

        frame = renderer.draw(frame, body)
        key = renderer.waitKey(delay=1)

        if key == 27 or key == ord("q"):
            break

    renderer.exit()
    tracker.exit()


if __name__ == "__main__":
    main()
