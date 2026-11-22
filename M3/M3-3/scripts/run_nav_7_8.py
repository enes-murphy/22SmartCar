#!/usr/bin/env python3
"""Verified preset: reach map (7, 8, 180 deg) via the bottom corridor."""

import sys

from run_nav_via import main


if __name__ == "__main__":
    sys.exit(main([
        "--via", "3", "0", "0",
        "--goal-x", "7", "--goal-y", "8", "--goal-yaw-deg", "180",
        "--min-waypoint-travel", "2.5", "--min-final-travel", "5",
    ]))
