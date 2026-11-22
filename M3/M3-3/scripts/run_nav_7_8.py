#!/usr/bin/env python3
"""Navigate from the launch pose to map (7, 8, 180 deg) via the bottom corridor.

The direct route turns toward the island immediately and has repeatedly aborted
near its southwest corner. The intermediate pose (3, 0, 0 deg) keeps the car
heading east along the bottom corridor before Nav2 plans the final leg.
"""

from pathlib import Path
import subprocess
import sys


TRIAL = Path(__file__).with_name("run_nav_trial.py")
LEGS = (
    ("bottom corridor", (3.0, 0.0, 0.0), 2.5, False),
    ("final goal", (7.0, 8.0, 180.0), 5.0, True),
)


def main():
    for label, (x, y, yaw), min_travel, from_current in LEGS:
        print(f"Starting {label}: map ({x:g}, {y:g}, {yaw:g} deg)", flush=True)
        command = [
            sys.executable, str(TRIAL),
            "--goal-x", str(x), "--goal-y", str(y),
            "--goal-yaw-deg", str(yaw), "--min-travel", str(min_travel),
        ]
        if from_current:
            command.append("--from-current")
        result = subprocess.run(command, check=False)
        if result.returncode:
            print(f"FAILED: {label} navigation; stop and inspect the log", flush=True)
            return result.returncode
    print("SUCCEEDED: reached map (7, 8, 180 deg) through both Nav2 legs", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
