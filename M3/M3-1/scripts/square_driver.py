#!/usr/bin/env python3
"""Exam-facing entry point: drive and record N independent one-lap trials."""

import argparse
import shutil
import subprocess
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--laps", type=int, required=True,
                        help="Number of recorded one-lap trials")
    parser.add_argument("--side", type=float, required=True,
                        help="Outer width of the rounded square in metres")
    parser.add_argument("--out", type=Path, required=True,
                        help="New directory for run_01, run_02, ...")
    parser.add_argument("--radius", type=float, default=0.65)
    parser.add_argument("--speed", type=float, default=0.1)
    args = parser.parse_args()
    if args.laps < 1:
        parser.error("--laps must be at least 1")
    flat_files = [args.out / f"run_{index:02d}_{suffix}.csv"
                  for index in range(1, args.laps + 1)
                  for suffix in ("odom", "truth")]
    if any(path.exists() for path in flat_files):
        parser.error("Flat trial CSV already exists; choose a new --out directory")
    runner = Path(__file__).with_name("run_trials.py")
    subprocess.run([
        sys.executable, str(runner), "--runs", str(args.laps),
        "--side", str(args.side), "--radius", str(args.radius),
        "--speed", str(args.speed), "--out", str(args.out),
    ], check=True)
    # Keep the structured run folders for metadata and analysis while also
    # supplying the flat CSV filenames shown in the assessment handout.
    for index in range(1, args.laps + 1):
        run = f"run_{index:02d}"
        for source, suffix in (("odom.csv", "odom"), ("truth.csv", "truth")):
            destination = args.out / f"{run}_{suffix}.csv"
            shutil.copyfile(args.out / run / source, destination)
    print(f"Saved {args.laps} paired flat CSV files in {args.out}")


if __name__ == "__main__":
    main()
