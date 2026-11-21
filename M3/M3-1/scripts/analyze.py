#!/usr/bin/env python3
"""Exam-facing entry point: summarize recorded laps and render deliverables."""

import argparse
import subprocess
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--logs", type=Path, required=True,
                        help="Directory containing run_01, run_02, ...")
    parser.add_argument("--out", type=Path, required=True,
                        help="Directory for table, statistics, and figures")
    parser.add_argument("--expected-runs", type=int, default=10)
    args = parser.parse_args()
    analyzer = Path(__file__).with_name("analyze_repeats.py")
    subprocess.run([
        sys.executable, str(analyzer), "--runs-dir", str(args.logs),
        "--expected-runs", str(args.expected_runs), "--out", str(args.out),
    ], check=True)


if __name__ == "__main__":
    main()
