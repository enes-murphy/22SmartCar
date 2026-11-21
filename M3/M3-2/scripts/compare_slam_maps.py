#!/usr/bin/env python3
"""Compare two M3-2 replay maps in matched physical regions of the world."""

import argparse
import json
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

from eval_map import line_quality


def load(path):
    meta = yaml.safe_load(path.read_text())
    data = np.asarray(Image.open(path.parent / meta["image"]))
    return meta, data


def measures(path, reference):
    meta, data = load(path)
    ref_meta, ref_data = load(reference)
    resolution = float(meta["resolution"])
    if abs(resolution - float(ref_meta["resolution"])) > 1e-9:
        raise ValueError("map resolutions differ")
    dx = round((ref_meta["origin"][0] - meta["origin"][0]) / resolution)
    dy = data.shape[0] - ref_data.shape[0] + round(
        (meta["origin"][1] - ref_meta["origin"][1]) / resolution)
    occupied = data < 100
    free = data == 254
    walls = {}
    squared_residual_sum = 0.0
    wall_sample_count = 0
    for name, axis, lo, hi, search_lo, search_hi, exclude in [
        ("north", "x", 50, 231, 35, 48, None),
        ("south", "x", 50, 231, 154, 168, None),
        ("west", "y", 50, 151, 34, 48, None),
        ("east excluding gate", "y", 50, 151, 234, 251, (88, 114)),
    ]:
        primary = dx if axis == "x" else dy
        secondary = dy if axis == "x" else dx
        shifted_exclude = None if exclude is None else (
            exclude[0] + primary, exclude[1] + primary)
        try:
            n, rms, p95 = line_quality(
                occupied, axis, lo + primary, hi + primary,
                search_lo + secondary, search_hi + secondary,
                resolution, shifted_exclude)
            walls[name] = {"samples": n, "rms_m": round(rms, 4),
                           "p95_m": round(p95, 4)}
            squared_residual_sum += n * rms * rms
            wall_sample_count += n
        except RuntimeError:
            walls[name] = None
    spans = []
    for col in range(50 + dx, 231 + dx):
        for low, high in ((35, 48), (154, 168)):
            cells = np.flatnonzero(occupied[low + dy:high + dy, col])
            if len(cells):
                spans.append(int(np.ptp(cells)) + 1)
    for row in list(range(50 + dy, 88 + dy)) + list(range(114 + dy, 151 + dy)):
        cells = np.flatnonzero(occupied[row, 234 + dx:251 + dx])
        if len(cells):
            spans.append(int(np.ptp(cells)) + 1)
    widths = []
    for row in range(98 + dy, 104 + dy):
        inner = np.flatnonzero(occupied[row, 245 + dx:256 + dx]) + 245 + dx
        outer = np.flatnonzero(occupied[row, 267 + dx:280 + dx]) + 267 + dx
        if len(inner) and len(outer):
            left, right = int(inner.max()), int(outer.min())
            if right > left and np.all(free[row, left + 1:right]):
                widths.append(right - left - 1)
    return {
        "map": str(path), "width": int(data.shape[1]), "height": int(data.shape[0]),
        "resolution_m": resolution, "origin": meta["origin"],
        "occupied_cells": int(occupied.sum()), "free_cells": int(free.sum()),
        "wall_weighted_rms_m": round(
            (squared_residual_sum / wall_sample_count) ** 0.5, 4
        ) if wall_sample_count else None,
        "walls": walls,
        "max_wall_span_m": round(max(spans) * resolution, 3) if spans else None,
        "gate_known_free_width_m": round(min(widths) * resolution, 3) if widths else None,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--on", type=Path, default=Path("results/loop_closure_compare/replay_on/map.yaml"))
    parser.add_argument("--off", type=Path, default=Path("results/loop_closure_compare/replay_off/map.yaml"))
    parser.add_argument("--reference", type=Path, default=Path("maps/map.yaml"))
    parser.add_argument("--out", type=Path, default=Path("results/loop_closure_compare/metrics.json"))
    args = parser.parse_args()
    results = {"loop_on": measures(args.on, args.reference),
               "loop_off": measures(args.off, args.reference)}
    args.out.write_text(json.dumps(results, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(results, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
