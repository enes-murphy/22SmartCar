#!/usr/bin/env python3
"""Render four time-stamped route snapshots from one recorded obstacle trial."""

import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path, help='run_XX directory containing summary.json and data/')
    parser.add_argument('--out', type=Path, help='output directory; defaults to run/process_frames')
    args = parser.parse_args()
    summary = json.loads((args.run / 'summary.json').read_text())
    data = args.run / 'data'
    with (data / 'truth.csv').open(newline='') as stream:
        truth = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(stream)]
    plans = [json.loads(line) for line in (data / 'plans.jsonl').read_text().splitlines()]
    start = summary['goal_start_sim_s']
    spawn = summary['obstacle_sim_s']
    end = truth[-1]['sim_s']
    if spawn is None or end <= spawn:
        parser.error('a completed obstacle trial is required')
    # These are data reconstructions, not screenshots from Gazebo.
    stages = [
        ('01_approach', min(spawn - 0.25, start + 1.5)),
        ('02_detection', min(end, spawn + 0.5)),
        ('03_detour', min(end, spawn + (end - spawn) * 0.35)),
        ('04_after_obstacle', min(end, spawn + (end - spawn) * 0.75)),
    ]
    out = args.out or args.run / 'process_frames'
    out.mkdir(parents=True, exist_ok=True)
    x_box, y_box = summary['obstacle_world_xy']
    sx, sy = summary['obstacle_size_xy']
    route = [r for r in truth if r['sim_s'] >= start]
    for label, t in stages:
        visible = [r for r in route if r['sim_s'] <= t]
        if not visible:
            continue
        plan = next((p for p in reversed(plans) if p['sim_s'] <= t), None)
        fig, ax = plt.subplots(figsize=(9, 5.6))
        ax.plot([r['x'] for r in route], [r['y'] for r in route],
                color='0.8', lw=1.2, label='whole recorded route')
        ax.plot([r['x'] for r in visible], [r['y'] for r in visible],
                color='#1767a0', lw=2, label='route up to this time')
        ax.scatter(visible[-1]['x'], visible[-1]['y'], color='#1767a0', s=75,
                   zorder=4, label='car centre')
        if plan:
            ax.plot([xy[0] - 5 for xy in plan['xy']],
                    [xy[1] - 4 for xy in plan['xy']], color='#d18b18',
                    lw=1.3, alpha=.8, label='latest global plan')
        if t >= spawn:
            ax.add_patch(Rectangle((x_box - sx/2, y_box - sy/2), sx, sy,
                                   facecolor='#bb3434', edgecolor='black',
                                   alpha=.9, label='inserted obstacle'))
        ax.set(xlim=(-5, 3.6), ylim=(-4.8, -2.6), xlabel='Gazebo world x (m)',
               ylabel='Gazebo world y (m)',
               title=f'{label.replace("_", " ")}  |  {t-start:.2f} s after goal acceptance')
        ax.set_aspect('equal', adjustable='box')
        ax.grid(alpha=.25)
        ax.legend(loc='upper left', fontsize=8)
        fig.tight_layout()
        fig.savefig(out / f'{label}.png', dpi=150)
        plt.close(fig)
    print(out)


if __name__ == '__main__':
    main()
