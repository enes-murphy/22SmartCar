#!/usr/bin/env python3
"""Plot all successful Gazebo truth paths and the inserted obstacle."""

import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


def read_paths(experiment):
    paths = []
    settings = None
    for run_dir in sorted(experiment.glob('run_*')):
        summary_file = run_dir / 'summary.json'
        if not summary_file.exists():
            continue
        summary = json.loads(summary_file.read_text())
        if settings is None:
            settings = summary
        if summary.get('outcome') != 'SUCCEEDED' or summary.get('obstacle_sim_s') is None:
            continue
        with (run_dir / 'data' / 'truth.csv').open(newline='') as stream:
            rows = list(csv.DictReader(stream))
        rows = [r for r in rows if float(r['sim_s']) >= summary['goal_start_sim_s']]
        if rows:
            paths.append(([float(r['x']) for r in rows], [float(r['y']) for r in rows]))
    return paths, settings


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('experiments', nargs='+', type=Path,
                        help='experiment directories in display order')
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    colors = ['#1261a0', '#d04b25', '#21815c']
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.4), constrained_layout=True)
    settings = None
    for index, experiment in enumerate(args.experiments):
        paths, current = read_paths(experiment)
        if current is not None:
            settings = current
        color = colors[index % len(colors)]
        for ax in axes:
            for xs, ys in paths:
                ax.plot(xs, ys, color=color, alpha=0.35, linewidth=1.0)
            if paths:
                ax.plot([], [], color=color, linewidth=2,
                        label=f'{experiment.name}: {len(paths)} successes')
    if settings is None:
        parser.error('no run summaries found')
    x, y = settings['obstacle_world_xy']
    sx, sy = settings['obstacle_size_xy']
    for ax in axes:
        ax.add_patch(Rectangle((x - sx / 2, y - sy / 2), sx, sy,
                               facecolor='#c31b1b', edgecolor='black', alpha=0.8,
                               label='inserted box'))
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel('Gazebo world x (m)')
        ax.set_ylabel('Gazebo world y (m)')
        ax.grid(alpha=0.25)
    axes[0].set_title('All successful goal-to-result paths')
    axes[0].set_xlim(-5.0, 3.4)
    axes[0].set_ylim(-4.25, -2.8)
    axes[1].set_title('At the inserted obstacle')
    axes[1].set_xlim(x - 1.15, x + 1.15)
    axes[1].set_ylim(y - 0.8, y + 0.55)
    axes[0].legend(loc='upper left', fontsize=8)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=180)
    print(args.out)


if __name__ == '__main__':
    main()
