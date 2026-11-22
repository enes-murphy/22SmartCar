#!/usr/bin/env python3
"""Plot the four M3-4 comparison metrics from analyzed trial folders."""

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('experiments', type=Path, nargs='+')
    parser.add_argument('--labels', nargs='+')
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    if args.labels and len(args.labels) != len(args.experiments):
        parser.error('one label is needed for each experiment')
    reports = [json.loads((p / 'analysis.json').read_text()) for p in args.experiments]
    if any(r['runs'] == 0 for r in reports):
        parser.error('each analysis must contain completed runs')
    labels = args.labels or [p.name for p in args.experiments]
    metrics = [
        ('Success rate', [r['success_rate'] * 100 for r in reports], '%', None),
        ('Mean time, successful runs',
         [r['metrics_on_successful_runs']['duration_sim_s']['mean'] for r in reports],
         's', 'duration_sim_s'),
        ('Path excess over endpoint chord',
         [r['metrics_on_successful_runs']['extra_path_over_chord_m']['mean']
          for r in reports], 'm', 'extra_path_over_chord_m'),
        ('Curvature-change RMS',
         [r['metrics_on_successful_runs']['smoothness_rms_curvature_change_per_m']['mean']
          for r in reports], '1/m', 'smoothness_rms_curvature_change_per_m'),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(12, 7.5))
    colors = ['#286d96', '#bd7a26', '#6b7890']
    for ax, (title, values, unit, key) in zip(axes.flat, metrics):
        if any(v is None for v in values):
            ax.text(.5, .5, 'Reference run needed', ha='center', va='center',
                    transform=ax.transAxes, fontsize=12)
            ax.set_xticks([])
        else:
            bars = ax.bar(labels, values, color=[colors[i % len(colors)]
                                                  for i in range(len(values))])
            for i, (bar, value) in enumerate(zip(bars, values)):
                n = reports[i]['runs'] if key is None else reports[i][
                    'metrics_on_successful_runs'][key]['n']
                ax.annotate(f'{value:.1f}{unit}' if key is None else f'{value:.3f}{unit}',
                            (bar.get_x() + bar.get_width()/2, bar.get_height()),
                            xytext=(0, 3), textcoords='offset points',
                            ha='center', fontsize=10)
                ax.annotate(f'n={n}', (bar.get_x()+bar.get_width()/2, 0),
                            xytext=(0, 4), textcoords='offset points', ha='center',
                            va='bottom', fontsize=9, color='white')
            ax.set_ylim(bottom=min(0, min(values) * 1.15),
                        top=max(values) * 1.2 if max(values) else 1)
            ax.tick_params(axis='x', labelrotation=12)
        ax.set_title(title, fontsize=12)
        ax.set_ylabel(unit)
        ax.grid(axis='y', alpha=.2)
    fig.suptitle('M3-4 fixed-scene comparison: actual trial records', fontsize=16)
    fig.text(.02, .02,
             'Time, path and smoothness use successful runs only. Path excess uses the endpoint chord, '
             'so it is a redundancy proxy rather than obstacle detour.\n'
             'Failed accepted goals remain in the success-rate denominator.',
             fontsize=9)
    fig.tight_layout(rect=(0, .065, 1, .95))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=160)
    print(args.out)


if __name__ == '__main__':
    main()
