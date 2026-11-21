#!/usr/bin/env python3
"""Compute directly comparable metrics from the raw M3-4 trial records."""

import argparse
import csv
import json
import math
from pathlib import Path
import statistics


def read_csv(path):
    with path.open(newline='') as stream:
        return [{key: float(value) for key, value in row.items()}
                for row in csv.DictReader(stream)]


def path_length(rows):
    return sum(math.hypot(b['x'] - a['x'], b['y'] - a['y'])
               for a, b in zip(rows, rows[1:]))


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def smoothness(rows, step=0.1):
    """RMS change of path curvature at approximately 0.1 m spacing."""
    sampled = []
    travelled = 0.0
    last = None
    for row in rows:
        if last is not None:
            travelled += math.hypot(row['x'] - last['x'], row['y'] - last['y'])
        if not sampled or travelled >= step:
            sampled.append(row)
            travelled = 0.0
        last = row
    curvatures = []
    for a, b in zip(sampled, sampled[1:]):
        distance = math.hypot(b['x'] - a['x'], b['y'] - a['y'])
        if distance >= step * 0.8:
            curvatures.append(angle_diff(b['yaw_rad'], a['yaw_rad']) / distance)
    changes = [b - a for a, b in zip(curvatures, curvatures[1:])]
    return math.sqrt(statistics.mean(x * x for x in changes)) if changes else None


def analyze_run(run_dir, reference_length):
    data = run_dir / 'data'
    summary = json.loads((run_dir / 'summary.json').read_text())
    truth = read_csv(data / 'truth.csv') if (data / 'truth.csv').exists() else []
    if summary.get('goal_start_sim_s') is not None:
        truth = [row for row in truth if row['sim_s'] >= summary['goal_start_sim_s']]
    plans = [json.loads(line) for line in (data / 'plans.jsonl').read_text().splitlines()]
    spawn = summary.get('obstacle_sim_s')
    obstacle_x, obstacle_y = summary.get('obstacle_world_xy', [None, None])
    obstacle_sx, obstacle_sy = summary.get('obstacle_size_xy', [None, None])
    # The M3-2 map's origin is aligned so world x=-5 is near map x=0.
    box_map_x = obstacle_x + 5.0 if obstacle_x is not None else None

    def corridor_y(plan):
        values = [y for x, y in plan['xy']
                  if box_map_x is not None and abs(x - box_map_x) <= 0.5]
        return statistics.median(values) if values else None

    before = [corridor_y(p) for p in plans if spawn and p['sim_s'] < spawn]
    before = [value for value in before if value is not None]
    after = [p for p in plans if spawn and p['sim_s'] >= spawn]
    baseline_y = statistics.median(before[-3:]) if before else None
    # A 0.10 m shift at the obstruction exceeds the small pre-injection
    # path jitter while capturing the narrower detour around an offset box.
    changed = [p for p in after if baseline_y is not None and corridor_y(p) is not None
               and abs(corridor_y(p) - baseline_y) > 0.10]
    # Reject a single transient path sample: two consecutive detour plans must agree.
    first_persistent = next((a for a, b in zip(after, after[1:])
                             if a in changed and b in changed), None)
    end = summary.get('end_map_base_link_xy')
    goal_error = math.dist(end, summary.get('goal_map_xy', [11.0, 3.0])) if end else None
    length = path_length(truth) if len(truth) >= 2 else None
    chord = math.hypot(truth[-1]['x'] - truth[0]['x'],
                       truth[-1]['y'] - truth[0]['y']) if len(truth) >= 2 else None
    min_clearance = None
    if truth and obstacle_x is not None and not summary.get('no_obstacle'):
        # Distance between the robot-centre point and the axis-aligned box edge;
        # negative means the centre entered the box. Not a footprint collision test.
        after_spawn = [p for p in truth if spawn is None or p['sim_s'] >= spawn]
        if after_spawn:
            min_clearance = min(math.hypot(max(abs(p['x'] - obstacle_x) - obstacle_sx / 2, 0),
                                           max(abs(p['y'] - obstacle_y) - obstacle_sy / 2, 0))
                                for p in after_spawn)
    success = (summary.get('outcome') == 'SUCCEEDED' and
               (spawn is not None or summary.get('no_obstacle')) and
               goal_error is not None and goal_error <= 0.25)
    log_file = run_dir / 'launch.log'
    evidence_file = run_dir / 'failure_events.txt'
    launch_log = (log_file.read_text(errors='replace') if log_file.exists()
                  else evidence_file.read_text(errors='replace') if evidence_file.exists()
                  else '')
    if 'Controller patience exceeded' in launch_log:
        nav_failure = 'Controller patience exceeded after collision warnings'
    elif 'Starting point in lethal space' in launch_log:
        nav_failure = 'Hybrid-A* rejected start in lethal costmap cell'
    elif 'Failed to make progress' in launch_log:
        nav_failure = 'Progress checker: failed to make progress'
    elif 'Goal failed' in launch_log:
        nav_failure = 'Nav2 goal failed; see launch.log'
    else:
        nav_failure = 'Navigation status/goal check failed'
    return {
        'run': summary['run'], 'success': success, 'outcome': summary.get('outcome'),
        'failure_reason': summary.get('harness_error') or summary.get('error') or
                          (None if success else nav_failure),
        'duration_wall_s': summary.get('elapsed_wall_s'),
        'duration_sim_s': summary.get('elapsed_sim_s'),
        'truth_path_m': length,
        'endpoint_chord_m': chord,
        'extra_path_over_chord_m': length - chord
                if length is not None and chord is not None else None,
        'extra_path_vs_unobstructed_mean_m': length - reference_length
                if length is not None and reference_length is not None else None,
        'smoothness_rms_curvature_change_per_m': smoothness(truth) if truth else None,
        'goal_error_m': goal_error,
        'min_robot_center_to_box_edge_m': min_clearance,
        'plans_before_injection': len(before), 'plans_after_injection': len(after),
        'first_plan_latency_sim_s': after[0]['sim_s'] - spawn if after else None,
        'first_detour_latency_sim_s': first_persistent['sim_s'] - spawn
                                      if first_persistent else None,
        'baseline_y_at_box_m': baseline_y,
        'largest_signed_y_shift_at_box_m': max((abs(corridor_y(p) - baseline_y)
                         for p in after if baseline_y is not None and corridor_y(p) is not None),
                         default=None),
    }


def aggregate(rows, key, successful_only=True):
    values = [row[key] for row in rows if row[key] is not None and
              (row['success'] or not successful_only)]
    return {'n': len(values), 'mean': statistics.mean(values) if values else None,
            'stdev': statistics.stdev(values) if len(values) > 1 else None}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('experiment', type=Path)
    parser.add_argument('--reference-length', type=float)
    args = parser.parse_args()
    rows = [analyze_run(path, args.reference_length) for path in sorted(args.experiment.glob('run_*'))
            if (path / 'summary.json').exists()]
    report = {'runs': len(rows), 'successes': sum(x['success'] for x in rows),
              'success_rate': statistics.mean(x['success'] for x in rows) if rows else None,
              'metrics_on_successful_runs': {
                  key: aggregate(rows, key) for key in (
                      'duration_sim_s', 'duration_wall_s', 'truth_path_m', 'endpoint_chord_m',
                      'extra_path_over_chord_m', 'extra_path_vs_unobstructed_mean_m',
                      'smoothness_rms_curvature_change_per_m', 'goal_error_m',
                      'first_plan_latency_sim_s', 'first_detour_latency_sim_s')},
              'replanning_on_all_runs': {
                  key: aggregate(rows, key, successful_only=False) for key in (
                      'first_plan_latency_sim_s', 'first_detour_latency_sim_s')},
              'runs_detail': rows}
    (args.experiment / 'analysis.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
