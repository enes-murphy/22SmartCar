#!/usr/bin/env python3
"""Repeat one obstacle experiment from a fresh simulation for each run."""

import argparse
import json
import os
import signal
from pathlib import Path
import shutil
import subprocess
import sys
import time
import psutil

from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'M3-3' / 'scripts'))
from run_repeats import stop_group, wait_for_navigation  # noqa: E402


def stop_entire_group(process):
    """Stop the launcher and any children it left after exiting itself."""
    if process is None:
        return
    try:
        children = psutil.Process(process.pid).children(recursive=True)
    except psutil.NoSuchProcess:
        children = []
    stop_group(process, timeout=5)
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    for child in children:
        try:
            child.terminate()
        except psutil.NoSuchProcess:
            pass
    _, alive = psutil.wait_procs(children, timeout=3)
    for child in alive:
        try:
            child.kill()
        except psutil.NoSuchProcess:
            pass


def run_one(index, run_dir, config, domain, args):
    run_dir.mkdir(parents=True)
    log_dir = run_dir / 'ros_logs'
    log_dir.mkdir()
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOG_DIR=str(log_dir.resolve()),
               GAZEBO_MASTER_URI=f'http://127.0.0.1:{11370 + domain}')
    launch = navigation = trial = None
    code = None
    error = None
    try:
        with (run_dir / 'launch.log').open('w') as stream:
            launch = subprocess.Popen(
                ['ros2', 'launch', 'smart_car_navigation', 'localization.launch.py',
                 f'gui:={str(args.gui).lower()}', 'rviz:=false', 'navigation:=false'], env=env, stdout=stream,
                stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + 90
            while time.monotonic() < deadline:
                if launch.poll() is not None:
                    raise RuntimeError(f'world launch exited with {launch.returncode}')
                state = subprocess.run(['ros2', 'lifecycle', 'get', '/amcl'], env=env,
                                       capture_output=True, text=True, timeout=8)
                if 'active [3]' in state.stdout:
                    break
                time.sleep(1)
            else:
                raise TimeoutError('AMCL did not become active within 90 s')
            # Establish a live map->odom transform before creating any Nav2
            # costmap. This avoids accepting a stale first TF in fresh launches.
            warmup = subprocess.run(
                [sys.executable, '../M3-2/scripts/drive_route.py', '--straight-test',
                 '0.3', '--straight-speed', '0.1'], env=env, capture_output=True,
                text=True, timeout=20)
            (run_dir / 'prewarm.log').write_text(warmup.stdout + warmup.stderr)
            if warmup.returncode:
                raise RuntimeError(f'AMCL prewarm failed with {warmup.returncode}')
            nav_share = Path(get_package_share_directory('smart_car_navigation'))
            resolved = config.read_text().replace(
                '__NAV_BT__', str(nav_share / 'behavior_trees' / 'navigate_ackermann.xml'))
            resolved = resolved.replace(
                '__NAV_THROUGH_BT__',
                str(nav_share / 'behavior_trees' / 'navigate_through_poses_ackermann.xml'))
            nav_file = run_dir / 'resolved_navigation.yaml'
            nav_file.write_text(resolved)
            navigation = subprocess.Popen(
                ['ros2', 'launch', 'nav2_bringup', 'navigation_launch.py',
                 f'params_file:={nav_file.resolve()}', 'use_sim_time:=true',
                 'autostart:=true', 'use_composition:=False'],
                env=env, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
            # Lifecycle queries can briefly time out while the composed Nav2
            # container configures its costmaps. Retry instead of aborting a
            # healthy launch on one slow service response.
            ready = False
            deadline = time.monotonic() + 90
            while time.monotonic() < deadline:
                try:
                    wait_for_navigation(env, navigation, timeout=8)
                    ready = True
                    break
                except subprocess.TimeoutExpired:
                    continue
                except TimeoutError:
                    continue
            if not ready:
                raise TimeoutError('Nav2 and AMCL did not become active within 90 s')
            readings = []
            for node, parameter in (
                ('/controller_server', 'FollowPath.lookahead_dist'),
                ('/local_costmap/local_costmap', 'plugins'),
                ('/local_costmap/local_costmap', 'obstacle_layer.observation_sources'),
                ('/global_costmap/global_costmap', 'plugins'),
                ('/global_costmap/global_costmap', 'obstacle_layer.observation_sources'),
            ):
                value = subprocess.run(['ros2', 'param', 'get', node, parameter], env=env,
                                       capture_output=True, text=True, timeout=10)
                readings.append(f'{node} {parameter}: {value.stdout.strip()} {value.stderr.strip()}')
            (run_dir / 'parameter_readback.txt').write_text('\n'.join(readings) + '\n')
            with (run_dir / 'trial.log').open('w') as trial_log:
                trial = subprocess.Popen(
                    [sys.executable, 'scripts/dynamic_trial.py', '--out', str(run_dir / 'data'),
                     '--delay', str(args.delay), '--world-x', str(args.world_x),
                     '--world-y', str(args.world_y), '--size-x', str(args.size_x),
                     '--size-y', str(args.size_y), '--goal-x', str(args.goal_x),
                     '--goal-y', str(args.goal_y), '--goal-yaw-deg', str(args.goal_yaw_deg),
                     *(['--no-obstacle'] if args.no_obstacle else [])], env=env, stdout=trial_log,
                    stderr=subprocess.STDOUT, start_new_session=True)
                try:
                    code = trial.wait(timeout=205)
                except subprocess.TimeoutExpired:
                    error = 'Trial exceeded 205 wall-clock seconds'
    except Exception as exc:
        error = str(exc)
    finally:
        stop_entire_group(trial)
        # The trial has already saved its action result and measurements. A
        # bounded SIGINT shutdown avoids long lifecycle-service timeouts and
        # leaves no Nav2 process for the next isolated run.
        stop_entire_group(navigation)
        stop_entire_group(launch)
    summary_file = run_dir / 'data' / 'summary.json'
    summary = json.loads(summary_file.read_text()) if summary_file.exists() else {}
    summary.update({'run': index, 'process_exit_code': code, 'harness_error': error})
    (run_dir / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(f'Run {index}: {summary.get("outcome", "NO DATA")}; '
          f'time={summary.get("elapsed_wall_s")}; box={summary.get("obstacle_sim_s")}; '
          f'error={error or summary.get("error")}', flush=True)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', choices=['short', 'long', 'long_tuned',
                                             'guide_short', 'guide_long', 'guide_long_tuned'],
                        required=True)
    parser.add_argument('--runs', type=int, default=10)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--first-domain', type=int, default=100)
    parser.add_argument('--delay', type=float, default=8.0)
    parser.add_argument('--world-x', type=float, default=-1.0)
    parser.add_argument('--world-y', type=float, default=-3.35)
    parser.add_argument('--size-x', type=float, default=0.30)
    parser.add_argument('--size-y', type=float, default=0.30)
    parser.add_argument('--goal-x', type=float, default=8.0)
    parser.add_argument('--goal-y', type=float, default=0.4)
    parser.add_argument('--goal-yaw-deg', type=float, default=0.0)
    parser.add_argument('--no-obstacle', action='store_true')
    parser.add_argument('--gui', action='store_true',
                        help='show Gazebo during a live demonstration')
    parser.add_argument('--max-preflight-attempts', type=int, default=20)
    parser.add_argument('--resume', action='store_true',
                        help='continue an interrupted batch without repeating completed runs')
    parser.add_argument('--max-new-runs', type=int, default=0,
                        help='stop after this many new completed runs; 0 means no limit')
    args = parser.parse_args()
    if args.runs < 1 or args.max_preflight_attempts < 1 or args.max_new_runs < 0 or not (
        1 <= args.first_domain <= 230 - args.runs * args.max_preflight_attempts):
        parser.error('invalid run count or ROS domain range')
    if shutil.which('ros2') is None:
        parser.error('source ROS and M3-1, M3-2, M3-3 workspaces')
    config = (Path('config') / f'{args.config}.yaml').resolve()
    if not config.exists():
        parser.error(f'missing {config}')
    if args.resume:
        saved = json.loads((args.out / 'experiment.json').read_text())
        for key in ('config', 'runs', 'first_domain', 'delay', 'world_x', 'world_y',
                    'size_x', 'size_y', 'goal_x', 'goal_y', 'goal_yaw_deg',
                    'no_obstacle', 'max_preflight_attempts', 'gui'):
            previous = saved.get(key, False) if key == 'gui' else saved[key]
            if previous != getattr(args, key):
                parser.error(f'--resume mismatch for {key}: saved {previous}')
        if (args.out / 'navigation.yaml').read_bytes() != config.read_bytes():
            parser.error('--resume configuration differs from saved navigation.yaml')
    else:
        args.out.mkdir(parents=True, exist_ok=False)
        shutil.copyfile(config, args.out / 'navigation.yaml')
        (args.out / 'experiment.json').write_text(json.dumps(vars(args) | {'out': str(args.out),
                                                             'config_path': str(config)}, indent=2) + '\n')
    rows = []
    new_runs = 0
    for i in range(1, args.runs + 1):
        completed = args.out / f'run_{i:02d}' / 'summary.json'
        if completed.exists():
            row = json.loads(completed.read_text())
            if row.get('goal_start_sim_s') is None:
                parser.error(f'completed run {i} has no accepted navigation goal')
            rows.append(row)
            print(f'{args.config} {i}/{args.runs}: already completed, keeping saved data',
                  flush=True)
            continue
        for attempt in range(1, args.max_preflight_attempts + 1):
            domain = args.first_domain + (i - 1) * args.max_preflight_attempts + attempt - 1
            print(f'{args.config} {i}/{args.runs}, preflight attempt {attempt}, '
                  f'ROS domain {domain}', flush=True)
            attempt_dir = args.out / 'attempts' / f'run_{i:02d}_attempt_{attempt:02d}'
            if attempt_dir.exists():
                old_summary = attempt_dir / 'summary.json'
                if old_summary.exists() and json.loads(old_summary.read_text()).get(
                        'goal_start_sim_s') is None:
                    print(f'Run {i}: prior attempt {attempt} failed before goal; '
                          'keeping its record', flush=True)
                    continue
                interrupted = args.out / 'interrupted'
                interrupted.mkdir(exist_ok=True)
                destination = interrupted / attempt_dir.name
                suffix = 2
                while destination.exists():
                    destination = interrupted / f'{attempt_dir.name}_{suffix}'
                    suffix += 1
                attempt_dir.rename(destination)
                print(f'Run {i}: moved incomplete attempt to {destination}', flush=True)
            row = run_one(i, attempt_dir, config, domain, args)
            # A trial only counts once its infrastructure passed the fixed
            # before-goal readiness test. Keep rejected attempts for audit.
            preflight_failed = row.get('goal_start_sim_s') is None
            if preflight_failed:
                if attempt == args.max_preflight_attempts:
                    raise RuntimeError(
                        f'Run {i} could not pass preflight in {attempt} fresh launches; '
                        f'see {args.out / "attempts"}')
                print(f'Run {i}: rejected before goal, restarting fresh simulation', flush=True)
                continue
            attempt_dir.rename(args.out / f'run_{i:02d}')
            row['preflight_attempts'] = attempt
            (args.out / f'run_{i:02d}' / 'summary.json').write_text(
                json.dumps(row, indent=2) + '\n')
            rows.append(row)
            new_runs += 1
            break
        if args.max_new_runs and new_runs >= args.max_new_runs:
            break
    success = sum(row.get('outcome') == 'SUCCEEDED' and
                  (args.no_obstacle or row.get('obstacle_sim_s') is not None)
                  for row in rows)
    complete = len(rows) == args.runs
    (args.out / 'summary.json').write_text(json.dumps({
        'planned_runs': args.runs, 'completed_runs': len(rows), 'complete': complete,
        'successes': success, 'success_rate': success / len(rows) if rows else None,
    }, indent=2) + '\n')
    return 0 if not complete or success == args.runs else 1


if __name__ == '__main__':
    sys.exit(main())
