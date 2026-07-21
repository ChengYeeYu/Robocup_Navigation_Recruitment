#!/usr/bin/env python3
"""Benchmarks each (world, planner) combo against goals.yaml and scores the results."""
import argparse
import csv
import math
import os
import signal
import subprocess
import sys
import time
from datetime import datetime

import yaml

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.normpath(os.path.join(THIS_DIR, '..', '..'))
GOALS_FILE = os.path.join(THIS_DIR, 'goals.yaml')
CONFIG_DIR = os.path.join(THIS_DIR, 'config')
RESULTS_DIR = os.path.join(THIS_DIR, 'results')

ALL_WORLDS = ['world1_arena', 'world2_house']
ALL_PLANNERS = ['default', 'cpp_custom', 'python_custom']

# Per-goal score (0-100), a weighted quality signal for grading a global
# planner. Failure (no plan, or nav doesn't SUCCEED) scores 0 -- a planner
# that can't reliably finish the task shouldn't score decently just for
# being fast on the runs it does complete. Successful runs are scored on:
#   path_efficiency -- how direct the path is (100 / length_ratio); the
#     core "is this a good plan" signal, weighted highest.
#   plan_speed -- scaled against the ~10s budget already stated in
#     README's planner rules ("must return within ~10 seconds").
#   nav_speed -- scaled against --nav-timeout.
# Retune by editing these two constants; weights must sum to 1.0.
SCORE_WEIGHTS = {
    'path_efficiency': 0.40,
    'plan_speed': 0.30,
    'nav_speed': 0.30,
}
PLAN_TIME_BUDGET_SEC = 10.0


def load_goals():
    """Load goals.yaml."""
    with open(GOALS_FILE) as f:
        return yaml.safe_load(f)


def load_start_pose(world):
    """Map-frame pose for seeding AMCL (amcl_initial_pose if set, else robot_start_pose)."""
    with open(os.path.join(CONFIG_DIR, f'{world}.yaml')) as f:
        spec = yaml.safe_load(f)
    p = spec.get('amcl_initial_pose', spec['robot_start_pose'])
    return p['x'], p['y'], p.get('yaw_deg', 0.0)


def launch_stack(world, planner, model, headless, use_rviz):
    """Launch bringup.launch.py as a background process."""
    cmd = [
        'ros2', 'launch', 'bringup', 'bringup.launch.py',
        f'world:={world}', f'planner:={planner}', f'model:={model}',
        f'use_rviz:={"true" if use_rviz else "false"}',
        f'headless:={"true" if headless else "false"}',
    ]
    proc = subprocess.Popen(
        cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid, cwd=REPO_ROOT)
    return proc


def shutdown_stack(proc, grace_sec=20):
    """Stop a launched stack, killing any leftover Gazebo processes."""
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=grace_sec)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            pass

    # gzserver can outlive SIGKILL to its process group; a leftover one
    # holds Gazebo's fixed ports and breaks the next combo's launch
    subprocess.run(
        ['pkill', '-9', '-f', 'gzserver|gzclient'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def _wait_for_node_active(navigator, node_name, deadline):
    """Bounded, single-threaded wait for a lifecycle node to reach 'active'."""
    from lifecycle_msgs.srv import GetState
    import rclpy

    client = navigator.create_client(GetState, f'{node_name}/get_state')
    while time.time() < deadline:
        if client.wait_for_service(timeout_sec=1.0):
            break
    else:
        return False

    while time.time() < deadline:
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(navigator, future, timeout_sec=2.0)
        result = future.result()
        if result is not None and result.current_state.label == 'active':
            return True
        time.sleep(0.5)
    return False


def publish_initial_pose(navigator, x, y, yaw_deg):
    """Publish /initialpose with a small deliberate (non-zero) covariance."""
    from geometry_msgs.msg import PoseWithCovarianceStamped
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = navigator.get_clock().now().to_msg()
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    yaw = math.radians(yaw_deg)
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    msg.pose.covariance[0] = 0.0225   # x variance, ~0.15m std dev
    msg.pose.covariance[7] = 0.0225   # y variance
    msg.pose.covariance[35] = 0.003   # yaw variance, ~3 deg std dev
    navigator.initial_pose_pub.publish(msg)


def _wait_for_initial_pose(navigator, deadline, x, y, yaw_deg):
    """Republish the initial pose until AMCL acknowledges it."""
    import rclpy
    while time.time() < deadline and not navigator.initial_pose_received:
        publish_initial_pose(navigator, x, y, yaw_deg)
        rclpy.spin_once(navigator, timeout_sec=1.0)
    return navigator.initial_pose_received


def wait_until_nav2_active(navigator, timeout_sec, x, y, yaw_deg):
    """Bounded equivalent of navigator.waitUntilNav2Active()."""
    deadline = time.time() + timeout_sec
    if not _wait_for_node_active(navigator, 'amcl', deadline):
        return False
    if not _wait_for_initial_pose(navigator, deadline, x, y, yaw_deg):
        return False
    return _wait_for_node_active(navigator, 'bt_navigator', deadline)


def path_length(path):
    """Sum the Euclidean distance between consecutive path poses."""
    pts = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
    return sum(math.hypot(x2 - x1, y2 - y1) for (x1, y1), (x2, y2) in zip(pts, pts[1:]))


def compute_score(row, nav_timeout):
    """Compute a goal's 0-100 weighted quality score (see SCORE_WEIGHTS above)."""
    if not row['plan_success'] or row['nav_result'] != 'SUCCEEDED':
        return 0.0
    ratio = row['length_ratio']
    efficiency = 100.0 if ratio in ('', None) else max(0.0, min(100.0, 100.0 / ratio))
    plan_speed = max(0.0, min(100.0, 100.0 * (1.0 - row['plan_time_sec'] / PLAN_TIME_BUDGET_SEC)))
    nav_speed = max(0.0, min(100.0, 100.0 * (1.0 - row['nav_time_sec'] / nav_timeout)))
    return round(
        SCORE_WEIGHTS['path_efficiency'] * efficiency
        + SCORE_WEIGHTS['plan_speed'] * plan_speed
        + SCORE_WEIGHTS['nav_speed'] * nav_speed,
        1)


def make_pose_stamped(navigator, x, y, yaw_deg=0.0):
    """Build a PoseStamped in the map frame from x/y/yaw_deg."""
    from geometry_msgs.msg import PoseStamped
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    yaw = math.radians(yaw_deg)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def run_combo(world, planner, model, goals, headless, use_rviz, startup_timeout, nav_timeout):
    """Launch one (world, planner) combo and test it against all its goals."""
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

    rows = []
    print(f'\n=== {world} / {planner} ===')
    proc = launch_stack(world, planner, model, headless, use_rviz)
    navigator = BasicNavigator()
    # must match the stack's sim clock or AMCL rejects our stamped messages
    from rclpy.parameter import Parameter
    navigator.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    try:
        sx, sy, syaw = load_start_pose(world)
        start_pose = make_pose_stamped(navigator, sx, sy, syaw)

        print(f'  waiting up to {startup_timeout:.0f}s for the sim to become active...')
        ready = wait_until_nav2_active(navigator, startup_timeout, sx, sy, syaw)
        if not ready:
            print('  STARTUP TIMEOUT')
            rows.append({
                'world': world, 'planner': planner, 'goal_label': '',
                'goal_x': '', 'goal_y': '', 'plan_success': False,
                'plan_time_sec': '', 'path_length_m': '', 'straight_line_m': '',
                'length_ratio': '', 'nav_result': 'STARTUP_TIMEOUT', 'nav_time_sec': '',
                'score': 0.0,
            })
            return rows

        for goal_spec in goals:
            label = goal_spec.get('label', '')
            gx, gy = goal_spec['x'], goal_spec['y']
            goal_pose = make_pose_stamped(navigator, gx, gy)
            print(f'  goal "{label}" ({gx}, {gy})...')

            # --- planning-only metrics (ComputePathToPose) ---
            t0 = time.time()
            path = navigator.getPath(start_pose, goal_pose, planner_id='GridBased', use_start=False)
            plan_time = time.time() - t0
            plan_success = path is not None and len(path.poses) > 0
            if plan_success:
                length = path_length(path)
                p0 = path.poses[0].pose.position
                straight = math.hypot(gx - p0.x, gy - p0.y)
                ratio = (length / straight) if straight > 1e-6 else ''
            else:
                length = straight = ratio = ''
            print(f'    plan: success={plan_success} time={plan_time:.2f}s '
                  f'length={length if length == "" else round(length, 2)}')

            # --- full navigation metrics (NavigateToPose) ---
            nav_start = time.time()
            navigator.goToPose(goal_pose)
            timed_out = False
            while not navigator.isTaskComplete():
                if time.time() - nav_start > nav_timeout:
                    navigator.cancelTask()
                    timed_out = True
                    break
            nav_time = time.time() - nav_start
            if timed_out:
                nav_result = 'TIMEOUT'
                time.sleep(1.0)
            else:
                result = navigator.getResult()
                nav_result = {
                    TaskResult.SUCCEEDED: 'SUCCEEDED',
                    TaskResult.FAILED: 'FAILED',
                    TaskResult.CANCELED: 'CANCELED',
                }.get(result, 'UNKNOWN')
            print(f'    nav:  result={nav_result} time={nav_time:.2f}s')

            row = {
                'world': world, 'planner': planner, 'goal_label': label,
                'goal_x': gx, 'goal_y': gy,
                'plan_success': plan_success, 'plan_time_sec': round(plan_time, 3),
                'path_length_m': round(length, 3) if length != '' else '',
                'straight_line_m': round(straight, 3) if straight != '' else '',
                'length_ratio': round(ratio, 3) if ratio != '' else '',
                'nav_result': nav_result, 'nav_time_sec': round(nav_time, 3),
            }
            row['score'] = compute_score(row, nav_timeout)
            print(f'    score: {row["score"]}/100')
            rows.append(row)
    finally:
        try:
            navigator.destroy_node()
        except Exception:
            pass
        shutdown_stack(proc)
        time.sleep(5)  # let ports/resources fully release before the next combo
    return rows


FIELDNAMES = [
    'world', 'planner', 'goal_label', 'goal_x', 'goal_y',
    'plan_success', 'plan_time_sec', 'path_length_m', 'straight_line_m',
    'length_ratio', 'nav_result', 'nav_time_sec', 'score',
]

SUMMARY_FIELDNAMES = [
    'world', 'planner', 'goals', 'plan_success_rate', 'nav_success_rate',
    'avg_plan_time_sec', 'avg_nav_time_sec', 'avg_length_ratio', 'avg_score',
]

# Unified schema for the one results CSV -- detail and summary rows share a
# single file (row_type distinguishes them), and each run is APPENDED
# rather than overwriting, so successive tuning attempts on the same
# world/planner stay comparable instead of erasing each other. run_at
# groups all rows from one invocation together.
CSV_FIELDNAMES = ['run_at', 'row_type'] + FIELDNAMES + [
    'goals', 'plan_success_rate', 'nav_success_rate',
    'avg_plan_time_sec', 'avg_nav_time_sec', 'avg_length_ratio', 'avg_score',
]


def print_table(rows):
    """Print the detail rows as an aligned table."""
    if not rows:
        print('(no results)')
        return
    widths = {f: max(len(f), *(len(str(r.get(f, ''))) for r in rows)) for f in FIELDNAMES}
    header = ' | '.join(f.ljust(widths[f]) for f in FIELDNAMES)
    print(header)
    print('-' * len(header))
    for r in rows:
        print(' | '.join(str(r.get(f, '')).ljust(widths[f]) for f in FIELDNAMES))


def print_summary(rows):
    """Print and return one aggregated row per (world, planner)."""
    print('\n=== Summary (per world / planner) ===')
    combos = sorted({(r['world'], r['planner']) for r in rows})
    summary_rows = []
    for world, planner in combos:
        subset = [r for r in rows if r['world'] == world and r['planner'] == planner]
        n = len(subset)
        plan_ok = [r for r in subset if r['plan_success']]
        nav_ok = [r for r in subset if r['nav_result'] == 'SUCCEEDED']
        ratios = [r['length_ratio'] for r in plan_ok if r['length_ratio'] != '']
        scores = [r['score'] for r in subset if isinstance(r['score'], (int, float))]
        summary_rows.append({
            'world': world, 'planner': planner, 'goals': n,
            'plan_success_rate': f'{len(plan_ok)}/{n}',
            'nav_success_rate': f'{len(nav_ok)}/{n}',
            'avg_plan_time_sec': round(sum(r['plan_time_sec'] for r in subset if isinstance(r['plan_time_sec'], (int, float))) / n, 3) if n else '',
            'avg_nav_time_sec': round(sum(r['nav_time_sec'] for r in subset if isinstance(r['nav_time_sec'], (int, float))) / n, 3) if n else '',
            'avg_length_ratio': round(sum(ratios) / len(ratios), 3) if ratios else '',
            'avg_score': round(sum(scores) / len(scores), 1) if scores else '',
        })
    widths = {f: max(len(f), *(len(str(r.get(f, ''))) for r in summary_rows)) for f in SUMMARY_FIELDNAMES}
    header = ' | '.join(f.ljust(widths[f]) for f in SUMMARY_FIELDNAMES)
    print(header)
    print('-' * len(header))
    for r in summary_rows:
        print(' | '.join(str(r.get(f, '')).ljust(widths[f]) for f in SUMMARY_FIELDNAMES))
    return summary_rows


def main():
    """Parse args, run every requested (world, planner) combo, print and save results."""
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--worlds', nargs='+', default=ALL_WORLDS, choices=ALL_WORLDS)
    parser.add_argument('--planners', nargs='+', default=ALL_PLANNERS, choices=ALL_PLANNERS)
    parser.add_argument('--model', default='burger', choices=['burger', 'waffle', 'waffle_pi'])
    parser.add_argument('--gazebo', action='store_true', help='show Gazebo GUI (default: headless)')
    parser.add_argument('--rviz', action='store_true', help='also show RViz (default: off)')
    parser.add_argument('--startup-timeout', type=float, default=240.0,
                         help='world2_house (turtlebot3_house) alone can take ~2.5min '
                              'to spawn on first load; default is generous to cover that')
    parser.add_argument('--nav-timeout', type=float, default=60.0)
    parser.add_argument('--out', default=None,
                         help='CSV path (default: src/tools/results/benchmark.csv). '
                              'Appended to, not overwritten, if it already exists.')
    args = parser.parse_args()

    if 'ROS_DISTRO' not in os.environ:
        print('ERROR: ROS 2 environment not sourced. Run:\n'
              '  source /opt/ros/humble/setup.bash && source install/setup.bash', file=sys.stderr)
        sys.exit(1)

    os.makedirs(RESULTS_DIR, exist_ok=True)
    # one fixed file, appended to (not overwritten) -- see CSV_FIELDNAMES
    out_path = args.out or os.path.join(RESULTS_DIR, 'benchmark.csv')

    import rclpy
    rclpy.init()

    goals_by_world = load_goals()
    all_rows = []
    try:
        for world in args.worlds:
            for planner in args.planners:
                rows = run_combo(
                    world, planner, args.model, goals_by_world.get(world, []),
                    headless=not args.gazebo,
                    use_rviz=args.rviz,
                    startup_timeout=args.startup_timeout,
                    nav_timeout=args.nav_timeout)
                all_rows.extend(rows)
    finally:
        rclpy.shutdown()

    print('\n=== Detail ===')
    print_table(all_rows)
    summary_rows = print_summary(all_rows)

    run_at = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    csv_rows = [{'run_at': run_at, 'row_type': 'detail', **r} for r in all_rows]
    csv_rows += [{'run_at': run_at, 'row_type': 'summary', **r} for r in summary_rows]

    write_header = not os.path.exists(out_path) or os.path.getsize(out_path) == 0
    with open(out_path, 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDNAMES)
        if write_header:
            writer.writeheader()
        writer.writerows(csv_rows)

    print(f'\nAppended this run to: {out_path}')


if __name__ == '__main__':
    main()
