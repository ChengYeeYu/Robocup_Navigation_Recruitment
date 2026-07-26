#!/usr/bin/env python3
"""Robustness check for a C++-track submission (instructor tool).

This launches the cpp_custom Nav2 stack (exercises whatever custom_planner.cpp is currently built into cpp_custom) 
and fires the same 5 edge-case goals at the REAL plugin via the ComputePathToPose
action (getPath), reporting PASS/FAIL per case. Because it drives the live
stack it needs a running sim (~30-90s startup + a few seconds per case)

Each case is judged by three observable outcomes:
  * PATH  -- planner returned a non-empty path
  * EMPTY -- planner returned no path AND planner_server is still 'active'
             (a clean give-up: correct for impossible/bad goals)
  * CRASH -- planner_server is no longer 'active' after the call (segfault etc.)
  * HANG  -- getPath didn't return within --timeout (infinite loop)

Edge-case coordinates (a wall cell, an enclosed/unreachable free cell) are
discovered from the live /map at runtime, so it adapts to whichever --world
you point it at. If the map has no sealed-off free region the 'unreachable'
case is skipped with a note.

Usage (inside the ROS 2 env, from repo root):
    python3 src/tools/robustness_check_cpp.py
"""

import argparse
import os
import sys
import threading
import time
from collections import deque

import numpy as np
import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import benchmark as B

EMPTY = 'EMPTY'        # must give up cleanly (no route / bad goal)
NO_CRASH = 'ANY'       # any well-formed return is fine; must not crash or hang


def planner_active(navigator, timeout=3.0):
    """True if planner_server's lifecycle state is still 'active'."""
    from lifecycle_msgs.srv import GetState
    client = navigator.create_client(GetState, 'planner_server/get_state')
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            return False
        fut = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(navigator, fut, timeout_sec=timeout)
        res = fut.result()
        return res is not None and res.current_state.label == 'active'
    finally:
        navigator.destroy_client(client)


def find_edge_cells(gv, start_xy):
    """Discover (wall_world, unreachable_world_or_None) from the static map."""
    data = np.asarray(gv.data).reshape(gv.height, gv.width)
    occupied = (data < 0) | (data >= 50)
    free = ~occupied

    # a wall cell nearest the map centre (robust: always exists via borders)
    wall_world = None
    ys, xs = np.nonzero(occupied)
    if len(xs):
        cx, cy = gv.width / 2.0, gv.height / 2.0
        i = int(np.argmin((xs - cx) ** 2 + (ys - cy) ** 2))
        wall_world = gv.grid_to_world(int(xs[i]), int(ys[i]))

    # an enclosed free cell: flood-fill free space from the start, then any
    # free cell not reached is walled off (correct answer = EMPTY, unreachable)
    sgx, sgy = gv.world_to_grid(*start_xy)
    unreachable_world = None
    if 0 <= sgx < gv.width and 0 <= sgy < gv.height and free[sgy, sgx]:
        seen = np.zeros_like(free)
        q = deque([(sgx, sgy)])
        seen[sgy, sgx] = True
        while q:
            x, y = q.popleft()
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = x + dx, y + dy
                if 0 <= nx < gv.width and 0 <= ny < gv.height \
                        and free[ny, nx] and not seen[ny, nx]:
                    seen[ny, nx] = True
                    q.append((nx, ny))
        walled = free & ~seen
        if walled.any():
            wy, wx = np.argwhere(walled)[len(np.argwhere(walled)) // 2]
            unreachable_world = gv.grid_to_world(int(wx), int(wy))
    return wall_world, unreachable_world


def run_case(navigator, start_pose, goal_pose, timeout):
    """Return ('PATH'|'EMPTY'|'CRASH'|'HANG', detail) for one getPath call."""
    holder = {}

    def call():
        try:
            holder['path'] = navigator.getPath(
                start_pose, goal_pose, planner_id='GridBased', use_start=True)
        except Exception as exc:  # noqa: BLE001
            holder['exc'] = repr(exc)
        finally:
            holder['done'] = True

    t = threading.Thread(target=call, daemon=True)
    t.start()
    t.join(timeout)
    if t.is_alive():
        return 'HANG', f'>{timeout:.0f}s (infinite loop?)'
    if not planner_active(navigator):
        return 'CRASH', holder.get('exc', 'planner_server no longer active')
    path = holder.get('path')
    if path is not None and len(path.poses) > 0:
        return 'PATH', ''
    return 'EMPTY', ''


def grade(expected, got):
    if got in ('CRASH', 'HANG'):
        return False
    if expected == NO_CRASH:
        return True
    return got == expected


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--world', default='world2_house',
                        help='world to run the live stack in (default: world2_house)')
    parser.add_argument('--model', default='burger')
    parser.add_argument('--timeout', type=float, default=15.0,
                        help='per-case seconds before it counts as a HANG')
    parser.add_argument('--points', type=float, default=20.0,
                        help='total points these tests are worth (scaled to PASS count)')
    parser.add_argument('--startup-timeout', type=float, default=240.0)
    args = parser.parse_args()

    rclpy.init()
    from nav2_simple_commander.robot_navigator import BasicNavigator
    from rclpy.parameter import Parameter

    proc = B.launch_stack(args.world, 'cpp_custom', args.model,
                          headless=True, use_rviz=False)
    navigator = BasicNavigator()
    navigator.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

    try:
        sx, sy, syaw = B.load_start_pose(args.world)
        print(f'\nRobustness check (C++ / live stack) -- world={args.world} '
              f'planner=cpp_custom, {args.timeout:.0f}s timeout/case')
        print(f'waiting up to {args.startup_timeout:.0f}s for the sim to become active...')
        if not B.wait_until_nav2_active(navigator, args.startup_timeout, sx, sy, syaw):
            print('STARTUP TIMEOUT -- could not bring up the stack')
            return 1

        gv = B._load_grid_view(navigator)
        if gv is None:
            print('could not load /map -- cannot build edge cases')
            return 1
        wall_world, unreachable_world = find_edge_cells(gv, (sx, sy))

        free_start = B.make_pose_stamped(navigator, sx, sy, syaw)
        # (label, start_pose, goal_pose_or_None, expected, note)
        cases = []
        if unreachable_world is not None:
            cases.append(('unreachable', free_start,
                          B.make_pose_stamped(navigator, *unreachable_world), EMPTY,
                          'free goal walled off from start -> must give up, not fake a path'))
        else:
            cases.append(('unreachable', None, None, None,
                          'SKIPPED: this map has no sealed-off free region to test'))
        if wall_world is not None:
            cases.append(('goal_on_wall', free_start,
                          B.make_pose_stamped(navigator, *wall_world), EMPTY,
                          'goal is a wall cell -> must return empty'))
            cases.append(('start_on_wall',
                          B.make_pose_stamped(navigator, *wall_world), free_start, NO_CRASH,
                          'start on an occupied cell -> must not crash'))
        cases.append(('goal==start', free_start, free_start, NO_CRASH,
                      'trivial goal -> no crash / no divide-by-zero'))
        cases.append(('goal_out_map', free_start,
                      B.make_pose_stamped(navigator, 500.0, 500.0), EMPTY,
                      'goal outside the map -> empty, not an out-of-range crash'))

        header = f'{"case":<15} {"expected":<10} {"got":<7} {"result":<6}  note'
        print('\n' + header)
        print('-' * len(header))
        passed = graded = 0
        for label, s, g, expected, note in cases:
            if expected is None:  # skipped
                print(f'{label:<15} {"-":<10} {"SKIP":<7} {"-":<6}  {note}')
                continue
            graded += 1
            got, detail = run_case(navigator, s, g, args.timeout)
            ok = grade(expected, got)
            passed += ok
            exp_disp = 'ANY' if expected == NO_CRASH else expected
            extra = f'  [{detail}]' if detail else ''
            print(f'{label:<15} {exp_disp:<10} {got:<7} '
                  f'{"PASS" if ok else "FAIL":<6}  {note}{extra}')

        score = round(args.points * passed / graded, 1) if graded else 0.0
        print('-' * len(header))
        print(f'\n{passed}/{graded} passed  ->  {score}/{args.points:.0f} points\n')
        return 0 if graded and passed == graded else 1
    finally:
        try:
            navigator.destroy_node()
        except Exception:
            pass
        B.shutdown_stack(proc)
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
