#!/usr/bin/env python3
"""Robustness check for a Python-track submission (instructor tool).

Runs the candidate's generate_path() against 5 edge cases on a small, fixed
synthetic map and reports PASS/FAIL per case -- no Gazebo, no ROS sim, ~1s.
Each case runs in a subprocess with a timeout so an infinite loop is caught
as a HANG (not something that wedges the grader).

Grades what the happy-path benchmark can't: does the planner give up cleanly
on impossible goals, refuse to fabricate a path through walls, and survive
bad input without crashing or hanging?

Usage (inside the ROS 2 env, from repo root):
    python3 src/tools/robustness_check.py
It imports planner_py.custom_planner -- so it tests whatever custom_planner.py
"""
import argparse
import multiprocessing as mp
import types

import numpy as np

from planner_py.occupancy_grid_view import OccupancyGridView

# expected outcomes
EMPTY = 'EMPTY'        # must return an empty path (no route exists / bad goal)
NO_CRASH = 'ANY'       # any well-formed return is fine; must not crash or hang


def build_map():
    """A 40x30 free arena with a border wall and one fully sealed room."""
    W, H, res = 40, 30, 0.05
    data = np.zeros((H, W), dtype=np.int8)
    data[0, :] = data[-1, :] = 100          # top/bottom border
    data[:, 0] = data[:, -1] = 100          # left/right border
    # a sealed room (walls all around, NO door) -> interior is free but unreachable
    x0, x1, y0, y1 = 28, 36, 8, 18
    data[y0:y1 + 1, x0] = 100
    data[y0:y1 + 1, x1] = 100
    data[y0, x0:x1 + 1] = 100
    data[y1, x0:x1 + 1] = 100
    flat = [int(v) for row in data for v in row]
    ns = types.SimpleNamespace
    msg = ns(info=ns(resolution=res, width=W, height=H,
                     origin=ns(position=ns(x=0.0, y=0.0))),
             data=flat)
    return OccupancyGridView(msg)


def pose(x, y):
    """Duck-typed PoseStamped with just the fields generate_path reads."""
    ns = types.SimpleNamespace
    return ns(pose=ns(position=ns(x=float(x), y=float(y))))


def _worker(q, start_pose, goal_pose, grid):
    try:
        from planner_py.custom_planner import generate_path
        res = generate_path(start_pose, goal_pose, grid)
        q.put(('ok', bool(res) and len(res) > 0))
    except Exception as exc:  # noqa: BLE001 -- report, don't propagate
        q.put(('exc', repr(exc)))


def run_case(start_pose, goal_pose, grid, timeout):
    """Return ('EMPTY'|'PATH'|'CRASH'|'HANG', detail)."""
    q = mp.Queue()
    p = mp.Process(target=_worker, args=(q, start_pose, goal_pose, grid))
    p.start()
    p.join(timeout)
    if p.is_alive():
        p.terminate()
        p.join()
        return 'HANG', f'>{timeout}s (infinite loop?)'
    if q.empty():
        return 'CRASH', 'process died without returning'
    kind, val = q.get()
    if kind == 'exc':
        return 'CRASH', val
    return ('PATH' if val else 'EMPTY'), ''


def grade(expected, got):
    if got in ('CRASH', 'HANG'):
        return False
    if expected == NO_CRASH:
        return True
    return got == expected


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--timeout', type=float, default=10.0,
                        help='per-case seconds before it counts as a HANG')
    parser.add_argument('--points', type=float, default=20.0,
                        help='total points these tests are worth (scaled to PASS count)')
    args = parser.parse_args()

    grid = build_map()
    gw = grid.grid_to_world
    # (label, start_pose, goal_pose, expected, what it checks)
    cases = [
        ('unreachable',   pose(*gw(5, 15)),  pose(*gw(32, 13)), EMPTY,
         'free goal sealed inside a walled room -> must give up, not fake a path'),
        ('goal_on_wall',  pose(*gw(5, 15)),  pose(*gw(0, 0)),   EMPTY,
         'goal is a wall cell -> must return empty'),
        ('start_on_wall', pose(*gw(0, 0)),   pose(*gw(6, 15)),  NO_CRASH,
         'start on an occupied cell (map noise) -> must not crash'),
        ('goal==start',   pose(*gw(5, 15)),  pose(*gw(5, 15)),  NO_CRASH,
         'trivial goal -> no divide-by-zero / no crash'),
        ('goal_out_map',  pose(*gw(5, 15)),  pose(500.0, 500.0), EMPTY,
         'goal outside the map -> must return empty, not index out of range'),
    ]

    print(f'\nRobustness check ({len(cases)} cases, {args.timeout:.0f}s timeout each)\n')
    header = f'{"case":<15} {"expected":<10} {"got":<7} {"result":<6}  note'
    print(header)
    print('-' * len(header))
    passed = 0
    for label, s, g, expected, note in cases:
        got, detail = run_case(s, g, grid, args.timeout)
        ok = grade(expected, got)
        passed += ok
        exp_disp = 'ANY' if expected == NO_CRASH else expected
        extra = f'  [{detail}]' if detail else ''
        print(f'{label:<15} {exp_disp:<10} {got:<7} {"PASS" if ok else "FAIL":<6}  {note}{extra}')

    score = round(args.points * passed / len(cases), 1)
    print('-' * len(header))
    print(f'\n{passed}/{len(cases)} passed  ->  {score}/{args.points:.0f} points\n')
    return 0 if passed == len(cases) else 1


if __name__ == '__main__':
    raise SystemExit(main())
