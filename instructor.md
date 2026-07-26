# Instructor Guide

Setup tasks and grading tools for running this repo as an assignment.
Freshies don't need anything in this file -- see [`README.md`](README.md)
for their side. For a deep technical dive into how the three planner
modes actually work internally, see [`structure.md`](structure.md).

---

## Building maps for a world

The repo ships a default map for each world under `src/bringup/maps/`.
Maps are **not** generated from the world layout -- they're built for
real, the same way you would on actual hardware: run SLAM, drive the
robot around, save the result. Do this once per world (or whenever a
world's layout changes).

1. If you're adding a new world, hand-write a `.world` file under
   `src/bringup/worlds/`:
   - **Official/pre-made Gazebo model** (like `world1_arena` = TB3's
     `turtlebot3_world`, `world2_house` = `turtlebot3_house`): just
     `<include>`s the model (copy an existing one as a template).
   - **Custom (box-wall) world**: author the wall `<model>` blocks
     directly (see any stock TurtleBot3 `.world` file for the SDF format).

   Either way, add a matching config file under `src/tools/config/` with
   `name`, `description`, and `robot_start_pose` (see the existing files
   for the format).
2. Launch Gazebo + SLAM for the world you want to map. Uses `slam_toolbox`
   (async online mapping mode; see `src/bringup/params/slam_toolbox_params.yaml`):
   ```bash
   ros2 launch bringup slam.launch.py world:=world1_arena
   ```
   RViz opens with the **SlamToolboxPlugin** panel (bottom-left). If drift
   builds up as you drive (the map looks shifted/doubled where you've
   already been), drive back over an already-mapped area and slam_toolbox
   will usually loop-close on its own; if it doesn't, use the panel's
   **Manual "Loop Closure"** tab to nudge two matching scans together by
   hand.
3. In another terminal (new terminal = `source install/setup.bash` and
   `export TURTLEBOT3_MODEL=burger` again, neither carries over from the
   `slam.launch.py` terminal), drive the robot around until the whole world
   is mapped:
   ```bash
   export TURTLEBOT3_MODEL=burger
   ros2 run turtlebot3_teleop teleop_keyboard
   ```
4. Save the map:
   ```bash
   ros2 run nav2_map_server map_saver_cli -f src/bringup/maps/world1_arena --ros-args -p save_map_timeout:=10.0
   ```
5. Rebuild so the new map is picked up:
   ```bash
   colcon build --symlink-install --packages-select bringup
   ```
6. **Verify the spawn point is still valid in the new map.** SLAM maps can
   shift slightly as later loop-closure corrections land, and the cell under
   the robot's start pose can end up marked occupied -- or just too close to
   a wall -- even though the robot really started there. This bit
   `world2_house` twice: the stock TB3 spawn (-2.0, -0.5) landed on an
   occupied cell, and its replacement (0.0, 0.0) sat only ~0.05 m from a wall
   (< the burger's 0.105 m radius), so the robot spawned *inside* wall
   inflation -- it couldn't move (every nav aborted) and Gazebo's physics
   flung it over. The spawn is now **(0.25, -2.33)**, the most open spot in
   the house (~2.2 m clearance). Rule of thumb: pick a spawn whose clearance
   to the nearest wall comfortably exceeds the robot radius, and confirm all
   goals are reachable from it. Sanity check after any remap:
   ```bash
   ros2 launch bringup bringup.launch.py world:=world1_arena use_rviz:=false
   # in another terminal, after it's up:
   ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
     "{header: {frame_id: 'map'}, pose: {pose: {position: {x: <spawn_x>, y: <spawn_y>}, orientation: {w: 1.0}}}}"
   ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
     "{goal: {header: {frame_id: 'map'}, pose: {position: {x: <some_goal_x>, y: <some_goal_y>}}}, use_start: false}"
   ```
   If that aborts immediately for every goal you try, the start cell itself
   is the problem -- update `robot_start_pose` in the world's config file
   (and `WORLD_SPAWN_POSE` in `src/bringup/launch/bringup.launch.py`) to a
   nearby point that works, rather than re-running SLAM.

---

## Benchmarking

`src/tools/benchmark.py` drives the full sim through every `(world, planner)`
combination you ask for, sends the same set of goals to each, and logs
the results. Freshies can also run this themselves to self-check (see
README §7) -- this section covers the full reference.

```bash
source install/setup.bash
python3 src/tools/benchmark.py
```

Runs all 2 worlds x all 3 planners (6 combinations) headless by default.
Each goal is measured two ways:
- **Planning** (`ComputePathToPose` called directly): found a path or not,
  how long planning took, and `length_ratio` = path length / straight-line
  distance (1.0 = perfectly direct, higher = more detour).
- **Full navigation** (`NavigateToPose`, same as RViz's Nav2 Goal button):
  `SUCCEEDED` / `FAILED` / `CANCELED` / `TIMEOUT`, and total time.

Appends both detail (one row per goal) and summary (one row per
`(world, planner)`) rows to a single file, `src/tools/results/benchmark.csv`
-- `row_type` (`detail`/`summary`) tells the two apart, and `run_at` groups
all rows from one invocation together. Each run **adds** rows rather than
overwriting, so successive tuning attempts on the same world/planner stay
side by side for comparison. Use `--out` to point at a different path,
e.g. one file per submission when grading (still append-on-existing).

### How grading works

Each goal gets a **score from 0-100** (`score` in the detail CSV). Each
`(world, planner)` combo gets the **average of its goals' scores**
(`avg_score` in the summary CSV) -- that's the number to use for grading a
submission.

**Gate:** if `ComputePathToPose` doesn't find a path, or `NavigateToPose`
doesn't finish `SUCCEEDED`, the goal scores **0**. A planner that can't
reliably finish the task doesn't get partial credit for being fast on the
goals it does solve.

**For goals that succeed**, the score is a weighted mix of three
components (weights live in `SCORE_WEIGHTS` near the top of
`src/tools/benchmark.py`):

| Component | Weight | Formula | Rewards |
|---|---|---|---|
| Path efficiency | 40% | `100 * grid_optimal_m / path_length_m` | A direct path, close to the **grid-optimal** length for that goal -- the shortest 8-connected free-cell path on the static map (see below). Not straight-line distance, since that's rarely achievable once walls are in the way |
| Planning speed | 30% | `100 * (1 - plan_time_sec / 10)` | Returning a plan quickly (10s budget matches README's planner rules) |
| Navigation speed | 30% | `100 * (1 - nav_time_sec / nav_timeout)` | Actually driving there quickly |

**Where the reference length comes from:** for each goal, `benchmark.py`
computes `grid_optimal_m` -- the shortest 8-connected path over free cells
of the static `/map`, start to goal (an offline A* in
`reference_optimal_length()`). This is the geometric optimum: what the best
possible planner could achieve on this map, computed once and needing no
tuning. Efficiency is `path_length_m` measured against it, so a planner that
returns near-optimal paths scores near 100. This replaced an earlier design
that ran the stock `default` (NavFn) planner first and used *its* path as the
reference -- that coupled the yardstick to costmap/controller tuning and
scored everyone 0 on any goal NavFn itself couldn't reach. If the grid
optimum can't be computed for a goal (e.g. `/map` didn't arrive, or an
endpoint is blocked), that goal falls back to the straight-line ratio
(`100 / length_ratio`, where `length_ratio = path_length_m / straight_line_m`).

Each component is clamped to `[0, 100]` before weighting, so a slow-but-direct
path and a fast-but-wandering path can end up with similar scores --
efficiency is weighted highest since it's the clearest signal of "is this
actually a good plan," but no single metric dominates completely.

**To retune:** edit `SCORE_WEIGHTS` (must sum to 1.0) or
`PLAN_TIME_BUDGET_SEC` at the top of `src/tools/benchmark.py`. No other code
changes needed -- every score is recomputed from the same underlying
`plan_time_sec`/`length_ratio`/`nav_time_sec` measurements already in the
detail CSV.

### CSV field reference

`benchmark.csv` -- `row_type` is `detail` or `summary`; each row only
populates the fields relevant to its type, the rest are blank:

```
run_at           # timestamp of the run this row came from (all rows from
                 # one invocation share the same run_at)
row_type         # detail (one per goal) or summary (one per world/planner)
world            # world this row is about
planner          # default / cpp_custom / python_custom

# --- detail rows only ---
goal_label          # label from src/tools/goals.yaml
goal_x, goal_y      # goal position, 'map' frame, meters
plan_success        # True/False -- did ComputePathToPose return a path
plan_time_sec       # time for that ComputePathToPose call to return
path_length_m       # arc length of the returned path
straight_line_m     # straight-line start->goal distance (fallback reference)
length_ratio        # path_length_m / straight_line_m
grid_optimal_m      # shortest 8-connected free-cell path length on the static
                    # map (the grid optimum); the primary efficiency reference.
                    # Blank if it couldn't be computed (then scoring falls back
                    # to length_ratio)
nav_result          # SUCCEEDED / FAILED / CANCELED / TIMEOUT
nav_time_sec        # time from sending the NavigateToPose goal to it finishing
score               # 0-100, this goal's weighted quality score -- see
                    # SCORE_WEIGHTS in benchmark.py. 0 if plan_success is
                    # False or nav_result isn't SUCCEEDED; otherwise a
                    # weighted mix of path efficiency, planning speed, and
                    # navigation speed.

# --- summary rows only ---
goals               # number of goals tested for this world/planner
plan_success_rate   # e.g. "2/2"
nav_success_rate    # e.g. "1/2"
avg_plan_time_sec   # mean plan_time_sec
avg_nav_time_sec    # mean nav_time_sec
avg_length_ratio    # mean length_ratio, over goals that planned successfully
avg_score           # mean score across all of this world's goals -- the
                     # single number for "how good is this planner here"
```

### Flags

```bash
# grade one student's submission across all worlds
python3 src/tools/benchmark.py --out src/tools/results/student_jane.csv

# just one world, watching it run (Gazebo GUI + RViz)
python3 src/tools/benchmark.py --worlds world1_arena --gazebo --rviz

# slower machine? give it more time
python3 src/tools/benchmark.py --startup-timeout 120 --nav-timeout 90
```

Test goals per world live in `src/tools/goals.yaml` -- edit to add more. Each
combination launches and tears down its own Gazebo + Nav2 stack, so a full
6-combination run takes a while (~1-2 min per combination). Use
`--worlds`/`--planners` to narrow it down while iterating.

---

## Robustness checks

The benchmark measures the happy path (good goals, does it plan a direct path
and drive there). It does **not** catch a planner that crashes, hangs, or
fabricates a route on degenerate input. Two separate tools cover that, one per
track -- both report PASS/FAIL per case and a `points`-scaled score:

| Track | Tool | How it works | Speed |
|---|---|---|---|
| Python | `src/tools/robustness_check.py` | Imports the candidate's `generate_path()` and calls it in-process against 5 edge cases on a synthetic map. Each case runs in a subprocess with a timeout, so an infinite loop is caught as HANG. | ~1s, no sim |
| C++ | `src/tools/robustness_check_cpp.py` | Launches the `cpp_custom` stack and fires the same 5 cases at the real compiled plugin via the `ComputePathToPose` action; edge-case coordinates (a wall cell, a walled-off free cell) are discovered from the live `/map`. Distinguishes a clean empty return from a CRASH by checking `planner_server` is still `active`, and a HANG via a per-case `--timeout`. | needs sim (~30-90s startup + a few s/case) |

```bash
python3 src/tools/robustness_check.py                      # Python submission
python3 src/tools/robustness_check_cpp.py                  # C++ submission
python3 src/tools/robustness_check_cpp.py --world world1_arena --timeout 15
```

The 5 cases (same for both): **unreachable** (a free goal walled off from the
start -> must return empty), **goal_on_wall**, **goal_out_map** (both -> empty),
**start_on_wall** and **goal==start** (must not crash/hang; any well-formed
return is fine). Both starter planners are a plain straight line, so both score
**2/5** out of the box -- they fabricate paths through walls and off the map by
design. A real search that gives up cleanly on impossible goals scores higher;
that gap is the point of the check. Scale `--points` to whatever you want these
worth in the overall grade.

---

## How the launch system works (short version)

`bringup.launch.py` is the single entry point for all 3 planner modes. It
always brings up Gazebo + localization (map_server + AMCL, stock Nav2) the
same way, then branches only on navigation:

- `planner:=default` / `cpp_custom` -> stock `nav2_bringup/navigation_launch.py`, params file only differs in which plugin `planner_server` loads.
- `planner:=python_custom` -> our own `navigation.launch.py`, a fork with `planner_server` removed and `planner_py`'s action server launched instead.

For the full file-by-file breakdown of all three modes -- including why
`python_custom` needed two custom behavior-tree files and a second action
server -- see [`structure.md`](structure.md).

---

## Repo layout

```
robocup_nav_rec/
└── src/
    ├── tools/
    │   ├── install_dependencies.sh   # freshie-facing: one-shot setup for a bare machine
    │   ├── ci_build_check.sh         # local pre-flight build + lint check
    │   ├── benchmark.py              # grading/self-check benchmark, see above
    │   ├── robustness_check.py       # Python-track edge-case check (in-process)
    │   ├── robustness_check_cpp.py   # C++-track edge-case check (live stack)
    │   ├── goals.yaml                # test goals per world, used by benchmark.py
    │   ├── results/                  # benchmark.py output (detail + summary CSVs)
    │   └── config/                   # per-world spawn pose + description, used by benchmark.py
    ├── bringup/                # launch files, worlds, maps, nav2 params, rviz config, behavior trees
    ├── planner_cpp/            # C++ Nav2 planner plugin (freshie edits custom_planner.cpp)
    └── planner_py/             # Python planner action server (freshie edits custom_planner.py)
```
