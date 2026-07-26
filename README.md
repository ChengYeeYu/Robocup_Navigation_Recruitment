# RoboCup Nav2 Starter Kit

>

A TurtleBot3 + Nav2 simulation starter kit. Run it as-is and it just works
-- no code needed. If you want to write your own path-planning algorithm,
in **C++ or Python**, you can plug it in with one launch argument, editing
exactly one function.

> Instructor? See [`instructor.md`](instructor.md) for setup, maps, and
> grading tools.

> **TL;DR for assignment:**
>
> 1. Pick **one** track, **C++ or Python**
>    and write your own global path planner.
> 2. **Submit your code + a short write-up** [(S8)](#8-what-to-submit). You're
>    scored on path efficiency, planning speed, and navigation success
>    ([S7](#7-check-your-planners-quality)), and separately checked for clean
>    handling of impossible/invalid goals.
> 3. Start your journey now from [(S1)](#1-one-time-setup)

## Package structure

```
navigation_recruitment/
├── README.md                  <- you are here
├── instructor.md               for instructors -- setup, maps, grading tools
├── explanation.md               the write-up you'll fill in and submit
└── src/
    ├── bringup/                  launch files, maps, and Nav2 settings -- you won't need to touch these
    │   ├── launch/
    │   │   ├── bringup.launch.py       <- this is the file you'll actually run
    │   │   ├── gazebo.launch.py          starts Gazebo and spawns the robot into it
    │   │   ├── navigation.launch.py      a variant of Nav2 used only when planner:=python_custom
    │   │   ├── rviz.launch.py            opens RViz with the map and displays already set up
    │   │   └── slam.launch.py            for building new maps -- not needed for this assignment
    │   ├── behavior_trees/*.xml        Nav2 behavior trees used by the python_custom track
    │   ├── maps/
    │   │   ├── world1_arena.yaml       one of the two worlds you're graded on (the easier one)
    │   │   └── world2_house.yaml       the other world you're graded on (harder, more rooms)
    │   ├── params/nav2_params_*.yaml   Nav2's tuning settings, one set per robot model and track
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    ├── planner_cpp/                 the C++ track
    │   ├── include/planner_cpp/
    │   │   └── custom_planner.hpp    declares the planner class -- part of the plugin scaffolding
    │   ├── src/
    │   │   └── custom_planner.cpp    <- this is the file you edit if you're doing C++
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── plugins.xml
    │
    ├── planner_py/                   the Python track
    │   ├── planner_py/
    │   │   ├── custom_planner.py             <- this is the file you edit if you're doing Python
    │   │   ├── occupancy_grid_view.py          the `grid` object you're given
    │   │   ├── compute_path_to_pose_server.py  runs your planner as a ROS action server, so you don't have to write that plumbing yourself
    │   │   └── __init__.py
    │   ├── package.xml
    │   └── setup.py
    │
    └── tools/                        setup and grading scripts -- you'll only run a couple of these yourself
        ├── install_dependencies.sh   installs everything you need; run this once, at the start
        ├── benchmark.py              run this yourself to see how your planner scores
        ├── goals.yaml                the start/goal points used when benchmarking each world
        ├── config/*.yaml             the "perfect" path lengths benchmark.py compares your planner against
        ├── robustness_check.py       instructors use this to check your Python planner handles bad goals gracefully
        └── robustness_check_cpp.py   the same check, but for the C++ planner
```

In short: you'll only ever edit one of two files -- **`custom_planner.cpp`**
if you're doing C++ ([S4](#4-rules-for-writing-your-own-planner),
[S5](#5-writing-your-own-c-planner)) or **`custom_planner.py`** if you're
doing Python ([S4](#4-rules-for-writing-your-own-planner),
[S6](#6-writing-your-own-python-planner)) -- plus writing up
`explanation.md` ([S8](#8-what-to-submit)). Everything else under `src/`
is scaffolding that makes the simulation and grading work; the only script
under `src/tools/` you'll run yourself is `benchmark.py`, to check your own
score ([S7](#7-check-your-planners-quality)).

---

## 1. One-time setup

On a bare Ubuntu 22.04 machine, install ROS 2 Humble, Gazebo, and Nav2:

```bash
sudo bash src/tools/install_dependencies.sh
```

Open a new terminal and check it worked:

```bash
echo $ROS_DISTRO
gazebo --version
```

(Already have ROS 2 Humble + Gazebo + Nav2 installed? Skip this step.)

## 2. Build the repo

```bash
git clone <this-repo-url> robocup_nav_rec
cd robocup_nav_rec
colcon build --symlink-install
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

> **Every new terminal needs both `source install/setup.bash` and
> `export TURTLEBOT3_MODEL=burger`** -- they don't carry over between
> terminal windows/tabs. Add both lines to `~/.bashrc` so new terminals
> pick them up automatically, or just re-run them each time you open one.

## 3. Run it (no code needed)

This is just a sanity check that your setup works and a chance to get a
feel for the sim -- it's not part of the assignment. You're graded on
**both** `world1_arena` and `world2_house` ([S7](#7-check-your-planners-quality)), so it's worth
launching each at least once here before you start editing anything.

```bash
ros2 launch bringup bringup.launch.py
```

You can override any of these launch arguments, e.g.
`ros2 launch bringup bringup.launch.py world:=world2_house planner:=cpp_custom`:

| Argument   | What it controls                    | Options                                                                 |
| ---------- | ----------------------------------- | ----------------------------------------------------------------------- |
| `world`    | which simulated environment loads   | `world1_arena` (default, easier) or `world2_house` (harder, more rooms) |
| `planner`  | which path planner Nav2 uses        | `default` (default, Nav2's built-in), `cpp_custom`, or `python_custom`  |
| `use_rviz` | whether RViz opens                  | `true` (default) or `false`                                             |
| `headless` | whether the Gazebo window is hidden | `false` (default, window shown) or `true` (hidden)                      |

In RViz:

1. Click **2D Pose Estimate**, then click-drag on the map where the robot
   actually is in Gazebo, matching its facing direction. Getting this
   right matters -- a wrong estimate won't throw an error, it'll just make
   navigation flaky in ways that look like a bad planner. To check it:
   the cloud of small particles around the robot should collapse tightly
   within a second or two, not stay spread out; if the laser scan display
   is on, it should hug the map's walls, not float through them.
2. Click **Nav2 Goal**, then click-drag anywhere on the map you want the
   robot to go.

## 4. Rules for writing your own planner

Read this before editing anything. Both the C++ and Python tracks follow
the same contract:

| #   | Rule                                         | Detail                                                                                                                                                                                                                                                                                                                                                          |
| --- | -------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 1   | Edit only your one file                      | C++: [`custom_planner.cpp`](src/planner_cpp/src/custom_planner.cpp) -- just `computePlan(start, goal)`. Python: [`custom_planner.py`](src/planner_py/planner_py/custom_planner.py) -- just `generate_path(start_pose, goal_pose, grid)`. Don't touch launch files, params, `CMakeLists.txt`, or other boilerplate.                                              |
| 2   | Keep the function signature exactly as given | Same inputs (start pose, goal pose, map/costmap data), same output (ordered waypoints).                                                                                                                                                                                                                                                                         |
| 3   | Return a valid path                          | Waypoints ordered start → goal, real map coordinates. No path found? Return an empty list -- don't crash or throw.                                                                                                                                                                                                                                              |
| 4   | Be fast                                      | Must return within ~10 seconds. No infinite loops, no waiting for user input.                                                                                                                                                                                                                                                                                   |
| 5   | Don't touch anything else in Nav2            | No editing costmap, controller, or localization settings -- only the path-planning step is yours.                                                                                                                                                                                                                                                               |
| 6   | No new dependencies                          | Beyond what's already installed.                                                                                                                                                                                                                                                                                                                                |
| 7   | Don't delete the surrounding boilerplate     | Only the body of `computePlan()` / `generate_path()` is yours to rewrite -- replacing its straight-line logic (which cuts through walls on purpose) is the whole point, not something to preserve. Leave the includes, lifecycle callbacks (`configure`/`activate`/...), and plugin registration untouched -- that's what makes the file a drop-in Nav2 plugin. |

### Inputs & outputs

The function signature is already written for you (in the starter code) --
you never declare these names or types yourself, you just use them:

```python
def generate_path(start_pose: PoseStamped, goal_pose: PoseStamped, grid: OccupancyGridView) -> list:
```

```cpp
std::vector<geometry_msgs::msg::PoseStamped> computePlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
```

|         | Python name  | Python type               | C++ name            | C++ type                          | Meaning                                                                  |
| ------- | ------------ | ------------------------- | ------------------- | --------------------------------- | ------------------------------------------------------------------------ |
| **in**  | `start_pose` | `PoseStamped`             | `start`             | `geometry_msgs::msg::PoseStamped` | Where the robot is now, in the `map` frame.                              |
| **in**  | `goal_pose`  | `PoseStamped`             | `goal`              | `geometry_msgs::msg::PoseStamped` | Where it needs to go, in the `map` frame.                                |
| **in**  | `grid`       | `OccupancyGridView`       | `costmap_` (member) | `nav2_costmap_2d::Costmap2D*`     | Obstacle data -- see helper methods below.                               |
| **out** | return value | `list` of `(x, y)` tuples | return value        | `std::vector<PoseStamped>`        | Ordered waypoints, start → goal, real map coords. Empty = no path found. |

A pose (`PoseStamped`) is a struct/object, not a plain number -- to get the
x/y you write `start_pose.pose.position.x` (Python) or
`start.pose.position.x` (C++), same field path in both languages.

Helper methods on `grid` / `costmap_`:

| Python                                         | C++                         | Meaning                                                                                                                                                                                                                                                                                                                                                                                      |
| ---------------------------------------------- | --------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `grid.is_occupied(gx, gy)`                     | `costmap_->getCost(mx, my)` | Is this cell blocked?                                                                                                                                                                                                                                                                                                                                                                        |
| `grid.world_to_grid(x, y)`                     | `costmap_->worldToMap(...)` | Real coords → grid cell.                                                                                                                                                                                                                                                                                                                                                                     |
| `grid.grid_to_world(gx, gy)`                   | `costmap_->mapToWorld(...)` | Grid cell → real coords.                                                                                                                                                                                                                                                                                                                                                                     |
| `grid.cost(gx, gy)` / `grid.move_cost(gx, gy)` | `costmap_->getCost(mx, my)` | Traversal cost for entering cell `(gx, gy)`: `None` on a wall, `1.0` in the open, rising near walls -- use this instead of `is_occupied` if you want your search to prefer clearance from obstacles (Python only; the C++ costmap already carries this via inflation). Both take the **same `(gx, gy)` args as `is_occupied`** -- e.g. for a neighbour tuple `n`, call `grid.move_cost(*n)`. |

### Getting unstuck

Both starters just draw a straight line, which cuts through walls -- that's
the baseline you replace, with a real search (BFS, Dijkstra, A\*, RRT,
whatever you like) in either language. The algorithm design is on you, but
a few mechanical things apply regardless of what you pick:

- **Coordinates.** Convert the poses you're given into grid cells with
  `grid.world_to_grid(x, y)` (Python) / `costmap_->worldToMap(wx, wy, mx,
my)` (C++) before searching, and convert back with `grid.grid_to_world(gx,
gy)` / `costmap_->mapToWorld(mx, my, wx, wy)` (C++) before returning -- a
  path in the wrong coordinate space is worse than no path.
- **Invalid start/goal.** If either cell is occupied, unknown, or out of
  bounds, return an empty path immediately.
- **No path found.** Return an empty list/vector -- don't throw, don't
  crash; an unreachable goal is a normal outcome.
- **Test incrementally.** Try `world1_arena` first (open room, easiest),
  then `world2_house` (tighter doorways, denser obstacles) before you
  consider it done.

## 5. Writing your own C++ planner

1. Edit [`custom_planner.cpp`](src/planner_cpp/src/custom_planner.cpp) -- the `computePlan()` function (look for the `TODO` comment).
2. Rebuild:
   ```bash
   colcon build --symlink-install --packages-select planner_cpp
   source install/setup.bash
   ```
3. Run with it:
   ```bash
   ros2 launch bringup bringup.launch.py planner:=cpp_custom
   ```

This is a standard [Nav2 planner plugin](https://navigation.ros.org/plugin_tutorials/docs/writing_new_nav2planner_plugin.html) -- everything except `computePlan()` is already written for you.

## 6. Writing your own Python planner

1. Edit [`custom_planner.py`](src/planner_py/planner_py/custom_planner.py) -- the `generate_path()` function.
2. No rebuild needed -- just re-run:
   ```bash
   ros2 launch bringup bringup.launch.py planner:=python_custom
   ```

**Note:** your Python planner only sees the static map, not a live obstacle-inflated costmap like the C++ track does. To close that gap, `grid` still gives you a _static_ approximation of wall clearance -- `grid.cost(gx, gy)`/`grid.move_cost(gx, gy)` (see the helper table in §4) return a cost that rises near walls, computed once from the map. If your search _minimizes this cost_ instead of just avoiding blocked cells, it can reach goals a plain `is_occupied`-only search would fail to drive to -- the controller can't track a plan that hugs walls too closely, so keeping clearance matters.

## 7. Check your planner's quality

```bash
python3 src/tools/benchmark.py --worlds world1_arena --planners cpp_custom
```

| Argument     | What it controls                    | Options                                                                             |
| ------------ | ----------------------------------- | ----------------------------------------------------------------------------------- |
| `--worlds`   | which world(s) to test on           | space-separated, e.g. `world1_arena world2_house` (default: `world1_arena`)         |
| `--planners` | which planner(s) to test            | space-separated, from `default`, `cpp_custom`, `python_custom` (default: `default`) |
| `--model`    | which TurtleBot3 robot is simulated | `burger` (default), `waffle`, or `waffle_pi`                                        |
| `--gazebo`   | show the Gazebo GUI                 | flag; default: hidden (headless)                                                    |
| `--rviz`     | also show RViz                      | flag; default: off                                                                  |
| `--out`      | where to write the results          | file path; default: `src/tools/results/benchmark.csv` (appended to)                 |

Prints results to the terminal and appends them to the CSV (`score` column,
0-100 per goal). Don't run this at the same time as `bringup.launch.py` --
both start their own Gazebo, and they'll conflict.

| Counts for       | Weight | Rewards                                                                                                                                         |
| ---------------- | ------ | ----------------------------------------------------------------------------------------------------------------------------------------------- |
| Path efficiency  | 40%    | Close to the **grid-optimal** length for that goal -- the shortest route any planner could take on this map (not a straight line through walls) |
| Planning speed   | 30%    | Returning a plan quickly                                                                                                                        |
| Navigation speed | 30%    | Actually driving there quickly                                                                                                                  |

Score is **0** if your planner didn't find a path, or the robot didn't
actually reach the goal. See
[`instructor.md`](instructor.md#how-grading-works) for the exact formula.

Beyond speed and directness, your planner is also checked for **robustness** on
impossible or invalid goals -- an unreachable (walled-off) goal, a goal on a
wall, a goal off the map, a start on a wall, a goal equal to the start. On
these it must **return an empty path** and must **not crash, hang, or fabricate
a route** (rules 3 and 4). The straight-line starter fails these by design;
a real search should give up cleanly.

## 8. What to submit

Your submission is **two files**:

1. **Your one planner file** for the track you chose -- **exactly one** of:
   - **C++:** [`custom_planner.cpp`](src/planner_cpp/src/custom_planner.cpp)
   - **Python:** [`custom_planner.py`](src/planner_py/planner_py/custom_planner.py)

   Pick one track and commit to it -- you are graded on a single track, not
   both. Change nothing else (see the rules in [S4](#4-rules-for-writing-your-own-planner))

2. **`explanation.md`** -- a short write-up covering:
   - **Which track** you chose (C++ or Python) and **why**.
   - **Your algorithm** -- what search you implemented (BFS, Dijkstra, A\*, ...)
     and, any relevant details.
   - **Limitations** -- where it could be faster, more optimal
     and what you'd do with more time.

Before you submit, sanity-check both:

```bash
# does it plan good paths and drive there? (pick your track)
python3 src/tools/benchmark.py --worlds world1_arena world2_house --planners cpp_custom
python3 src/tools/benchmark.py --worlds world1_arena world2_house --planners python_custom
```

## 9. Troubleshooting

| Problem                                                   | Fix                                                                                                                                                                                                                                                                       |
| --------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Robot doesn't move after Nav2 Goal                        | Did you click **2D Pose Estimate** first?                                                                                                                                                                                                                                 |
| `KeyError: 'TURTLEBOT3_MODEL'`                            | This terminal doesn't have the env var set -- run `export TURTLEBOT3_MODEL=burger` in it (see step 2). Needed in _every_ new terminal, not just the one you first built in.                                                                                               |
| C++ changes don't show up                                 | Rebuild (`colcon build --packages-select planner_cpp`) and `source install/setup.bash` again.                                                                                                                                                                             |
| Gazebo won't close / next launch fails                    | `pkill -9 gzserver; pkill -9 gzclient`, then try again.                                                                                                                                                                                                                   |
| `world2_house` first launch shows "Gazebo Not Responding" | Expected -- it's loading several meshes it hasn't cached yet, can take 1-3 minutes. Don't force-quit, just wait. (`install_dependencies.sh` pre-fetches these so this normally shouldn't happen; if it does, the fetch may have failed at install time, e.g. no network.) |
