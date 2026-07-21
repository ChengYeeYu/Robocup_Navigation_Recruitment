# RoboCup Nav2 Starter Kit

A TurtleBot3 + Nav2 simulation starter kit. Run it as-is and it just works
-- no code needed. If you want to write your own path-planning algorithm,
in **C++ or Python**, you can plug it in with one launch argument, editing
exactly one function.

> Instructor? See [`instructor.md`](instructor.md) for setup, maps, and
> grading tools.

---

## 1. One-time setup

On a bare Ubuntu 22.04 machine, install ROS 2 Humble, Gazebo, and Nav2:

```bash
sudo bash src/tools/install_dependencies.sh
```

Open a new terminal and check it worked:

```bash
ros2 --version
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

```bash
ros2 launch bringup bringup.launch.py
```

| Argument | Values (default first) |
|---|---|
| `world` | `world1_arena` (easier) / `world2_house` (harder, more rooms) |
| `planner` | `default` / `cpp_custom` / `python_custom` |
| `model` | `burger` / `waffle` / `waffle_pi` |
| `use_rviz` | `true` / `false` |
| `headless` | `false` / `true` -- skip the Gazebo window |

e.g. `ros2 launch bringup bringup.launch.py world:=world2_house planner:=cpp_custom`

In RViz:
1. Click **2D Pose Estimate**, then click-drag on the map where the robot
   actually is in Gazebo.
2. Click **Nav2 Goal**, then click-drag anywhere on the map you want the
   robot to go.

## 4. Rules for writing your own planner

Read this before editing anything. Both the C++ and Python tracks follow
the same contract:

| # | Rule | Detail |
|---|---|---|
| 1 | Edit only your one file | C++: [`custom_planner.cpp`](src/planner_cpp/src/custom_planner.cpp) -- just `computePlan(start, goal)`. Python: [`custom_planner.py`](src/planner_py/planner_py/custom_planner.py) -- just `generate_path(start_pose, goal_pose, grid)`. Don't touch launch files, params, `CMakeLists.txt`, or other boilerplate. |
| 2 | Keep the function signature exactly as given | Same inputs (start pose, goal pose, map/costmap data), same output (ordered waypoints). |
| 3 | Return a valid path | Waypoints ordered start → goal, real map coordinates. No path found? Return an empty list -- don't crash or throw. |
| 4 | Be fast | Must return within ~10 seconds. No infinite loops, no waiting for user input. |
| 5 | Don't touch anything else in Nav2 | No editing costmap, controller, or localization settings -- only the path-planning step is yours. |
| 6 | No new dependencies | Beyond what's already installed. |
| 7 | Don't delete the starter code | It's the straight-line fallback your algorithm is compared against. |

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

| | Python name | Python type | C++ name | C++ type | Meaning |
|---|---|---|---|---|---|
| **in** | `start_pose` | `PoseStamped` | `start` | `geometry_msgs::msg::PoseStamped` | Where the robot is now, in the `map` frame. |
| **in** | `goal_pose` | `PoseStamped` | `goal` | `geometry_msgs::msg::PoseStamped` | Where it needs to go, in the `map` frame. |
| **in** | `grid` | `OccupancyGridView` | `costmap_` (member) | `nav2_costmap_2d::Costmap2D*` | Obstacle data -- see helper methods below. |
| **out** | return value | `list` of `(x, y)` tuples | return value | `std::vector<PoseStamped>` | Ordered waypoints, start → goal, real map coords. Empty = no path found. |

A pose (`PoseStamped`) is a struct/object, not a plain number -- to get the
x/y you write `start_pose.pose.position.x` (Python) or
`start.pose.position.x` (C++), same field path in both languages.

Helper methods on `grid` / `costmap_`:

| Python | C++ | Meaning |
|---|---|---|
| `grid.is_occupied(gx, gy)` | `costmap_->getCost(mx, my)` | Is this cell blocked? |
| `grid.world_to_grid(x, y)` | `costmap_->worldToMap(...)` | Real coords → grid cell. |
| `grid.grid_to_world(gx, gy)` | `costmap_->mapToWorld(...)` | Grid cell → real coords. |

### Suggested approach (works for any algorithm)

The starter code just draws a straight line, which cuts through walls. These
steps are the general shape of a planner that actually avoids obstacles --
they apply whether you implement BFS, Dijkstra, A*, RRT, or anything else,
in either language:

1. **Convert start/goal to grid cells.** Use `grid.world_to_grid(x, y)`
   (Python) or `costmap_->worldToMap(wx, wy, mx, my)` (C++) to turn the
   poses you're given into grid coordinates you can search over.
2. **Check start and goal aren't themselves blocked.** If either cell is
   occupied, unknown, or out of bounds, return an empty path immediately --
   no algorithm can do better than that.
3. **Search outward, but only into free cells.** Whatever algorithm you
   pick, the obstacle-avoidance happens here: before adding a neighboring
   cell to your search, check `grid.is_occupied(gx, gy)` (Python) or
   `costmap_->getCost(mx, my)` (C++) and skip it if it's blocked. Never let
   the search step into an occupied cell.
4. **Remember how you reached each cell** (e.g. a `cell -> parent` map) as
   you search, so you can reconstruct the path once you reach the goal --
   most search algorithms need this regardless of how they pick which cell
   to expand next.
5. **Walk backward from the goal to the start** through those parent links
   once you reach the goal, then reverse the result so it reads start →
   goal.
6. **Convert grid cells back to world coordinates** before returning --
   `grid.grid_to_world(gx, gy)` (Python) or `costmap_->mapToWorld(mx, my,
   wx, wy)` (C++). A path in the wrong coordinate space is worse than no
   path.
7. **If the search never reaches the goal, return an empty list/vector.**
   Don't throw, don't crash -- an unreachable goal is a normal outcome.
8. **Test incrementally.** Try `world1_arena` first (open room, easiest),
   then `world2_house` (tighter doorways) -- confirm your planner still
   finds a path as obstacles get denser before you consider it done.

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

**Note:** your Python planner only sees the static map, not a live obstacle-inflated costmap like the C++ track does. If you want closer parity, use `grid.is_occupied(...)` to add your own safety margin around obstacles.

## 7. Check your planner's quality

```bash
python3 src/tools/benchmark.py --worlds world1_arena --planners cpp_custom
```

| Argument | Values (default first) |
|---|---|
| `--worlds` | `world1_arena world2_house` (space-separated, pick any) |
| `--planners` | `default cpp_custom python_custom` (space-separated, pick any) |
| `--model` | `burger` / `waffle` / `waffle_pi` |
| `--gazebo` | flag -- show Gazebo GUI (default: headless) |
| `--rviz` | flag -- also show RViz (default: off) |
| `--out` | CSV path (default: `src/tools/results/benchmark.csv`, appended to) |

Prints results to the terminal and appends them to the CSV (`score` column,
0-100 per goal). Don't run this at the same time as `bringup.launch.py` --
both start their own Gazebo, and they'll conflict.

| Counts for | Weight | Rewards |
|---|---|---|
| Path directness | 40% | Close to straight-line distance |
| Planning speed | 30% | Returning a plan quickly |
| Navigation speed | 30% | Actually driving there quickly |

Score is **0** if your planner didn't find a path, or the robot didn't
actually reach the goal. See
[`instructor.md`](instructor.md#how-grading-works) for the exact formula.

## 8. Troubleshooting

| Problem | Fix |
|---|---|
| Robot doesn't move after Nav2 Goal | Did you click **2D Pose Estimate** first? |
| `KeyError: 'TURTLEBOT3_MODEL'` | This terminal doesn't have the env var set -- run `export TURTLEBOT3_MODEL=burger` in it (see step 2). Needed in *every* new terminal, not just the one you first built in. |
| C++ changes don't show up | Rebuild (`colcon build --packages-select planner_cpp`) and `source install/setup.bash` again. |
| Gazebo won't close / next launch fails | `pkill -9 gzserver; pkill -9 gzclient`, then try again. |
