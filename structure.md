# structure.md — How the Default and Custom Planners Work

Deep technical reference for instructors. Explains exactly what happens
for `planner:=default`, `planner:=cpp_custom`, and `planner:=python_custom`,
file by file. Freshies don't need this -- see [`README.md`](README.md).
For setup/grading tasks, see [`instructor.md`](instructor.md).

---

## 1. The single entry point

Every mode starts the same way:

```bash
ros2 launch bringup bringup.launch.py world:=<world> planner:=<planner> model:=<model>
```

[`bringup.launch.py`](src/bringup/launch/bringup.launch.py) does four things, in order, for **every** value of `planner`:

```
bringup.launch.py
├── 1. gazebo.launch.py          (Gazebo + robot spawn — identical in all 3 modes)
├── 2. nav2_bringup/localization_launch.py   (map_server + AMCL — identical in all 3 modes)
├── 3. ONE of:
│     ├── nav2_bringup/navigation_launch.py     (planner:=default)
│     ├── nav2_bringup/navigation_launch.py     (planner:=cpp_custom, different params file)
│     └── bringup/navigation.launch.py    (planner:=python_custom, forked launch file)
└── 4. rviz.launch.py            (RViz — identical in all 3 modes, skippable with use_rviz:=false)
```

Step 3 is the only place the three modes diverge. Steps 1, 2, and 4 are
byte-for-byte the same regardless of `planner:=`.

**How the branching actually works:** `bringup.launch.py` declares all
three `IncludeLaunchDescription` actions for step 3, each guarded by
`condition=LaunchConfigurationEquals('planner', '<value>')`. At launch time,
only the one whose value matches actually runs; the other two are no-ops.
This is why nothing needs an `if/else` in Python — it's a launch-time
condition evaluated by the `launch` framework itself.

**World and map selection:** `world:=world1_arena` (etc.) is resolved into
two file paths via `PathJoinSubstitution`:
- `src/bringup/worlds/<world>.world` — passed to Gazebo.
- `src/bringup/maps/<world>.yaml` — passed to `map_server`.

Both are just string concatenation done at launch time; nothing generates
them on the fly. See [`instructor.md`](instructor.md) for how the
`.world`/`.yaml` files themselves get created (world: scripted from a
config file; map: SLAM + `map_saver_cli`, run manually by the instructor).

**Model selection:** `model:=burger` (etc., default from `$TURTLEBOT3_MODEL`)
picks which of the 9 params files in `src/bringup/params/` gets used
(`nav2_params_<model>_default.yaml`, `nav2_params_<model>_custom_cpp.yaml`,
or `nav2_params_<model>_custom_python.yaml`, selected together with
`planner:=`).

---

## 2. Steps 1, 2, 4 — identical across all modes

### Step 1: `gazebo.launch.py`

Starts `gzserver` + `gzclient` on the selected `.world` file, then includes
two **unmodified stock TurtleBot3 launch files**:
- `turtlebot3_gazebo/launch/robot_state_publisher.launch.py` — publishes the
  robot's URDF/TF tree.
- `turtlebot3_gazebo/launch/spawn_turtlebot3.launch.py` — spawns the robot
  model into Gazebo at `(x_pose, y_pose)`.

Two things worth knowing:
- Those two stock files read `TURTLEBOT3_MODEL` from the **process
  environment variable**, not a launch argument. So `gazebo.launch.py`
  uses `SetEnvironmentVariable('TURTLEBOT3_MODEL', model)` to make our
  `model:=` launch argument actually take effect for them.
- The `gzserver.launch.py` we include (from `gazebo_ros`) declares its own
  launch argument also named `world`. Launch arguments are global by name
  across an entire launch tree, so passing our world's `.world` file path
  under the key `'world'` would silently overwrite *our* `world:=world1_arena`
  argument for the rest of the launch, breaking the later map-path lookup
  (which also reads `world`). Fixed by wrapping that one
  `IncludeLaunchDescription` in a scoped `GroupAction`, containing the leak
  to just that sub-launch.
- `x_pose`/`y_pose` per world come from a small `WORLD_SPAWN_POSE` dict in
  `bringup.launch.py`, resolved via `OpaqueFunction` (needed because the
  `world` argument's value isn't known until launch time, so a plain
  Python dict lookup can't happen at import time).

### Step 2: `nav2_bringup/localization_launch.py` (100% stock, unmodified)

Standard Nav2 localization: `map_server` (loads the `.yaml`/`.pgm` map),
`amcl` (particle-filter localization against `/scan`), and
`lifecycle_manager_localization` (brings both up through
unconfigured → inactive → active). This never changes based on `planner:=`
— your custom planner never touches localization.

### Step 4: `rviz.launch.py`

Thin wrapper around `rviz2` using `src/bringup/rviz/default_view.rviz`
(a copy of nav2_bringup's stock view). Skippable with `use_rviz:=false`
(used during automated testing, since there's no point rendering a GUI).

---

## 3. `planner:=default` — stock Nav2, nothing of ours involved

Step 3 includes **unmodified** `nav2_bringup/navigation_launch.py` with
`params_file` set to `nav2_params_<model>_default.yaml`. That launch file starts:

```
controller_server   (DWB local planner — follows the path, avoids obstacles)
smoother_server     (light path smoothing after planning)
planner_server      (GLOBAL PATH PLANNING — see below)
behavior_server     (recovery behaviors: spin, back up, wait, etc.)
bt_navigator        (runs the Behavior Tree that sequences the above)
waypoint_follower
velocity_smoother
lifecycle_manager_navigation   (brings all the above to "active")
```

**The global planner in this mode** is `nav2_navfn_planner/NavfnPlanner`, a
stock Nav2 plugin (Dijkstra/A* grid search over the costmap) — set in
[`nav2_params_burger_default.yaml`](src/bringup/params/nav2_params_burger_default.yaml)
under `planner_server.GridBased.plugin`. None of our code runs anywhere in
this mode; it's exactly what you'd get from `nav2_bringup` directly, just
pointed at our world/map/robot.

**Request flow** (what happens when RViz's "Nav2 Goal" is clicked):
1. RViz publishes a `PoseStamped` on `/goal_pose`.
2. `bt_navigator`'s active Behavior Tree
   (`navigate_to_pose_w_replanning_and_recovery.xml`, stock, unmodified)
   calls the `ComputePathToPose` BT node, which calls the
   `compute_path_to_pose` **action**.
3. `planner_server` is the only thing listening on that action name in this
   mode. It runs `NavfnPlanner::createPlan(start, goal)` against the live
   `global_costmap` (inflated, obstacle-aware) and returns a `nav_msgs/Path`.
4. The BT feeds that path into `FollowPath`, served by `controller_server`
   (DWB), which streams `cmd_vel` commands to drive the robot.
5. This repeats at 1 Hz (replanning) until the goal is reached.

---

## 4. `planner:=cpp_custom` — pluginlib swap inside `planner_server`

Step 3 includes the **same** stock `navigation_launch.py` as default mode —
the only difference is `params_file` is
[`nav2_params_<model>_custom_cpp.yaml`](src/bringup/params/nav2_params_burger_custom_cpp.yaml),
which differs from the default params file in exactly one block:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "planner_cpp/CustomPlanner"   # <- was nav2_navfn_planner/NavfnPlanner
      interpolation_resolution: 0.1
```

**Node topology is identical to default mode** — same 8 processes, same
`planner_server` executable. Only *which shared library `planner_server`
dlopens for its `GridBased` plugin* differs.

### How the plugin gets discovered and loaded

1. `src/planner_cpp/CMakeLists.txt` compiles
   [`custom_planner.cpp`](src/planner_cpp/src/custom_planner.cpp)
   into `libplanner_cpp_plugin.so`, and calls
   `pluginlib_export_plugin_description_file(nav2_core plugins.xml)`. This
   registers
   [`plugins.xml`](src/planner_cpp/plugins.xml) — which maps the string
   ID `"planner_cpp/CustomPlanner"` to the C++ class
   `planner_cpp::CustomPlanner` — into the ament resource index under
   `share/ament_index/resource_index/nav2_core__pluginlib__plugin/`.
2. At startup, `planner_server` reads `GridBased.plugin` from the params
   file, asks `pluginlib::ClassLoader<nav2_core::GlobalPlanner>` to load
   that string ID, which searches the resource index across every
   installed package, finds `planner_cpp`'s entry, `dlopen`s
   `libplanner_cpp_plugin.so`, and instantiates `CustomPlanner`.
3. `planner_server` then calls the same `nav2_core::GlobalPlanner`
   lifecycle any plugin gets: `configure()` → `activate()` → repeated
   `createPlan(start, goal)` calls.

### What's boilerplate vs. what freshies edit, in `custom_planner.cpp`

| Method | Who wrote it | What it does |
|---|---|---|
| `configure()` | boilerplate | Stores a weak pointer to the parent node, caches `costmap_ = costmap_ros->getCostmap()` and `global_frame_`, declares the `interpolation_resolution` parameter (default `0.1`). |
| `cleanup()` / `activate()` / `deactivate()` | boilerplate | Log statements only — required by the interface, nothing to configure. |
| `createPlan(start, goal)` | boilerplate | Validates both poses are in `global_frame_`, then calls `computePlan()` and wraps the result in a `nav_msgs::msg::Path` with a proper header. |
| `computePlan(start, goal)` | **freshie edits this** | Out-of-the-box: straight-line interpolation at `interpolation_resolution_` spacing between start and goal, with a per-point `costmap_->getCost(mx, my) >= nav2_costmap_2d::LETHAL_OBSTACLE` check that logs a `WARN` (but does not reroute) if the line crosses an obstacle. Freshies replace this with a real search algorithm (BFS/Dijkstra/A*/RRT/etc.), using `costmap_` for collision checks. |

The bottom of the file has `PLUGINLIB_EXPORT_CLASS(planner_cpp::CustomPlanner, nav2_core::GlobalPlanner)`
— this macro is what actually generates the `dlopen`-visible factory
function pluginlib calls into; freshies never need to touch it.

**Request flow** is identical to §3 above, except step 3 (`createPlan`)
runs our `CustomPlanner::computePlan()` instead of `NavfnPlanner`'s
internal Dijkstra search. Everything downstream (smoothing, BT, DWB
control) is unchanged.

---

## 5. `planner:=python_custom` — a standalone node replaces `planner_server` entirely

This is the mode with real launch-graph differences, because Nav2
pluginlib plugins must be C++ — there is no way to `dlopen` Python. Instead,
step 3 includes `bringup`'s own
[`navigation.launch.py`](src/bringup/launch/navigation.launch.py),
a **forked copy** of stock `navigation_launch.py` with three changes:

```
controller_server    (same as default)
smoother_server       (same as default)
custom_planner     (planner_py's node -- REPLACES nav2_planner/planner_server)
behavior_server       (same as default)
bt_navigator          (same as default, but pointed at trimmed BT XMLs -- see below)
waypoint_follower     (same as default)
velocity_smoother     (same as default)
lifecycle_manager_navigation   (node_names list has "planner_server" REMOVED)
```

`planner_server` (the `nav2_planner` executable) is **never started** in
this mode. There is consequently **no `global_costmap` process** either,
since `global_costmap` lives inside `planner_server` (it's not a separate
node) — this has a real, documented consequence, see the costmap caveat
below.

### Why this works from the BT Navigator's point of view

`bt_navigator`'s Behavior Tree doesn't know or care which node is behind
the `compute_path_to_pose` action — it just calls the action by name. Our
node ([`compute_path_to_pose_server.py`](src/planner_py/planner_py/compute_path_to_pose_server.py),
run under the ROS node name `custom_planner`) advertises an
`ActionServer` under that exact name, so it's a transparent substitute.
It's a **plain `rclpy.Node`, not a `nav2_util::LifecycleNode`** — it has no
configure/activate states, it just starts serving the action the moment it
spins up. That's why it's *not* in `lifecycle_manager_navigation`'s
`node_names` list — there's no lifecycle to manage.

### Why bt_navigator ALSO needs `compute_path_through_poses`

The BT Navigator internally maintains two Behavior Trees: one for single
`NavigateToPose` goals, one for multi-pose `NavigateThroughPoses` goals. It
loads **both** at configure time regardless of which one you actually use --
if either tree references an action server that doesn't exist,
`bt_navigator`'s `on_configure()` throws and the whole node fails to
activate, even though RViz's "Nav2 Goal" button only ever sends
`NavigateToPose`. That's why `compute_path_to_pose_server.py` implements
**two** action servers, not one — `compute_path_through_poses` is handled by
calling `generate_path()` once per leg (`start→goals[0]`,
`goals[0]→goals[1]`, ...) and concatenating the results, dropping the
duplicate pose at each junction. Freshies never see this — it's boilerplate
built entirely on top of the same `generate_path()` function.

### Why bt_navigator needs different Behavior Tree XML files in this mode

Nav2's stock BT XMLs (`navigate_to_pose_w_replanning_and_recovery.xml` and
`navigate_through_poses_w_replanning_and_recovery.xml`) contain
`ClearEntireCostmap` recovery nodes that call the service
`global_costmap/clear_entirely_global_costmap`. That service is normally
advertised by `planner_server`'s embedded `global_costmap` — which, as
noted above, doesn't exist in this mode. `bt_navigator` fails to even parse
the stock XML without that service being resolvable.

Fix: `navigation.launch.py` points `bt_navigator` at
[`navigate_to_pose_no_global_costmap.xml`](src/bringup/behavior_trees/navigate_to_pose_no_global_costmap.xml) /
[`navigate_through_poses_no_global_costmap.xml`](src/bringup/behavior_trees/navigate_through_poses_no_global_costmap.xml)
instead of the stock trees. These are byte-for-byte identical except every
`ClearEntireCostmap ... global_costmap` node is replaced with a no-op
`<Wait wait_duration="0.1"/>` (or simply removed where it's one item in a
list of recovery actions). Local costmap clearing (served by
`controller_server`, which always runs) is untouched.

This is wired up as a **second `parameters=[]` entry on just the
`bt_navigator` `Node(...)`** in `navigation.launch.py` — `[configured_params,
bt_xml_overrides]` — not via a `RewrittenYaml` rewrite of the shared params
file. That distinction matters: `RewrittenYaml` only rewrites keys that
already exist in the source YAML, so making this work that way would
require `default_nav_to_pose_bt_xml`/`default_nav_through_poses_bt_xml` to
be declared (even as `""`) in the params file shared by all three modes —
and an explicit `""` there is treated as the real value by ROS 2's
parameter system, not as "fall back to bt_navigator's own compiled
default." That earlier approach silently broke `default`/`cpp_custom`
(bt_navigator would throw `Empty Tree` trying to load a blank path). The
current approach only touches `bt_navigator`'s parameters, and only in
`python_custom`'s own launch file, so `default`/`cpp_custom` never see it.

### What's boilerplate vs. what freshies edit, in `planner_py`

| File | Who wrote it | What it does |
|---|---|---|
| [`compute_path_to_pose_server.py`](src/planner_py/planner_py/compute_path_to_pose_server.py) | boilerplate | The `rclpy.Node` + two `ActionServer`s. Subscribes `/map` (QoS matching `map_server`'s `TRANSIENT_LOCAL` publisher), looks up the robot's current pose via TF when a goal doesn't include a start pose, wraps everything in a `MultiThreadedExecutor` + `ReentrantCallbackGroup` (needed so the blocking action callback doesn't starve the `/map` subscription callback), calls `generate_path()`, packages the result into `nav_msgs/Path`, catches any exception the student code raises so a bug can't crash the whole nav stack (aborts the goal instead). |
| [`occupancy_grid_view.py`](src/planner_py/planner_py/occupancy_grid_view.py) | boilerplate | `OccupancyGridView` — wraps the raw `OccupancyGrid` message into `.resolution`/`.width`/`.height`, a numpy 2D array, and `world_to_grid()` / `grid_to_world()` / `is_occupied()` helpers. |
| [`custom_planner.py`](src/planner_py/planner_py/custom_planner.py) | **freshie edits this** | One function, `generate_path(start_pose, goal_pose, grid) -> list[(x, y)]`. Out-of-the-box: the same straight-line interpolation as the C++ default (no obstacle avoidance), for symmetry between the two tracks. |

### Costmap caveat (important, documented in the README too)

Because `planner_server`/`global_costmap` never runs in this mode, the
Python planner only has the **static, instructor-provided `/map`** to work
with — not a live, obstacle-inflated costmap. `OccupancyGridView` is built
straight from the `/map` topic. This is a deliberate simplification to keep
the boilerplate implementable in pure `rclpy` + `numpy` (reimplementing
`nav2_costmap_2d`'s layered inflation in Python was judged out of scope for
a freshman assignment). A student wanting closer parity with the C++ track
can implement their own dilation/margin around obstacles using
`grid.is_occupied(...)`.

**Request flow:**
1. RViz publishes `/goal_pose` → `bt_navigator` (running the trimmed BT)
   calls `compute_path_to_pose`.
2. `custom_planner` resolves the start pose (TF lookup to `base_link` if
   not given), waits for `/map`, builds an `OccupancyGridView`, and calls
   `generate_path(start_pose, goal_pose, grid)`.
3. The returned waypoint list is packaged into a `nav_msgs/Path` and
   returned as the action result.
4. Same as before from here: `FollowPath` (DWB) drives the robot.

---

## 6. Side-by-side summary

| | `default` | `cpp_custom` | `python_custom` |
|---|---|---|---|
| `planner_server` process running? | Yes (stock `NavfnPlanner`) | Yes (our plugin loaded inside it) | **No** |
| Global costmap available to the planner? | Yes, live/inflated | Yes, live/inflated | No — static `/map` only |
| Freshie-edited file | *(none)* | `custom_planner.cpp` → `computePlan()` | `custom_planner.py` → `generate_path()` |
| Rebuild needed after editing? | — | Yes: `colcon build --packages-select planner_cpp` | No (with `--symlink-install`) |
| `bt_navigator`'s BT XML | stock | stock | trimmed (no global costmap clearing) |
| `params_file` used | `nav2_params_<model>_default.yaml` | `nav2_params_<model>_custom_cpp.yaml` | `nav2_params_<model>_custom_python.yaml` |
| Node count in navigation group | 8 | 8 | 8 (7 stock/renamed + `custom_planner`, `planner_server` absent) |

---

## 7. File index

```
src/bringup/launch/
  bringup.launch.py     top-level entry point; all 3 modes start here
  gazebo.launch.py      Gazebo + spawn, identical across all modes
  navigation.launch.py  forked nav2 launch file, ONLY used for python_custom
  rviz.launch.py        RViz wrapper, identical across all modes
src/bringup/params/
  nav2_params_<model>_default.yaml         used by default
  nav2_params_<model>_custom_cpp.yaml      used by cpp_custom (only planner_server block differs)
  nav2_params_<model>_custom_python.yaml   used by python_custom (starts identical to
                                            default -- planner_server never launches in
                                            this mode, so tune controller/costmap/BT
                                            settings here without touching default)
src/bringup/behavior_trees/
  navigate_to_pose_no_global_costmap.xml           used ONLY by python_custom
  navigate_through_poses_no_global_costmap.xml     used ONLY by python_custom
src/planner_cpp/
  plugins.xml                     pluginlib registration
  include/.../custom_planner.hpp
  src/custom_planner.cpp       computePlan() is the freshie TODO
src/planner_py/
  planner_py/compute_path_to_pose_server.py   boilerplate action server node
  planner_py/occupancy_grid_view.py           boilerplate map helper
  planner_py/custom_planner.py               generate_path() is the freshie TODO
```
