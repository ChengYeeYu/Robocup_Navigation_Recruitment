"""Freshie-editable path planner for the Python track."""
from geometry_msgs.msg import PoseStamped

from planner_py.occupancy_grid_view import OccupancyGridView


def _heuristic(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


def generate_path(
    start_pose: PoseStamped,
    goal_pose: PoseStamped,
    grid: OccupancyGridView,
) -> list:
    """Freshie TODO: straight-line default, replace with a real algorithm.

    Interpolates a straight line from start to goal and returns it as ordered
    (x, y) waypoints. Replace this with a real search (BFS, Dijkstra, A*, ...) that uses
    `grid` to route around obstacles -- see the README's "Suggested approach"
    and the grid helpers (`grid.is_occupied`, `grid.world_to_grid`,
    `grid.grid_to_world`, and the clearance-aware `grid.cost`/`grid.move_cost`).
    """
    sx = start_pose.pose.position.x
    sy = start_pose.pose.position.y
    gx = goal_pose.pose.position.x
    gy = goal_pose.pose.position.y

    interpolation_resolution = 0.1  
    dx = gx - sx
    dy = gy - sy
    total_distance = (dx * dx + dy * dy) ** 0.5
    n_steps = max(1, int(total_distance / interpolation_resolution))

    return [
        (sx + (i / n_steps) * dx, sy + (i / n_steps) * dy)
        for i in range(n_steps + 1)
    ]
