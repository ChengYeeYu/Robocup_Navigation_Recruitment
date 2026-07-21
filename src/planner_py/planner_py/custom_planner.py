"""Freshie-editable path planner for the Python track."""
from collections import deque

from geometry_msgs.msg import PoseStamped

from planner_py.occupancy_grid_view import OccupancyGridView


def generate_path(
    start_pose: PoseStamped,
    goal_pose: PoseStamped,
    grid: OccupancyGridView,
) -> list:
    """BFS from start_pose to goal_pose; ordered (x, y) waypoints, empty if unreachable."""
    start = grid.world_to_grid(start_pose.pose.position.x, start_pose.pose.position.y)
    goal = grid.world_to_grid(goal_pose.pose.position.x, goal_pose.pose.position.y)

    if grid.is_occupied(*start) or grid.is_occupied(*goal):
        return []

    visited = {start: None}   # cell -> parent cell
    queue = deque([start])
    neighbors = [(1, 0), (-1, 0), (0, 1), (0, -1)]

    while queue:
        cur = queue.popleft()
        if cur == goal:
            break
        for dx, dy in neighbors:
            nxt = (cur[0] + dx, cur[1] + dy)
            if nxt not in visited and not grid.is_occupied(*nxt):
                visited[nxt] = cur
                queue.append(nxt)

    if goal not in visited:
        return []

    path_cells = []
    node = goal
    while node is not None:
        path_cells.append(node)
        node = visited[node]
    path_cells.reverse()

    return [grid.grid_to_world(gx, gy) for gx, gy in path_cells]
