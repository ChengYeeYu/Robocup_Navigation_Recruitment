"""Freshie-editable path planner for the Python track."""
import heapq

from geometry_msgs.msg import PoseStamped

from planner_py.occupancy_grid_view import OccupancyGridView


def generate_path(
    start_pose: PoseStamped,
    goal_pose: PoseStamped,
    grid: OccupancyGridView,
) -> list:
    """A* from start_pose to goal_pose minimizing grid.move_cost() (which
    rises near walls), so the path prefers clearance from obstacles instead
    of just avoiding them -- ordered (x, y) waypoints, empty if unreachable.
    """
    start = grid.world_to_grid(start_pose.pose.position.x, start_pose.pose.position.y)
    goal = grid.world_to_grid(goal_pose.pose.position.x, goal_pose.pose.position.y)

    if grid.is_occupied(*start) or grid.is_occupied(*goal):
        return []

    neighbors = [(1, 0), (-1, 0), (0, 1), (0, -1)]

    def heuristic(cell):
        # Manhattan distance: admissible since move_cost() is always >= 1.0
        # per 4-connected step.
        return abs(cell[0] - goal[0]) + abs(cell[1] - goal[1])

    g_score = {start: 0.0}
    came_from = {}
    open_set = [(heuristic(start), start)]

    while open_set:
        _, cur = heapq.heappop(open_set)
        if cur == goal:
            break
        for dx, dy in neighbors:
            nxt = (cur[0] + dx, cur[1] + dy)
            step_cost = grid.move_cost(cur, nxt)
            if step_cost is None:
                continue
            tentative_g = g_score[cur] + step_cost
            if tentative_g < g_score.get(nxt, float('inf')):
                g_score[nxt] = tentative_g
                came_from[nxt] = cur
                heapq.heappush(open_set, (tentative_g + heuristic(nxt), nxt))

    if goal not in g_score:
        return []

    path_cells = [goal]
    node = goal
    while node != start:
        node = came_from[node]
        path_cells.append(node)
    path_cells.reverse()

    return [grid.grid_to_world(gx, gy) for gx, gy in path_cells]
