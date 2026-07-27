"""Read-only wrapper around nav_msgs/OccupancyGrid, passed to generate_path() as `grid`."""
from collections import deque

import numpy as np


class OccupancyGridView:

    # Matches inflation_radius/robot_radius from
    # src/bringup/params/nav2_params_burger_custom_python.yaml (the file
    # planner:=python_custom actually loads), so cost() shapes a similar
    # clearance preference to what the C++ track's real (live) inflated
    # costmap gives -- burger-only, hardcoded for simplicity since this
    # track never learns which model is running.
    _INFLATION_RADIUS_M = 0.55
    _OCCUPIED_THRESH = 50

    def __init__(self, msg):
        """Wrap an OccupancyGrid message."""
        self.resolution = msg.info.resolution
        self.width = msg.info.width
        self.height = msg.info.height
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        # row-major, values 0-100 (occupancy prob.), -1 = unknown
        self.data = np.asarray(msg.data, dtype=np.int8).reshape(self.height, self.width)
        # NOT the live Nav2 costmap -- a static approximation of wall
        # clearance computed once from this map, so paths built with
        # cost()/move_cost() prefer clearance from walls the way a plan
        # against a real inflated costmap would (see cost()'s docstring).
        self._clearance = self._compute_clearance()

    def world_to_grid(self, x: float, y: float) -> tuple:
        """World (map frame) coordinates -> (grid_x, grid_y) cell indices."""
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> tuple:
        """Cell indices -> world (map frame) coordinates of the cell center."""
        x = self.origin_x + (gx + 0.5) * self.resolution
        y = self.origin_y + (gy + 0.5) * self.resolution
        return x, y

    def in_bounds(self, gx: int, gy: int) -> bool:
        """Return True if (gx, gy) is within the grid."""
        return 0 <= gx < self.width and 0 <= gy < self.height

    def is_occupied(self, gx: int, gy: int, occupied_thresh: int = 50) -> bool:
        """Return True if occupied, unknown, or out of bounds."""
        if not self.in_bounds(gx, gy):
            return True
        value = int(self.data[gy, gx])
        return value < 0 or value >= occupied_thresh

    def _compute_clearance(self) -> np.ndarray:
        """Multi-source BFS outward from every wall cell -- clearance[gy, gx] = cells to nearest wall."""
        occupied = (self.data < 0) | (self.data >= self._OCCUPIED_THRESH)
        clearance = np.full((self.height, self.width), -1, dtype=np.float32)
        queue = deque()
        for gy, gx in zip(*np.nonzero(occupied)):
            clearance[gy, gx] = 0
            queue.append((int(gx), int(gy)))

        neighbors = ((1, 0), (-1, 0), (0, 1), (0, -1))
        while queue:
            x, y = queue.popleft()
            next_d = clearance[y, x] + 1
            for dx, dy in neighbors:
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.width and 0 <= ny < self.height and clearance[ny, nx] < 0:
                    clearance[ny, nx] = next_d
                    queue.append((nx, ny))
        return clearance

    def cost(self, gx: int, gy: int):
        """Traversal cost for entering (gx, gy): None if a wall (or out of bounds/unknown),
        else >= 1.0, rising steeply as clearance to the nearest wall shrinks.

        Deliberately a gradient rather than binary wall-thickening: thicken
        walls enough to guarantee real clearance and you close narrow
        doorways outright; thicken less and a near-wall goal becomes
        permanently unreachable. A gradient keeps every non-wall cell
        traversable at some finite cost, so a search can still squeeze
        through a doorway when that's the only option, while still
        preferring routes that keep their distance from walls -- the same
        thing a real inflated costmap rewards.
        """
        if self.is_occupied(gx, gy):
            return None
        clearance_m = self._clearance[gy, gx] * self.resolution
        if clearance_m >= self._INFLATION_RADIUS_M:
            return 1.0
        return 1.0 + (self._INFLATION_RADIUS_M / max(clearance_m, self.resolution)) ** 2

    def move_cost(self, gx: int, gy: int):
        """Cost of stepping INTO cell (gx, gy); None if blocked. Alias for cost().

        Takes the same (gx, gy) coordinates as cost() and is_occupied() -- pass
        the cell you're moving into, e.g. `grid.move_cost(*neighbor)` where
        `neighbor` is a (gx, gy) tuple.
        """
        return self.cost(gx, gy)
