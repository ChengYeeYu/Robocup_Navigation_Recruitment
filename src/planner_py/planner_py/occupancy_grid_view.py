"""Read-only wrapper around nav_msgs/OccupancyGrid, passed to generate_path() as `grid`."""
import numpy as np


class OccupancyGridView:

    def __init__(self, msg):
        """Wrap an OccupancyGrid message."""
        self.resolution = msg.info.resolution
        self.width = msg.info.width
        self.height = msg.info.height
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        # row-major, values 0-100 (occupancy prob.), -1 = unknown
        self.data = np.asarray(msg.data, dtype=np.int8).reshape(self.height, self.width)

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
