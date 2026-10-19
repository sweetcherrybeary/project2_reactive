"""
Log-odds occupancy grid (no ROS code, so it can be unit tested alone).

Each cell stores the log-odds that it is occupied: 0 means unknown,
positive means probably occupied, negative means probably free. For every
lidar beam, the cells the beam passed through become more "free" and the
cell where it hit something becomes more "occupied".

The beams are traced with NumPy: each beam is sampled every half cell from
the sensor to its end point, which visits the same cells as a Bresenham
line trace but processes all 640 beams at once instead of in a Python loop.
"""

import numpy as np


class OccupancyGrid:
    """A 2-D grid of occupancy log-odds, centered on the robot's start pose."""

    def __init__(self, resolution=0.05, width_m=30.0, height_m=30.0,
                 origin_x=-15.0, origin_y=-15.0, hit_log_odds=0.85, miss_log_odds=-0.4,
                 min_log_odds=-4.0, max_log_odds=4.0):
        """Allocate the grid. ``origin`` is the world position of cell (0, 0)."""
        self.resolution = resolution
        self.width = int(round(width_m / resolution))       # cells along x
        self.height = int(round(height_m / resolution))     # cells along y
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.hit = hit_log_odds
        self.miss = miss_log_odds
        self.min_log_odds = min_log_odds
        self.max_log_odds = max_log_odds
        # Row = y index, column = x index (the layout nav_msgs/OccupancyGrid uses).
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)

    def _flat_indices(self, xs, ys):
        """Convert world coordinates to flat cell indices, dropping off-grid points."""
        cols = np.floor((xs - self.origin_x) / self.resolution).astype(np.int64)
        rows = np.floor((ys - self.origin_y) / self.resolution).astype(np.int64)
        inside = (cols >= 0) & (cols < self.width) & (rows >= 0) & (rows < self.height)
        return rows[inside] * self.width + cols[inside]

    def integrate_scan(self, sensor_x, sensor_y, sensor_yaw, angles, ranges,
                       range_min, range_max, max_mapping_range=5.0):
        """
        Update the grid with one scan.

        ``sensor_x/y/yaw`` is the lidar's pose in the world; ``angles`` and
        ``ranges`` are NumPy arrays in the lidar's own frame.
        """
        ranges = np.asarray(ranges, dtype=np.float64)
        angles = np.asarray(angles, dtype=np.float64)
        limit = min(range_max, max_mapping_range)

        # A beam that hit something inside the mapping range marks an obstacle.
        hit = np.isfinite(ranges) & (ranges >= range_min) & (ranges < limit)
        # A beam with no return ("inf") means free space all the way out.
        no_return = np.isposinf(ranges) | (np.isfinite(ranges) & (ranges >= limit))
        use = hit | no_return
        if not np.any(use):
            return
        free_length = np.where(hit, ranges, limit)[use]
        world_angles = sensor_yaw + angles[use]
        cos_a = np.cos(world_angles)
        sin_a = np.sin(world_angles)

        # Free cells: sample every half cell up to (not including) the end point.
        steps = np.arange(0.0, limit, self.resolution * 0.5)
        dist = steps[np.newaxis, :]                               # (1, S)
        before_end = dist < (free_length[:, np.newaxis] - self.resolution)
        xs = sensor_x + dist * cos_a[:, np.newaxis]
        ys = sensor_y + dist * sin_a[:, np.newaxis]
        free = np.unique(self._flat_indices(xs[before_end], ys[before_end]))

        # Occupied cells: the end point of each beam that hit something.
        hit_used = hit[use]
        end_x = sensor_x + free_length[hit_used] * cos_a[hit_used]
        end_y = sensor_y + free_length[hit_used] * sin_a[hit_used]
        occupied = np.unique(self._flat_indices(end_x, end_y))
        free = np.setdiff1d(free, occupied, assume_unique=True)

        flat = self.log_odds.reshape(-1)
        flat[free] += self.miss
        flat[occupied] += self.hit
        np.clip(self.log_odds, self.min_log_odds, self.max_log_odds, out=self.log_odds)

    def to_ros_data(self):
        """
        Return the grid as int8 values for nav_msgs/OccupancyGrid.

        -1 = unknown, 0..100 = probability of being occupied (percent).
        """
        probability = 1.0 - 1.0 / (1.0 + np.exp(self.log_odds))
        data = np.rint(probability * 100.0).astype(np.int8)
        data[self.log_odds == 0.0] = -1
        return data.reshape(-1)
