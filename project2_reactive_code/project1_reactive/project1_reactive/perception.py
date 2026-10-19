"""
Perception: turn a raw lidar scan into what the behaviors need to know.

The behaviors never look at the scan directly. Instead, ``ObstacleDetector``
summarizes it as two numbers: the clearance to the nearest obstacle in the
front-left sector and in the front-right sector (``FrontObstacles``).

Project 2 (physical robot) additions, all aimed at noisy real lidar data:

* Self-filter: returns from inside the robot's own outline (the TurtleBot 4's
  top-plate standoffs and cables) are thrown away instead of being read as
  an obstacle at negative clearance.
* Outlier rejection: in each sector the ``outlier_beams`` smallest
  clearances are ignored, so one or two spurious short readings cannot
  trigger escape or avoid on their own.
* ``ClearanceFilter``: a median over the last few scans, so a single bad
  scan cannot flip the robot between behaviors.
"""

from collections import deque
from dataclasses import dataclass
import math
import statistics

from project1_reactive.geometry import FEET_TO_METERS, LidarMount


@dataclass
class FrontObstacles:
    """
    Nearest obstacles in front of the robot.

    Clearances are measured from the robot's body (its center distance minus
    its radius), in meters. ``math.inf`` means nothing was seen on that side.
    """

    left: float = math.inf
    right: float = math.inf

    def nearest(self):
        """Return the smaller of the two clearances."""
        return min(self.left, self.right)


def robust_minimum(values, outliers):
    """
    Return the smallest value after discarding the ``outliers`` smallest.

    If there are no more than ``outliers`` values, they are all treated as
    noise and ``math.inf`` (nothing seen) is returned. A real obstacle within
    1 ft spans many lidar beams (about 0.56 degrees apart on the TurtleBot 4),
    so only isolated glitches are lost.
    """
    if len(values) <= outliers:
        return math.inf
    return sorted(values)[outliers]


class ObstacleDetector:
    """
    Find obstacles in the two front sectors of a laser scan.

    The front of the robot is split into a left sector (0 to +half_angle)
    and a right sector (0 to -half_angle). A narrow center band belongs to
    both sectors, so an obstacle dead ahead counts on both sides.
    """

    def __init__(self, mount, robot_radius=0.17, front_half_angle=math.radians(60.0),
                 center_band=math.radians(5.0), threshold=FEET_TO_METERS,
                 symmetry_tolerance=0.05, self_filter_margin=0.0, outlier_beams=0):
        """
        Configure the sectors (radians) and the distance thresholds (meters).

        ``self_filter_margin``: returns closer to the robot's center than
        ``robot_radius - self_filter_margin`` are treated as the robot's own
        body and ignored (0 ignores everything inside the outline).
        ``outlier_beams``: number of smallest clearances per sector to ignore.
        """
        self.mount = mount if mount is not None else LidarMount()
        self.robot_radius = robot_radius
        self.front_half_angle = front_half_angle
        self.center_band = center_band
        self.threshold = threshold
        self.symmetry_tolerance = symmetry_tolerance
        self.self_filter_radius = max(0.0, robot_radius - self_filter_margin)
        self.outlier_beams = max(0, int(outlier_beams))

    def detect(self, angle_min, angle_increment, ranges, range_min, range_max):
        """Return the ``FrontObstacles`` seen in one scan."""
        left = []
        right = []
        for i, scan_range in enumerate(ranges):
            # Real lidars report inf/NaN for "no return" and 0 or tiny values
            # for invalid beams; range_min/range_max reject both.
            if not math.isfinite(scan_range) or not range_min <= scan_range <= range_max:
                continue
            scan_angle = angle_min + i * angle_increment
            x, y = self.mount.to_robot_frame(scan_angle, scan_range)
            if x <= 0.0:
                continue                      # beside or behind the robot
            distance = math.hypot(x, y)
            if distance < self.self_filter_radius:
                continue                      # a part of the robot itself
            bearing = math.atan2(y, x)
            if abs(bearing) > self.front_half_angle:
                continue                      # outside the front arc
            clearance = distance - self.robot_radius
            if bearing >= -self.center_band:
                left.append(clearance)
            if bearing <= self.center_band:
                right.append(clearance)
        return FrontObstacles(robust_minimum(left, self.outlier_beams),
                              robust_minimum(right, self.outlier_beams))

    def is_near(self, clearance):
        """Return True if a clearance is within the obstacle threshold (1 ft)."""
        return clearance <= self.threshold

    def is_symmetric(self, obstacles):
        """Return True for obstacles close on BOTH sides at roughly equal distance."""
        return (self.is_near(obstacles.left) and self.is_near(obstacles.right)
                and abs(obstacles.left - obstacles.right) <= self.symmetry_tolerance)

    def is_asymmetric(self, obstacles):
        """Return True if something is within 1 ft in front but not symmetric."""
        return self.is_near(obstacles.nearest()) and not self.is_symmetric(obstacles)


class ClearanceFilter:
    """
    Median of the last few ``FrontObstacles``, side by side.

    A window of 1 passes scans through unchanged. A window of 3 removes any
    single-scan glitch at the cost of about one scan period of delay.
    """

    def __init__(self, window=3):
        """``window``: number of recent scans to take the median over."""
        window = max(1, int(window))
        self.left = deque(maxlen=window)
        self.right = deque(maxlen=window)

    def add(self, obstacles):
        """Add one scan's result and return the filtered ``FrontObstacles``."""
        self.left.append(obstacles.left)
        self.right.append(obstacles.right)
        return FrontObstacles(statistics.median_low(self.left),
                              statistics.median_low(self.right))

    def clear(self):
        """Forget the history (e.g. after the scan stream was interrupted)."""
        self.left.clear()
        self.right.clear()
