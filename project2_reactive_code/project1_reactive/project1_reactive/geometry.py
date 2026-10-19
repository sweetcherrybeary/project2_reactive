"""
Geometry helpers shared by the controller and the mapper.

This module contains no ROS code, so every class here can be unit tested
without a running simulator.

* ``wrap_angle`` / ``FEET_TO_METERS``: small math helpers.
* ``LidarMount``: where the lidar sits on the robot and which way it faces.
* ``PoseTracker``: the robot's pose (x, y, yaw) plus a forward-only odometer.
"""

import math

FEET_TO_METERS = 0.3048


def wrap_angle(angle):
    """Wrap an angle in radians to the interval [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(x, y, z, w):
    """Return the yaw (rotation about z) of a quaternion, in radians."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class LidarMount:
    """
    Pose of the lidar relative to the robot's base frame.

    The TurtleBot 4 lidar is mounted rotated by +90 degrees about z and
    4 cm behind the robot's center, so scan angle 0 does NOT point forward.
    ``to_robot_frame`` converts a (scan angle, range) reading to an (x, y)
    point in the robot frame (x forward, y left).

    The values start from ROS parameters and are replaced by the real
    transform from TF when one becomes available, so the code keeps working
    if the lidar is moved or rotated.
    """

    def __init__(self, x_offset=-0.04, y_offset=0.0, yaw_offset=math.pi / 2.0):
        """Store the lidar's offset (m) and yaw (rad) in the robot frame."""
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.yaw_offset = yaw_offset
        self.from_tf = False

    def set_from_transform(self, x, y, yaw):
        """Replace the parameter values with a transform read from TF."""
        self.x_offset = x
        self.y_offset = y
        self.yaw_offset = yaw
        self.from_tf = True

    def to_robot_frame(self, scan_angle, scan_range):
        """Convert one lidar reading to an (x, y) point in the robot frame."""
        robot_angle = scan_angle + self.yaw_offset
        return (self.x_offset + scan_range * math.cos(robot_angle),
                self.y_offset + scan_range * math.sin(robot_angle))

    def sensor_pose_in_world(self, robot_x, robot_y, robot_yaw):
        """Return the lidar's (x, y, yaw) in the world, given the robot's pose."""
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        return (robot_x + cos_yaw * self.x_offset - sin_yaw * self.y_offset,
                robot_y + sin_yaw * self.x_offset + cos_yaw * self.y_offset,
                robot_yaw + self.yaw_offset)


class PoseTracker:
    """
    Robot pose in the odometry frame, plus a forward-only odometer.

    The pose normally comes from ``/odom``. If odometry stops arriving
    (for example while the simulator is still starting), the tracker falls
    back to dead reckoning from the velocity commands being sent, so the
    behaviors and the mapper never have to deal with a missing pose.
    """

    def __init__(self, odom_timeout=0.5):
        """Start at the origin; odometry older than ``odom_timeout`` s is stale."""
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odometer = 0.0        # total forward distance driven (m)
        self.odom_timeout = odom_timeout
        self._last_odom_time = None
        self._last_cmd_time = None

    def odom_is_fresh(self, now):
        """Return True if an odometry message arrived recently."""
        return (self._last_odom_time is not None
                and now - self._last_odom_time < self.odom_timeout)

    def update_from_odom(self, x, y, yaw, now):
        """Set the pose from an odometry message."""
        self._move_to(x, y, yaw)
        self._last_odom_time = now

    def update_from_command(self, linear, angular, now):
        """Dead-reckon from a velocity command, but only while odometry is stale."""
        if self._last_cmd_time is not None and not self.odom_is_fresh(now):
            dt = min(now - self._last_cmd_time, 0.5)
            if dt > 0.0:
                yaw_mid = self.yaw + 0.5 * angular * dt
                self._move_to(self.x + linear * dt * math.cos(yaw_mid),
                              self.y + linear * dt * math.sin(yaw_mid),
                              wrap_angle(self.yaw + angular * dt))
        self._last_cmd_time = now

    def _move_to(self, x, y, yaw):
        """Update the pose and add any forward motion to the odometer."""
        forward = (x - self.x) * math.cos(self.yaw) + (y - self.y) * math.sin(self.yaw)
        # Ignore big jumps (e.g. the first odom message after dead reckoning).
        if 0.0 < forward < 0.5:
            self.odometer += forward
        self.x = x
        self.y = y
        self.yaw = yaw
