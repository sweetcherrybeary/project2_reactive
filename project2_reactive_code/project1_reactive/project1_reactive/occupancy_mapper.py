"""
ROS 2 node for the mapping behavior.

Mapping runs in parallel with the reactive behaviors: it never moves the
robot, it only listens. Each laser scan is added to an ``OccupancyGrid``
at the robot's current odometry pose, and the grid is published on /map
(frame ``odom``) about once per second for RViz.
"""

import array
import math
import time

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg
from nav_msgs.msg import Odometry
import numpy as np
from project1_reactive.geometry import LidarMount, PoseTracker, yaw_from_quaternion
from project1_reactive.lidar_tf import LidarMountFromTf
from project1_reactive.occupancy_grid import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

PARAMETERS = {
    'scan_topic': 'scan',
    'odom_topic': 'odom',
    'cmd_vel_topic': 'cmd_vel',         # only used if odometry is missing
    'cmd_vel_stamped': True,
    'map_topic': 'map',
    'map_frame': 'odom',
    'resolution': 0.05,                 # m per cell
    'map_size': 30.0,                   # m; the grid is centered on the start pose
    'max_mapping_range': 5.0,           # m; longer beams are less accurate
    'max_update_rate': 5.0,             # scans per second added to the map
    'publish_period': 1.0,              # s between /map messages
    'base_frame': 'base_link',
    'use_tf_for_lidar': True,
    'lidar_x_offset': -0.04,
    'lidar_y_offset': 0.0,
    'lidar_yaw_offset_deg': 90.0,
}


class OccupancyMapper(Node):
    """Build an occupancy grid from /scan and /odom and publish it on /map."""

    def __init__(self):
        """Declare parameters, create the grid, and wire up ROS topics."""
        super().__init__('occupancy_mapper')
        for name, default in PARAMETERS.items():
            self.declare_parameter(name, default)
        p = {name: self.get_parameter(name).value for name in PARAMETERS}
        self.params = p

        half = p['map_size'] / 2.0
        self.grid = OccupancyGrid(resolution=p['resolution'], width_m=p['map_size'],
                                  height_m=p['map_size'], origin_x=-half, origin_y=-half)
        self.pose = PoseTracker()
        self.mount = LidarMount(p['lidar_x_offset'], p['lidar_y_offset'],
                                math.radians(p['lidar_yaw_offset_deg']))
        self.lidar_tf = LidarMountFromTf(self, self.mount, p['base_frame'],
                                         p['use_tf_for_lidar'])
        self.last_update = 0.0
        self.scans_used = 0

        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGridMsg, p['map_topic'], map_qos)
        self.create_subscription(LaserScan, p['scan_topic'], self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, p['odom_topic'], self.on_odom,
                                 qos_profile_sensor_data)
        cmd_type = TwistStamped if p['cmd_vel_stamped'] else Twist
        self.create_subscription(cmd_type, p['cmd_vel_topic'], self.on_cmd_vel, 10)
        self.create_timer(p['publish_period'], self.publish_map)
        self.get_logger().info('Occupancy mapper started.')

    def on_odom(self, msg):
        """Track the robot pose from wheel odometry."""
        position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pose.update_from_odom(position.x, position.y,
                                   yaw_from_quaternion(q.x, q.y, q.z, q.w),
                                   time.monotonic())

    def on_cmd_vel(self, msg):
        """Dead-reckon from velocity commands if odometry is missing."""
        twist = msg.twist if hasattr(msg, 'twist') else msg
        self.pose.update_from_command(twist.linear.x, twist.angular.z, time.monotonic())

    def on_scan(self, msg):
        """Add a scan to the grid (at most ``max_update_rate`` times per second)."""
        now = time.monotonic()
        if now - self.last_update < 1.0 / self.params['max_update_rate']:
            return
        self.last_update = now
        self.lidar_tf.try_update(msg.header.frame_id)
        sensor_x, sensor_y, sensor_yaw = self.mount.sensor_pose_in_world(
            self.pose.x, self.pose.y, self.pose.yaw)
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        angles = msg.angle_min + msg.angle_increment * np.arange(len(ranges))
        self.grid.integrate_scan(sensor_x, sensor_y, sensor_yaw, angles, ranges,
                                 msg.range_min, msg.range_max,
                                 self.params['max_mapping_range'])
        self.scans_used += 1

    def publish_map(self):
        """Publish the current grid as a nav_msgs/OccupancyGrid."""
        if self.scans_used == 0:
            return
        msg = OccupancyGridMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.params['map_frame']
        msg.info.resolution = float(self.grid.resolution)
        msg.info.width = self.grid.width
        msg.info.height = self.grid.height
        msg.info.origin.position.x = float(self.grid.origin_x)
        msg.info.origin.position.y = float(self.grid.origin_y)
        msg.info.origin.orientation.w = 1.0
        msg.data = array.array('b', self.grid.to_ros_data().tobytes())
        self.map_pub.publish(msg)


def main(args=None):
    """Start the node and spin until Ctrl+C."""
    rclpy.init(args=args)
    node = OccupancyMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
