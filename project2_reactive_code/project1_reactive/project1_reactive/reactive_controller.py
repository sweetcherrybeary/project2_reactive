"""
ROS 2 node that runs the reactive behaviors on the TurtleBot 4.

Data flow (one control cycle, 20 times per second):

    /scan, /odom, /hazard_detection, /key_vel
        -> SensorState  (filled in by the subscription callbacks)
        -> PriorityArbiter.step()  (asks each behavior, highest priority first)
        -> /cmd_vel

At start-up the TurtleBot 4 is sitting on its charging dock, so the node
first asks the robot to undock and only starts the behaviors afterwards.

Project 2 (physical robot) changes:

* Topic names are relative (``scan``, not ``/scan``), so the node follows a
  robot namespace such as ``/tb4_3`` set in the launch file.
* Scans are self-filtered, outlier-filtered and median-filtered over the
  last few scans (see ``perception.py``) before the behaviors see them.
* The bumper counts as pressed only while BUMP reports keep arriving
  (``bump_timeout``), so a missed "released" message cannot halt the robot
  forever.
* The Create 3's built-in bump reflexes are switched off at start-up, so
  that our halt behavior, not the base, decides what happens on contact.
* Ctrl+C sends a final stop command before the node exits.
"""

import math
import random
import time

from geometry_msgs.msg import Twist, TwistStamped
from irobot_create_msgs.action import Undock
from irobot_create_msgs.msg import DockStatus, HazardDetection, HazardDetectionVector
from nav_msgs.msg import Odometry
from project1_reactive.arbiter import PriorityArbiter
from project1_reactive.behaviors import build_behaviors, SensorState, VelocityCommand
from project1_reactive.geometry import LidarMount, yaw_from_quaternion
from project1_reactive.lidar_tf import LidarMountFromTf
from project1_reactive.perception import ClearanceFilter
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

# ROS parameters and their defaults. Distances in meters, angles in degrees,
# times in seconds, speeds in m/s or rad/s.
# Topic names are relative so that a robot namespace (e.g. /tb4_3) applies.
PARAMETERS = {
    'cmd_vel_topic': 'cmd_vel',
    'cmd_vel_stamped': True,            # Jazzy TurtleBot 4 expects TwistStamped
    'teleop_topic': 'key_vel',
    'teleop_stamped': True,
    'scan_topic': 'scan',
    'odom_topic': 'odom',
    'hazard_topic': 'hazard_detection',
    'dock_status_topic': 'dock_status',
    'undock_action': 'undock',
    'auto_undock': True,
    'startup_timeout': 60.0,
    'base_frame': 'base_link',
    'use_tf_for_lidar': True,
    'lidar_x_offset': -0.04,
    'lidar_y_offset': 0.0,
    'lidar_yaw_offset_deg': 90.0,
    'robot_radius': 0.17,
    'obstacle_distance': 0.3048,
    'front_half_angle_deg': 60.0,
    'center_band_deg': 5.0,
    'symmetry_tolerance': 0.05,
    'forward_speed': 0.2,
    'turn_speed': 1.0,
    'escape_turn_deg': 180.0,
    'escape_jitter_deg': 30.0,
    'random_turn_step': 0.3048,
    'random_turn_max_deg': 15.0,
    'halt_hold_time': 1.0,
    'teleop_timeout': 1.0,
    'control_rate': 20.0,
    'random_seed': -1,                  # -1 = different every run
    # Added for the physical robot (Project 2).
    'self_filter_margin': 0.0,          # m; ignore lidar returns inside the robot
    'outlier_beams': 2,                 # smallest readings ignored per front sector
    'scan_filter_window': 3,            # scans in the median filter (1 = off)
    'avoid_turn_speed': 0.8,            # rad/s
    'avoid_hysteresis': 0.05,           # m; avoid releases at 1 ft + this
    'min_turn_speed': 0.3,              # rad/s
    'turn_tolerance_deg': 3.0,
    'turn_timeout_factor': 3.0,         # give up a stalled turn (0 = never)
    'bump_timeout': 0.25,               # s; BUMP must keep being reported
    'disable_reflexes': True,           # turn off the Create 3 bump reflexes
    'motion_control_node': 'motion_control',
    'log_clearances': False,            # log left/right clearances at 2 Hz
}

BEHAVIOR_SETTINGS = (
    'robot_radius', 'obstacle_distance', 'front_half_angle_deg', 'center_band_deg',
    'symmetry_tolerance', 'forward_speed', 'turn_speed', 'escape_turn_deg',
    'escape_jitter_deg', 'random_turn_step', 'random_turn_max_deg', 'halt_hold_time',
    'teleop_timeout', 'self_filter_margin', 'outlier_beams', 'avoid_turn_speed',
    'avoid_hysteresis', 'min_turn_speed', 'turn_tolerance_deg', 'turn_timeout_factor')


class ReactiveController(Node):
    """Collect sensor data, run the arbiter, and publish velocity commands."""

    def __init__(self):
        """Declare parameters, build the behaviors, and wire up ROS topics."""
        super().__init__('reactive_controller')
        for name, default in PARAMETERS.items():
            self.declare_parameter(name, default)
        p = {name: self.get_parameter(name).value for name in PARAMETERS}
        self.params = p

        self.mount = LidarMount(p['lidar_x_offset'], p['lidar_y_offset'],
                                math.radians(p['lidar_yaw_offset_deg']))
        self.lidar_tf = LidarMountFromTf(self, self.mount, p['base_frame'],
                                         p['use_tf_for_lidar'])

        seed = p['random_seed']
        rng = random.Random(None if seed < 0 else seed)
        settings = {key: p[key] for key in BEHAVIOR_SETTINGS}
        settings['mount'] = self.mount
        behaviors, self.detector = build_behaviors(settings, rng)
        self.arbiter = PriorityArbiter(behaviors)
        self.state = SensorState()
        self.last_winner = None
        self.clearance_filter = ClearanceFilter(p['scan_filter_window'])
        self.last_clearance_log = 0.0

        # Publishers
        cmd_type = TwistStamped if p['cmd_vel_stamped'] else Twist
        self.cmd_pub = self.create_publisher(cmd_type, p['cmd_vel_topic'], 10)
        self.behavior_pub = self.create_publisher(String, 'active_behavior', 10)

        # Subscribers
        sensor_qos = qos_profile_sensor_data
        self.create_subscription(LaserScan, p['scan_topic'], self.on_scan, sensor_qos)
        self.create_subscription(Odometry, p['odom_topic'], self.on_odom, sensor_qos)
        self.create_subscription(HazardDetectionVector, p['hazard_topic'],
                                 self.on_hazards, sensor_qos)
        teleop_type = TwistStamped if p['teleop_stamped'] else Twist
        self.create_subscription(teleop_type, p['teleop_topic'], self.on_teleop, 10)

        # Undocking at start-up
        self.ready = not p['auto_undock']
        self.start_time = time.monotonic()
        self.dock_status = None
        self.undock_goal_sent = False
        self.undock_done = False
        if p['auto_undock']:
            self.create_subscription(DockStatus, p['dock_status_topic'],
                                     self.on_dock_status, sensor_qos)
            self.undock_client = ActionClient(self, Undock, p['undock_action'])

        # Create 3 bump reflexes would back the robot away on contact,
        # fighting the halt behavior, so they are switched off.
        if p['disable_reflexes']:
            self.reflex_client = self.create_client(
                SetParameters, p['motion_control_node'] + '/set_parameters')
            self.reflexes_off = False
            self.reflex_request_pending = False
        else:
            self.reflexes_off = True

        self.create_timer(1.0 / p['control_rate'], self.control_cycle)
        self.get_logger().info('Reactive controller started.')

    # ------------------------------------------------------------------
    # Sensor callbacks: they only store data in self.state.
    # ------------------------------------------------------------------
    def on_scan(self, msg):
        """Summarize a laser scan as front-left / front-right clearances."""
        self.lidar_tf.try_update(msg.header.frame_id)
        now = time.monotonic()
        if now - self.state.scan_time > 1.0:
            self.clearance_filter.clear()   # stream was interrupted: start afresh
        raw = self.detector.detect(
            msg.angle_min, msg.angle_increment, msg.ranges, msg.range_min, msg.range_max)
        self.state.obstacles = self.clearance_filter.add(raw)
        self.state.scan_time = now
        if self.params['log_clearances'] and now - self.last_clearance_log >= 0.5:
            self.last_clearance_log = now
            self.get_logger().info(
                f'clearance left={self.state.obstacles.left:.3f} m '
                f'right={self.state.obstacles.right:.3f} m')

    def on_odom(self, msg):
        """Update the robot pose from wheel odometry."""
        position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.state.pose.update_from_odom(position.x, position.y,
                                         yaw_from_quaternion(q.x, q.y, q.z, q.w),
                                         time.monotonic())

    def on_hazards(self, msg):
        """
        Record the time of the latest bumper report.

        The Create 3 lists every active hazard in each message. Whether the
        bumper is pressed is decided in ``control_cycle`` from how recently
        a BUMP was reported, which works whether or not the robot sends an
        empty message when the bumper is released.
        """
        if any(d.type == HazardDetection.BUMP for d in msg.detections):
            self.state.last_bump_time = time.monotonic()

    def on_teleop(self, msg):
        """Store the latest keyboard command."""
        twist = msg.twist if hasattr(msg, 'twist') else msg
        self.state.teleop_command = VelocityCommand(twist.linear.x, twist.angular.z)
        self.state.teleop_time = time.monotonic()

    def on_dock_status(self, msg):
        """Remember whether the robot is on its dock."""
        self.dock_status = msg

    # ------------------------------------------------------------------
    # Start-up: undock before running the behaviors.
    # ------------------------------------------------------------------
    def check_ready(self):
        """Return True once the robot is off the dock (or we gave up waiting)."""
        if self.ready:
            return True
        waited = time.monotonic() - self.start_time
        if self.undock_done:
            self.get_logger().info('Undocked; starting behaviors.')
            self.ready = True
        elif self.dock_status is not None and not self.dock_status.is_docked \
                and not self.undock_goal_sent:
            self.get_logger().info('Robot is not docked; starting behaviors.')
            self.ready = True
        elif (not self.undock_goal_sent and self.undock_client.server_is_ready()
              and (self.dock_status is not None and self.dock_status.is_docked
                   or self.dock_status is None and waited > 5.0)):
            # Docked, or no dock status after 5 s: undocking is harmless if the
            # robot is not actually docked (the request just fails).
            self.get_logger().info('Sending undock request.')
            future = self.undock_client.send_goal_async(Undock.Goal())
            future.add_done_callback(self.on_undock_accepted)
            self.undock_goal_sent = True
        elif waited > self.params['startup_timeout']:
            self.get_logger().warn('Undock did not finish in time; starting behaviors anyway.')
            self.ready = True
        return self.ready

    def on_undock_accepted(self, future):
        """Wait for the undock action to finish once it is accepted."""
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn('Undock request rejected; starting behaviors.')
            self.undock_done = True
            return
        handle.get_result_async().add_done_callback(self.on_undock_finished)

    def on_undock_finished(self, _future):
        """Mark undocking as finished."""
        self.undock_done = True

    # ------------------------------------------------------------------
    # Create 3 reflexes
    # ------------------------------------------------------------------
    def try_disable_reflexes(self):
        """Ask the Create 3 to turn off its bump reflexes (once, when available)."""
        if self.reflexes_off or self.reflex_request_pending:
            return
        if not self.reflex_client.service_is_ready():
            return
        value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=False)
        request = SetParameters.Request(
            parameters=[Parameter(name='reflexes_enabled', value=value)])
        self.reflex_request_pending = True
        self.reflex_client.call_async(request).add_done_callback(self.on_reflexes_set)

    def on_reflexes_set(self, future):
        """Log whether the reflexes were turned off."""
        self.reflex_request_pending = False
        self.reflexes_off = True            # do not keep retrying either way
        response = future.result()
        if response is not None and response.results and response.results[0].successful:
            self.get_logger().info('Create 3 bump reflexes disabled.')
        else:
            reason = response.results[0].reason if response and response.results else ''
            self.get_logger().warn(f'Could not disable Create 3 reflexes: {reason}')

    # ------------------------------------------------------------------
    # Control cycle
    # ------------------------------------------------------------------
    def control_cycle(self):
        """Run the arbiter once and publish the winning command."""
        if self.params['disable_reflexes']:
            self.try_disable_reflexes()
        if not self.check_ready():
            return                      # do not fight the undock action
        self.state.now = time.monotonic()
        self.state.bumper_pressed = (
            self.state.now - self.state.last_bump_time < self.params['bump_timeout'])
        winner, command = self.arbiter.step(self.state)
        self.publish_command(command)
        self.state.pose.update_from_command(command.linear, command.angular, self.state.now)
        if winner != self.last_winner:
            self.get_logger().info(f'Active behavior: {winner}')
            self.behavior_pub.publish(String(data=winner))
            self.last_winner = winner

    def publish_command(self, command):
        """Send a VelocityCommand as Twist or TwistStamped."""
        twist = Twist()
        twist.linear.x = float(command.linear)
        twist.angular.z = float(command.angular)
        if self.params['cmd_vel_stamped']:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.params['base_frame']
            msg.twist = twist
            self.cmd_pub.publish(msg)
        else:
            self.cmd_pub.publish(twist)

    def stop_robot(self):
        """Send one zero-velocity command (used at shutdown)."""
        self.publish_command(VelocityCommand(0.0, 0.0))


def main(args=None):
    """Start the node and spin until Ctrl+C."""
    # Handle Ctrl+C ourselves: with rclpy's own handler the context is already
    # shut down when KeyboardInterrupt arrives, and the final stop command
    # could not be sent, leaving a real robot rolling.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ReactiveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
