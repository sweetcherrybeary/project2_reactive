"""Look up the lidar's mounting pose from TF (shared by both nodes)."""

import time

from project1_reactive.geometry import yaw_from_quaternion
from rclpy.time import Time
import tf2_ros


class LidarMountFromTf:
    """
    Fill in a ``LidarMount`` from the TF tree when TF is available.

    Until the transform is found, the mount keeps the values from the ROS
    parameters. Lookups are rate-limited to one per second.
    """

    def __init__(self, node, mount, base_frame, enabled=True):
        """Create the TF listener (only if ``enabled``)."""
        self.node = node
        self.mount = mount
        self.base_frame = base_frame
        self.enabled = enabled
        self._last_try = 0.0
        if enabled:
            self.buffer = tf2_ros.Buffer()
            self.listener = tf2_ros.TransformListener(self.buffer, node)

    def try_update(self, scan_frame):
        """Try once (per second) to read base_frame -> scan_frame from TF."""
        if not self.enabled or self.mount.from_tf or not scan_frame:
            return
        now = time.monotonic()
        if now - self._last_try < 1.0:
            return
        self._last_try = now
        try:
            tf = self.buffer.lookup_transform(self.base_frame, scan_frame,
                                              Time())
        except tf2_ros.TransformException:
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        self.mount.set_from_transform(t.x, t.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        self.node.get_logger().info(
            f'Lidar pose from TF: x={t.x:.3f} m, y={t.y:.3f} m, '
            f'yaw={self.mount.yaw_offset:.3f} rad')
