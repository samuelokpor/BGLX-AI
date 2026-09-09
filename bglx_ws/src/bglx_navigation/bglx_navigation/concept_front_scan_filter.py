"""Remove trike self-returns while retaining the real scan frame and ray origin."""
import copy
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException
from .concept_scan_geometry import collision_shapes, transform_matrix, scan_points, self_mask, usable_scan


class ConceptFrontScanFilter(Node):
    def __init__(self):
        super().__init__('bglx_front_scan_filter')
        self.declare_parameter('robot_description', '')
        self.shapes = collision_shapes(self.get_parameter('robot_description').value)
        self.frames_needed = {s[0] for s in self.shapes}
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pending = None
        self.last_stamp = -1
        self.publisher = self.create_publisher(LaserScan, '/etrike/front_scan', qos_profile_sensor_data)
        self.subscription = self.create_subscription(LaserScan, '/etrike/front_scan_raw', self.receive, qos_profile_sensor_data)
        self.timer = self.create_timer(0.02, self.process)
        self.get_logger().info('Filtering /etrike/front_scan_raw -> /etrike/front_scan with stamped TF and physical collision solids')

    def receive(self, msg):
        self.pending = msg

    def process(self):
        msg = self.pending
        if msg is None:
            return
        stamp = Time.from_msg(msg.header.stamp)
        age = (self.get_clock().now()-stamp).nanoseconds*1e-9
        if age < -0.05 or age > 0.25 or stamp.nanoseconds <= 0 or not usable_scan(msg):
            self.pending = None
            return  # no heartbeat on bad/stale geometry; downstream watchdog stops
        try:
            frames = {'base_link': np.eye(4)}
            for name in self.frames_needed | {msg.header.frame_id}:
                if name != 'base_link':
                    frames[name] = transform_matrix(self.buffer.lookup_transform('base_link', name, stamp).transform)
        except (TransformException, ValueError):
            return  # retry next timer tick; never substitute latest TF
        points, indices = scan_points(msg, frames[msg.header.frame_id])
        mask = self_mask(points, self.shapes, frames)
        filtered = copy.deepcopy(msg)
        ranges = list(msg.ranges)
        for index in indices[mask]:
            ranges[int(index)] = math.nan
        filtered.ranges = ranges
        # Keep intensities and all angle/time/range metadata unchanged.
        self.publisher.publish(filtered)
        self.pending = None


def main(args=None):
    rclpy.init(args=args)
    node = ConceptFrontScanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
