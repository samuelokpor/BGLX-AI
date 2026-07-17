#!/usr/bin/env python3
"""BGLX autonomous cmd_vel loop.

Drives the trike toward a list of (x, y) waypoints in the odom frame and
publishes /cmd_vel. A lidar front-cone watchdog forces a hard stop when
anything is closer than obstacle_stop_distance. Controller-agnostic: works
with the diff-drive Gazebo plugin today and with tricycle_controller later.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        # ---- Parameters ----
        self.declare_parameter('cmd_vel_topic', '/etrike/cmd_vel')
        self.declare_parameter('odom_topic', '/etrike/odom')
        self.declare_parameter('scan_topic', '/etrike/scan')
        # waypoints as a flat list [x0, y0, x1, y1, ...] in the odom frame
        self.declare_parameter('waypoints', [2.0, 0.0])
        self.declare_parameter('goal_tolerance', 0.25)        # m
        self.declare_parameter('max_linear', 0.35)            # m/s (creep / walking pace)
        self.declare_parameter('max_angular', 0.6)            # rad/s
        self.declare_parameter('k_linear', 0.6)
        self.declare_parameter('k_angular', 1.5)
        self.declare_parameter('heading_tolerance', 0.25)     # rad; above this, turn in place
        self.declare_parameter('obstacle_stop_distance', 0.8) # m
        self.declare_parameter('obstacle_cone_deg', 60.0)     # total front cone
        self.declare_parameter('min_valid_range', 0.35)       # ignore returns closer than this (robot self-hits)
        self.declare_parameter('control_rate', 20.0)          # Hz
        self.declare_parameter('loop', False)

        g = self.get_parameter
        self.cmd_topic = g('cmd_vel_topic').value
        self.odom_topic = g('odom_topic').value
        self.scan_topic = g('scan_topic').value
        flat = list(g('waypoints').value)
        self.waypoints = [(flat[i], flat[i + 1]) for i in range(0, len(flat) - 1, 2)]
        self.goal_tol = g('goal_tolerance').value
        self.max_lin = g('max_linear').value
        self.max_ang = g('max_angular').value
        self.k_lin = g('k_linear').value
        self.k_ang = g('k_angular').value
        self.head_tol = g('heading_tolerance').value
        self.stop_dist = g('obstacle_stop_distance').value
        self.cone = math.radians(g('obstacle_cone_deg').value)
        self.min_valid = g('min_valid_range').value
        rate = g('control_rate').value
        self.loop = g('loop').value

        # ---- State ----
        self.have_odom = False
        self.x = self.y = self.yaw = 0.0
        self.obstacle = False
        self.min_front = float('inf')
        self.wp_idx = 0
        self.done = False

        # ---- ROS I/O ----
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.on_scan,
                                 qos_profile_sensor_data)
        self.create_timer(1.0 / rate, self.control_step)

        if not self.waypoints:
            self.get_logger().error('No waypoints given - nothing to do.')
        self.get_logger().info(
            f'Waypoint follower up: {len(self.waypoints)} waypoint(s), '
            f'cmd->{self.cmd_topic}, stop@{self.stop_dist:.2f} m, '
            f'creep<= {self.max_lin:.2f} m/s')

    # ---- Callbacks ----
    def on_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.have_odom = True

    def on_scan(self, msg):
        half = self.cone / 2.0
        closest = float('inf')
        ang = msg.angle_min
        for r in msg.ranges:
            if -half <= ang <= half and max(msg.range_min, self.min_valid) <= r <= msg.range_max \
                    and not math.isinf(r) and not math.isnan(r):
                if r < closest:
                    closest = r
            ang += msg.angle_increment
        self.min_front = closest
        self.obstacle = closest < self.stop_dist

    # ---- Helpers ----
    def stop(self):
        self.cmd_pub.publish(Twist())

    def finish(self):
        self.stop()
        self.done = True
        self.get_logger().info('All waypoints reached - stopping.')

    # ---- Control loop ----
    def control_step(self):
        if self.done or not self.have_odom:
            return

        # Safety first: lidar obstacle stop.
        if self.obstacle:
            self.stop()
            self.get_logger().warn(
                f'Obstacle at {self.min_front:.2f} m - holding.',
                throttle_duration_sec=1.0)
            return

        if self.wp_idx >= len(self.waypoints):
            self.finish()
            return

        gx, gy = self.waypoints[self.wp_idx]
        dx, dy = gx - self.x, gy - self.y
        dist = math.hypot(dx, dy)

        if dist < self.goal_tol:
            self.get_logger().info(
                f'Reached waypoint {self.wp_idx + 1} ({gx:.2f}, {gy:.2f}).')
            self.wp_idx += 1
            if self.wp_idx >= len(self.waypoints):
                if self.loop:
                    self.wp_idx = 0
                else:
                    self.finish()
            return

        yaw_err = normalize_angle(math.atan2(dy, dx) - self.yaw)
        cmd = Twist()
        cmd.angular.z = clamp(self.k_ang * yaw_err, -self.max_ang, self.max_ang)
        if abs(yaw_err) <= self.head_tol:
            # Pointed roughly at the goal: creep forward while correcting.
            cmd.linear.x = clamp(self.k_lin * dist, 0.0, self.max_lin)
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
