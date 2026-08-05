#!/usr/bin/env python3
"""
cmd_vel_limiter  (Track A: speed-dependent steering limit + Nav2->controller bridge)
DEST: bglx_ws/src/bglx_navigation/bglx_navigation/cmd_vel_limiter.py

Sits between Nav2's /etrike/cmd_vel output and the tricycle_steering_controller.
Two jobs:

  1) SAFETY: clamp the commanded angular velocity so the implied steering angle
     and lateral acceleration stay within limits. This tightens the allowed
     turn as speed rises (delta-trike rollover mitigation) and forbids
     in-place pivots at ~zero speed (a steered trike physically can't).

        w_max = min( |v|*tan(delta_max)/L ,   a_lat_max/|v| )
                 \____ steering-angle cap ___/ \__ rollover cap __/

  2) BRIDGE: republish the (clamped) Twist to the steering controller's
     input topic (~/reference_unstamped). This also transparently handles the
     bglx_autonomy waypoint follower, which publishes /etrike/cmd_vel directly.

Publishes zeros on input timeout (watchdog).
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class CmdVelLimiter(Node):
    def __init__(self):
        super().__init__('cmd_vel_limiter')

        # --- parameters ---
        self.declare_parameter('wheelbase', 1.33)              # m
        self.declare_parameter('max_steering_angle', 1.047)    # rad (60 deg) - keep in sync with front_fork.xacro
        self.declare_parameter('max_lateral_accel', 1.5)       # m/s^2 (rollover cap)
        self.declare_parameter('max_linear_vel', 2.78)         # m/s (~10 km/h)
        self.declare_parameter('min_speed_for_steer', 0.05)    # m/s
        self.declare_parameter('input_topic', '/etrike/cmd_vel')
        self.declare_parameter('output_topic',
                               '/tricycle_steering_controller/reference_unstamped')
        self.declare_parameter('input_timeout', 0.5)           # s
        self.declare_parameter('publish_rate', 20.0)           # Hz (watchdog)

        gp = self.get_parameter
        self.L = float(gp('wheelbase').value)
        self.delta_max = float(gp('max_steering_angle').value)
        self.a_lat_max = float(gp('max_lateral_accel').value)
        self.v_max = float(gp('max_linear_vel').value)
        self.v_steer_min = float(gp('min_speed_for_steer').value)
        self.timeout = float(gp('input_timeout').value)
        in_topic = gp('input_topic').value
        out_topic = gp('output_topic').value

        self.pub = self.create_publisher(Twist, out_topic, 10)
        self.sub = self.create_subscription(Twist, in_topic, self.on_cmd, 10)

        self._last_out = Twist()
        self._last_stamp = None
        self.create_timer(1.0 / float(gp('publish_rate').value), self.on_timer)

        self.get_logger().info(
            f'cmd_vel_limiter: {in_topic} -> {out_topic} | '
            f'L={self.L} delta_max={self.delta_max} a_lat_max={self.a_lat_max} '
            f'v_max={self.v_max}')

    def limit(self, v_in, w_in):
        v = clamp(v_in, -self.v_max, self.v_max)
        av = abs(v)
        if av < self.v_steer_min:
            # Too slow to steer a real wheel; forbid in-place rotation.
            return v, 0.0
        w_steer = av * math.tan(self.delta_max) / self.L   # steering-angle cap
        w_accel = self.a_lat_max / av                       # rollover cap
        w_max = min(w_steer, w_accel)
        return v, clamp(w_in, -w_max, w_max)

    def on_cmd(self, msg: Twist):
        v, w = self.limit(msg.linear.x, msg.angular.z)
        out = Twist()
        out.linear.x = v
        out.angular.z = w
        self._last_out = out
        self._last_stamp = self.get_clock().now()
        self.pub.publish(out)

    def on_timer(self):
        # Watchdog: if input went stale, command a stop.
        if self._last_stamp is None:
            return
        age = (self.get_clock().now() - self._last_stamp).nanoseconds * 1e-9
        if age > self.timeout:
            self.pub.publish(Twist())  # zero
        else:
            self.pub.publish(self._last_out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelLimiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()