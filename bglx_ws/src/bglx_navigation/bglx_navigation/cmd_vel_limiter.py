#!/usr/bin/env python3
"""
cmd_vel_limiter  (Track A: speed-dependent steering limit + Nav2->controller
bridge + 60 deg/s steering ACTUATOR model)

Sits between Nav2's /etrike/cmd_vel and the tricycle_steering_controller.
Jobs:
  1) SAFETY: clamp angular velocity so steering angle and lateral accel stay
     in limits; forbid in-place pivots at ~zero speed.
  2) ACTUATOR MODEL: rate-limit + low-pass the angular command so the implied
     steering can only move as fast as a real 60 deg/s (1.047 rad/s) actuator.
     A physical leadscrew/worm actuator cannot snap instantly to a new angle,
     so it physically filters MPPI's high-frequency jitter instead of chasing
     it. This is what a self-locking, rate-limited actuator does - modelled on
     the command path so it does NOT disturb the tricycle controller's
     odometry (which nav, EKF and the agent all depend on).
  3) BRIDGE + watchdog: republish to ~/reference_unstamped; zeros on timeout.
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

        self.declare_parameter('wheelbase', 1.33)
        self.declare_parameter('max_steering_angle', 1.047)
        self.declare_parameter('max_lateral_accel', 1.5)
        self.declare_parameter('max_linear_vel', 2.78)
        self.declare_parameter('min_speed_for_steer', 0.05)
        self.declare_parameter('steer_slew_rate', 1.047)      # rad/s = 60 deg/s
        self.declare_parameter('steer_lowpass_alpha', 0.4)    # 0..1 lower=smoother
        self.declare_parameter('input_topic', '/etrike/cmd_vel')
        self.declare_parameter('output_topic',
                               '/tricycle_steering_controller/reference_unstamped')
        self.declare_parameter('input_timeout', 0.5)
        self.declare_parameter('publish_rate', 50.0)

        gp = self.get_parameter
        self.L = float(gp('wheelbase').value)
        self.delta_max = float(gp('max_steering_angle').value)
        self.a_lat_max = float(gp('max_lateral_accel').value)
        self.v_max = float(gp('max_linear_vel').value)
        self.v_steer_min = float(gp('min_speed_for_steer').value)
        self.steer_rate = float(gp('steer_slew_rate').value)
        self.lp_alpha = clamp(float(gp('steer_lowpass_alpha').value), 0.01, 1.0)
        self.timeout = float(gp('input_timeout').value)
        self.rate = float(gp('publish_rate').value)
        in_topic = gp('input_topic').value
        out_topic = gp('output_topic').value

        self.pub = self.create_publisher(Twist, out_topic, 10)
        self.sub = self.create_subscription(Twist, in_topic, self.on_cmd, 10)

        self._tgt_v = 0.0
        self._tgt_w = 0.0
        self._last_w = 0.0
        self._last_stamp = None
        self.dt = 1.0 / self.rate
        self.create_timer(self.dt, self.on_timer)

        self.get_logger().info(
            'cmd_vel_limiter+actuator: %s -> %s | L=%.2f delta_max=%.3f '
            'a_lat_max=%.2f slew=%.3frad/s(%.0fdeg/s) lp=%.2f'
            % (in_topic, out_topic, self.L, self.delta_max, self.a_lat_max,
               self.steer_rate, math.degrees(self.steer_rate), self.lp_alpha))

    def _safety_w(self, v, w_in):
        av = abs(v)
        if av < self.v_steer_min:
            return 0.0
        w_steer = av * math.tan(self.delta_max) / self.L
        w_accel = self.a_lat_max / av
        w_max = min(w_steer, w_accel)
        return clamp(w_in, -w_max, w_max)

    def on_cmd(self, msg: Twist):
        self._tgt_v = clamp(float(msg.linear.x), -self.v_max, self.v_max)
        self._tgt_w = float(msg.angular.z)
        self._last_stamp = self.get_clock().now()

    def on_timer(self):
        if self._last_stamp is None:
            return
        age = (self.get_clock().now() - self._last_stamp).nanoseconds * 1e-9
        if age > self.timeout:
            self._last_w = 0.0
            self.pub.publish(Twist())
            return

        v = self._tgt_v
        w_target = self._safety_w(v, self._tgt_w)

        # Actuator model: max angular-velocity step a steer_rate-limited
        # actuator allows at this speed. dw/dt ~= (v/L)*dd/dt for small angle.
        # Floor keeps it from freezing near zero speed.
        max_dw = max(0.05, abs(v) * self.steer_rate / self.L) * self.dt
        w = clamp(w_target, self._last_w - max_dw, self._last_w + max_dw)
        # light low-pass to kill residual jitter
        w = self.lp_alpha * w + (1.0 - self.lp_alpha) * self._last_w
        self._last_w = w

        out = Twist()
        out.linear.x = v
        out.angular.z = w
        self.pub.publish(out)


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
