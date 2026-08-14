#!/usr/bin/env python3
"""
cmd_vel_limiter

Nav2 -> tricycle controller bridge with:
  1) steering / lateral-acceleration limits,
  2) steering actuator slew + low-pass model,
  3) command watchdog,
  4) HARD forward low-LiDAR stop.

The hard stop is independent of Nav2's costmap. If the low front LiDAR sees
something inside the emergency envelope, positive forward velocity is
suppressed even if Nav2 is still commanding motion.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


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
        self.declare_parameter('steer_slew_rate', 1.047)
        self.declare_parameter('steer_lowpass_alpha', 0.4)

        self.declare_parameter('input_topic', '/etrike/cmd_vel')
        self.declare_parameter(
            'output_topic',
            '/tricycle_steering_controller/reference_unstamped')

        self.declare_parameter('input_timeout', 0.5)
        self.declare_parameter('publish_rate', 50.0)

        # Independent low-LiDAR collision guard.
        self.declare_parameter('front_scan_topic', '/etrike/front_scan')
        self.declare_parameter('front_stop_distance', 1.0)
        self.declare_parameter('front_stop_half_angle_deg', 45.0)
        self.declare_parameter('front_scan_timeout', 0.75)
        self.declare_parameter('fail_closed_on_front_scan_loss', True)

        # Independent reverse-motion safety guard.
        self.declare_parameter(
            'rear_scan_topic',
            '/etrike/rear_depth/scan')
        self.declare_parameter(
            'rear_stop_distance',
            0.80)
        self.declare_parameter(
            'rear_stop_half_angle_deg',
            20.0)
        self.declare_parameter(
            'rear_scan_timeout',
            0.75)
        self.declare_parameter(
            'fail_closed_on_rear_scan_loss',
            True)

        # Independent forward terrain safety guard.
        #
        # Terrain hazards restrict forward motion only so the
        # vehicle retains the ability to reverse away from a
        # curb, drop-off, low obstacle, or unsafe slope.
        self.declare_parameter(
            'terrain_hazard_topic',
            '/etrike/terrain/hard_stop')
        self.declare_parameter(
            'terrain_hazard_timeout',
            0.60)
        self.declare_parameter(
            'fail_closed_on_terrain_loss',
            True)

        gp = self.get_parameter

        self.L = float(gp('wheelbase').value)
        self.delta_max = float(gp('max_steering_angle').value)
        self.a_lat_max = float(gp('max_lateral_accel').value)
        self.v_max = float(gp('max_linear_vel').value)
        self.v_steer_min = float(gp('min_speed_for_steer').value)
        self.steer_rate = float(gp('steer_slew_rate').value)

        self.lp_alpha = clamp(
            float(gp('steer_lowpass_alpha').value),
            0.01,
            1.0)

        self.timeout = float(gp('input_timeout').value)
        self.rate = float(gp('publish_rate').value)

        in_topic = gp('input_topic').value
        out_topic = gp('output_topic').value

        self.front_scan_topic = gp('front_scan_topic').value
        self.front_stop_distance = float(
            gp('front_stop_distance').value)

        self.front_half_angle = math.radians(
            float(gp('front_stop_half_angle_deg').value))

        self.front_scan_timeout = float(
            gp('front_scan_timeout').value)

        self.fail_closed = bool(
            gp('fail_closed_on_front_scan_loss').value)

        self.rear_scan_topic = gp(
            'rear_scan_topic').value
        self.rear_stop_distance = float(
            gp('rear_stop_distance').value)
        self.rear_half_angle = math.radians(
            float(gp('rear_stop_half_angle_deg').value))
        self.rear_scan_timeout = float(
            gp('rear_scan_timeout').value)
        self.rear_fail_closed = bool(
            gp('fail_closed_on_rear_scan_loss').value)

        self.terrain_hazard_topic = gp(
            'terrain_hazard_topic').value
        self.terrain_hazard_timeout = float(
            gp('terrain_hazard_timeout').value)
        self.terrain_fail_closed = bool(
            gp('fail_closed_on_terrain_loss').value)

        self.pub = self.create_publisher(
            Twist,
            out_topic,
            10)

        self.sub = self.create_subscription(
            Twist,
            in_topic,
            self.on_cmd,
            10)

        self.front_scan_sub = self.create_subscription(
            LaserScan,
            self.front_scan_topic,
            self.on_front_scan,
            qos_profile_sensor_data)

        self.rear_scan_sub = self.create_subscription(
            LaserScan,
            self.rear_scan_topic,
            self.on_rear_scan,
            qos_profile_sensor_data)

        self.terrain_hazard_sub = self.create_subscription(
            Bool,
            self.terrain_hazard_topic,
            self.on_terrain_hazard,
            10)

        self._tgt_v = 0.0
        self._tgt_w = 0.0
        self._last_w = 0.0
        self._last_stamp = None

        self._front_near = None
        self._front_scan_stamp = None

        self._rear_near = None
        self._rear_scan_stamp = None

        self._terrain_hazard = None
        self._terrain_hazard_stamp = None

        self._hard_stop_active = False
        self._last_stop_log_time = -1e9

        self.dt = 1.0 / self.rate
        self.create_timer(self.dt, self.on_timer)

        self.get_logger().info(
            'cmd_vel_limiter: %s -> %s | '
            'L=%.2f delta_max=%.3f a_lat_max=%.2f '
            'slew=%.3frad/s(%.0fdeg/s) lp=%.2f'
            % (
                in_topic,
                out_topic,
                self.L,
                self.delta_max,
                self.a_lat_max,
                self.steer_rate,
                math.degrees(self.steer_rate),
                self.lp_alpha))

        self.get_logger().info(
            'HARD FRONT STOP enabled: '
            'scan=%s distance=%.2fm arc=+/-%.0fdeg '
            'scan_timeout=%.2fs fail_closed=%s'
            % (
                self.front_scan_topic,
                self.front_stop_distance,
                math.degrees(self.front_half_angle),
                self.front_scan_timeout,
                self.fail_closed))

    def _now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_front_scan(self, msg: LaserScan):
        """
        Find nearest valid low-LiDAR return inside the
        forward collision-safety arc.
        """
        nearest = None

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue

            if r < msg.range_min or r > msg.range_max:
                continue

            angle = msg.angle_min + i * msg.angle_increment

            if abs(angle) > self.front_half_angle:
                continue

            if nearest is None or r < nearest:
                nearest = float(r)

        self._front_near = nearest
        self._front_scan_stamp = self.get_clock().now()

    def _front_guard_reason(self, v):
        """
        Return a reason if forward movement must be stopped.
        Otherwise return None.
        """

        # This front sensor does not restrict reverse motion.
        if v <= 0.0:
            return None

        if self._front_scan_stamp is None:
            if self.fail_closed:
                return 'no front_scan received yet'
            return None

        age = (
            self.get_clock().now() -
            self._front_scan_stamp
        ).nanoseconds * 1e-9

        if age > self.front_scan_timeout:
            if self.fail_closed:
                return 'front_scan stale (%.2fs old)' % age
            return None

        if (
            self._front_near is not None
            and self._front_near <= self.front_stop_distance
        ):
            return 'low obstacle %.2fm ahead' % self._front_near

        return None

    def on_rear_scan(self, msg: LaserScan):
        """
        Find the nearest valid return inside the rear
        collision-safety arc.

        rear_depth_link is physically rotated to face -X,
        so angle zero in this LaserScan is straight behind
        the vehicle.
        """
        nearest = None

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue

            if r < msg.range_min or r > msg.range_max:
                continue

            angle = msg.angle_min + i * msg.angle_increment

            if abs(angle) > self.rear_half_angle:
                continue

            if nearest is None or r < nearest:
                nearest = float(r)

        self._rear_near = nearest
        self._rear_scan_stamp = self.get_clock().now()


    def _rear_guard_reason(self, v):
        """
        Return a reason if reverse movement must be stopped.

        Rear sensor state never blocks forward escape.
        """

        # This rear sensor only restricts reverse motion.
        if v >= 0.0:
            return None

        if self._rear_scan_stamp is None:
            if self.rear_fail_closed:
                return 'no rear_scan received yet'
            return None

        age = (
            self.get_clock().now() -
            self._rear_scan_stamp
        ).nanoseconds * 1e-9

        if age > self.rear_scan_timeout:
            if self.rear_fail_closed:
                return 'rear_scan stale (%.2fs old)' % age
            return None

        if (
            self._rear_near is not None
            and self._rear_near <= self.rear_stop_distance
        ):
            return (
                'rear obstacle %.2fm behind'
                % self._rear_near
            )

        return None


    def on_terrain_hazard(self, msg: Bool):
        """
        Cache the latest terrain hazard decision.

        Bool has no header, so freshness is measured from the
        local receive time.
        """
        self._terrain_hazard = bool(msg.data)
        self._terrain_hazard_stamp = self.get_clock().now()


    def _terrain_guard_reason(self, v):
        """
        Return a reason if forward movement must be stopped by
        terrain perception.

        Terrain state never blocks reverse escape.
        """

        if v <= 0.0:
            return None

        if self._terrain_hazard_stamp is None:
            if self.terrain_fail_closed:
                return 'no terrain hazard state received yet'
            return None

        age = (
            self.get_clock().now() -
            self._terrain_hazard_stamp
        ).nanoseconds * 1e-9

        if age > self.terrain_hazard_timeout:
            if self.terrain_fail_closed:
                return (
                    'terrain hazard state stale '
                    '(%.2fs old)' % age
                )
            return None

        if self._terrain_hazard:
            return 'terrain hazard reported'

        return None


    def _safety_w(self, v, w_in):
        av = abs(v)

        if av < self.v_steer_min:
            return 0.0

        w_steer = (
            av *
            math.tan(self.delta_max) /
            self.L
        )

        w_accel = self.a_lat_max / av

        w_max = min(
            w_steer,
            w_accel)

        return clamp(
            w_in,
            -w_max,
            w_max)

    def on_cmd(self, msg: Twist):
        self._tgt_v = clamp(
            float(msg.linear.x),
            -self.v_max,
            self.v_max)

        self._tgt_w = float(msg.angular.z)

        self._last_stamp = self.get_clock().now()

    def on_timer(self):
        if self._last_stamp is None:
            return

        age = (
            self.get_clock().now() -
            self._last_stamp
        ).nanoseconds * 1e-9

        # Existing cmd_vel watchdog.
        if age > self.timeout:
            self._last_w = 0.0
            self.pub.publish(Twist())
            return

        v = self._tgt_v

        # ---------------------------------------------------------
        # HARD LOW-LIDAR SAFETY GATE
        #
        # This happens AFTER Nav2.
        # Nav2 may make a bad decision; this layer still stops.
        # ---------------------------------------------------------

        front_reason = self._front_guard_reason(v)
        terrain_reason = self._terrain_guard_reason(v)
        rear_reason = self._rear_guard_reason(v)

        if front_reason is not None:
            stop_reason = front_reason
            stop_direction = 'FRONT'
        elif terrain_reason is not None:
            stop_reason = terrain_reason
            stop_direction = 'TERRAIN'
        elif rear_reason is not None:
            stop_reason = rear_reason
            stop_direction = 'REAR'
        else:
            stop_reason = None
            stop_direction = None

        if stop_reason is not None:
            self._last_w = 0.0

            # Zero linear AND angular command immediately.
            self.pub.publish(Twist())

            now_sec = self._now_sec()

            # Avoid flooding the terminal.
            if (
                not self._hard_stop_active
                or now_sec - self._last_stop_log_time >= 1.0
            ):
                self.get_logger().warn(
                    'HARD %s STOP: %s'
                    % (stop_direction, stop_reason))

                self._last_stop_log_time = now_sec

            self._hard_stop_active = True
            return

        if self._hard_stop_active:
            self.get_logger().info(
                'HARD SAFETY STOP cleared')

            self._hard_stop_active = False

        # ---------------------------------------------------------
        # Existing steering safety / actuator model
        # ---------------------------------------------------------

        w_target = self._safety_w(
            v,
            self._tgt_w)

        # Apply the safe yaw-rate command immediately.
        #
        # Additional yaw-rate slew / low-pass filtering caused excessive
        # steering lag on the tricycle: the vehicle moved forward before
        # developing the curvature requested by Nav2. The downstream
        # steering-angle and lateral-acceleration limits in _safety_w()
        # remain active.
        w = w_target

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
