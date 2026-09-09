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
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from .concept_scan_geometry import transform_matrix, scan_points, corridor_clearance, usable_scan


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class CmdVelLimiter(Node):
    def __init__(self):
        super().__init__('cmd_vel_limiter')

        self.declare_parameter('wheelbase', 1.20)
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
        self.declare_parameter('front_corridor_half_width', 0.52)
        # --- speed-scaled clearance cap (test feature, default off) ---
        self.declare_parameter('use_speed_scaled_cap', False)
        self.declare_parameter('cap_margin', 0.25)
        self.declare_parameter('cap_latency', 0.30)
        self.declare_parameter('cap_brake_accel', 1.0)
        self.declare_parameter('cap_hard_floor', 0.10)
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

        # Independent side-sensor health watchdogs.
        #
        # Collision Monitor on ROS 2 Humble does not reliably
        # fail closed when an observation source disappears.
        # Therefore the final velocity limiter independently
        # verifies that both side safety scans remain alive.
        self.declare_parameter(
            'left_scan_topic',
            '/etrike/left_depth/scan')
        self.declare_parameter(
            'left_scan_timeout',
            0.75)
        self.declare_parameter(
            'fail_closed_on_left_scan_loss',
            True)

        self.declare_parameter(
            'right_scan_topic',
            '/etrike/right_depth/scan')
        self.declare_parameter(
            'right_scan_timeout',
            0.75)
        self.declare_parameter(
            'fail_closed_on_right_scan_loss',
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

        self.front_corridor_half_width = float(
            gp('front_corridor_half_width').value)

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

        self.left_scan_topic = gp(
            'left_scan_topic').value
        self.left_scan_timeout = float(
            gp('left_scan_timeout').value)
        self.left_fail_closed = bool(
            gp('fail_closed_on_left_scan_loss').value)

        self.right_scan_topic = gp(
            'right_scan_topic').value
        self.right_scan_timeout = float(
            gp('right_scan_timeout').value)
        self.right_fail_closed = bool(
            gp('fail_closed_on_right_scan_loss').value)

        self.terrain_hazard_topic = gp(
            'terrain_hazard_topic').value
        self.terrain_hazard_timeout = float(
            gp('terrain_hazard_timeout').value)
        self.terrain_fail_closed = bool(
            gp('fail_closed_on_terrain_loss').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

        self.left_scan_sub = self.create_subscription(
            LaserScan,
            self.left_scan_topic,
            self.on_left_scan,
            qos_profile_sensor_data)

        self.right_scan_sub = self.create_subscription(
            LaserScan,
            self.right_scan_topic,
            self.on_right_scan,
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

        self._left_scan_stamp = None
        self._right_scan_stamp = None

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

    def _project_scan(self, msg, timeout):
        stamp = Time.from_msg(msg.header.stamp)
        age = (self.get_clock().now()-stamp).nanoseconds*1e-9
        if stamp.nanoseconds <= 0 or age < -0.05 or age > timeout or not usable_scan(msg):
            raise ValueError('Unusable or stale scan')
        transform = self.tf_buffer.lookup_transform('base_link', msg.header.frame_id, stamp)
        points, _ = scan_points(msg, transform_matrix(transform.transform))
        return points, stamp

    def on_front_scan(self, msg: LaserScan):
        """Distance beyond the front footprint edge, using measured steering TF."""
        try:
            points, stamp = self._project_scan(msg, self.front_scan_timeout)
            near = corridor_clearance(points, True, self.front_corridor_half_width)
        except (TransformException, ValueError):
            self._front_scan_stamp = None
            self._front_near = None
            return
        self._front_near = near
        self._front_scan_stamp = stamp

        # Diagnostic only: report points responsible for near-zero clearance.
        # Failures here must not change the safety decision.
        try:
            floor = float(self.get_parameter('cap_hard_floor').value)
            now = self._now_sec()
            if (near is not None and near <= floor and
                    now - getattr(self, '_last_scan_debug', -1e9) >= 0.25):
                self._last_scan_debug = now
                candidates = [
                    point for point in points
                    if point[0] >= 0.0
                    and abs(point[1]) <= self.front_corridor_half_width
                ]
                candidates.sort(key=lambda point: float(point[0]))
                coords = '; '.join(
                    '(%.3f, %.3f, %.3f)' % tuple(point)
                    for point in candidates[:6]
                )
                sensor_tf = self.tf_buffer.lookup_transform(
                    'base_link', msg.header.frame_id, stamp)
                q = sensor_tf.transform.rotation
                sensor_yaw = math.degrees(math.atan2(
                    2.0*(q.w*q.z + q.x*q.y),
                    1.0 - 2.0*(q.y*q.y + q.z*q.z)))
                steering_text = 'unavailable'
                try:
                    steering_tf = self.tf_buffer.lookup_transform(
                        'base_link', 'caster_mount', stamp)
                    q = steering_tf.transform.rotation
                    steering_text = '%.1f' % math.degrees(math.atan2(
                        2.0*(q.w*q.z + q.x*q.y),
                        1.0 - 2.0*(q.y*q.y + q.z*q.z)))
                except TransformException:
                    pass
                self.get_logger().warn(
                    'SCAN_EVIDENCE stamp=%.6f frame=%s '
                    'sensor_yaw=%.1fdeg fork_yaw=%sdeg '
                    'clearance=%.3f candidates=%d '
                    'target_v=%.3f target_w=%.3f '
                    'base_xyz=[%s]' % (
                        stamp.nanoseconds * 1e-9,
                        msg.header.frame_id, sensor_yaw, steering_text,
                        near, len(candidates),
                        self._tgt_v, self._tgt_w, coords))
        except Exception as exc:
            now = self._now_sec()
            if now - getattr(self, '_last_debug_error', -1e9) >= 2.0:
                self._last_debug_error = now
                self.get_logger().warn('SCAN_EVIDENCE error: %s' % exc)

    def _front_guard_reason(self, v):
        """
        Return a reason if forward movement must be stopped.
        Otherwise return None.
        """

        # This front sensor does not restrict reverse motion.
        if v <= 0.0:
            return None

        side_reason = self._side_scan_guard_reason()

        if side_reason is not None:
            return side_reason

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

        if bool(self.get_parameter('use_speed_scaled_cap').value):
            # Distance restriction is applied as a speed cap in on_timer().
            # Staleness / fail-closed vetoes above still apply.
            return None

        if (
            self._front_near is not None
            and self._front_near <= float(
                self.get_parameter('front_stop_distance').value)
        ):
            return 'low obstacle %.2fm ahead' % self._front_near

        return None

    def _front_speed_cap(self):
        """
        Largest forward speed whose stopping envelope fits the measured
        corridor clearance:

            d_required(v) = margin + v * latency + v**2 / (2 * brake)

        solved for the largest v with d_required(v) <= clearance.

        Returns None when no clearance reading exists; that case is
        already covered by the validity vetoes in _front_guard_reason().
        """

        near = self._front_near

        if near is None:
            return None

        margin = float(self.get_parameter('cap_margin').value)
        latency = float(self.get_parameter('cap_latency').value)
        brake = float(self.get_parameter('cap_brake_accel').value)

        usable = near - margin

        if usable <= 0.0 or brake <= 0.0:
            return 0.0

        at = brake * latency

        return max(0.0, min(-at + math.sqrt(at * at + 2.0 * brake * usable),
                            self.v_max))


    def on_rear_scan(self, msg: LaserScan):
        """Clearance from the rear footprint, including the cargo sensor tilt."""
        try:
            # gazebo_ros LaserScan uses the centre vertical row.
            points, stamp = self._project_scan(msg, self.rear_scan_timeout)
            near = corridor_clearance(points, False, self.front_corridor_half_width)
        except (TransformException, ValueError):
            self._rear_scan_stamp = None
            self._rear_near = None
            return
        self._rear_near = near
        self._rear_scan_stamp = stamp


    def _rear_guard_reason(self, v):
        """
        Return a reason if reverse movement must be stopped.

        Rear sensor state never blocks forward escape.
        """

        # This rear sensor only restricts reverse motion.
        if v >= 0.0:
            return None

        side_reason = self._side_scan_guard_reason()

        if side_reason is not None:
            return side_reason

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


    def on_left_scan(self, msg: LaserScan):
        stamp = Time.from_msg(msg.header.stamp)
        age = (self.get_clock().now()-stamp).nanoseconds*1e-9
        self._left_scan_stamp = (stamp if stamp.nanoseconds > 0 and
            -0.05 <= age <= self.left_scan_timeout and usable_scan(msg) else None)


    def on_right_scan(self, msg: LaserScan):
        stamp = Time.from_msg(msg.header.stamp)
        age = (self.get_clock().now()-stamp).nanoseconds*1e-9
        self._right_scan_stamp = (stamp if stamp.nanoseconds > 0 and
            -0.05 <= age <= self.right_scan_timeout and usable_scan(msg) else None)


    def _side_scan_guard_reason(self):
        """
        Fail closed if either required side safety scan is
        missing or stale.

        Unlike the front/rear geometric guards, side-source
        loss blocks motion in either direction because 360
        degree lateral clearance can no longer be guaranteed.
        """

        now = self.get_clock().now()

        if self._left_scan_stamp is None:

            if self.left_fail_closed:
                return 'no left_scan received yet'

        else:

            age = (
                now -
                self._left_scan_stamp
            ).nanoseconds * 1e-9

            if (
                age > self.left_scan_timeout and
                self.left_fail_closed
            ):
                return (
                    'left_scan stale '
                    '(%.2fs old)' % age
                )

        if self._right_scan_stamp is None:

            if self.right_fail_closed:
                return 'no right_scan received yet'

        else:

            age = (
                now -
                self._right_scan_stamp
            ).nanoseconds * 1e-9

            if (
                age > self.right_scan_timeout and
                self.right_fail_closed
            ):
                return (
                    'right_scan stale '
                    '(%.2fs old)' % age
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

        # ---------------------------------------------------------
        # SPEED-SCALED CLEARANCE CAP
        #
        # Caps forward speed by measured clearance instead of vetoing
        # motion. Angular velocity is scaled with linear velocity so the
        # commanded curvature (and therefore steering angle) is preserved.
        # Never raises speed; never converts a zero command into motion.
        # ---------------------------------------------------------

        w_in = self._tgt_w

        if v > 0.0 and bool(self.get_parameter('use_speed_scaled_cap').value):
            v_cap = self._front_speed_cap()

            if v_cap is not None and v_cap < v:

                floor = float(self.get_parameter('cap_hard_floor').value)

                if self._front_near is not None and self._front_near <= floor:
                    # Inside the absolute floor: no forward motion at all.
                    self._last_w = 0.0
                    self.pub.publish(Twist())
                    now_sec = self._now_sec()
                    if (not self._hard_stop_active
                            or now_sec - self._last_stop_log_time >= 1.0):
                        self.get_logger().warn(
                            'CAP FLOOR STOP: clearance %.2fm <= floor %.2fm'
                            % (self._front_near, floor))
                        self._last_stop_log_time = now_sec
                    self._hard_stop_active = True
                    return

                if v_cap < self.v_steer_min:
                    # Creep instead of stopping, so the steering angle
                    # stays commandable and the vehicle can turn out of
                    # the situation. Braking distance at this speed is a
                    # couple of centimetres, far inside the clearance
                    # that remains above the absolute floor.
                    v_cap = self.v_steer_min

                if False:
                    # Below the speed at which steering can be commanded,
                    # creeping straight ahead is not a safe option.
                    self._last_w = 0.0
                    self.pub.publish(Twist())

                    now_sec = self._now_sec()

                    if (
                        not self._hard_stop_active
                        or now_sec - self._last_stop_log_time >= 1.0
                    ):
                        self.get_logger().warn(
                            'CAP STOP: clearance %.2fm permits %.3fm/s, '
                            'below steering threshold %.3fm/s'
                            % (self._front_near, v_cap, self.v_steer_min))

                        self._last_stop_log_time = now_sec

                    self._hard_stop_active = True
                    return

                w_in = w_in * (v_cap / v)
                v = v_cap

        if self._hard_stop_active:
            self.get_logger().info(
                'HARD SAFETY STOP cleared')

            self._hard_stop_active = False

        # ---------------------------------------------------------
        # Existing steering safety / actuator model
        # ---------------------------------------------------------

        w_target = self._safety_w(
            v,
            w_in)

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
