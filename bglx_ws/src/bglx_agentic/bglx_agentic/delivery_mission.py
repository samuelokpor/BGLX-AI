#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String


def quat_from_yaw(yaw):
    return (
        math.sin(yaw / 2.0),
        math.cos(yaw / 2.0),
    )


class DeliveryMission(Node):

    def __init__(self):

        super().__init__('bglx_delivery_mission')

        # ==================================================
        # Mission configuration
        # ==================================================

        self.declare_parameter('frame', 'map')

        self.declare_parameter(
            'leg_timeout',
            120.0
        )

        self.declare_parameter(
            'pickup_dwell',
            3.0
        )

        self.declare_parameter(
            'delivery_dwell',
            3.0
        )

        self.declare_parameter(
            'max_leg_attempts',
            2
        )

        self.declare_parameter(
            'retry_delay',
            2.0
        )

        # --------------------------------------------------
        # Validated Oxford mission coordinates.
        #
        # Arrival headings are aligned with the incoming
        # direction of travel to avoid forcing the tricycle
        # into unnecessary end-of-goal turning.
        # --------------------------------------------------

        self.declare_parameter(
            'home_x',
            0.000
        )

        self.declare_parameter(
            'home_y',
            0.000
        )

        self.declare_parameter(
            'home_yaw_deg',
            -132.14
        )

        self.declare_parameter(
            'pickup_x',
            5.514
        )

        self.declare_parameter(
            'pickup_y',
            0.098
        )

        self.declare_parameter(
            'pickup_yaw_deg',
            1.02
        )

        self.declare_parameter(
            'delivery_x',
            4.969
        )

        self.declare_parameter(
            'delivery_y',
            5.492
        )

        self.declare_parameter(
            'delivery_yaw_deg',
            95.77
        )

        gp = self.get_parameter

        self.frame = str(
            gp('frame').value
        )

        self.leg_timeout = float(
            gp('leg_timeout').value
        )

        self.pickup_dwell = float(
            gp('pickup_dwell').value
        )

        self.delivery_dwell = float(
            gp('delivery_dwell').value
        )

        self.max_leg_attempts = max(
            1,
            int(
                gp('max_leg_attempts').value
            )
        )

        self.retry_delay = max(
            0.0,
            float(
                gp('retry_delay').value
            )
        )

        self.home = (
            float(gp('home_x').value),
            float(gp('home_y').value),
            math.radians(
                float(
                    gp('home_yaw_deg').value
                )
            ),
        )

        self.pickup = (
            float(gp('pickup_x').value),
            float(gp('pickup_y').value),
            math.radians(
                float(
                    gp('pickup_yaw_deg').value
                )
            ),
        )

        self.delivery = (
            float(gp('delivery_x').value),
            float(gp('delivery_y').value),
            math.radians(
                float(
                    gp('delivery_yaw_deg').value
                )
            ),
        )

        # ==================================================
        # ROS interfaces
        # ==================================================

        self.state_pub = self.create_publisher(
            String,
            '/bglx/mission/state',
            10
        )

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        self.active_goal_handle = None

        self.last_feedback_print = 0.0
        self.current_recoveries = 0

    # ======================================================
    # Mission state
    # ======================================================

    def set_state(self, state):

        msg = String()
        msg.data = state

        self.state_pub.publish(msg)

        print()
        print(
            'MISSION STATE: %s'
            % state
        )

    # ======================================================
    # Navigation feedback
    # ======================================================

    def feedback_cb(self, msg):

        fb = msg.feedback
        now = time.monotonic()

        self.current_recoveries = int(
            fb.number_of_recoveries
        )

        if (
            now - self.last_feedback_print
            < 2.0
        ):
            return

        p = fb.current_pose.pose.position

        print(
            '  pose=(%.2f, %.2f) '
            'remaining=%.2fm '
            'recoveries=%d'
            % (
                p.x,
                p.y,
                fb.distance_remaining,
                self.current_recoveries,
            )
        )

        self.last_feedback_print = now

    # ======================================================
    # Cancel active navigation goal
    # ======================================================

    def cancel_active_goal(self):

        if self.active_goal_handle is None:
            return

        try:

            cancel_future = (
                self.active_goal_handle.
                cancel_goal_async()
            )

            rclpy.spin_until_future_complete(
                self,
                cancel_future,
                timeout_sec=3.0
            )

        except Exception:
            pass

        self.active_goal_handle = None

    # ======================================================
    # Navigate one mission leg
    # ======================================================

    def navigate(
        self,
        destination_name,
        waypoint
    ):

        x, y, yaw = waypoint

        self.current_recoveries = 0
        self.last_feedback_print = 0.0

        print()
        print(
            '--------------------------------------'
        )

        print(
            'NAVIGATING TO %s'
            % destination_name
        )

        print(
            'goal=(%.3f, %.3f, %.2f deg)'
            % (
                x,
                y,
                math.degrees(yaw),
            )
        )

        print(
            '--------------------------------------'
        )

        goal = NavigateToPose.Goal()

        goal.pose.header.frame_id = (
            self.frame
        )

        # Use the same zero-stamp convention as robot_tools.py.
        # This lets Nav2 transform the goal using the latest TF.
        goal.pose.header.stamp = rclpy.time.Time().to_msg()

        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y

        qz, qw = quat_from_yaw(yaw)

        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        send_future = (
            self.nav_client.
            send_goal_async(
                goal,
                feedback_callback=
                self.feedback_cb
            )
        )

        rclpy.spin_until_future_complete(
            self,
            send_future,
            timeout_sec=5.0
        )

        if not send_future.done():
            print(
                'FAIL: Nav2 did not answer goal request '
                'within 5 seconds'
            )
            return False

        handle = send_future.result()

        if (
            handle is None
            or not handle.accepted
        ):

            print(
                'FAIL: %s goal rejected'
                % destination_name
            )

            return False

        self.active_goal_handle = handle

        print(
            'Goal accepted by Nav2.'
        )

        result_future = (
            handle.get_result_async()
        )

        start = time.monotonic()

        while (
            rclpy.ok()
            and not result_future.done()
        ):

            rclpy.spin_once(
                self,
                timeout_sec=0.10
            )

            if (
                time.monotonic() - start
                > self.leg_timeout
            ):

                print(
                    'TIMEOUT: %s'
                    % destination_name
                )

                self.cancel_active_goal()

                return False

        wrapped = result_future.result()

        self.active_goal_handle = None

        if wrapped is None:

            print(
                'FAIL: no result from %s'
                % destination_name
            )

            return False

        status = wrapped.status

        print(
            '%s result: '
            'status=%d recoveries=%d'
            % (
                destination_name,
                status,
                self.current_recoveries,
            )
        )

        if (
            status
            == GoalStatus.STATUS_SUCCEEDED
        ):

            print(
                'ARRIVED: %s'
                % destination_name
            )

            return True

        return False

    # ======================================================
    # Navigate with controlled retry
    # ======================================================

    def navigate_with_retry(
        self,
        destination_name,
        waypoint
    ):

        for attempt in range(
            1,
            self.max_leg_attempts + 1
        ):

            print()
            print(
                'ATTEMPT %d/%d: %s'
                % (
                    attempt,
                    self.max_leg_attempts,
                    destination_name,
                )
            )

            if self.navigate(
                destination_name,
                waypoint
            ):
                return True

            if (
                attempt
                < self.max_leg_attempts
            ):

                self.set_state(
                    'RETRYING_%s'
                    % destination_name
                )

                print(
                    'Retrying %s in %.1f seconds...'
                    % (
                        destination_name,
                        self.retry_delay,
                    )
                )

                time.sleep(
                    self.retry_delay
                )

        print(
            'FAIL: %s exhausted %d attempt(s)'
            % (
                destination_name,
                self.max_leg_attempts,
            )
        )

        return False

    # ======================================================
    # Parcel operation
    # ======================================================

    def dwell(
        self,
        description,
        seconds
    ):

        seconds = max(
            0.0,
            float(seconds)
        )

        print(
            '%s for %.1f seconds...'
            % (
                description,
                seconds,
            )
        )

        time.sleep(seconds)

    # ======================================================
    # Full delivery mission
    # ======================================================

    def run_mission(self):

        print()
        print(
            '======================================'
        )

        print(
            '       BGLX DELIVERY MISSION'
        )

        print(
            '======================================'
        )

        print(
            'HOME       '
            '(%.3f, %.3f, %.2f deg)'
            % (
                self.home[0],
                self.home[1],
                math.degrees(
                    self.home[2]
                ),
            )
        )

        print(
            'PICKUP_A   '
            '(%.3f, %.3f, %.2f deg)'
            % (
                self.pickup[0],
                self.pickup[1],
                math.degrees(
                    self.pickup[2]
                ),
            )
        )

        print(
            'DELIVERY_A '
            '(%.3f, %.3f, %.2f deg)'
            % (
                self.delivery[0],
                self.delivery[1],
                math.degrees(
                    self.delivery[2]
                ),
            )
        )

        print()

        if not self.nav_client.wait_for_server(
            timeout_sec=10.0
        ):

            self.set_state(
                'MISSION_ABORTED'
            )

            print(
                'FAIL: NavigateToPose '
                'server unavailable'
            )

            return False

        # ==================================================
        # HOME -> PICKUP
        # ==================================================

        self.set_state(
            'NAVIGATING_TO_PICKUP'
        )

        if not self.navigate_with_retry(
            'PICKUP_A',
            self.pickup
        ):

            self.set_state(
                'MISSION_ABORTED'
            )

            print(
                'ABORT: failed to reach '
                'PICKUP_A'
            )

            return False

        # ==================================================
        # PICKUP
        # ==================================================

        self.set_state(
            'AT_PICKUP'
        )

        self.set_state(
            'LOADING'
        )

        self.dwell(
            'Loading parcel',
            self.pickup_dwell
        )

        self.set_state(
            'PARCEL_LOADED'
        )

        # ==================================================
        # PICKUP -> DELIVERY
        # ==================================================

        self.set_state(
            'NAVIGATING_TO_DELIVERY'
        )

        if not self.navigate_with_retry(
            'DELIVERY_A',
            self.delivery
        ):

            self.set_state(
                'MISSION_ABORTED'
            )

            print(
                'ABORT: failed to reach '
                'DELIVERY_A'
            )

            return False

        # ==================================================
        # DELIVERY
        # ==================================================

        self.set_state(
            'AT_DELIVERY'
        )

        self.set_state(
            'UNLOADING'
        )

        self.dwell(
            'Unloading parcel',
            self.delivery_dwell
        )

        self.set_state(
            'PARCEL_DELIVERED'
        )

        # ==================================================
        # DELIVERY -> HOME
        # ==================================================

        self.set_state(
            'RETURNING_HOME'
        )

        if not self.navigate_with_retry(
            'HOME',
            self.home
        ):

            self.set_state(
                'MISSION_ABORTED'
            )

            print(
                'ABORT: failed to return HOME'
            )

            return False

        # ==================================================
        # COMPLETE
        # ==================================================

        self.set_state(
            'AT_HOME'
        )

        self.set_state(
            'MISSION_COMPLETE'
        )

        print()
        print(
            '======================================'
        )

        print(
            '     DELIVERY MISSION COMPLETE'
        )

        print(
            '======================================'
        )

        return True


def main(args=None):

    rclpy.init(args=args)

    node = DeliveryMission()

    success = False

    try:

        success = node.run_mission()

    except KeyboardInterrupt:

        print()
        print(
            'Operator interrupted mission.'
        )

        node.cancel_active_goal()

        node.set_state(
            'MISSION_INTERRUPTED'
        )

    except Exception as exc:

        node.cancel_active_goal()

        node.set_state(
            'MISSION_ABORTED'
        )

        node.get_logger().error(
            'Mission exception: %s'
            % exc
        )

    finally:

        node.cancel_active_goal()

        node.destroy_node()

        rclpy.shutdown()

    if not success:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
