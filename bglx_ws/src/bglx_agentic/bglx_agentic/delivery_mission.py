#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

from action_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import BackUp, NavigateToPose
from std_msgs.msg import String

import tf2_ros
from tf2_ros import TransformException

from bglx_agentic.mission_waypoints import (
    build_delivery_route,
    build_multi_stop_route,
)
from bglx_agentic.mission_history import (
    append_mission_record,
    new_mission_id,
    utc_now_iso,
)


MAP_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Goal-directed SLAM exploration.
#
# A mission destination may be outside the CURRENT map. Instead of asking
# Nav2 to plan directly to an impossible goal, drive to a safe intermediate
# point near the current frontier, let SLAM extend /map, then continue toward
# the ORIGINAL mission waypoint.
EXPLORATION_EDGE_MARGIN = 1.25
EXPLORATION_MIN_STAGE = 0.75
EXPLORATION_MIN_PROGRESS = 0.40
EXPLORATION_MAP_WAIT = 4.0
EXPLORATION_SETTLE = 0.75
EXPLORATION_MAX_STAGES = 40


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
        # Early tricycle unstuck / turnaround assistance.
        #
        # These operate ABOVE Nav2. The proven Nav2/BT
        # configuration remains unchanged.
        # --------------------------------------------------

        self.declare_parameter(
            'turnaround_assist_enabled',
            True
        )

        self.declare_parameter(
            'turnaround_angle_deg',
            150.0
        )

        self.declare_parameter(
            'turnaround_min_goal_distance',
            2.0
        )

        self.declare_parameter(
            'early_unstuck_enabled',
            True
        )

        self.declare_parameter(
            'early_unstuck_window',
            4.0
        )

        self.declare_parameter(
            'early_unstuck_min_movement',
            0.15
        )

        self.declare_parameter(
            'early_unstuck_min_goal_distance',
            1.0
        )

        self.declare_parameter(
            'early_unstuck_max_per_goal',
            3
        )

        self.declare_parameter(
            'unstuck_backup_distance',
            0.90
        )

        self.declare_parameter(
            'unstuck_backup_speed',
            0.30
        )

        self.declare_parameter(
            'unstuck_backup_timeout',
            10.0
        )

        self.declare_parameter(
            'unstuck_cooldown',
            1.0
        )

        # --------------------------------------------------
        # Named mission locations.
        #
        # XY positions live in mission_waypoints.py.
        # Arrival yaw is calculated from the incoming
        # direction of travel.
        # --------------------------------------------------

        self.declare_parameter(
            'home_name',
            'HOME'
        )

        self.declare_parameter(
            'pickup_name',
            'PICKUP_A'
        )

        self.declare_parameter(
            'delivery_name',
            'DELIVERY_A'
        )

        self.declare_parameter(
            'delivery_names_csv',
            ''
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

        self.turnaround_assist_enabled = bool(
            gp('turnaround_assist_enabled').value
        )

        self.turnaround_angle_deg = max(
            0.0,
            min(
                180.0,
                float(
                    gp('turnaround_angle_deg').value
                )
            )
        )

        self.turnaround_min_goal_distance = max(
            0.0,
            float(
                gp(
                    'turnaround_min_goal_distance'
                ).value
            )
        )

        self.early_unstuck_enabled = bool(
            gp('early_unstuck_enabled').value
        )

        self.early_unstuck_window = max(
            1.0,
            float(
                gp('early_unstuck_window').value
            )
        )

        self.early_unstuck_min_movement = max(
            0.01,
            float(
                gp(
                    'early_unstuck_min_movement'
                ).value
            )
        )

        self.early_unstuck_min_goal_distance = max(
            0.0,
            float(
                gp(
                    'early_unstuck_min_goal_distance'
                ).value
            )
        )

        self.early_unstuck_max_per_goal = max(
            0,
            int(
                gp(
                    'early_unstuck_max_per_goal'
                ).value
            )
        )

        self.unstuck_backup_distance = max(
            0.05,
            float(
                gp('unstuck_backup_distance').value
            )
        )

        self.unstuck_backup_speed = max(
            0.05,
            float(
                gp('unstuck_backup_speed').value
            )
        )

        self.unstuck_backup_timeout = max(
            1.0,
            float(
                gp('unstuck_backup_timeout').value
            )
        )

        self.unstuck_cooldown = max(
            0.0,
            float(
                gp('unstuck_cooldown').value
            )
        )

        self.home_name = str(
            gp('home_name').value
        )

        self.pickup_name = str(
            gp('pickup_name').value
        )

        self.delivery_name = str(
            gp('delivery_name').value
        )

        self.delivery_names_csv = str(
            gp('delivery_names_csv').value
        ).strip()

        if self.delivery_names_csv:

            requested_delivery_names = [
                name.strip()
                for name
                in self.delivery_names_csv.split(',')
                if name.strip()
            ]

        else:

            requested_delivery_names = [
                self.delivery_name
            ]

        route = build_multi_stop_route(
            self.home_name,
            self.pickup_name,
            requested_delivery_names
        )

        # Store canonical names returned by the registry.
        self.home_name = route[
            'home_name'
        ]

        self.pickup_name = route[
            'pickup_name'
        ]

        self.delivery_names = route[
            'delivery_names'
        ]

        self.pickup = route[
            'pickup'
        ]

        self.delivery_stops = route[
            'deliveries'
        ]

        # Backward compatibility for any code still expecting
        # the original single self.delivery pose.
        self.delivery = self.delivery_stops[
            0
        ][
            'pose'
        ]

        self.home = route[
            'home'
        ]

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

        self.backup_client = ActionClient(
            self,
            BackUp,
            '/backup'
        )

        # --------------------------------------------------
        # Live navigation context for staged SLAM exploration.
        #
        # /map = growing SLAM world representation.
        # /global_costmap/costmap = area Nav2 can currently plan in.
        # --------------------------------------------------

        self.base_frame = 'base_footprint'

        self.slam_bounds = None
        self.costmap_bounds = None
        self.slam_grid = None
        self.slam_info = None

        self.create_subscription(
            OccupancyGrid,
            '/map',
            self._on_slam_map,
            MAP_QOS
        )

        self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self._on_global_costmap,
            MAP_QOS
        )

        self.tf_buffer = tf2_ros.Buffer()

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        self.active_goal_handle = None

        self.last_feedback_print = 0.0
        self.current_recoveries = 0

    # ======================================================
    # Live SLAM / Nav2 navigation context
    # ======================================================

    @staticmethod
    def _bounds_from_grid(msg):

        info = msg.info

        min_x = info.origin.position.x
        min_y = info.origin.position.y

        max_x = (
            min_x
            + info.width * info.resolution
        )

        max_y = (
            min_y
            + info.height * info.resolution
        )

        return (
            min_x,
            min_y,
            max_x,
            max_y,
        )

    def _on_slam_map(self, msg):

        self.slam_bounds = (
            self._bounds_from_grid(msg)
        )

        self.slam_grid = list(
            msg.data
        )

        self.slam_info = msg.info

    def _on_global_costmap(self, msg):

        self.costmap_bounds = (
            self._bounds_from_grid(msg)
        )

    def _navigation_bounds(self):
        """Area currently represented by BOTH SLAM and Nav2.

        A mission staging waypoint must lie inside this intersection.
        """

        slam = self.slam_bounds
        costmap = self.costmap_bounds

        if slam is None:
            return costmap

        if costmap is None:
            return slam

        bounds = (
            max(
                slam[0],
                costmap[0]
            ),
            max(
                slam[1],
                costmap[1]
            ),
            min(
                slam[2],
                costmap[2]
            ),
            min(
                slam[3],
                costmap[3]
            ),
        )

        if (
            bounds[0] >= bounds[2]
            or bounds[1] >= bounds[3]
        ):
            return None

        return bounds

    @staticmethod
    def _point_in_bounds(
        x,
        y,
        bounds
    ):

        if bounds is None:
            return False

        return (
            bounds[0] <= float(x) <= bounds[2]
            and bounds[1] <= float(y) <= bounds[3]
        )

    def _current_pose(self):
        """Current map-frame x, y, yaw from TF."""

        try:

            tf = self.tf_buffer.lookup_transform(
                self.frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(
                    seconds=0.25
                )
            )

        except TransformException:

            return None

        t = tf.transform.translation
        q = tf.transform.rotation

        yaw = math.atan2(
            2.0 * (
                q.w * q.z
                + q.x * q.y
            ),
            1.0 - 2.0 * (
                q.y * q.y
                + q.z * q.z
            )
        )

        return (
            t.x,
            t.y,
            yaw,
        )

    @staticmethod
    def _normalize_angle(angle):
        """Normalize angle to [-pi, +pi]."""

        return math.atan2(
            math.sin(angle),
            math.cos(angle)
        )

    def _safe_backup(
        self,
        reason
    ):
        """Run Nav2 BackUp only when no NavigateToPose is active."""

        if self.active_goal_handle is not None:

            print(
                '[unstuck] REFUSED backup: '
                'NavigateToPose is still active.'
            )

            return False

        if not self.backup_client.wait_for_server(
            timeout_sec=1.0
        ):

            print(
                '[unstuck] /backup action server '
                'is unavailable; continuing with Nav2.'
            )

            return False

        print()
        print(
            '[unstuck] BACKUP: %s'
            % reason
        )

        print(
            '[unstuck] requesting %.2fm reverse '
            'at %.2fm/s'
            % (
                self.unstuck_backup_distance,
                self.unstuck_backup_speed,
            )
        )

        goal = BackUp.Goal()

        # BackUp target is expressed behind the robot.
        goal.target.x = (
            -abs(
                self.unstuck_backup_distance
            )
        )
        goal.target.y = 0.0
        goal.target.z = 0.0

        goal.speed = abs(
            self.unstuck_backup_speed
        )

        seconds = int(
            self.unstuck_backup_timeout
        )

        nanoseconds = int(
            (
                self.unstuck_backup_timeout
                - seconds
            )
            * 1_000_000_000
        )

        goal.time_allowance.sec = seconds
        goal.time_allowance.nanosec = nanoseconds

        send_future = (
            self.backup_client.
            send_goal_async(goal)
        )

        rclpy.spin_until_future_complete(
            self,
            send_future,
            timeout_sec=3.0
        )

        if not send_future.done():

            print(
                '[unstuck] FAIL: /backup did not '
                'answer within 3 seconds.'
            )

            return False

        handle = send_future.result()

        if (
            handle is None
            or not handle.accepted
        ):

            print(
                '[unstuck] FAIL: backup goal rejected.'
            )

            return False

        result_future = (
            handle.get_result_async()
        )

        deadline = (
            time.monotonic()
            + self.unstuck_backup_timeout
            + 2.0
        )

        while (
            rclpy.ok()
            and not result_future.done()
            and time.monotonic() < deadline
        ):

            rclpy.spin_once(
                self,
                timeout_sec=0.10
            )

        if not result_future.done():

            print(
                '[unstuck] FAIL: backup timed out.'
            )

            try:

                cancel_future = (
                    handle.cancel_goal_async()
                )

                rclpy.spin_until_future_complete(
                    self,
                    cancel_future,
                    timeout_sec=2.0
                )

            except Exception:
                pass

            return False

        wrapped = result_future.result()

        if (
            wrapped is not None
            and wrapped.status
            == GoalStatus.STATUS_SUCCEEDED
        ):

            print(
                '[unstuck] BACKUP SUCCEEDED.'
            )

            return True

        status = (
            wrapped.status
            if wrapped is not None
            else -1
        )

        print(
            '[unstuck] backup finished '
            'without success, status=%d.'
            % status
        )

        return False

    def _maybe_turnaround_assist(
        self,
        destination_name,
        goal_x,
        goal_y
    ):
        """Back up before a goal lying almost directly behind the trike."""

        if not self.turnaround_assist_enabled:
            return False

        pose = self._current_pose()

        if pose is None:

            print(
                '[unstuck] turnaround check skipped: '
                'pose unavailable.'
            )

            return False

        px, py, yaw = pose

        dx = float(goal_x) - px
        dy = float(goal_y) - py

        distance = math.hypot(
            dx,
            dy
        )

        if (
            distance
            < self.turnaround_min_goal_distance
        ):
            return False

        desired_yaw = math.atan2(
            dy,
            dx
        )

        error = abs(
            self._normalize_angle(
                desired_yaw - yaw
            )
        )

        error_deg = math.degrees(
            error
        )

        if (
            error_deg
            < self.turnaround_angle_deg
        ):
            return False

        print()
        print(
            '[unstuck] TURNAROUND detected for %s: '
            'goal bearing is %.1f deg from current '
            'heading (threshold %.1f deg).'
            % (
                destination_name,
                error_deg,
                self.turnaround_angle_deg,
            )
        )

        return self._safe_backup(
            'proactive turnaround before %s'
            % destination_name
        )

    def _goal_cell_blocked(
        self,
        x,
        y
    ):
        """Known occupied SLAM cell?

        Unknown cells are intentionally allowed: that is exactly the space
        the active SLAM exploration is trying to reveal.
        """

        grid = self.slam_grid
        info = self.slam_info

        if (
            grid is None
            or info is None
        ):
            return False

        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y

        cx = int(
            (float(x) - ox)
            / res
        )

        cy = int(
            (float(y) - oy)
            / res
        )

        if not (
            0 <= cx < info.width
            and 0 <= cy < info.height
        ):
            return False

        value = grid[
            cy * info.width
            + cx
        ]

        # -1 means unknown and is intentionally permitted.
        return value >= 65

    def _frontier_stage_goal(
        self,
        start_x,
        start_y,
        final_x,
        final_y,
        bounds
    ):
        """Furthest safe intermediate point toward the final destination."""

        dx = (
            float(final_x)
            - float(start_x)
        )

        dy = (
            float(final_y)
            - float(start_y)
        )

        full_distance = math.hypot(
            dx,
            dy
        )

        if full_distance < 1e-6:

            return (
                float(final_x),
                float(final_y),
                0.0,
            )

        ux = dx / full_distance
        uy = dy / full_distance

        min_x, min_y, max_x, max_y = bounds

        distances = []
        eps = 1e-9

        if ux > eps:
            distances.append(
                (max_x - start_x)
                / ux
            )

        elif ux < -eps:
            distances.append(
                (min_x - start_x)
                / ux
            )

        if uy > eps:
            distances.append(
                (max_y - start_y)
                / uy
            )

        elif uy < -eps:
            distances.append(
                (min_y - start_y)
                / uy
            )

        positive = [
            distance
            for distance in distances
            if distance >= 0.0
        ]

        if not positive:

            return (
                float(start_x),
                float(start_y),
                0.0,
            )

        distance_to_edge = min(
            positive
        )

        if (
            full_distance
            <= distance_to_edge
        ):

            return (
                float(final_x),
                float(final_y),
                full_distance,
            )

        stage_distance = max(
            0.0,
            distance_to_edge
            - EXPLORATION_EDGE_MARGIN
        )

        # Pull the candidate backward if the frontier point happens to
        # coincide with a known occupied cell.
        while (
            stage_distance
            >= EXPLORATION_MIN_STAGE
        ):

            stage_x = (
                start_x
                + ux * stage_distance
            )

            stage_y = (
                start_y
                + uy * stage_distance
            )

            if not self._goal_cell_blocked(
                stage_x,
                stage_y
            ):

                return (
                    stage_x,
                    stage_y,
                    stage_distance,
                )

            stage_distance -= 0.25

        return (
            float(start_x),
            float(start_y),
            0.0,
        )

    def _wait_for_navigation_context(
        self,
        timeout=4.0
    ):

        deadline = (
            time.monotonic()
            + float(timeout)
        )

        while (
            rclpy.ok()
            and time.monotonic()
            < deadline
        ):

            rclpy.spin_once(
                self,
                timeout_sec=0.10
            )

            if (
                self._current_pose()
                is not None
                and self._navigation_bounds()
                is not None
            ):
                return True

        return False

    def _wait_for_more_exploration_room(
        self,
        final_x,
        final_y
    ):

        deadline = (
            time.monotonic()
            + EXPLORATION_MAP_WAIT
        )

        while (
            rclpy.ok()
            and time.monotonic()
            < deadline
        ):

            rclpy.spin_once(
                self,
                timeout_sec=0.10
            )

            bounds = (
                self._navigation_bounds()
            )

            pose = (
                self._current_pose()
            )

            if (
                bounds is None
                or pose is None
            ):
                continue

            if self._point_in_bounds(
                final_x,
                final_y,
                bounds
            ):
                return True

            _, _, stage_distance = (
                self._frontier_stage_goal(
                    pose[0],
                    pose[1],
                    final_x,
                    final_y,
                    bounds
                )
            )

            if (
                stage_distance
                >= EXPLORATION_MIN_STAGE
            ):
                return True

        return False

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

    def cancel_active_goal(
        self,
        result_future=None
    ):
        """Cancel active NavigateToPose and wait for it to settle."""

        if self.active_goal_handle is None:
            return True

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

        except Exception as exc:

            print(
                '[unstuck] navigation cancel error: %s'
                % exc
            )

            return False

        if (
            result_future is not None
            and not result_future.done()
        ):

            rclpy.spin_until_future_complete(
                self,
                result_future,
                timeout_sec=4.0
            )

        settled = (
            result_future is None
            or result_future.done()
        )

        if settled:

            self.active_goal_handle = None

        else:

            print(
                '[unstuck] WARNING: navigation action '
                'did not settle after cancellation. '
                'Backup will NOT be commanded.'
            )

        return settled

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

        # --------------------------------------------------
        # Proactive tricycle turnaround.
        #
        # If the new goal lies almost directly behind the
        # trike, create maneuvering room BEFORE asking the
        # forward-curvature planner to solve the new leg.
        # --------------------------------------------------

        self._maybe_turnaround_assist(
            destination_name,
            x,
            y
        )

        navigation_deadline = (
            time.monotonic()
            + self.leg_timeout
        )

        early_unstuck_count = 0
        assist_budget_logged = False

        # This loop permits the exact SAME mission goal to be
        # safely re-issued after an early BackUp maneuver.
        while rclpy.ok():

            if (
                time.monotonic()
                > navigation_deadline
            ):

                print(
                    'TIMEOUT: %s'
                    % destination_name
                )

                return False

            self.current_recoveries = 0
            self.last_feedback_print = 0.0

            goal = NavigateToPose.Goal()

            goal.pose.header.frame_id = (
                self.frame
            )

            # Use latest available TF.
            goal.pose.header.stamp = (
                rclpy.time.Time().to_msg()
            )

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

            anchor_pose = (
                self._current_pose()
            )

            anchor_time = (
                time.monotonic()
            )

            restart_same_goal = False

            while (
                rclpy.ok()
                and not result_future.done()
            ):

                rclpy.spin_once(
                    self,
                    timeout_sec=0.10
                )

                now = time.monotonic()

                # ------------------------------------------
                # Normal mission-leg timeout.
                # ------------------------------------------

                if (
                    now
                    > navigation_deadline
                ):

                    print(
                        'TIMEOUT: %s'
                        % destination_name
                    )

                    self.cancel_active_goal(
                        result_future
                    )

                    return False

                # ------------------------------------------
                # EARLY STUCK DETECTOR
                #
                # Nav2's existing progress checker / BT stays
                # untouched. This intervenes sooner only if
                # the physical trike has barely moved.
                # ------------------------------------------

                if (
                    self.early_unstuck_enabled
                    and early_unstuck_count
                    < self.early_unstuck_max_per_goal
                ):

                    pose = (
                        self._current_pose()
                    )

                    if pose is not None:

                        if anchor_pose is None:

                            anchor_pose = pose
                            anchor_time = now

                        moved = math.hypot(
                            pose[0] - anchor_pose[0],
                            pose[1] - anchor_pose[1],
                        )

                        remaining_direct = math.hypot(
                            x - pose[0],
                            y - pose[1],
                        )

                        # Any meaningful movement restarts
                        # the stagnation timer.
                        if (
                            moved
                            >= self.early_unstuck_min_movement
                        ):

                            anchor_pose = pose
                            anchor_time = now

                        elif (
                            remaining_direct
                            > self.early_unstuck_min_goal_distance
                            and (
                                now - anchor_time
                                >= self.early_unstuck_window
                            )
                        ):

                            early_unstuck_count += 1

                            print()
                            print(
                                '[unstuck] EARLY STUCK on %s: '
                                'moved %.2fm in %.1fs, '
                                'goal is %.2fm away.'
                                % (
                                    destination_name,
                                    moved,
                                    now - anchor_time,
                                    remaining_direct,
                                )
                            )

                            print(
                                '[unstuck] assist %d/%d: '
                                'cancel Nav2 -> BackUp -> '
                                'resend SAME goal'
                                % (
                                    early_unstuck_count,
                                    self.early_unstuck_max_per_goal,
                                )
                            )

                            settled = (
                                self.cancel_active_goal(
                                    result_future
                                )
                            )

                            if not settled:

                                print(
                                    '[unstuck] navigation '
                                    'cancellation did not '
                                    'settle; refusing to '
                                    'start BackUp.'
                                )

                                return False

                            self._safe_backup(
                                'early stuck while navigating '
                                'to %s'
                                % destination_name
                            )

                            # Small settling period while still
                            # spinning ROS callbacks.
                            settle_deadline = (
                                time.monotonic()
                                + self.unstuck_cooldown
                            )

                            while (
                                rclpy.ok()
                                and time.monotonic()
                                < settle_deadline
                            ):

                                rclpy.spin_once(
                                    self,
                                    timeout_sec=0.10
                                )

                            print(
                                '[unstuck] re-sending '
                                'original %s goal.'
                                % destination_name
                            )

                            restart_same_goal = True
                            break

                elif (
                    self.early_unstuck_enabled
                    and not assist_budget_logged
                    and self.early_unstuck_max_per_goal > 0
                ):

                    print(
                        '[unstuck] early-assist budget '
                        'exhausted for %s; leaving any '
                        'further recovery to the normal '
                        'Nav2 BT.'
                        % destination_name
                    )

                    assist_budget_logged = True

            if restart_same_goal:
                continue

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
                'status=%d recoveries=%d '
                'early_assists=%d'
                % (
                    destination_name,
                    status,
                    self.current_recoveries,
                    early_unstuck_count,
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
    # Goal-directed SLAM exploration for one mission leg
    # ======================================================

    def navigate_with_exploration(
        self,
        destination_name,
        waypoint
    ):
        """Navigate one mission leg, extending SLAM toward it if necessary.

        The destination never changes. Intermediate goals are only temporary
        staging points selected along the direction of the original waypoint.
        """

        final_x, final_y, final_yaw = waypoint

        if not self._wait_for_navigation_context(
            timeout=4.0
        ):

            print(
                'EXPLORATION CONTEXT unavailable; '
                'falling back to direct Nav2 goal.'
            )

            return self.navigate(
                destination_name,
                waypoint
            )

        initial_pose = (
            self._current_pose()
        )

        if initial_pose is None:

            print(
                'FAIL: no map-frame pose available '
                'for exploration'
            )

            return False

        print(
            '[explore] %s final goal=(%.3f, %.3f), '
            'distance=%.2fm'
            % (
                destination_name,
                final_x,
                final_y,
                math.hypot(
                    final_x - initial_pose[0],
                    final_y - initial_pose[1],
                ),
            )
        )

        completed_stages = 0

        for stage_number in range(
            1,
            EXPLORATION_MAX_STAGES + 1
        ):

            pose = self._current_pose()
            bounds = self._navigation_bounds()

            if (
                pose is None
                or bounds is None
            ):

                print(
                    'FAIL: exploration lost pose '
                    'or map bounds'
                )

                return False

            remaining_before = math.hypot(
                final_x - pose[0],
                final_y - pose[1],
            )

            # The real destination has entered the current map.
            if self._point_in_bounds(
                final_x,
                final_y,
                bounds
            ):

                print(
                    '[explore] %s final goal is now '
                    'inside the live map after %d stage(s).'
                    % (
                        destination_name,
                        completed_stages,
                    )
                )

                return self.navigate(
                    destination_name,
                    waypoint
                )

            (
                stage_x,
                stage_y,
                stage_distance,
            ) = self._frontier_stage_goal(
                pose[0],
                pose[1],
                final_x,
                final_y,
                bounds
            )

            if (
                stage_distance
                < EXPLORATION_MIN_STAGE
            ):

                print(
                    '[explore] %s reached the current '
                    'frontier; waiting for SLAM expansion...'
                    % destination_name
                )

                if self._wait_for_more_exploration_room(
                    final_x,
                    final_y
                ):
                    continue

                print(
                    'FAIL: %s exploration stopped safely: '
                    'SLAM did not expose more usable map '
                    'within %.1fs.'
                    % (
                        destination_name,
                        EXPLORATION_MAP_WAIT,
                    )
                )

                return False

            stage_yaw = math.atan2(
                final_y - stage_y,
                final_x - stage_x
            )

            stage_name = (
                '%s_EXPLORE_%d'
                % (
                    destination_name,
                    stage_number,
                )
            )

            print(
                '[explore] stage %d for %s: '
                'current=(%.2f, %.2f) '
                'stage=(%.2f, %.2f) '
                'stage_distance=%.2fm '
                'final_remaining=%.2fm'
                % (
                    stage_number,
                    destination_name,
                    pose[0],
                    pose[1],
                    stage_x,
                    stage_y,
                    stage_distance,
                    remaining_before,
                )
            )

            if not self.navigate(
                stage_name,
                (
                    stage_x,
                    stage_y,
                    stage_yaw,
                )
            ):

                print(
                    'FAIL: exploration stage %d '
                    'toward %s failed'
                    % (
                        stage_number,
                        destination_name,
                    )
                )

                return False

            completed_stages += 1

            after = self._current_pose()

            if after is None:

                print(
                    'FAIL: pose unavailable after '
                    'exploration stage %d'
                    % stage_number
                )

                return False

            remaining_after = math.hypot(
                final_x - after[0],
                final_y - after[1],
            )

            progress = (
                remaining_before
                - remaining_after
            )

            print(
                '[explore] stage %d complete: '
                'reached=(%.2f, %.2f), '
                'progress=%.2fm, '
                'remaining=%.2fm'
                % (
                    stage_number,
                    after[0],
                    after[1],
                    progress,
                    remaining_after,
                )
            )

            if (
                progress
                < EXPLORATION_MIN_PROGRESS
            ):

                print(
                    'FAIL: exploration stage %d made '
                    'only %.2fm progress toward %s; '
                    'refusing to loop.'
                    % (
                        stage_number,
                        progress,
                        destination_name,
                    )
                )

                return False

            settle_deadline = (
                time.monotonic()
                + EXPLORATION_SETTLE
            )

            while (
                rclpy.ok()
                and time.monotonic()
                < settle_deadline
            ):

                rclpy.spin_once(
                    self,
                    timeout_sec=0.10
                )

        print(
            'FAIL: %s exploration exceeded safety '
            'limit of %d stages'
            % (
                destination_name,
                EXPLORATION_MAX_STAGES,
            )
        )

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

            if self.navigate_with_exploration(
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

        route_names = (
            [
                self.home_name,
                self.pickup_name,
            ]
            + list(
                self.delivery_names
            )
            + [
                self.home_name,
            ]
        )

        print(
            'ROUTE: %s'
            % ' -> '.join(
                route_names
            )
        )

        print()

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
            'PICKUP     %s '
            '(%.3f, %.3f, %.2f deg)'
            % (
                self.pickup_name,
                self.pickup[0],
                self.pickup[1],
                math.degrees(
                    self.pickup[2]
                ),
            )
        )

        for index, stop in enumerate(
            self.delivery_stops,
            start=1
        ):

            pose = stop[
                'pose'
            ]

            print(
                'DELIVERY %d %s '
                '(%.3f, %.3f, %.2f deg)'
                % (
                    index,
                    stop['name'],
                    pose[0],
                    pose[1],
                    math.degrees(
                        pose[2]
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
            self.pickup_name,
            self.pickup
        ):

            self.set_state(
                'MISSION_ABORTED'
            )

            print(
                'ABORT: failed to reach %s'
                % self.pickup_name
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
        # ORDERED DELIVERY STOPS
        # ==================================================

        total_stops = len(
            self.delivery_stops
        )

        for index, stop in enumerate(
            self.delivery_stops,
            start=1
        ):

            destination_name = stop[
                'name'
            ]

            waypoint = stop[
                'pose'
            ]

            if total_stops == 1:

                navigating_state = (
                    'NAVIGATING_TO_DELIVERY'
                )

                arrived_state = (
                    'AT_DELIVERY'
                )

                unloading_state = (
                    'UNLOADING'
                )

                delivered_state = (
                    'PARCEL_DELIVERED'
                )

            else:

                navigating_state = (
                    'NAVIGATING_TO_DELIVERY_%d_OF_%d'
                    % (
                        index,
                        total_stops,
                    )
                )

                arrived_state = (
                    'AT_DELIVERY_%d_OF_%d'
                    % (
                        index,
                        total_stops,
                    )
                )

                unloading_state = (
                    'UNLOADING_%d_OF_%d'
                    % (
                        index,
                        total_stops,
                    )
                )

                delivered_state = (
                    'DELIVERY_%d_OF_%d_COMPLETE'
                    % (
                        index,
                        total_stops,
                    )
                )

            self.set_state(
                navigating_state
            )

            if not self.navigate_with_retry(
                destination_name,
                waypoint
            ):

                self.set_state(
                    'MISSION_ABORTED'
                )

                print(
                    'ABORT: failed to reach delivery '
                    'stop %d/%d: %s'
                    % (
                        index,
                        total_stops,
                        destination_name,
                    )
                )

                return False

            self.set_state(
                arrived_state
            )

            self.set_state(
                unloading_state
            )

            self.dwell(
                'Unloading at %s'
                % destination_name,
                self.delivery_dwell
            )

            self.set_state(
                delivered_state
            )

        if total_stops > 1:

            self.set_state(
                'ALL_DELIVERIES_COMPLETE'
            )

        # ==================================================
        # LAST DELIVERY -> HOME
        # ==================================================

        self.set_state(
            'RETURNING_HOME'
        )

        if not self.navigate_with_retry(
            self.home_name,
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

    mission_id = new_mission_id()

    started_epoch = time.time()
    started_at = utc_now_iso()

    home_name = str(
        node.get_parameter(
            'home_name'
        ).value
    )

    pickup_name = str(
        node.get_parameter(
            'pickup_name'
        ).value
    )

    delivery_name = str(
        node.get_parameter(
            'delivery_name'
        ).value
    )

    final_status = 'MISSION_ABORTED'
    error_text = None

    print(
        'MISSION ID: %s'
        % mission_id
    )

    try:

        success = node.run_mission()

        if success:

            final_status = (
                'MISSION_COMPLETE'
            )

        else:

            final_status = (
                'MISSION_ABORTED'
            )

    except KeyboardInterrupt:

        print()
        print(
            'Operator interrupted mission.'
        )

        node.cancel_active_goal()

        node.set_state(
            'MISSION_INTERRUPTED'
        )

        final_status = (
            'MISSION_INTERRUPTED'
        )

    except Exception as exc:

        node.cancel_active_goal()

        node.set_state(
            'MISSION_ABORTED'
        )

        final_status = (
            'MISSION_ABORTED'
        )

        error_text = (
            '%s: %s'
            % (
                type(exc).__name__,
                exc,
            )
        )

        node.get_logger().error(
            'Mission exception: %s'
            % exc
        )

    finally:

        node.cancel_active_goal()

        finished_epoch = time.time()
        finished_at = utc_now_iso()

        delivery_names = list(
            node.delivery_names
        )

        record = {
            'mission_id': mission_id,
            'pickup': node.pickup_name,
            'delivery': (
                delivery_names[0]
                if len(delivery_names) == 1
                else None
            ),
            'deliveries': delivery_names,
            'route': (
                [
                    node.home_name,
                    node.pickup_name,
                ]
                + delivery_names
                + [
                    node.home_name,
                ]
            ),
            'status': final_status,
            'started_at': started_at,
            'finished_at': finished_at,
            'duration_sec': round(
                max(
                    0.0,
                    finished_epoch
                    - started_epoch
                ),
                3
            ),
        }

        if error_text is not None:

            record['error'] = (
                error_text
            )

        try:

            append_mission_record(
                record
            )

            print(
                'MISSION HISTORY SAVED: %s'
                % mission_id
            )

        except Exception as history_exc:

            node.get_logger().error(
                'Failed to save mission history: '
                '%s: %s'
                % (
                    type(
                        history_exc
                    ).__name__,
                    history_exc,
                )
            )

        node.destroy_node()

        rclpy.shutdown()

    if not success:

        raise SystemExit(1)

if __name__ == '__main__':
    main()
