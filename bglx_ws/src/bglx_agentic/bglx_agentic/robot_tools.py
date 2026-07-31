"""Tool layer for the BGLX agentic stack.

Confirmed against the running system:
    scan        /etrike/scan
    odometry    /tricycle_steering_controller/odometry
    costmap     /global_costmap/costmap
    plan        /plan
    cmd_vel     /etrike/cmd_vel      (consumed by cmd_vel_limiter)
    action      /navigate_to_pose
    base frame  base_footprint
    global      map

The agent sees only the public methods, and they return English.
"""

import math
import os
import threading
import time

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)

from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose

import tf2_ros
from tf2_ros import TransformException

from .observations import (summarise_scan, format_scan, diagnose_nav_failure,
                           MIN_TURN_RADIUS, MAX_LINEAR_VEL,
                           MIN_SPEED_FOR_YAW, max_yaw_rate)

SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=1)

MAP_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)

BASE_FRAME = 'base_footprint'
GLOBAL_FRAME = 'map'
ODOM_FRAME = 'odom'
FOOTPRINT_FORWARD_REACH = 1.25
NAV_TIMEOUT = 180.0


def rpy_from_quat(q):
    """Full roll/pitch/yaw. Yaw alone cannot tell you the robot has fallen."""
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return roll, pitch, math.atan2(siny, cosy)


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_from_yaw(yaw):
    return Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


class RobotTools(Node):

    def __init__(self):
        super().__init__('bglx_agentic_tools')

        self.declare_parameter('scan_topic', '/etrike/scan')
        self.declare_parameter('odom_topic',
                               '/tricycle_steering_controller/odometry')
        self.declare_parameter('costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('cmd_vel_topic', '/etrike/cmd_vel')
        self.declare_parameter('landmarks_file',
                               os.path.expanduser('~/.bglx_landmarks.yaml'))

        scan_topic = self.get_parameter('scan_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        costmap_topic = self.get_parameter('costmap_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.landmarks_path = self.get_parameter('landmarks_file').value

        self.cb_group = ReentrantCallbackGroup()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._scan = None
        self._odom = None
        self._bounds = None
        self._plan_len = 0
        self._counts = {'scan': 0, 'odom': 0, 'costmap': 0}
        self._lock = threading.Lock()
        self._last_failure = "No navigation attempted yet."

        self.create_subscription(LaserScan, scan_topic, self._on_scan,
                                 SENSOR_QOS, callback_group=self.cb_group)
        self.create_subscription(Odometry, odom_topic, self._on_odom,
                                 SENSOR_QOS, callback_group=self.cb_group)
        self.create_subscription(OccupancyGrid, costmap_topic,
                                 self._on_costmap, MAP_QOS,
                                 callback_group=self.cb_group)
        self.create_subscription(Path, '/plan', self._on_plan, 1,
                                 callback_group=self.cb_group)

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.nav_client = ActionClient(self, NavigateToPose,
                                       'navigate_to_pose',
                                       callback_group=self.cb_group)

        self.landmarks = self._load_landmarks()
        self.get_logger().info(
            "tools ready: scan=%s odom=%s cmd_vel=%s"
            % (scan_topic, odom_topic, cmd_vel_topic))

    # --- callbacks ---------------------------------------------------------
    def _on_scan(self, msg):
        with self._lock:
            self._scan = msg
            self._counts['scan'] += 1

    def _on_odom(self, msg):
        with self._lock:
            self._odom = msg
            self._counts['odom'] += 1

    def _on_plan(self, msg):
        with self._lock:
            self._plan_len = len(msg.poses)

    def _on_costmap(self, msg):
        i = msg.info
        with self._lock:
            self._bounds = (i.origin.position.x, i.origin.position.y,
                            i.origin.position.x + i.width * i.resolution,
                            i.origin.position.y + i.height * i.resolution)
            self._counts['costmap'] += 1

    # --- pose --------------------------------------------------------------
    def _lookup(self, ref):
        try:
            tf = self.tf_buffer.lookup_transform(
                ref, BASE_FRAME, rclpy.time.Time(),
                timeout=Duration(seconds=0.3))
        except TransformException:
            return None
        t = tf.transform.translation
        return t.x, t.y, yaw_from_quat(tf.transform.rotation)

    def _pose(self):
        for ref in (GLOBAL_FRAME, ODOM_FRAME):
            got = self._lookup(ref)
            if got:
                return got + (ref,)
        return None

    def _speed(self):
        with self._lock:
            od = self._odom
        return od.twist.twist.linear.x if od else 0.0

    # --- map bounds --------------------------------------------------------
    def check_map_bounds(self):
        pose = self._pose()
        with self._lock:
            bounds = self._bounds
        if pose is None or bounds is None:
            return None, "Map bounds unknown (no costmap or no pose yet)."
        x, y, _, frame = pose
        if frame != GLOBAL_FRAME:
            return None, ("Pose is in 'odom'; cannot compare against the map "
                          "costmap. SLAM may not have converged.")
        min_x, min_y, max_x, max_y = bounds
        if not (min_x <= x <= max_x and min_y <= y <= max_y):
            return False, (
                "OUTSIDE THE MAP: robot at (%.2f, %.2f) is beyond the costmap, "
                "which spans (%.2f, %.2f) to (%.2f, %.2f). No path can be "
                "planned from here to ANY goal. Reverse back into mapped "
                "space first." % (x, y, min_x, min_y, max_x, max_y))
        margin = min(x - min_x, max_x - x, y - min_y, max_y - y)
        if margin < FOOTPRINT_FORWARD_REACH:
            return True, (
                "NEAR THE MAP EDGE: only %.2fm of margin, and the footprint "
                "reaches %.2fm ahead of the robot's origin. Planning may fail. "
                "Head back toward the middle of the mapped area."
                % (margin, FOOTPRINT_FORWARD_REACH))
        return True, ("Inside the mapped area, %.2fm from the nearest edge."
                      % margin)

    # --- landmarks ---------------------------------------------------------
    def _load_landmarks(self):
        if os.path.exists(self.landmarks_path):
            with open(self.landmarks_path) as f:
                return yaml.safe_load(f) or {}
        return {}

    def save_landmark(self, name):
        pose = self._pose()
        if pose is None:
            return "Cannot save landmark: robot pose unavailable."
        x, y, yaw, frame = pose
        if frame != GLOBAL_FRAME:
            return ("Refusing to save: pose is in '%s', not 'map'. A landmark "
                    "in odom will not survive the session." % frame)
        self.landmarks[name] = {'x': round(x, 3), 'y': round(y, 3),
                                'yaw': round(yaw, 3)}
        with open(self.landmarks_path, 'w') as f:
            yaml.safe_dump(self.landmarks, f)
        return "Saved landmark '%s' at (%.2f, %.2f)." % (name, x, y)

    def list_landmarks(self):
        if not self.landmarks:
            return ("No landmarks recorded yet. Use record_landmark to tag "
                    "locations before referring to them by name.")
        rows = ["- %s: (%.2f, %.2f)" % (k, v['x'], v['y'])
                for k, v in self.landmarks.items()]
        return "Known landmarks:\n" + "\n".join(rows)

    # --- sensing tools -----------------------------------------------------
    def check_attitude(self):
        """Is the vehicle upright?

        A tipped robot still publishes a pose, still accepts goals, and still
        reports plausible-looking failures. Without this the agent will keep
        commanding a vehicle lying on its side.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                GLOBAL_FRAME, 'base_link', rclpy.time.Time(),
                timeout=Duration(seconds=0.3))
        except TransformException:
            return None, "Attitude unknown (no transform to base_link)."
        roll, pitch, _ = rpy_from_quat(tf.transform.rotation)
        r, p = math.degrees(roll), math.degrees(pitch)

        if abs(r) > 45.0 or abs(p) > 45.0:
            return False, ("VEHICLE HAS TIPPED OVER: roll %.0f deg, pitch "
                           "%.0f deg. It is lying on its side and cannot "
                           "drive. NOTHING will work until it is righted. "
                           "Stop issuing movement commands and report this "
                           "to the operator." % (r, p))
        if abs(r) > 15.0 or abs(p) > 15.0:
            return True, ("UNSTABLE ATTITUDE: roll %.0f deg, pitch %.0f deg. "
                          "The vehicle is leaning heavily and close to "
                          "tipping. Slow down and avoid further turning until "
                          "it settles." % (r, p))
        return True, "Upright (roll %.0f deg, pitch %.0f deg)." % (r, p)

    def get_pose(self):
        pose = self._pose()
        if pose is None:
            return ("Pose unavailable: no transform to base_footprint from "
                    "'map' or 'odom'. SLAM or the controller is not running.")
        x, y, yaw, frame = pose
        msg = ("Position (%.2f, %.2f) in frame '%s', heading %.1f deg, "
               "speed %.2f m/s." % (x, y, frame, math.degrees(yaw),
                                    self._speed()))
        upright, att = self.check_attitude()
        if upright is not True:
            msg += " " + att
        ok, note = self.check_map_bounds()
        if ok is not True:
            msg += " " + note
        return msg

    def get_scan_summary(self):
        with self._lock:
            scan = self._scan
        return format_scan(summarise_scan(scan))

    def get_last_failure(self):
        return self._last_failure

    # --- navigation --------------------------------------------------------
    def navigate_to(self, x, y, yaw=None):
        """Send a Nav2 goal, then report in words what happened."""
        upright, att = self.check_attitude()
        if upright is False:
            self._last_failure = att
            return "navigate_to refused. " + att

        inside, bounds_note = self.check_map_bounds()
        if inside is False:
            self._last_failure = bounds_note
            return "navigate_to refused. " + bounds_note

        with self._lock:
            bounds = self._bounds
        if bounds:
            bx0, by0, bx1, by1 = bounds
            if not (bx0 <= float(x) <= bx1 and by0 <= float(y) <= by1):
                msg = ("navigate_to refused: goal (%.2f, %.2f) lies OUTSIDE "
                       "the mapped area, which spans (%.2f, %.2f) to "
                       "(%.2f, %.2f). No path can be planned to a goal the "
                       "costmap does not cover. Pick a goal inside the map, "
                       "or explore toward it in stages."
                       % (float(x), float(y), bx0, by0, bx1, by1))
                self._last_failure = msg
                return msg

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            return ("navigate_to failed: the navigate_to_pose action server is "
                    "not available. Nav2 is not running or not activated.")

        start = self._pose()
        if start is None:
            return "navigate_to failed: robot pose unavailable."

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = GLOBAL_FRAME
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation = quat_from_yaw(float(yaw or 0.0))

        with self._lock:
            self._plan_len = 0

        send = self.nav_client.send_goal_async(goal)
        t_send = time.time()
        while not send.done() and time.time() - t_send < 10.0:
            time.sleep(0.05)
        handle = send.result()
        if handle is None or not handle.accepted:
            self._last_failure = "Nav2 rejected the goal outright."
            return "navigate_to failed: " + self._last_failure

        t0 = time.time()
        travelled = 0.0
        last = (start[0], start[1])
        stalled_since = None
        stalled_max = 0.0
        yaw_at_low_speed = False
        left_map = False
        result_future = handle.get_result_async()

        while not result_future.done():
            time.sleep(0.2)
            cur = self._pose()
            if cur:
                d = math.hypot(cur[0] - last[0], cur[1] - last[1])
                travelled += d
                last = (cur[0], cur[1])
                if d < 0.005:
                    stalled_since = stalled_since or time.time()
                    stalled_max = max(stalled_max, time.time() - stalled_since)
                else:
                    stalled_since = None
            with self._lock:
                od = self._odom
            if od is not None:
                if (abs(od.twist.twist.linear.x) < MIN_SPEED_FOR_YAW
                        and abs(od.twist.twist.angular.z) > 0.05):
                    yaw_at_low_speed = True
            if self.check_map_bounds()[0] is False:
                left_map = True
            if time.time() - t0 > NAV_TIMEOUT:
                handle.cancel_goal_async()
                break

        elapsed = time.time() - t0
        res = result_future.result()
        status = {4: 'SUCCEEDED', 5: 'CANCELED',
                  6: 'ABORTED'}.get(getattr(res, 'status', 0), 'UNKNOWN')

        end = self._pose() or start
        remaining = math.hypot(x - end[0], y - end[1])

        if status == 'SUCCEEDED' and remaining < 0.75:
            self._last_failure = "Last navigation succeeded."
            return ("Arrived at (%.2f, %.2f), %.2fm from the requested goal, "
                    "in %.1fs. %s"
                    % (end[0], end[1], remaining, elapsed,
                       self.get_scan_summary()))

        with self._lock:
            scan_sum = summarise_scan(self._scan)
            had_plan = self._plan_len > 0

        msg = diagnose_nav_failure(
            status=status, dist_remaining=remaining, dist_travelled=travelled,
            stalled_seconds=stalled_max, elapsed=elapsed,
            scan_summary=scan_sum, had_global_plan=had_plan, goal_xy=(x, y),
            commanded_yaw_at_low_speed=yaw_at_low_speed,
            left_map=left_map)
        self._last_failure = msg
        return msg

    def navigate_relative(self, forward=0.0, left=0.0):
        """Move relative to the robot's CURRENT pose and heading.

        Exists because relative instructions ("go 4m ahead", "back up 2m")
        require trigonometry against the live heading, and a small model gets
        that wrong in a way that looks like success: it invents an absolute
        coordinate, reaches it, and reports the task done.
        """
        pose = self._pose()
        if pose is None:
            return "navigate_relative failed: robot pose unavailable."
        x, y, yaw, frame = pose
        if frame != GLOBAL_FRAME:
            return ("navigate_relative refused: pose is in '%s', not 'map'. "
                    "SLAM has not converged." % frame)

        forward = float(forward)
        left = float(left)
        gx = x + forward * math.cos(yaw) - left * math.sin(yaw)
        gy = y + forward * math.sin(yaw) + left * math.cos(yaw)

        note = ("Relative move: %.2fm forward, %.2fm left from (%.2f, %.2f) "
                "at %.1f deg -> absolute goal (%.2f, %.2f). "
                % (forward, left, x, y, math.degrees(yaw), gx, gy))
        return note + self.navigate_to(gx, gy, yaw)

    def turn_by(self, degrees):
        """Change heading by driving the arc directly, measuring after each try.

        Deliberately does NOT route through Nav2. Nav2's xy_goal_tolerance is
        0.30m and yaw_goal_tolerance 0.60 rad (34 deg), so a small commanded
        turn produces a goal inside BOTH tolerances: it reports SUCCEEDED
        without moving. A 7 deg arc at minimum radius puts the goal 0.15m
        away with a 7 deg heading change - invisible to the goal checker.

        Ackermann kinematics give the answer exactly: omega = v tan(d) / L, so
        a heading change of theta takes theta/omega seconds. Measure, compute
        the shortfall, repeat.

        No obstacle avoidance during the arc, so clearance is checked first.
        """
        target = float(degrees)
        if abs(target) < 2.0:
            return "Turn of %.0f deg is too small to be meaningful." % target

        upright, att = self.check_attitude()
        if upright is False:
            return "turn_by refused. " + att

        start = self._pose()
        if start is None:
            return "turn_by failed: robot pose unavailable."
        yaw0 = start[2]

        SPEED = 0.6
        TOL = 3.0
        omega_max = max_yaw_rate(SPEED)

        # Which way to drive? Forward if there is room, else reverse.
        summary = summarise_scan(self._scan)
        need = 1.5
        direction = 1.0
        if summary:
            front = min(summary.get("front", 99), summary.get("front-left", 99),
                        summary.get("front-right", 99))
            rear = min(summary.get("rear", 99), summary.get("rear-left", 99),
                       summary.get("rear-right", 99))
            if front < need and rear >= need:
                direction = -1.0
            elif front < need and rear < need:
                return ("turn_by refused: only %.1fm ahead and %.1fm behind. "
                        "This platform cannot pivot, so it needs about %.1fm "
                        "of clear space in one direction to swing round. Move "
                        "somewhere more open first." % (front, rear, need))

        achieved = 0.0
        log = []
        for attempt in range(6):
            remaining = target - achieved
            if abs(remaining) < TOL:
                break

            # Reversing mirrors the steering-to-heading relationship.
            sign = math.copysign(1.0, remaining) * direction
            duration = min(5.0, abs(math.radians(remaining)) / omega_max)

            before = self._pose()
            if before is None:
                log.append("lost pose"); break

            msg = Twist()
            msg.linear.x = SPEED * direction
            msg.angular.z = omega_max * sign * direction
            t0 = time.time()
            while time.time() - t0 < duration:
                self.cmd_pub.publish(msg)
                time.sleep(0.05)
            self.cmd_pub.publish(Twist())
            time.sleep(0.4)          # let it settle before measuring

            after = self._pose()
            if after is None:
                log.append("lost pose after arc"); break
            got = math.degrees((after[2] - before[2] + math.pi)
                               % (2 * math.pi) - math.pi)
            achieved += got
            log.append("arc %d: %.1fs %s, got %.0f (cumulative %.0f)"
                       % (attempt + 1, duration,
                          "forward" if direction > 0 else "reverse",
                          got, achieved))

            up, att2 = self.check_attitude()
            if up is False:
                return "Turn ABORTED. " + att2 + " " + " | ".join(log)

        err = target - achieved
        end = self._pose()
        moved = (math.hypot(end[0] - start[0], end[1] - start[1])
                 if end else 0.0)

        if abs(achieved) < 2.0:
            head = ("Turn FAILED: requested %.0f deg, the robot did not rotate "
                    "at all. Check check_systems before retrying; a smaller "
                    "angle will not help. " % target)
        elif abs(err) > 10.0:
            head = ("Turn INCOMPLETE: requested %.0f deg, achieved %.0f, still "
                    "%.0f short. Do NOT report this as done. " % (target, achieved, err))
        else:
            head = ("Turn complete. Requested %.0f deg, achieved %.0f (error "
                    "%.0f), travelled %.2fm. " % (target, achieved, err, moved))
        return head + " | ".join(log) + " " + self.get_scan_summary()

    def mark_here(self, name):
        """Record the current pose under a name so it can be returned to.

        Without this, 'come back to where you started' has nothing to bind
        to and the model invents a distance.
        """
        return self.save_landmark(name)

    def navigate_to_landmark(self, name):
        lm = self.landmarks.get(name)
        if lm is None:
            return "No landmark named '%s'. %s" % (name, self.list_landmarks())
        return self.navigate_to(lm['x'], lm['y'], lm.get('yaw', 0.0))

    # --- open-loop motion --------------------------------------------------
    def drive(self, linear_x, angular_z, duration):
        """Escape hatch. Goes THROUGH cmd_vel_limiter, not around it."""
        linear_x = max(-MAX_LINEAR_VEL, min(MAX_LINEAR_VEL, float(linear_x)))
        angular_z = float(angular_z)
        duration = max(0.0, min(10.0, float(duration)))

        if abs(linear_x) < MIN_SPEED_FOR_YAW and abs(angular_z) > 0.05:
            return ("REJECTED: rotation commanded at near-zero speed. This "
                    "tricycle has a %.2fm minimum turning radius and cannot "
                    "rotate in place. Command a forward or reverse speed of "
                    "at least 0.3 m/s together with the turn."
                    % MIN_TURN_RADIUS)

        cap = max_yaw_rate(linear_x)
        if abs(angular_z) > cap:
            angular_z = math.copysign(cap, angular_z)

        upright, att = self.check_attitude()
        if upright is False:
            return "drive refused. " + att

        before = self._pose()
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        t0 = time.time()
        while time.time() - t0 < duration:
            self.cmd_pub.publish(msg)
            time.sleep(0.05)
        self.cmd_pub.publish(Twist())

        after = self._pose()
        moved = None
        if before and after:
            moved = math.hypot(after[0] - before[0], after[1] - before[1])

        intended = abs(linear_x) * duration
        out = ("Drove at %.2f m/s for %.1fs. Commanded travel was "
               "%.2fm (speed x time)." % (linear_x, duration, intended))
        if moved is not None:
            out += " ACTUAL DISPLACEMENT: %.2fm." % moved
            if abs(moved - intended) > 0.25:
                out += (" WARNING: actual displacement differs from commanded "
                        "by %.2fm - the robot may have been obstructed or "
                        "slipped." % abs(moved - intended))
        out += (" Note: drive() moves speed x duration metres. To travel a "
                "specific distance, compute duration = distance / speed, and "
                "verify the result before reporting completion.")
        return out + " " + self.get_pose()

    def stop(self):
        self.cmd_pub.publish(Twist())
        return "Stopped."

    def wait(self, seconds):
        seconds = max(0.0, min(30.0, float(seconds)))
        time.sleep(seconds)
        return "Waited %.1fs. %s" % (seconds, self.get_pose())

    # --- health ------------------------------------------------------------
    def health(self):
        with self._lock:
            counts = dict(self._counts)
            scan = self._scan
            bounds = self._bounds
        lines = []
        for key, label in (('scan', 'LaserScan'), ('odom', 'Odometry'),
                           ('costmap', 'Global costmap')):
            n = counts[key]
            lines.append("%s messages: %d%s"
                         % (label, n, "" if n else "   <-- NOTHING ARRIVING"))
        if scan is not None:
            lines.append("Scan: %d rays, frame '%s', %.2f-%.2fm"
                         % (len(scan.ranges), scan.header.frame_id,
                            scan.range_min, scan.range_max))
        if bounds:
            lines.append("Costmap spans (%.2f, %.2f) to (%.2f, %.2f)" % bounds)
        for ref in (GLOBAL_FRAME, ODOM_FRAME):
            lines.append("TF %s -> %s: %s"
                         % (ref, BASE_FRAME,
                            "OK" if self._lookup(ref) else "MISSING"))
        lines.append("Nav2 action server: %s"
                     % ("OK" if self.nav_client.wait_for_server(
                         timeout_sec=2.0) else "NOT AVAILABLE"))
        lines.append(self.check_attitude()[1])
        lines.append(self.check_map_bounds()[1])
        return "\n".join(lines)


def _spin(tools):
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(tools)
    threading.Thread(target=ex.spin, daemon=True).start()


def main(argv=None):
    """Inspector: ros2 run bglx_agentic inspect"""
    rclpy.init(args=argv)
    tools = RobotTools()
    _spin(tools)
    time.sleep(2.0)
    print("\n=== HEALTH ===")
    print(tools.health())
    print("\n=== LIVE (Ctrl-C to stop) ===")
    try:
        while True:
            print("\n" + tools.get_pose())
            print(tools.get_scan_summary())
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


def record_landmark(argv=None):
    """ros2 run bglx_agentic record_landmark"""
    rclpy.init(args=argv)
    tools = RobotTools()
    _spin(tools)
    time.sleep(2.0)
    print("Drive the robot, then type a name to tag the spot.")
    print("Blank = show pose, 'list' = show landmarks, 'quit' = exit.\n")
    try:
        while True:
            name = input("landmark> ").strip()
            if not name:
                print(tools.get_pose())
            elif name in ("quit", "exit"):
                break
            elif name == "list":
                print(tools.list_landmarks())
            else:
                print(tools.save_landmark(name))
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()