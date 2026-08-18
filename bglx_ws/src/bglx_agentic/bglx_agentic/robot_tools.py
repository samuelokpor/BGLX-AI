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
import signal
import subprocess
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
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose

import tf2_ros
from tf2_ros import TransformException

from .vision import VisionTool
from .map_check import compare as compare_map, confidence_note
from .corridor import check_corridor, can_turn_around
from .mission_waypoints import (
    LOCATIONS,
    resolve_location_name,
    get_location_info,
    format_location_registry,
    save_custom_location,
    update_custom_location,
    delete_custom_location,
)
from .mission_history import format_mission_history
from .observations import (summarise_scan, format_scan, SECTORS, diagnose_nav_failure,
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
MISSION_HOME_START_TOLERANCE = 1.0
MISSION_TOOL_TIMEOUT = 900.0
MISSION_LOG_PATH = os.path.expanduser('~/.bglx/mission_latest.log')
LOCAL_COSTMAP_LETHAL = 90      # occupancy 0..100; >= this counts as an obstacle
COSTMAP_SENSE_MAX_RANGE = 6.0  # m: how far to ray-march the costmap per sector
REVERSE_STRAIGHT_MAX = 5.0      # m: reverse <= this goes straight-back, no reorientation
REVERSE_STRAIGHT_TOL = 0.30     # m: |left| below this counts as a pure straight reverse
REVERSE_SPEED = 0.4             # m/s: closed-loop straight-reverse speed
REVERSE_STOP_DIST = 1.6         # m: rear clearance floor (matches drive())


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
        self.declare_parameter('front_scan_topic', '/etrike/front_scan')
        self.declare_parameter('odom_topic',
                               '/tricycle_steering_controller/odometry')
        self.declare_parameter('costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('cmd_vel_topic', '/etrike/cmd_vel')
        self.declare_parameter('camera_topic', '/etrike/front/image_raw')
        self.declare_parameter('landmarks_file',
                               os.path.expanduser('~/.bglx_landmarks.yaml'))

        scan_topic = self.get_parameter('scan_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        costmap_topic = self.get_parameter('costmap_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        camera_topic = self.get_parameter('camera_topic').value
        self.landmarks_path = self.get_parameter('landmarks_file').value

        self.cb_group = ReentrantCallbackGroup()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._scan = None
        self._odom = None
        self._bounds = None
        self._static_grid = None
        self._static_info = None
        self._local_grid = None
        self._local_info = None
        self._cov = None
        self._plan_len = 0
        self._front_scan = None
        self._counts = {'scan': 0, 'front_scan': 0, 'odom': 0, 'costmap': 0}
        self._lock = threading.Lock()
        self._last_failure = "No navigation attempted yet."

        # Asynchronous deterministic delivery mission.
        self._mission_proc = None
        self._mission_state = 'IDLE'
        self._mission_route = None
        self._mission_started_at = None

        self.create_subscription(LaserScan, scan_topic, self._on_scan,
                                 SENSOR_QOS, callback_group=self.cb_group)
        self.create_subscription(
            LaserScan,
            self.get_parameter('front_scan_topic').value,
            self._on_front_scan, SENSOR_QOS, callback_group=self.cb_group)
        self.create_subscription(Odometry, odom_topic, self._on_odom,
                                 SENSOR_QOS, callback_group=self.cb_group)
        self.create_subscription(OccupancyGrid, costmap_topic,
                                 self._on_costmap, MAP_QOS,
                                 callback_group=self.cb_group)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                                 self._on_local_costmap, MAP_QOS,
                                 callback_group=self.cb_group)
        self.create_subscription(Path, '/plan', self._on_plan, 1,
                                 callback_group=self.cb_group)
        self.create_subscription(
            String,
            '/bglx/mission/state',
            self._on_mission_state,
            10,
            callback_group=self.cb_group
        )
        # The saved map, and how confident AMCL is that we are where we think.
        self.create_subscription(OccupancyGrid, '/map', self._on_static_map,
                                 MAP_QOS, callback_group=self.cb_group)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self._on_amcl, 10,
                                 callback_group=self.cb_group)

        self.vision = VisionTool(self.get_logger())
        self.create_subscription(Image, camera_topic, self.vision.on_image,
                                 SENSOR_QOS, callback_group=self.cb_group)

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.nav_client = ActionClient(self, NavigateToPose,
                                       'navigate_to_pose',
                                       callback_group=self.cb_group)

        self.landmarks = self._load_landmarks()
        self.get_logger().info(
            "tools ready: scan=%s odom=%s cmd_vel=%s"
            % (scan_topic, odom_topic, cmd_vel_topic))

    # --- callbacks ---------------------------------------------------------

    def _on_mission_state(self, msg):
        new_state = str(
            msg.data
        )

        with self._lock:
            old_state = self._mission_state
            self._mission_state = new_state

        # Keep the interactive agent terminal informative without
        # restoring the verbose child-process/Nav2 log flood.
        # Only print when the mission state actually changes.
        if (
            new_state
            and new_state != old_state
        ):
            print(
                "\n[mission] %s"
                % new_state,
                flush=True
            )

    def _on_scan(self, msg):
        with self._lock:
            self._scan = msg
            self._counts['scan'] += 1

    def _on_front_scan(self, msg):
        with self._lock:
            self._front_scan = msg
            self._counts['front_scan'] += 1

    def _on_odom(self, msg):
        with self._lock:
            self._odom = msg
            self._counts['odom'] += 1

    def _on_static_map(self, msg):
        with self._lock:
            self._static_grid = list(msg.data)
            self._static_info = msg.info

    def _on_amcl(self, msg):
        with self._lock:
            self._cov = list(msg.pose.covariance)

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

    def _on_local_costmap(self, msg):
        with self._lock:
            self._local_grid = list(msg.data)
            self._local_info = msg.info

    def _costmap_sectors(self, max_range=COSTMAP_SENSE_MAX_RANGE):
        """Ray-march the LOCAL costmap outward per sector -> {sector: nearest obstacle m}.
        Reads the fused costmap (LiDAR + front depth cam + perimeter ray sensors), so
        low obstacles (e.g. benches) the raw 2D LiDAR flies over are included. Returns
        None if costmap or pose is unavailable so the caller falls back to raw scan."""
        with self._lock:
            grid = self._local_grid
            info = self._local_info
        if grid is None or info is None:
            return None
        pose = self._lookup(ODOM_FRAME)   # local costmap is in the odom frame
        if pose is None:
            return None
        rx, ry, ryaw = pose
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        W, H = info.width, info.height
        step = res * 0.5
        n_rays = 5
        out = {}
        for name, lo, hi in SECTORS:
            best = None
            for k in range(n_rays):
                frac = (k + 0.5) / n_rays
                bearing = ryaw + math.radians(lo + frac * (hi - lo))
                d = step
                while d <= max_range:
                    wx = rx + d * math.cos(bearing)
                    wy = ry + d * math.sin(bearing)
                    cx = int((wx - ox) / res)
                    cy = int((wy - oy) / res)
                    if not (0 <= cx < W and 0 <= cy < H):
                        break
                    if grid[cy * W + cx] >= LOCAL_COSTMAP_LETHAL:
                        best = d if best is None else min(best, d)
                        break
                    d += step
            if best is not None:
                out[name] = round(best, 2) if name not in out else min(out[name], round(best, 2))
        return out if out else None

    def get_scan_summary(self):
        """Nearest obstacle per sector. Prefers the fused LOCAL costmap (catches low
        obstacles the 2D LiDAR misses); falls back to raw LiDAR if costmap unavailable."""
        cm = self._costmap_sectors()
        if cm is not None:
            return format_scan(cm)
        with self._lock:
            scan = self._scan
        return format_scan(summarise_scan(scan))

    def check_low_obstacles(self):
        """Low obstacles ahead, from the front LiDAR at ~0.35m.

        The main LiDAR sits at ~0.98m and sees straight over anything shorter
        than that: kerbs, boxes, planters, a crouching child. This sensor is
        the only one that catches them, so a clear answer here is not the same
        as a clear path, and a blocked answer here outranks a clear costmap.
        """
        with self._lock:
            scan = self._front_scan
        if scan is None:
            return ("No front LiDAR data. Low obstacles CANNOT be detected. "
                    "Treat the ground ahead as unverified.")
        hits = []
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
                continue
            a = math.degrees(scan.angle_min + i * scan.angle_increment)
            if -60.0 <= a <= 60.0:
                hits.append((a, r))
        if not hits:
            return ("Front LiDAR clear: nothing within %.1fm in the forward "
                    "120 degree arc, at 0.35m height."
                    % scan.range_max)
        ang, near = min(hits, key=lambda t: t[1])
        side = "ahead" if abs(ang) < 15 else ("left" if ang > 0 else "right")
        lvl = "BLOCKED" if near < 1.5 else ("CLOSE" if near < 3.0 else "clear")
        return ("Front LiDAR %s: nearest low obstacle %.2fm %s (%.0f degrees), "
                "measured at 0.35m height. %d returns in the forward arc."
                % (lvl, near, side, ang, len(hits)))

    def check_map_against_sensors(self):
        """Does the saved map still match what the LiDAR sees?"""
        pose = self._pose()
        if pose is None:
            return "Cannot compare: robot pose unavailable."
        if pose[3] != GLOBAL_FRAME:
            return ("Cannot compare: pose is in '%s', not 'map'. AMCL has not "
                    "localised yet." % pose[3])
        with self._lock:
            scan = self._scan
            grid = self._static_grid
            info = self._static_info
            cov = self._cov
        return compare_map(scan, grid, info, (pose[0], pose[1]), pose[2], cov)

    def localisation_confidence(self):
        with self._lock:
            cov = self._cov
        return confidence_note(cov)[1]

    def check_width(self, direction_deg=0.0):
        """Will the vehicle fit through the gap in that direction?"""
        with self._lock:
            scan = self._scan
        return check_corridor(scan, float(direction_deg))

    def check_turn_around(self):
        with self._lock:
            scan = self._scan
        return can_turn_around(scan)[1]

    def look(self, question=None):
        """Semantic description of what the front camera sees."""
        return self.vision.look(question)

    def get_last_failure(self):
        return self._last_failure

    # --- navigation --------------------------------------------------------
    def _goal_is_blocked(self, x, y):
        """Return a diagnosis if (x, y) is not free space, else None."""
        with self._lock:
            grid = self._static_grid
            info = self._static_info
        if grid is None or info is None:
            return None                     # no map to check against

        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        cx = int((x - ox) / res)
        cy = int((y - oy) / res)
        if not (0 <= cx < info.width and 0 <= cy < info.height):
            return None                     # bounds check already covers this

        v = grid[cy * info.width + cx]
        if v < 0:
            # Unknown space is allowed by the active Smac Hybrid planner.
            # Let Nav2 decide whether a path through unknown space is feasible.
            # Known occupied cells are still rejected below.
            return None
        if v >= 65:
            # How far back along the bearing is open ground?
            import math as _m
            pose = self._pose()
            free_at = None
            if pose is not None:
                dx, dy = x - pose[0], y - pose[1]
                dist = _m.hypot(dx, dy)
                if dist > 0.1:
                    ux, uy = dx / dist, dy / dist
                    step = res * 4
                    d = dist
                    while d > 0.5:
                        tx, ty = pose[0] + ux * d, pose[1] + uy * d
                        tcx = int((tx - ox) / res)
                        tcy = int((ty - oy) / res)
                        if (0 <= tcx < info.width and 0 <= tcy < info.height
                                and 0 <= grid[tcy * info.width + tcx] < 25):
                            free_at = d
                            break
                        d -= step
            note = ""
            if free_at is not None:
                note = (" The furthest open ground on that bearing is about "
                        "%.1fm away." % free_at)
            return ("navigate_to refused: the goal (%.2f, %.2f) is INSIDE an "
                    "obstacle according to the map - a wall or a building. "
                    "Nav2 would drive up to it and then fail.%s Choose a "
                    "reachable goal, or route around the obstruction rather "
                    "than through it." % (x, y, note))
        return None

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

        # Is the goal cell somewhere the vehicle could actually stand?
        #
        # Being inside the map is not the same as being in free space. A
        # relative move computed from a heading lands wherever the arithmetic
        # says, and if that is inside a building Nav2 will spend the full
        # timeout getting as close as it can before aborting - 60 seconds to
        # learn something the costmap knew immediately. Observed: a request to
        # go 10m ahead put the goal inside a wall; Nav2 drove 9.74m, stalled
        # against the building, and reported an approach-angle problem.
        blocked = self._goal_is_blocked(float(x), float(y))
        if blocked:
            self._last_failure = blocked
            return blocked


        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            return ("navigate_to failed: the navigate_to_pose action server is "
                    "not available. Nav2 is not running or not activated.")

        start = self._pose()
        if start is None:
            return "navigate_to failed: robot pose unavailable."

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = GLOBAL_FRAME
        # Zero stamp = "use the latest available transform". A concrete
        # timestamp pins the goal to one instant, and Nav2's behaviour tree
        # re-plans against the same message every cycle - so each retry asks
        # TF for a transform further into the past. Observed: the planner
        # requesting t=4803.000 while the buffer had advanced to t=4821, then
        # aborting, running backup, and failing the goal.
        goal.pose.header.stamp = rclpy.time.Time().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        # `yaw or 0.0` was silently turning "no heading preference" into
        # "arrive facing due east". A PoseStamped always carries an
        # orientation, so None has to become SOMETHING - and 0.0 is the worst
        # choice available. Observed: a trike heading 177 deg was given a goal
        # demanding 0 deg, and with use_rotate_to_heading false it can only
        # correct heading by driving arcs. It travelled 80.95m for a 5m goal.
        #
        # Pointing the goal along the direction of travel makes the constraint
        # self-satisfying: driving straight at a point arrives pointing at it.
        if yaw is None:
            start = self._pose()
            if start is not None:
                goal_yaw = math.atan2(float(y) - start[1], float(x) - start[0])
            else:
                goal_yaw = 0.0
        else:
            goal_yaw = float(yaw)
        goal.pose.pose.orientation = quat_from_yaw(goal_yaw)

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

    def _straight_reverse(self, distance):
        """Back up `distance` metres along the CURRENT heading, no turning.

        Closed-loop on odom displacement (not duration, which slips), with the
        same rear-lidar safety floor as drive(). Steering is held at zero, so
        the vehicle tracks a straight line backwards - exactly what a steered
        trike can do trivially, and what Nav2 refuses to do because a reverse
        pose-goal drags in heading convergence and a 30m loop.
        """
        distance = abs(float(distance))

        upright, att = self.check_attitude()
        if upright is False:
            self._last_failure = att
            return "Straight reverse refused. " + att

        before = self._pose()
        if before is None:
            return "Straight reverse failed: robot pose unavailable."

        # Pre-flight rear clearance.
        summary = summarise_scan(self._scan)
        if summary:
            rear = min(summary.get("rear", 99.0),
                       summary.get("rear-left", 99.0),
                       summary.get("rear-right", 99.0))
            if rear < REVERSE_STOP_DIST:
                msg = ("Straight reverse REFUSED: only %.2fm of clearance "
                       "behind, %.2fm needed. Nothing was moved."
                       % (rear, REVERSE_STOP_DIST))
                self._last_failure = msg
                return msg

        msg = Twist()
        msg.linear.x = -REVERSE_SPEED
        msg.angular.z = 0.0

        t0 = time.time()
        moved = 0.0
        aborted = None
        timeout = distance / REVERSE_SPEED + 6.0   # generous watchdog
        while True:
            cur = self._pose()
            if cur:
                moved = math.hypot(cur[0] - before[0], cur[1] - before[1])
            if moved >= distance:
                break
            summary = summarise_scan(self._scan)
            if summary:
                rear = min(summary.get("rear", 99.0),
                           summary.get("rear-left", 99.0),
                           summary.get("rear-right", 99.0))
                if rear < REVERSE_STOP_DIST:
                    aborted = (rear, moved, time.time() - t0)
                    break
            if time.time() - t0 > timeout:
                aborted = ("timeout", moved, time.time() - t0)
                break
            self.cmd_pub.publish(msg)
            time.sleep(0.05)
        self.cmd_pub.publish(Twist())

        out = ("Straight reverse: backed up %.2fm of %.2fm requested along "
               "heading (no reorientation)." % (moved, distance))
        if aborted is not None:
            reason, got, el = aborted
            if reason == "timeout":
                out += (" STOPPED: watchdog fired at %.1fs after %.2fm - the "
                        "vehicle may be obstructed or slipping." % (el, got))
            else:
                out += (" STOPPED EARLY after %.1fs: obstacle within %.2fm "
                        "behind. Straight reverse halts rather than backing "
                        "into it." % (el, reason))
        else:
            out += " Completed."
        self._last_failure = ("Last straight reverse completed."
                              if aborted is None else out)
        return out + " " + self.get_pose()

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

        # Straight-back rule: a short reverse with no sideways component is a
        # pure back-up the trike can do directly. Routing it through Nav2 as a
        # pose goal drags in heading convergence and produces a huge forward
        # loop (observed: 34m travelled for an 8m reverse). Above the threshold,
        # fall through to Nav2 so it can plan a reorienting Reeds-Shepp path.
        if (forward < 0.0 and abs(left) < REVERSE_STRAIGHT_TOL
                and abs(forward) <= REVERSE_STRAIGHT_MAX):
            return self._straight_reverse(abs(forward))

        gx = x + forward * math.cos(yaw) - left * math.sin(yaw)
        gy = y + forward * math.sin(yaw) + left * math.cos(yaw)

        note = ("Relative move: %.2fm forward, %.2fm left from (%.2f, %.2f) "
                "at %.1f deg -> absolute goal (%.2f, %.2f). "
                % (forward, left, x, y, math.degrees(yaw), gx, gy))

        # Do NOT constrain the goal heading for a straight-ahead move.
        #
        # Passing the current yaw as the goal yaw turns "go 8m ahead" into
        # "reach that point AND be pointing exactly this way". A few degrees of
        # drift then makes the heading constraint unsatisfiable without a loop,
        # because this platform cannot correct heading in place. Observed: an
        # 8m goal produced 22.39m of travel over 129.8s, finishing 90 degrees
        # off, in open ground with the nearest obstacle 6m away.
        #
        # For a pure forward move, where the vehicle ends up pointing is not
        # part of the request. Leave yaw free and let Nav2 arrive naturally.
        goal_yaw = None if abs(left) < REVERSE_STRAIGHT_TOL else yaw
        return note + self.navigate_to(gx, gy, goal_yaw)

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

    # --- deterministic delivery mission -----------------------------------

    def list_delivery_locations(self):
        """Return canonical delivery locations and friendly aliases."""

        return (
            "BGLX delivery locations:\n"
            + format_location_registry()
        )

    def record_delivery_location(
        self,
        name,
        location_type,
        aliases=None
    ):
        """Persist the robot's current map position as a mission location."""

        if self.mission_active():

            return (
                "Delivery location recording REFUSED: an active "
                "delivery mission currently owns vehicle movement."
            )

        pose = self._pose()

        if pose is None:

            return (
                "Delivery location recording FAILED: "
                "current map pose is unavailable."
            )

        name = str(
            name
        ).strip()

        location_type = str(
            location_type
        ).strip().lower()

        if aliases is None:
            aliases = []

        if not isinstance(
            aliases,
            list
        ):

            return (
                "Delivery location recording REFUSED: "
                "aliases must be a list of names."
            )

        aliases = [
            str(alias).strip()
            for alias in aliases
            if str(alias).strip()
        ]

        try:

            canonical = save_custom_location(
                name=name,
                x=pose[0],
                y=pose[1],
                location_type=location_type,
                aliases=aliases,
                display_name=name,
            )

        except ValueError as exc:

            return (
                "Delivery location recording REFUSED: %s"
                % exc
            )

        except Exception as exc:

            return (
                "Delivery location recording FAILED: %s: %s"
                % (
                    type(exc).__name__,
                    exc,
                )
            )

        info = get_location_info(
            canonical
        )

        return (
            "DELIVERY LOCATION SAVED: %s [%s] at "
            "(%.3f, %.3f) in map frame. "
            "Display name: %s. Aliases: %s"
            % (
                canonical,
                info['type'],
                info['x'],
                info['y'],
                info.get(
                    'display_name',
                    canonical
                ),
                ', '.join(
                    info.get(
                        'aliases',
                        []
                    )
                ) or 'none',
            )
        )


    def update_delivery_location(
        self,
        name
    ):
        """
        Explicitly move an existing custom delivery location
        to the robot's CURRENT map position.

        Existing type, display name and aliases are preserved.
        """

        if self.mission_active():

            return (
                "Delivery location update REFUSED: an active "
                "delivery mission currently owns vehicle movement."
            )

        pose = self._pose()

        if pose is None:

            return (
                "Delivery location update FAILED: "
                "current map pose is unavailable."
            )

        name = str(
            name
        ).strip()

        if not name:

            return (
                "Delivery location update REFUSED: "
                "location name cannot be empty."
            )

        try:

            canonical = update_custom_location(
                name=name,
                x=pose[0],
                y=pose[1],
            )

            info = get_location_info(
                canonical
            )

        except ValueError as exc:

            return (
                "Delivery location update REFUSED: %s"
                % exc
            )

        return (
            "DELIVERY LOCATION UPDATED: "
            "%s [%s] is now at (%.3f, %.3f) "
            "in map frame. Display name: %s."
            % (
                canonical,
                info['type'],
                info['x'],
                info['y'],
                info.get(
                    'display_name',
                    canonical
                ),
            )
        )


    def delete_delivery_location(
        self,
        name
    ):
        """
        Explicitly forget an existing custom delivery location.

        Built-in locations remain protected by the registry layer.
        """

        if self.mission_active():

            return (
                "Delivery location deletion REFUSED: an active "
                "delivery mission currently owns vehicle movement."
            )

        name = str(
            name
        ).strip()

        if not name:

            return (
                "Delivery location deletion REFUSED: "
                "location name cannot be empty."
            )

        try:

            canonical = delete_custom_location(
                name
            )

        except ValueError as exc:

            return (
                "Delivery location deletion REFUSED: %s"
                % exc
            )

        return (
            "DELIVERY LOCATION DELETED: %s. "
            "The saved custom location has been forgotten."
            % canonical
        )


    def get_mission_history(
        self,
        limit=5
    ):
        """Return persistent recent delivery mission history."""

        try:

            limit = int(
                limit
            )

        except Exception:

            limit = 5

        limit = max(
            1,
            min(
                limit,
                20
            )
        )

        return format_mission_history(
            limit=limit
        )

    def mission_active(self):
        """True while the deterministic mission process is alive."""

        proc = self._mission_proc

        return (
            proc is not None
            and proc.poll() is None
        )

    def get_mission_status(self):
        """Report live deterministic delivery mission state."""

        proc = self._mission_proc

        with self._lock:
            state = self._mission_state
            route = self._mission_route
            started = self._mission_started_at

        if proc is None:

            return (
                "MISSION STATUS: IDLE. "
                "No delivery mission has been started "
                "by this agent process."
            )

        code = proc.poll()

        if route is None:
            route_text = "unknown route"
        else:
            route_text = (
                "HOME -> %s -> %s -> HOME"
                % route
            )

        elapsed = None

        if started is not None:
            elapsed = max(
                0.0,
                time.time() - started
            )

        if code is None:

            if elapsed is None:
                return (
                    "MISSION STATUS: ACTIVE. "
                    "state=%s, route=%s."
                    % (
                        state,
                        route_text,
                    )
                )

            return (
                "MISSION STATUS: ACTIVE. "
                "state=%s, route=%s, elapsed=%.1fs."
                % (
                    state,
                    route_text,
                    elapsed,
                )
            )

        if (
            code == 0
            and state == 'MISSION_COMPLETE'
        ):

            return (
                "MISSION STATUS: COMPLETE. "
                "state=%s, route=%s."
                % (
                    state,
                    route_text,
                )
            )

        return (
            "MISSION STATUS: STOPPED. "
            "state=%s, route=%s, process_exit=%d."
            % (
                state,
                route_text,
                code,
            )
        )

    def cancel_delivery_mission(self):
        """Cancel the active deterministic mission and its Nav2 goal."""

        proc = self._mission_proc

        if (
            proc is None
            or proc.poll() is not None
        ):

            return (
                "No active delivery mission to cancel. "
                + self.get_mission_status()
            )

        self.get_logger().warning(
            "cancelling active deterministic delivery mission"
        )

        try:

            # delivery_mission.py handles KeyboardInterrupt by
            # cancelling its active Nav2 goal before shutting down.
            os.killpg(
                proc.pid,
                signal.SIGINT
            )

            try:

                proc.wait(
                    timeout=5.0
                )

            except subprocess.TimeoutExpired:

                self.get_logger().warning(
                    "mission did not exit after SIGINT; "
                    "sending SIGTERM"
                )

                os.killpg(
                    proc.pid,
                    signal.SIGTERM
                )

                try:
                    proc.wait(
                        timeout=3.0
                    )
                except subprocess.TimeoutExpired:
                    os.killpg(
                        proc.pid,
                        signal.SIGKILL
                    )
                    proc.wait(
                        timeout=2.0
                    )

        except ProcessLookupError:
            pass

        except Exception as exc:

            return (
                "MISSION CANCEL ERROR: %s: %s"
                % (
                    type(exc).__name__,
                    exc,
                )
            )

        # Cancellation is authoritative at the agent layer. The child
        # process may have been interrupted between two mission-state
        # publications, so do not leave the previous state (for example
        # UNLOADING or NAVIGATING_TO_DELIVERY) looking current.
        with self._lock:
            self._mission_state = 'MISSION_INTERRUPTED'

        # Final zero command as a belt-and-suspenders stop.
        self.cmd_pub.publish(
            Twist()
        )

        time.sleep(
            0.2
        )

        return (
            "DELIVERY MISSION CANCELLED. "
            + self.get_mission_status()
        )

    def run_delivery_mission(
        self,
        pickup,
        delivery
    ):
        """Start the deterministic delivery mission asynchronously."""

        pickup_input = str(
            pickup
        ).strip()

        delivery_input = str(
            delivery
        ).strip()

        if self.mission_active():

            return (
                "Delivery mission REFUSED: another delivery "
                "mission is already active. "
                + self.get_mission_status()
            )

        try:

            pickup = resolve_location_name(
                pickup_input
            )

        except ValueError:

            return (
                "Delivery mission REFUSED: unknown pickup "
                "location '%s'.\n%s"
                % (
                    pickup_input,
                    self.list_delivery_locations(),
                )
            )

        try:

            delivery = resolve_location_name(
                delivery_input
            )

        except ValueError:

            return (
                "Delivery mission REFUSED: unknown delivery "
                "location '%s'.\n%s"
                % (
                    delivery_input,
                    self.list_delivery_locations(),
                )
            )

        pickup_info = get_location_info(
            pickup
        )

        delivery_info = get_location_info(
            delivery
        )

        if pickup_info.get('type') != 'pickup':

            return (
                "Delivery mission REFUSED: '%s' resolves to %s, "
                "which is a %s location, not a pickup location."
                % (
                    pickup_input,
                    pickup,
                    pickup_info.get(
                        'type',
                        'generic'
                    ),
                )
            )

        if delivery_info.get('type') != 'delivery':

            return (
                "Delivery mission REFUSED: '%s' resolves to %s, "
                "which is a %s location, not a delivery location."
                % (
                    delivery_input,
                    delivery,
                    delivery_info.get(
                        'type',
                        'generic'
                    ),
                )
            )

        if pickup == delivery:

            return (
                "Delivery mission REFUSED: pickup and delivery "
                "cannot be the same location."
            )

        pose = self._pose()

        if pose is None:

            return (
                "Delivery mission REFUSED: current robot pose "
                "is unavailable."
            )

        hx, hy = LOCATIONS['HOME']

        home_distance = math.hypot(
            pose[0] - hx,
            pose[1] - hy
        )

        if (
            home_distance
            > MISSION_HOME_START_TOLERANCE
        ):

            self.get_logger().info(
                "delivery mission requested %.2fm from HOME; "
                "returning to HOME before mission start"
                % home_distance
            )

            print(
                "\n[mission] PREPARING_AT_HOME",
                flush=True
            )

            # Approach HOME along the direction of travel from the
            # current position. This keeps the preparation movement
            # compatible with the tricycle geometry rather than
            # imposing an arbitrary final heading.
            home_yaw = math.atan2(
                hy - pose[1],
                hx - pose[0]
            )

            home_result = self.navigate_to(
                hx,
                hy,
                home_yaw
            )

            pose = self._pose()

            if pose is None:

                return (
                    "Delivery mission REFUSED: automatic return "
                    "to HOME finished but the robot pose is "
                    "unavailable. "
                    + str(home_result)
                )

            home_distance = math.hypot(
                pose[0] - hx,
                pose[1] - hy
            )

            if (
                home_distance
                > MISSION_HOME_START_TOLERANCE
            ):

                return (
                    "Delivery mission REFUSED: automatic return "
                    "to HOME did not succeed. Robot remains %.2fm "
                    "from HOME. %s"
                    % (
                        home_distance,
                        home_result,
                    )
                )

            self.get_logger().info(
                "robot prepared at HOME; starting "
                "deterministic delivery mission"
            )

            print(
                "\n[mission] READY_AT_HOME",
                flush=True
            )

        upright, attitude = (
            self.check_attitude()
        )

        if upright is False:

            return (
                "Delivery mission REFUSED: "
                + attitude
            )

        if not self.nav_client.wait_for_server(
            timeout_sec=2.0
        ):

            return (
                "Delivery mission REFUSED: "
                "Nav2 NavigateToPose server is unavailable."
            )

        cmd = [
            'ros2',
            'run',
            'bglx_agentic',
            'delivery_mission',
            '--ros-args',
            '-p',
            'home_name:=HOME',
            '-p',
            'pickup_name:=%s' % pickup,
            '-p',
            'delivery_name:=%s' % delivery,
        ]

        self.get_logger().info(
            "starting asynchronous deterministic delivery: "
            "HOME -> %s -> %s -> HOME"
            % (
                pickup,
                delivery,
            )
        )

        with self._lock:
            self._mission_state = 'STARTING'
            self._mission_route = (
                pickup,
                delivery,
            )
            self._mission_started_at = (
                time.time()
            )

        try:

            # Keep the asynchronous mission's verbose Nav2/state output
            # away from the interactive agent prompt. The latest mission
            # always gets its own clean log file.
            os.makedirs(
                os.path.dirname(MISSION_LOG_PATH),
                exist_ok=True
            )

            mission_env = os.environ.copy()
            mission_env['PYTHONUNBUFFERED'] = '1'

            with open(
                MISSION_LOG_PATH,
                'w',
                buffering=1
            ) as mission_log:

                self._mission_proc = subprocess.Popen(
                    cmd,
                    stdout=mission_log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                    env=mission_env
                )

        except FileNotFoundError:

            self._mission_proc = None

            return (
                "DELIVERY MISSION FAILED: ros2 executable "
                "was not found in the agent environment."
            )

        except Exception as exc:

            self._mission_proc = None

            return (
                "DELIVERY MISSION FAILED to launch: %s: %s"
                % (
                    type(exc).__name__,
                    exc,
                )
            )

        return (
            "DELIVERY MISSION STARTED: "
            "HOME -> %s -> %s -> HOME. "
            "The deterministic mission controller now owns "
            "vehicle movement. Do not issue manual movement "
            "commands while it is active. Use "
            "get_mission_status to check progress. "
            "Detailed mission output is being written to %s."
            % (
                pickup,
                delivery,
                MISSION_LOG_PATH,
            )
        )

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

        # drive() bypasses Nav2 entirely: no global plan, no costmap, no
        # collision checking. That is the point - it is the escape hatch when
        # the planner cannot help. But an escape hatch with no floor is a
        # trapdoor: the agent, frustrated by repeated Nav2 stalls, used this
        # to push the vehicle into a wall twice, each time travelling 1.5m of
        # a commanded 5m and grinding for the remaining seconds.
        #
        # So: check the direction of travel before moving, and keep checking
        # while moving. This is not obstacle avoidance - it will not steer
        # around anything - it simply refuses to drive into what it can see.
        STOP_DIST = 1.6          # m, roughly the forward footprint plus margin
        direction = 0.0 if float(linear_x) >= 0 else 180.0
        summary = summarise_scan(self._scan)
        if summary:
            if direction == 0.0:
                near = min(summary.get("front", 99.0),
                           summary.get("front-left", 99.0),
                           summary.get("front-right", 99.0))
                where = "ahead"
            else:
                near = min(summary.get("rear", 99.0),
                           summary.get("rear-left", 99.0),
                           summary.get("rear-right", 99.0))
                where = "behind"
            if near < STOP_DIST:
                return ("drive REFUSED: only %.2fm of clearance %s and this "
                        "tool has no obstacle avoidance whatsoever - it would "
                        "drive straight into it. %.2fm is needed. Use "
                        "navigate_relative, which plans around obstacles, or "
                        "move somewhere with more room first."
                        % (near, where, STOP_DIST))

        before = self._pose()
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        t0 = time.time()
        aborted = None
        while time.time() - t0 < duration:
            # Keep watching. A pre-flight check is not enough: the vehicle can
            # close 1.6m in three seconds, and grinding against a wall for the
            # remainder of the commanded duration is how a 5m command produced
            # 1.5m of travel and a collision.
            summary = summarise_scan(self._scan)
            if summary:
                if float(linear_x) >= 0:
                    near = min(summary.get("front", 99.0),
                               summary.get("front-left", 99.0),
                               summary.get("front-right", 99.0))
                    where = "ahead"
                else:
                    near = min(summary.get("rear", 99.0),
                               summary.get("rear-left", 99.0),
                               summary.get("rear-right", 99.0))
                    where = "behind"
                if near < STOP_DIST:
                    aborted = (near, where, time.time() - t0)
                    break
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
        if aborted is not None:
            near, where, elapsed = aborted
            out += (" STOPPED EARLY after %.1fs: an obstacle came within "
                    "%.2fm %s. drive() has no obstacle avoidance, so it halts "
                    "rather than continuing into it. Do NOT simply retry - use "
                    "navigate_relative, which plans around obstacles."
                    % (elapsed, near, where))
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
        nf = self.vision.frames_received()
        lines.append("Camera frames: %d%s"
                     % (nf, "" if nf else "   <-- NOTHING ARRIVING"))
        lines.append(self.localisation_confidence())
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
