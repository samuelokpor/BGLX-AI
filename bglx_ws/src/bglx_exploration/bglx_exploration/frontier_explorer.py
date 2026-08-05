#!/usr/bin/env python3
"""BGLX frontier exploration — info-gain per A*-path-cost scoring.

Nav2 drives via NavigateToPose; this node only selects goals.

CENTROID-DEADLOCK FIX (2026-08):
Symptom: explorer re-sent the SAME goal (~0.4 m away), Nav2 reported it
"reached" instantly (already inside xy_goal_tolerance), robot never moved,
map never grew, node declared "no information gain — complete".
Cause: score = info_gain / (1 + path_cost) always favours the NEAREST
frontier, and the goal was the frontier *centroid*, which for the blob
wrapping the robot lands right next to it. The old anti-revisit only scaled
score down; it never excluded a goal, so a single frontier was re-picked
forever.
Fix (goal selection only; scoring maths unchanged):
  1. min_goal_distance — reject goals so close Nav2 auto-succeeds without
     moving. Forces travel; pushes the sensor horizon out so new frontiers
     appear.
  2. revisit_radius — HARD-exclude goals near ones already sent.
  3. Graded fallback — far+unvisited, then far-only, then any reachable —
     so it never wedges or completes prematurely.
"""
import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener


def yaw_to_quat(yaw):
    q = Quaternion(); q.z = math.sin(yaw / 2.0); q.w = math.cos(yaw / 2.0); return q


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        p = self.declare_parameter
        p('map_topic', '/map'); p('global_frame', 'map'); p('robot_base_frame', 'base_link')
        p('nav_action', 'navigate_to_pose'); p('min_frontier_perimeter', 0.5)
        p('occupancy_threshold', 65); p('safe_distance', 3.0); p('info_gain_threshold', 0.03)
        p('num_no_gain_attempts', 2); p('goal_timeout', 15.0); p('unknown_traversal_penalty', 3.0)
        p('inflation_radius', 0.25); p('info_radius', 0.6); p('period', 1.0)
        # --- centroid-deadlock fix params ---
        p('min_goal_distance', 1.2)   # >= goal_checker xy tolerance (0.30 m)
        p('revisit_radius', 1.5)      # hard-exclude goals near ones already sent
        g = lambda n: self.get_parameter(n).value
        self.map_topic = g('map_topic'); self.global_frame = g('global_frame'); self.base_frame = g('robot_base_frame')
        self.min_perim = g('min_frontier_perimeter'); self.occ_th = g('occupancy_threshold')
        self.safe_dist = g('safe_distance'); self.info_gain_th = g('info_gain_threshold')
        self.num_no_gain = g('num_no_gain_attempts'); self.goal_timeout = g('goal_timeout')
        self.unk_pen = g('unknown_traversal_penalty'); self.infl = g('inflation_radius'); self.info_radius = g('info_radius')
        self.min_goal_dist = g('min_goal_distance'); self.revisit_radius = g('revisit_radius')
        self.grid = None; self.res = None; self.ox = self.oy = 0.0
        self.explored_goals = []; self.expl_dir = (0.0, 0.0)
        self.last_info = None; self.no_gain = 0
        self.busy = False; self.goal_start = None; self._goal_handle = None
        self.goals_published = 0; self.consecutive_fail = 0
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, qos)
        self.tf_buffer = Buffer(); self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, g('nav_action'))
        self.create_timer(g('period'), self.tick)
        self.get_logger().info('Frontier explorer up — waiting for /map and Nav2...')

    def on_map(self, msg):
        w, h = msg.info.width, msg.info.height
        self.grid = np.array(msg.data, dtype=np.int16).reshape(h, w)
        self.res = msg.info.resolution
        self.ox = msg.info.origin.position.x; self.oy = msg.info.origin.position.y

    def robot_cell(self):
        try:
            t = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame, rclpy.time.Time())
        except Exception:
            return None
        wx = t.transform.translation.x; wy = t.transform.translation.y
        return (int((wx - self.ox) / self.res), int((wy - self.oy) / self.res), wx, wy)

    def cell_to_world(self, cx, cy):
        return (self.ox + (cx + 0.5) * self.res, self.oy + (cy + 0.5) * self.res)

    @staticmethod
    def _neigh_any(mask):
        H, W = mask.shape; out = np.zeros_like(mask)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0: continue
                s = np.zeros_like(mask)
                s[max(0, -dy):H - max(0, dy), max(0, -dx):W - max(0, dx)] = \
                    mask[max(0, dy):H - max(0, -dy), max(0, dx):W - max(0, -dx)]
                out |= s
        return out

    def _inflate(self, occ):
        n = int(round(self.infl / self.res)); m = occ.copy()
        for _ in range(n): m = self._neigh_any(m) | m
        return m

    def detect_frontiers(self, occ):
        unknown = self.grid < 0; free = self.grid == 0
        fmask = unknown & self._neigh_any(free) & (~self._neigh_any(occ))
        min_cells = max(1, int(self.min_perim / self.res))
        ys, xs = np.where(fmask); seen = np.zeros_like(fmask); frontiers = []
        for sx, sy in zip(xs.tolist(), ys.tolist()):
            if seen[sy, sx]: continue
            q = deque([(sx, sy)]); comp = []; seen[sy, sx] = True
            while q:
                x, y = q.popleft(); comp.append((x, y))
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < fmask.shape[1] and 0 <= ny < fmask.shape[0] and fmask[ny, nx] and not seen[ny, nx]:
                            seen[ny, nx] = True; q.append((nx, ny))
            if len(comp) >= min_cells:
                frontiers.append((sum(p[0] for p in comp) / len(comp), sum(p[1] for p in comp) / len(comp), len(comp)))
        return frontiers

    def dijkstra(self, occ, start):
        import heapq
        H, W = self.grid.shape; cost = np.full((H, W), np.inf); sx, sy = start
        if occ[sy, sx]:
            q = deque([(sx, sy)]); seen = {(sx, sy)}; found = None
            while q and found is None:
                x, y = q.popleft()
                if not occ[y, x]: found = (x, y); break
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < W and 0 <= ny < H and (nx, ny) not in seen:
                            seen.add((nx, ny)); q.append((nx, ny))
            if found: sx, sy = found
        unknown = self.grid < 0; cost[sy, sx] = 0.0; pq = [(0.0, sx, sy)]
        while pq:
            c, x, y = heapq.heappop(pq)
            if c > cost[y, x]: continue
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0: continue
                    nx, ny = x + dx, y + dy
                    if not (0 <= nx < W and 0 <= ny < H) or occ[ny, nx]: continue
                    step = math.hypot(dx, dy) * (self.unk_pen if unknown[ny, nx] else 1.0)
                    if c + step < cost[ny, nx]:
                        cost[ny, nx] = c + step; heapq.heappush(pq, (c + step, nx, ny))
        return cost

    def score(self, fr, cost_field, rwx, rwy):
        cx, cy, size = fr; pc = cost_field[int(round(cy)), int(round(cx))]
        if not np.isfinite(pc): return float('-inf'), None
        path_cost = pc * self.res
        info_gain = min(size / (self.min_perim / self.res * 10.0), 1.0)
        wx, wy = self.cell_to_world(cx, cy); dxu, dyu = wx - rwx, wy - rwy; mag = math.hypot(dxu, dyu)
        momentum = 0.0
        if mag > 0.1 and (self.expl_dir[0] or self.expl_dir[1]):
            momentum = max(0.0, (self.expl_dir[0] * dxu + self.expl_dir[1] * dyu) / mag)
        s = info_gain / (1.0 + path_cost) * (1.0 + 0.5 * momentum)
        if self.explored_goals:
            dmin = min(math.hypot(wx - gx, wy - gy) for gx, gy in self.explored_goals)
            if dmin < self.safe_dist: s *= dmin / self.safe_dist
        return s, (wx, wy)

    def count_info(self, occ):
        return int(np.sum(self.grid == 0) + np.sum(occ))

    def _candidates(self, frontiers, cost_field, rwx, rwy, min_dist, use_revisit):
        """Keep goals >= min_dist from the robot and (when use_revisit) not
        within revisit_radius of a goal already sent. Returns (score,(gx,gy))."""
        out = []
        for fr in frontiers:
            s, w = self.score(fr, cost_field, rwx, rwy)
            if s == float('-inf') or w is None:
                continue
            gx, gy = w
            if math.hypot(gx - rwx, gy - rwy) < min_dist:
                continue
            if use_revisit and any(
                    math.hypot(gx - ex, gy - ey) < self.revisit_radius
                    for ex, ey in self.explored_goals):
                continue
            out.append((s, (gx, gy)))
        return out

    def tick(self):
        if self.grid is None or self.busy:
            if self.busy and self.goal_start is not None and \
               (self.get_clock().now() - self.goal_start).nanoseconds * 1e-9 > self.goal_timeout:
                self.get_logger().warn('Goal timeout — replanning.'); self._cancel_goal()
            return
        rc = self.robot_cell()
        if rc is None: return
        rx, ry, rwx, rwy = rc; occ = self._inflate(self.grid >= self.occ_th)
        if len(self.explored_goals) > 5 and self.last_info is not None and self.last_info > 0:
            inc = (self.count_info(occ) - self.last_info) / self.last_info
            if inc < self.info_gain_th:
                self.no_gain += 1
                if self.no_gain >= self.num_no_gain:
                    self.get_logger().info('No information gain — exploration complete.'); self.no_gain = 0; return
            else: self.no_gain = 0
        frontiers = self.detect_frontiers(occ)
        if not frontiers:
            self.last_info = self.count_info(occ); self.consecutive_fail += 1
            if self.goals_published >= 2 and self.consecutive_fail >= 10:
                self.get_logger().info('No frontiers left — exploration complete.')
            return
        cost_field = self.dijkstra(occ, (rx, ry))
        # Graded selection (centroid-deadlock fix): far+unvisited, then
        # far-only, then any reachable. Robot is never handed a goal it is
        # already standing on.
        scored = self._candidates(frontiers, cost_field, rwx, rwy, self.min_goal_dist, True)
        if not scored:
            scored = self._candidates(frontiers, cost_field, rwx, rwy, self.min_goal_dist, False)
        if not scored:
            scored = self._candidates(frontiers, cost_field, rwx, rwy, 0.0, False)
        if not scored:
            self.get_logger().info(f'{len(frontiers)} frontiers, none reachable this tick.'); return
        scored.sort(key=lambda t: t[0], reverse=True); best_s, (gx, gy) = scored[0]
        mag = math.hypot(gx - rwx, gy - rwy)
        if mag > 0.1: self.expl_dir = ((gx - rwx) / mag, (gy - rwy) / mag)
        self.explored_goals.append((gx, gy)); self.last_info = self.count_info(occ); self.consecutive_fail = 0
        self.send_goal(gx, gy, math.atan2(gy - rwy, gx - rwx), best_s)

    def send_goal(self, x, y, yaw, s):
        if not self.nav.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 action server not available yet.'); return
        goal = NavigateToPose.Goal(); ps = PoseStamped()
        ps.header.frame_id = self.global_frame; ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(x); ps.pose.position.y = float(y)   # numpy -> float
        ps.pose.orientation = yaw_to_quat(yaw)
        goal.pose = ps; self.busy = True; self.goal_start = self.get_clock().now(); self.goals_published += 1
        self.get_logger().info(f'Goal #{self.goals_published}: ({x:.2f}, {y:.2f}) score={s:.4f}')
        self.nav.send_goal_async(goal).add_done_callback(self._goal_response)

    def _goal_response(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().warn('Goal rejected by Nav2.'); self.busy = False; return
        self._goal_handle = gh; gh.get_result_async().add_done_callback(self._goal_result)

    def _goal_result(self, fut):
        st = fut.result().status
        msg = {GoalStatus.STATUS_SUCCEEDED: 'reached', GoalStatus.STATUS_ABORTED: 'aborted',
               GoalStatus.STATUS_CANCELED: 'cancelled'}.get(st, str(st))
        self.get_logger().info(f'Goal {msg}; selecting next frontier.'); self.busy = False; self._goal_handle = None

    def _cancel_goal(self):
        if self._goal_handle is not None: self._goal_handle.cancel_goal_async()
        self.busy = False; self._goal_handle = None


def main(args=None):
    rclpy.init(args=args); node = FrontierExplorer()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
