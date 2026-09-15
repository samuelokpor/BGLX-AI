"""Observed-space staging above the existing Nav2 and recovery supervisor."""
import math
import time

import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from action_msgs.msg import GoalStatus

from .directional_frontier import Grid, candidates, checked_path


class ReachableExploration:
    def __init__(self, node):
        self.n = node
        self.client = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
        self.previous = None

    def spin_until(self, future, seconds):
        deadline = time.monotonic()+seconds
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.n, timeout_sec=.05)
        return future.done()

    def context(self):
        n = self.n
        pose = n._current_pose()
        maps = []
        for key in ('exploration_slam', 'exploration_costmap'):
            msg, received = getattr(n, key, (None, 0))
            if msg is None or time.monotonic()-received > 10:
                return None
            stamp = msg.header.stamp
            age = n.get_clock().now().nanoseconds/1e9-(stamp.sec+stamp.nanosec/1e9)
            if age < -.1 or age > 10 or msg.header.frame_id != n.frame:
                return None
            maps.append(msg)
        return (pose, *maps) if pose is not None else None

    def plan(self, target):
        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = self.n.frame
        goal.goal.pose.position.x, goal.goal.pose.position.y = target[:2]
        goal.goal.pose.orientation.z = math.sin(target[2]/2)
        goal.goal.pose.orientation.w = math.cos(target[2]/2)
        goal.planner_id = 'GridBased'
        goal.use_start = False
        send = self.client.send_goal_async(goal)
        if not self.spin_until(send, 5):
            # Planning never moves the robot, but do not leave an orphan search.
            def cancel_late(f):
                try:
                    handle = f.result()
                    if handle.accepted:
                        handle.cancel_goal_async()
                except Exception:
                    pass
            send.add_done_callback(cancel_late)
            raise RuntimeError('planner acceptance timeout; mission remains stopped')
        handle = send.result()
        if not handle.accepted:
            return None
        result = handle.get_result_async()
        if not self.spin_until(result, 6):
            cancel = handle.cancel_goal_async()
            self.spin_until(cancel, 2)
            if not self.spin_until(result, 3):
                raise RuntimeError('planner did not settle after cancellation')
            return None
        wrapped = result.result()
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            return None
        path = wrapped.result.path
        if not path.poses:
            return None
        end = path.poses[-1].pose.position
        if math.hypot(end.x-target[0], end.y-target[1]) > .35:
            return None
        return path

    def run(self, name, final):
        n = self.n
        deadline = time.monotonic()+n.leg_timeout
        # This readiness call preserves fresh stopped odometry, sensor and
        # lifecycle gates from mission_recovery_v1.
        n.recovery_guard.ready()
        if not self.client.wait_for_server(timeout_sec=5):
            print('[explore] FAIL: planning action unavailable', flush=True)
            return False
        context_deadline = time.monotonic()+12
        while self.context() is None and time.monotonic() < context_deadline:
            rclpy.spin_once(n, timeout_sec=.1)
        failures = getattr(n, '_exploration_failed_targets', [])
        for stage in range(1, 41):
            if time.monotonic() >= deadline:
                print('[explore] FAIL: total leg budget exhausted', flush=True)
                return False
            state = self.context()
            if state is None:
                print('[explore] FAIL: fresh map/costmap/pose unavailable; no blind goal', flush=True)
                return False
            pose, slam, costmap = state
            options = []
            # Map membership alone never authorizes the final goal: it must
            # also produce a path wholly in observed space below.
            if Grid(slam, 65).free(*final[:2]) and Grid(costmap, 99).free(*final[:2]):
                options.append(tuple(final))
            options += candidates(slam, costmap, pose, final, self.previous)
            chosen = None
            for target in options:
                if time.monotonic() >= deadline:
                    break
                if any(time.monotonic()-t < 60 and math.dist(target[:2], xy) < 1.5
                       for t, xy in failures):
                    continue
                print('[explore] PREFLIGHT target=(%.2f, %.2f), distance=%.2fm' %
                      (*target[:2], math.dist(pose[:2], target[:2])), flush=True)
                path = self.plan(target)
                fresh = self.context()
                if path is None or fresh is None:
                    print('[explore] candidate rejected: no valid planner result/context', flush=True)
                    continue
                if math.dist(fresh[0][:2], pose[:2]) > .15:
                    print('[explore] pose changed during preflight; refusing stale selection', flush=True)
                    return False
                ok, reason = checked_path(path, fresh[1], fresh[2])
                print('[explore] '+reason, flush=True)
                if ok:
                    chosen = target
                    break
            if chosen is None:
                print('[explore] FAIL: no forward-progress target with a checked known-space path; '
                      'no motion requested. A detour/exploration policy may be needed.', flush=True)
                return False
            is_final = chosen == tuple(final)
            label = name if is_final else name+'_EXPLORE_'+str(stage)
            print('[explore] COMMITTED %s (%.2f, %.2f); no target switching during this stage' %
                  (label, *chosen[:2]), flush=True)
            self.previous = math.atan2(chosen[1]-pose[1], chosen[0]-pose[0])
            # Bound the whole leg, not 600 seconds separately for every stage.
            old_timeout = n.leg_timeout
            try:
                n.leg_timeout = max(.1, deadline-time.monotonic())
                success = n.navigate(label, chosen)
            finally:
                n.leg_timeout = old_timeout
            if not success:
                failures.append((time.monotonic(), chosen[:2]))
                n._exploration_failed_targets = failures[-20:]
                return False
            if is_final:
                return True
            after = n._current_pose()
            if after is None or math.dist(after[:2], pose[:2]) < .4:
                print('[explore] FAIL: stage made insufficient measured movement', flush=True)
                return False
            # Existing navigate() has already proved stopped. Allow current
            # scans to update the two maps before selecting the next stage.
            until = time.monotonic()+.5
            while rclpy.ok() and time.monotonic() < until:
                rclpy.spin_once(n, timeout_sec=.05)
        print('[explore] FAIL: stage budget exhausted', flush=True)
        return False
