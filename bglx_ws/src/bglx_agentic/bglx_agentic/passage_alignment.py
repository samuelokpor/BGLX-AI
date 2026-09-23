"""Bounded, mission-owned local alignment through a Nav2-selected opening.

Nav2's navigation action is cancelled and settled before FollowPath takes
ownership. All velocity commands still pass through the existing collision
monitor and limiter. This module never publishes cmd_vel or spawns models.
"""
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from tf2_ros import TransformException

from .passage_geometry import Grid, aligned_path, candidate_gate, wrap, xf
from .recovery_geometry import yaw


class PassageAlignment:
    def __init__(self, node):
        self.node, self.guard = node, node.recovery_guard
        node.declare_parameter('passage_alignment_enabled', False)
        self.enabled = bool(node.get_parameter('passage_alignment_enabled').value)
        self.plan = None
        self.plan_sequence = 0
        self.min_sequence = 0
        self.goal = None
        self.last_poll = 0.
        self.last_speed_report = 0.
        self.confirmed = None
        self.handled = []
        self.attempts = 0
        self.client = ActionClient(node, FollowPath, '/follow_path')
        self.get = node.create_client(GetParameters, '/controller_server/get_parameters')
        self.set = node.create_client(SetParametersAtomically, '/controller_server/set_parameters_atomically')
        self.debug = node.create_publisher(Path, '/bglx/alignment_path', 1)
        node.create_subscription(Path, '/plan', self.receive, 1)

    def receive(self, msg):
        self.plan, self.plan_sequence = msg, self.plan_sequence+1

    def begin_leg(self, waypoint):
        goal = tuple(waypoint)
        if self.goal != goal:
            self.attempts, self.handled = 0, []
        self.goal = goal
        self.min_sequence = self.plan_sequence+1
        self.confirmed = None

    def poll(self):
        now = time.monotonic()
        if self.enabled and now-self.last_speed_report >= 5.:
            self.last_speed_report = now
            try:
                v = self.guard.fresh('odom', .5).twist.twist
                self.guard.event('DRIVE SPEED: measured v=%.3fm/s, yaw_rate=%.3frad/s'
                                 % (v.linear.x, v.angular.z))
            except RuntimeError:
                pass
        if (not self.enabled or self.attempts >= 2 or now-self.last_poll < .5
                or self.plan is None or self.plan_sequence < self.min_sequence):
            return None
        self.last_poll = now
        try:
            plan = self.plan
            if not plan.poses or plan.header.frame_id != self.node.frame:
                return None
            end = plan.poses[-1].pose.position
            if math.dist((end.x, end.y), self.goal[:2]) > .8:
                return None
            msg = self.guard.fresh('grid', .75)
            # Nonblocking TF while Nav2 owns motion; the mission loop must keep spinning.
            robot = self.guard._transform_once(msg.header.frame_id, 'base_footprint')
            transform = self.guard._transform_once(msg.header.frame_id, plan.header.frame_id)
            points = [xf((p.pose.position.x, p.pose.position.y, yaw(p.pose.orientation)), transform)
                      for p in plan.poses]
            if math.dist(points[-1][:2], robot[:2]) < 5:
                return None
            gate = candidate_gate(Grid(msg), points, robot)
            if gate is None or any(math.dist((gate.x, gate.y), q) < 3. for q in self.handled):
                self.confirmed = None
                return None
            # Do not interrupt a trike already aligned on the opening centreline.
            lateral = abs(-(robot[0]-gate.x)*math.sin(gate.heading)
                          +(robot[1]-gate.y)*math.cos(gate.heading))
            if abs(wrap(robot[2]-gate.heading)) < math.radians(12) and lateral < .12:
                return None
            previous = self.confirmed
            self.confirmed = (gate, msg.header.frame_id, now)
            if (previous is None or previous[1] != msg.header.frame_id or now-previous[2] > 1.5
                    or math.dist((gate.x, gate.y), (previous[0].x, previous[0].y)) > .25
                    or abs(wrap(gate.heading-previous[0].heading)) > math.radians(10)):
                return None
            return gate, msg.header.frame_id
        except (RuntimeError, ValueError, TransformException):
            self.confirmed = None
            return None

    def request(self, client, request):
        if not client.wait_for_service(timeout_sec=3.):
            raise RuntimeError('alignment controller parameter service unavailable')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.)
        if not future.done() or future.result() is None:
            # The update may still arrive. Block subsequent motion on uncertainty.
            self.guard.fail('alignment parameter response unresolved')
        return future.result()

    def set_values(self, parameters):
        result = self.request(self.set, SetParametersAtomically.Request(parameters=parameters)).result
        if not result.successful:
            raise RuntimeError('alignment controller settings refused: '+result.reason)

    def make_path(self, points, frame):
        msg = Path()
        msg.header.frame_id = frame
        for x, y, angle in points:
            p = PoseStamped()
            p.header.frame_id = frame
            p.pose.position.x, p.pose.position.y = x, y
            p.pose.orientation.z, p.pose.orientation.w = math.sin(angle/2), math.cos(angle/2)
            msg.poses.append(p)
        return msg

    def perform(self, candidate, leg_deadline):
        gate, frame = candidate
        if self.node.active_goal_handle is not None:
            self.guard.fail('alignment requested before Nav2 cancellation settled')
        self.attempts += 1
        self.handled.append((gate.x, gate.y))
        self.min_sequence = self.plan_sequence+1
        self.confirmed = None
        self.guard.ready()
        self.guard.event('PASSAGE ALIGN: live opening in %s at (%.2f, %.2f), width %.2fm'
                         % (frame, gate.x, gate.y, gate.width))

        def prepare():
            msg = self.guard.fresh('grid', .75)
            if msg.header.frame_id != frame:
                raise RuntimeError('local costmap frame changed during alignment')
            ok, reason = self.guard.reverse_audit(0.)
            if not ok:
                raise RuntimeError('alignment context: '+reason)
            robot = self.guard.transform(frame, 'base_footprint')
            grid = Grid(msg)
            measured = grid.section((gate.x, gate.y, gate.heading))
            if measured is None or abs(measured.width-gate.width) > .20:
                raise RuntimeError('opening changed before alignment')
            return aligned_path(grid, robot, gate)

        points = prepare()
        if points is None:
            self.guard.event('PASSAGE ALIGN: no forward connector; trying one audited straight backup')
            if not self.guard.backup('create room for live passage alignment'):
                return False
            self.guard.spin(.3)
            points = prepare()
        if points is None:
            self.guard.event('PASSAGE ALIGN refused: no footprint-clear, forward-curvature connector')
            return False
        return self.follow_checked(points, frame, leg_deadline, gate.heading)

    def follow_checked(self, points, frame, leg_deadline, target_heading):
        """Shared existing follower for passage and observation manoeuvres."""
        if self.node.active_goal_handle is not None:
            self.guard.fail('manoeuvre requested before navigation settled')
        self.guard.ready()
        if time.monotonic() >= leg_deadline:
            return False
        if not self.client.wait_for_server(timeout_sec=3.):
            raise RuntimeError('FollowPath unavailable for alignment')
        names = ['FollowPath.desired_linear_vel', 'FollowPath.use_velocity_scaled_lookahead_dist',
                 'FollowPath.lookahead_dist']
        values = self.request(self.get, GetParameters.Request(names=names)).values
        if len(values) != len(names) or any(v.type == 0 for v in values):
            raise RuntimeError('alignment settings not declared')
        saved = [ParameterMsg(name=name, value=value) for name, value in zip(names, values)]
        temporary = [Parameter(name, value=value).to_parameter_msg() for name, value in
                     zip(names, (.25, False, .6))]
        changed = False
        ok = False
        try:
            # Mark before requesting: a lost response may still mean an applied update.
            changed = True
            self.set_values(temporary)
            self.guard.spin(.1)
            current = self.guard.transform(frame, 'base_footprint')
            if math.dist(current[:2], points[0][:2]) > .03 or abs(wrap(current[2]-points[0][2])) > .03:
                raise RuntimeError('alignment start changed while preparing')
            if not Grid(self.guard.fresh('grid', .75)).path_clear(points):
                raise RuntimeError('alignment path changed before dispatch')
            self.guard.spin(.15)
            current = self.guard.transform(frame, 'base_footprint')
            if math.dist(current[:2], points[0][:2]) > .03:
                raise RuntimeError('alignment start moved during audit')
            goal = FollowPath.Goal()
            goal.path = self.make_path(points, frame)
            goal.controller_id = 'FollowPath'
            goal.goal_checker_id = 'general_goal_checker'
            self.debug.publish(goal.path)
            handle = self.guard.resolve_send(self.client.send_goal_async(goal), 'passage alignment', 5.)
            if handle is None or not handle.accepted:
                return False
            self.node.active_goal_handle = handle
            result = handle.get_result_async()
            self.guard.nav_result = result
            self.guard.event('PASSAGE ALIGN: following audited manoeuvre at <=0.25m/s')
            deadline = min(leg_deadline, time.monotonic()+90.)
            last_index, last_check = 0, 0.
            while rclpy.ok() and not result.done():
                self.guard.spin(.05)
                now = time.monotonic()
                if now > deadline:
                    raise RuntimeError('alignment deadline exceeded')
                if now-last_check < .15:
                    continue
                last_check = now
                # No startup TF retry while moving.
                pose = self.guard._transform_once(frame, 'base_footprint')
                odom = self.guard.fresh('odom', .35)
                if odom.twist.twist.linear.x < -.03 or abs(odom.twist.twist.linear.x) > .32:
                    raise RuntimeError('alignment speed/direction outside bound')
                ok_context, reason = self.guard.reverse_audit(0., moving=True)
                if not ok_context:
                    raise RuntimeError(reason)
                # A simple, non-looping forward path: limit nearest-point search to its remainder.
                index = min(range(last_index, len(points)), key=lambda i: math.dist(pose[:2], points[i][:2]))
                if math.dist(pose[:2], points[index][:2]) > .10:
                    raise RuntimeError('alignment tracking error exceeds 0.10m')
                if abs(wrap(pose[2]-points[index][2])) > math.radians(15):
                    raise RuntimeError('alignment heading tracking error exceeds 15 degrees')
                last_index = index
                # Check the next 1.5m, including stopping room, in each fresh local grid.
                segment = [pose]
                length = 0.
                for p in points[index:]:
                    length += math.dist(segment[-1][:2], p[:2])
                    segment.append(p)
                    if length >= 1.5:
                        break
                msg = self.guard.fresh('grid', .75)
                if msg.header.frame_id != frame or not Grid(msg).path_clear(segment, margin=.05):
                    raise RuntimeError('alignment swept path no longer clear')
            ok = result.done() and result.result().status == GoalStatus.STATUS_SUCCEEDED
        finally:
            settled = self.guard.cancel_navigation()
            if changed and settled and self.guard.pending is None:
                try:
                    self.set_values(saved)
                except Exception as exc:
                    self.guard.fail('cruise settings restoration failed: '+str(exc))
            elif changed:
                self.guard.fail('alignment not settled; retained low-speed settings')
        if ok:
            final = self.guard.transform(frame, 'base_footprint')
            if math.dist(final[:2], points[-1][:2]) > .4 or abs(wrap(final[2]-target_heading)) > .15:
                raise RuntimeError('alignment endpoint outside checked exit tolerance')
            self.guard.event('AUDITED MANOEUVRE complete')
        return ok
