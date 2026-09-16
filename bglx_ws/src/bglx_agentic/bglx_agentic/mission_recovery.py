"""Mission-side recovery ownership, stop confirmation and reverse audit.

Forward routing remains Nav2's job. This module never publishes cmd_vel.
It deliberately does not reuse the course's Gazebo pillar geometry.
"""
import math
import time

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import BackUp
from action_msgs.msg import GoalStatus
from rcl_interfaces.msg import Log

from .recovery_geometry import audit, yaw


class RecoveryGuard:
    def __init__(self, node, map_qos):
        self.node = node
        self.data = {}
        self.backup_handle = None
        self.backup_result = None
        self.nav_result = None
        self.pending = None
        self.fault = None
        self.last_audit = 0.0
        self.log_times = {}
        node.create_subscription(Log, '/rosout', self.server_log, 100)
        self.clients = {name: node.create_client(GetState, name+'/get_state')
                        for name in ('/planner_server', '/controller_server',
                                     '/collision_monitor', '/behavior_server')}
        for kind, topic, key, qos in (
            (Odometry, '/tricycle_steering_controller/odometry', 'odom', qos_profile_sensor_data),
            (OccupancyGrid, '/local_costmap/costmap', 'grid', map_qos),
            (PolygonStamped, '/local_costmap/published_footprint', 'footprint', qos_profile_sensor_data),
            *((LaserScan, topic, topic, qos_profile_sensor_data) for topic in (
                '/etrike/front_scan', '/etrike/left_depth/scan',
                '/etrike/right_depth/scan', '/etrike/rear_depth/scan')),
        ):
            node.create_subscription(kind, topic,
                                     lambda msg, k=key: self.receive(k, msg), qos)

    def server_log(self, msg):
        # Preserve the server's actual reason alongside generic action status=6.
        if msg.level < 30 or not any(name in msg.name for name in (
                'planner_server', 'controller_server', 'bt_navigator',
                'collision_monitor', 'cmd_vel_limiter', 'behavior_server')):
            return
        key = (msg.name, msg.msg)
        now = time.monotonic()
        if now-self.log_times.get(key, 0) >= 2.0:
            self.event('SERVER %s: %s' % (msg.name, msg.msg[:1000]))
            if len(self.log_times) > 200:
                self.log_times.clear()
            self.log_times[key] = now

    def receive(self, key, msg):
        self.data[key] = (msg, time.monotonic())

    def event(self, text):
        print('[mission-supervisor] '+text, flush=True)

    def spin(self, seconds):
        deadline = time.monotonic()+seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def fresh(self, key, limit):
        msg, received = self.data.get(key, (None, 0))
        if msg is None or time.monotonic()-received > limit:
            raise RuntimeError('missing/stale '+key)
        stamp = msg.header.stamp
        age = self.node.get_clock().now().nanoseconds/1e9 - (stamp.sec+stamp.nanosec/1e9)
        if age < -0.1 or age > limit:
            raise RuntimeError('stale/future timestamp '+key)
        return msg

    def stopped(self, timeout=4.0):
        deadline = time.monotonic()+timeout
        since = None
        first_stamp = None
        while rclpy.ok() and time.monotonic() < deadline:
            self.spin(0.05)
            try:
                msg = self.fresh('odom', 0.5)
                t = msg.twist.twist
                velocity = (t.linear.x, t.linear.y, t.angular.z)
                quiet = (all(math.isfinite(v) for v in velocity)
                         and math.hypot(*velocity[:2]) < 0.02 and abs(velocity[2]) < 0.03)
            except RuntimeError:
                quiet = False
            if quiet:
                since = since or time.monotonic()
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
                first_stamp = stamp if first_stamp is None else first_stamp
                if time.monotonic()-since >= 0.4 and stamp-first_stamp >= 0.25:
                    self.event('STOP CONFIRMED by fresh odometry')
                    return True
            else:
                since = None
                first_stamp = None
        self.event('STOP NOT CONFIRMED; further motion requests blocked')
        return False

    def fail(self, reason):
        self.fault = reason
        raise RuntimeError('Recovery ownership/stop fault: '+reason)

    def ready(self):
        if self.fault or self.pending is not None or self.backup_handle is not None:
            self.fail(self.fault or 'another motion request is unresolved')
        if not self.stopped():
            self.fail('robot not confirmed stopped before action')
        for name, client in self.clients.items():
            if not client.wait_for_service(timeout_sec=1.0):
                raise RuntimeError('lifecycle unavailable: '+name)
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
            if not future.done() or future.result().current_state.id != 3:
                raise RuntimeError('lifecycle not active/responding: '+name)

    def transform(self, target, source, stamp=None):
        import time
        from tf2_ros import TransformException
        if stamp is not None:
            return self._transform_once(target, source, stamp)
        deadline = time.monotonic() + 3.0
        while rclpy.ok():
            try:
                return self._transform_once(target, source)
            except (TransformException, RuntimeError):
                if time.monotonic() >= deadline:
                    raise
                self.spin(0.1)
        raise RuntimeError("ROS stopped while waiting for fresh TF")

    def _transform_once(self, target, source, stamp=None):
        tf = self.node.tf_buffer.lookup_transform(
            target, source, Time.from_msg(stamp) if stamp else Time(),
            timeout=Duration(seconds=0.0))
        t = tf.transform
        # Reject stale dynamic latest transforms (zero stamp is valid for static TF).
        if stamp is None and (tf.header.stamp.sec or tf.header.stamp.nanosec):
            age = self.node.get_clock().now().nanoseconds/1e9 - (
                tf.header.stamp.sec+tf.header.stamp.nanosec/1e9)
            if age > 0.5 or age < -0.1:
                raise RuntimeError('stale/future TF')
        return t.translation.x, t.translation.y, yaw(t.rotation)

    def reverse_audit(self, distance, moving=False):
        try:
            for key in ('/etrike/front_scan', '/etrike/left_depth/scan',
                        '/etrike/right_depth/scan', '/etrike/rear_depth/scan'):
                scan = self.fresh(key, 0.75)
                if not scan.ranges or not any(not math.isnan(v) and v > 0 for v in scan.ranges):
                    raise RuntimeError('invalid scan '+key)
            self.fresh('odom', 0.5)
            grid = self.fresh('grid', 1.0)
            footprint = self.fresh('footprint', 1.0)
            # Verify the LIVE footprint matches the prototype this audit uses.
            fx, fy, fa = self.transform('base_footprint', footprint.header.frame_id,
                                        footprint.header.stamp)
            c, s = math.cos(fa), math.sin(fa)
            points = [(fx+c*p.x-s*p.y, fy+s*p.x+c*p.y) for p in footprint.polygon.points]
            expected = [(-.20, -.285), (1.04, -.285), (1.04, .285), (-.20, .285)]
            if (len(points) != 4 or not all(math.isfinite(v) for p in points for v in p)
                    or any(min(math.dist(p, q) for p in points) > .015 for q in expected)):
                raise RuntimeError('live footprint differs from prototype; audit refused')
            pose = (self._transform_once if moving else self.transform)(
                grid.header.frame_id, 'base_footprint')
            # Cover the permitted 3cm lateral / 3deg heading tracking envelope.
            return audit(grid, pose, distance, margin=.12)
        except Exception as exc:
            return False, str(exc)

    def resolve_send(self, future, label, timeout):
        """A timed-out send is NOT a rejected goal: quarantine and cancel late acceptance."""
        self.pending = future
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if not future.done():
            def late(f):
                try:
                    handle = f.result()
                    if handle and handle.accepted:
                        handle.cancel_goal_async()
                        self.event('cancel requested for late '+label+' acceptance')
                except Exception as exc:
                    self.event('late action cleanup: '+str(exc))
            future.add_done_callback(late)
            self.fail(label+' acceptance timed out; no subsequent motion allowed')
        self.pending = None
        return future.result()

    def cancel(self, handle, result, label):
        if handle is None:
            return True
        try:
            result = result or handle.get_result_async()
            if not result.done():
                future = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
                rclpy.spin_until_future_complete(self.node, result, timeout_sec=4.0)
            if not result.done() or result.result() is None:
                raise RuntimeError('terminal action result missing')
            if not self.stopped():
                raise RuntimeError('odometry did not confirm stop')
            self.event(label+' settled')
            return True
        except Exception as exc:
            self.fault = label+' cancellation/stop unresolved: '+str(exc)
            self.event(self.fault)
            return False

    def cancel_navigation(self, result=None):
        ok = self.cancel(self.node.active_goal_handle, result or self.nav_result, 'navigation')
        if ok:
            self.node.active_goal_handle = None
            self.nav_result = None
        return ok

    def cleanup(self):
        ok = self.cancel_navigation()
        if self.backup_handle is not None:
            ok = self.cancel(self.backup_handle, self.backup_result, 'backup') and ok
            if ok:
                self.backup_handle = None
        # Give a delayed goal response a bounded opportunity to be cancelled.
        if self.pending is not None:
            self.spin(3.0)
            if not self.pending.done():
                self.event('UNRESOLVED ACTION REQUEST: stop the navigation launch before restarting this mission')
                return False
        return ok

    def backup(self, reason):
        if self.node.active_goal_handle is not None:
            self.fail('navigation still owns motion before backup')
        self.ready()
        distance = self.node.unstuck_backup_distance
        speed = self.node.unstuck_backup_speed
        if not (0 < distance <= .90 and 0 < speed <= .40):
            self.event('BACKUP REFUSED: supported limits are 0.90m and 0.40m/s')
            return False
        # Reserve travel during a 1.0s grid age + 0.15s audit interval,
        # plus braking at a conservative 0.5m/s^2 and 0.05m margin.
        # These are simulation assumptions, not measured actuator guarantees.
        stopping_room = speed*1.15 + speed*speed/(2*.5) + .05
        ok, detail = self.reverse_audit(distance+stopping_room)
        self.event('REVERSE AUDIT: %s; stopping reserve=%.2fm, requested speed=%.2fm/s'
                   % (detail, stopping_room, speed))
        if not ok:
            return False
        if not self.node.backup_client.wait_for_server(timeout_sec=1.0):
            return False
        start = self.fresh('odom', .5)
        initial = start.pose.pose
        self.check_odom_pose(initial)
        angle = yaw(initial.orientation)
        goal = BackUp.Goal()
        goal.target.x = -distance
        goal.speed = speed
        allowance = self.node.unstuck_backup_timeout
        goal.time_allowance.sec = int(allowance)
        goal.time_allowance.nanosec = int((allowance-int(allowance))*1e9)
        self.event('AUDITED BACKUP: '+reason)
        handle = self.resolve_send(self.node.backup_client.send_goal_async(goal), 'backup', 3.0)
        if handle is None or not handle.accepted:
            return False
        self.backup_handle = handle
        self.backup_result = handle.get_result_async()
        deadline = time.monotonic()+allowance+2.0
        failure = None
        try:
            while rclpy.ok() and not self.backup_result.done():
                self.spin(.05)
                if time.monotonic() > deadline:
                    failure = 'backup deadline exceeded'
                    break
                current = self.fresh('odom', .5)
                velocity = current.twist.twist.linear.x
                if not math.isfinite(velocity) or velocity < -speed-.08 or velocity > .03:
                    failure = 'reverse velocity outside requested bound'
                    break
                p = current.pose.pose
                self.check_odom_pose(p)
                dx, dy = p.position.x-initial.position.x, p.position.y-initial.position.y
                reverse = -(math.cos(angle)*dx+math.sin(angle)*dy)
                lateral = abs(-math.sin(angle)*dx+math.cos(angle)*dy)
                error = abs(math.atan2(math.sin(yaw(p.orientation)-angle),
                                       math.cos(yaw(p.orientation)-angle)))
                if current.header.frame_id != start.header.frame_id or lateral > .03 or error > math.radians(3):
                    failure = 'straight reverse deviated from audited corridor'
                    break
                if reverse < -.03 or reverse > distance+.05:
                    failure = 'reverse displacement outside bound'
                    break
                ok, detail = self.reverse_audit(max(0.0, distance-reverse)+stopping_room, moving=True)
                if not ok:
                    failure = detail
                    break
        except Exception as exc:
            failure = str(exc)
        finally:
            # Also runs on Ctrl-C. Never let another action overlap an unsettled backup.
            if not self.cancel(handle, self.backup_result, 'backup'):
                self.fail(self.fault)
            self.backup_handle = None
        status = self.backup_result.result().status
        self.event('BACKUP RESULT: status=%s; %s' % (status, failure or 'completed'))
        return failure is None and status == GoalStatus.STATUS_SUCCEEDED

    @staticmethod
    def check_odom_pose(p):
        q = p.orientation
        if (not all(math.isfinite(v) for v in (p.position.x, p.position.y, q.x, q.y, q.z, q.w))
                or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.0) > .02):
            raise RuntimeError('invalid odometry pose')
