#!/usr/bin/env python3
"""Live opening-triggered vision using the installed BGLX mission controllers."""
import argparse
from collections import deque
from datetime import datetime
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import numpy as np

EXISTING = Path.home() / 'BGLX_Existing_Mission_v01'
sys.path.insert(0, str(EXISTING))
from live_policy import evaluate


class InspectOpening(Exception):
    """Unwind the current mission before cancelling its owned Nav2 action."""


def transit_then_inspect(node, bridge):
    try:
        arrived = node.navigate_with_exploration('DELIVERY_C', bridge.destination)
        bridge.event('transit_finished_without_vision_crossing', arrived=bool(arrived))
        return False
    except InspectOpening:
        bridge.event('handoff_cancel_requested')
        if not node.recovery_guard.cancel_navigation():
            raise RuntimeError('Navigation cancellation/stop not confirmed')
        bridge.event('handoff_stopped')
        return bool(bridge.inspect(True))


class LiveVision:
    def __init__(self, node, destination, out, model, event):
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, LaserScan
        self.node, self.guard = node, node.recovery_guard
        self.destination, self.out, self.model, self.event = destination, out, model, event
        self.scans = deque(maxlen=8)
        self.info = None
        self.original_poll = node.passage_alignment.poll
        self.armed, self.crossed = True, False
        self.last_poll = self.last_report = 0.
        self.last_valid_sample = time.monotonic()
        self.confirmed = None
        self.deadline = time.monotonic() + 600.
        self.subscriptions = [
            node.create_subscription(LaserScan, '/etrike/scan',
                lambda m: self.scans.append((m, time.monotonic())), qos_profile_sensor_data),
            node.create_subscription(CameraInfo, '/etrike/front/camera_info',
                self.receive_info, qos_profile_sensor_data)]

    def receive_info(self, msg):
        self.info = msg, time.monotonic()

    def sample(self):
        from rclpy.time import Time
        from tf2_ros import TransformException
        from analyze import matrix
        from openings import find_openings
        from bglx_agentic.passage_geometry import xf
        grid = self.guard.fresh('grid', .75)
        frame = grid.header.frame_id
        now = self.node.get_clock().now().nanoseconds / 1e9
        last = 'no recent scan with timestamp-supported TF'
        for scan, receipt in reversed(self.scans):
            stamp = scan.header.stamp.sec + scan.header.stamp.nanosec / 1e9
            if not -.1 <= now - stamp <= .4 or time.monotonic() - receipt > .4:
                continue
            try:
                def tf(source):
                    value = self.node.tf_buffer.lookup_transform(frame, source, Time.from_msg(scan.header.stamp))
                    return matrix(value.transform)
                lidar, base = tf(scan.header.frame_id), tf('base_footprint')
                map_to_grid = self.guard._transform_once(frame, 'map', stamp=scan.header.stamp)
            except TransformException as error:
                last = str(error)
                continue
            robot = (float(base[0, 3]), float(base[1, 3]), math.atan2(base[1, 0], base[0, 0]))
            goal = xf(self.destination, map_to_grid)
            targets = []
            for gap in find_openings(scan.ranges, scan.angle_min, scan.angle_increment,
                                     scan.range_min, scan.range_max):
                mouth = lidar @ np.array([*gap['mouth_scan_xy'], 0., 1.])
                mouth[2] -= base[2, 3]
                normal = lidar[:2, :2] @ np.asarray(gap['outward_normal_scan_xy'])
                norm = np.linalg.norm(normal)
                if not np.isfinite(norm) or norm < .9:
                    continue
                normal /= norm
                targets.append(dict(mouth=mouth[:3].tolist(), normal=normal.tolist(),
                                    width=gap['width_at_scan_height_m']))
            eligible, rows = evaluate(targets, robot, goal)
            self.last_valid_sample = time.monotonic()
            return dict(frame=frame, stamp=stamp, robot=robot, eligible=eligible, rows=rows)
        raise RuntimeError('Live inspection data not ready: ' + last)

    def detect(self):
        now = time.monotonic()
        if now - self.last_poll < .2:
            return False
        self.last_poll = now
        try:
            sample = self.sample()
        except RuntimeError as error:
            self.confirmed = None
            if now - self.last_valid_sample > 2.5:
                raise RuntimeError('Live opening monitor unavailable for 2.5s: ' + str(error))
            if now - self.last_report > 3.:
                self.event('inspection_wait', reason=str(error))
                self.last_report = now
            return False
        if sample['rows'] and now - self.last_report >= 1.:
            self.event('live_openings', frame=sample['frame'], robot=sample['robot'],
                       observations=sample['rows'])
            self.last_report = now
        if not sample['eligible']:
            self.confirmed = None
            return False
        target = sample['eligible'][0]['target']
        previous = self.confirmed
        self.confirmed = (sample['stamp'], sample['frame'], target, now)
        ready = (previous is not None and sample['stamp'] > previous[0]
                 and previous[1] == sample['frame'] and now - previous[3] < 1.
                 and math.dist(target['mouth'][:2], previous[2]['mouth'][:2]) < .35)
        if ready:
            self.event('inspection_trigger', target=target, frame=sample['frame'],
                       source='two current lidar scans; no observation waypoint')
        return ready

    def poll(self):
        if self.armed and self.detect():
            raise InspectOpening()
        return self.original_poll()

    def fresh_stationary_sample(self):
        deadline = time.monotonic() + 1.5
        last = ''
        while time.monotonic() < deadline:
            try:
                return self.sample()
            except RuntimeError as error:
                last = str(error)
                self.guard.spin(.03)
        raise RuntimeError(last)

    def process(self, command, label, duration=None):
        log = self.out / (label + '.log')
        with log.open('w') as stream:
            proc = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                    start_new_session=True)
            end = min(self.deadline, time.monotonic() + (duration or 110.))
            try:
                while proc.poll() is None and time.monotonic() < end:
                    self.guard.spin(.05)
                if proc.poll() is None:
                    if duration is None or time.monotonic() >= self.deadline:
                        raise RuntimeError(label + ' timed out')
                    os.killpg(proc.pid, signal.SIGINT)
                    flush_end = min(self.deadline, time.monotonic() + 15.)
                    while proc.poll() is None and time.monotonic() < flush_end:
                        self.guard.spin(.05)
                    if proc.poll() is None:
                        raise RuntimeError('Recorder did not finish writing')
                allowed = (0, -signal.SIGINT) if duration is not None else (0,)
                if proc.returncode not in allowed:
                    stream.flush()
                    tail = '\n'.join(log.read_text(errors='replace').splitlines()[-25:])
                    raise RuntimeError(label + ' failed:\n' + tail)
            finally:
                if proc.poll() is None:
                    os.killpg(proc.pid, signal.SIGTERM)
                    try:
                        proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid, signal.SIGKILL)
                        proc.wait(timeout=3)

    def inspect(self, execute):
        from rclpy.time import Time
        from analyze import matrix
        from bglx_tf_sync import grid_context
        from bglx_agentic.passage_geometry import Grid, wrap, xf
        from look_at_opening import choose, project
        from resume import TOPICS
        import vision_handoff
        self.armed = False
        self.guard.ready()
        sample = self.fresh_stationary_sample()
        msg, robot, map_to_grid = grid_context(self.node, event=self.event)
        if sample['frame'] != msg.header.frame_id:
            raise RuntimeError('Costmap frame changed before inspection')
        targets = [row['target'] for row in sample['rows']]
        eligible, rows = evaluate(targets, robot, xf(self.destination, map_to_grid))
        self.event('stationary_openings', frame=msg.header.frame_id, observations=rows)
        if not eligible:
            raise RuntimeError('No eligible opening after stopping; no passage requested')
        if self.info is None:
            raise RuntimeError('Camera calibration missing')
        info, receipt = self.info
        stamp = info.header.stamp.sec + info.header.stamp.nanosec / 1e9
        age = self.node.get_clock().now().nanoseconds / 1e9 - stamp
        if time.monotonic() - receipt > .75 or not -.1 <= age <= .75:
            raise RuntimeError('Camera calibration stale')
        if any(abs(d) > 1e-8 for d in info.d):
            raise RuntimeError('Camera must be rectified')
        k = np.asarray(info.k).reshape(3, 3)
        if not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
            raise RuntimeError('Invalid camera intrinsics')
        camera = matrix(self.node.tf_buffer.lookup_transform(
            info.header.frame_id, 'base_footprint', Time.from_msg(msg.header.stamp)).transform)
        grid = Grid(msg)
        selected = None
        already_visible = False
        for row in eligible:
            target = row['target']
            heading = math.atan2(target['normal'][1], target['normal'][0])
            uv = project(target['mouth'], robot, camera, k)
            if (uv is not None and .15 * info.width < uv[0] < .85 * info.width
                    and .1 * info.height < uv[1] < .9 * info.height
                    and abs(wrap(robot[2] - heading)) <= math.radians(70)):
                already_visible = True
                self.event('opening_in_view', target=target)
                break

            class ViewGrid:
                def path_clear(self, points):
                    return (abs(wrap(points[-1][2] - heading)) <= math.radians(70)
                            and grid.path_clear(points))

            selected = choose(robot, [target], camera, k, info.width, info.height, ViewGrid())
            if selected is not None:
                break
            self.event('view_candidate_rejected', target=target,
                       reason='no existing audited observation curve with suitable final heading')
        if not already_visible:
            if selected is None:
                raise RuntimeError('No audited view of any eligible opening; stopped')
            points, target, turn = selected
            plan = dict(frame=msg.header.frame_id, points=points, target=target, turn_degrees=turn)
            (self.out / 'view_plan.json').write_text(json.dumps(plan, indent=2))
            self.node.passage_alignment.debug.publish(self.node.passage_alignment.make_path(points, msg.header.frame_id))
            self.event('view_plan', target=target, turn_degrees=turn, endpoint=points[-1], execute=execute)
            if not execute:
                self.event('plan_only', scope='viewing curve checked; crossing requires fresh vision after moving')
                return True
            if not self.node.passage_alignment.follow_checked(
                    points, msg.header.frame_id, min(self.deadline, time.monotonic() + 45.), points[-1][2]):
                raise RuntimeError('Existing observation manoeuvre failed')
        if time.monotonic() >= self.deadline:
            raise RuntimeError('Inspection deadline expired')

        def event(name, **data):
            if name == 'vision_crossing_complete':
                self.crossed = True
            self.event(name, **data)

        # This routine owns alignment and dispatches C only after crossing confirmation.
        return vision_handoff.run(self.node, self.process, event, self.out, EXISTING,
                                  TOPICS, self.model, execute=execute)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--cruise', type=float, default=1.5)
    parser.add_argument('--model', default='qwen2.5vl:7b')
    args = parser.parse_args()
    if not math.isfinite(args.cruise) or not 0 < args.cruise <= 1.5:
        parser.error('Simulation cruise must be >0 and <=1.5m/s')
    for name, expected in json.loads((EXISTING / 'source_hashes.json').read_text()).items():
        module = importlib.import_module('bglx_agentic.' + name)
        if hashlib.sha256(Path(module.__file__).read_bytes()).hexdigest() != expected:
            raise RuntimeError('Unreviewed motion source: ' + name)
    if 'bglx_grid_context' not in (EXISTING / 'vision_handoff.py').read_text():
        raise RuntimeError('Run this package install.py first')
    out = Path.home() / 'bglx_navtest/logs' / ('live_vision_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    out.mkdir(parents=True)
    def event(name, **data):
        row = dict(event=name, wall_time=time.time(), **data)
        print('[live-vision]', json.dumps(row), flush=True)
        with (out / 'events.jsonl').open('a') as stream:
            stream.write(json.dumps(row) + '\n')
    print('RESULTS:', out, flush=True)
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.parameter import Parameter
    from rcl_interfaces.srv import GetParameters, SetParametersAtomically
    from bglx_agentic.delivery_mission import DeliveryMission
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true', '-p', 'passage_alignment_enabled:=true',
                    '-p', 'turnaround_assist_enabled:=true', '-p', 'early_unstuck_enabled:=true',
                    '-p', 'leg_timeout:=600.0'], signal_handler_options=SignalHandlerOptions.NO)
    def interrupted(*_):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM, interrupted)
    node = DeliveryMission()
    guard = node.recovery_guard
    bridge = LiveVision(node, (28., 82., 1.3028507292104001), out, args.model, event)
    success = stopped = False
    try:
        guard.ready()
        if not node.nav_client.wait_for_server(timeout_sec=10):
            raise RuntimeError('NavigateToPose unavailable')
        check = Path.home() / 'projects/BGLX/bglx_ws/tools/prototype_resize/stationary_check.py'
        subprocess.run([sys.executable, str(check)], check=True, timeout=60)
        request = node.passage_alignment.request
        names = ['use_sim_time', 'FollowPath.use_collision_detection', 'FollowPath.desired_linear_vel']
        values = request(node.passage_alignment.get, GetParameters.Request(names=names)).values
        if (len(values) != 3 or values[0].type != 1 or not values[0].bool_value
                or values[1].type != 1 or not values[1].bool_value):
            raise RuntimeError('Requires simulated controller with collision detection')
        limiter = node.create_client(GetParameters, '/cmd_vel_limiter/get_parameters')
        try:
            limit = request(limiter, GetParameters.Request(names=['max_linear_vel'])).values
        finally:
            node.destroy_client(limiter)
        if len(limit) != 1 or limit[0].type not in (2, 3):
            raise RuntimeError('Limiter ceiling unavailable')
        maximum = limit[0].double_value if limit[0].type == 3 else limit[0].integer_value
        if not math.isfinite(maximum) or maximum < args.cruise:
            raise RuntimeError('Requested cruise exceeds limiter ceiling')
        if args.execute:
            setting = Parameter('FollowPath.desired_linear_vel', value=args.cruise).to_parameter_msg()
            result = request(node.passage_alignment.set, SetParametersAtomically.Request(parameters=[setting])).result
            if not result.successful:
                raise RuntimeError('Cruise setting refused: ' + result.reason)
            actual = request(node.passage_alignment.get, GetParameters.Request(names=['FollowPath.desired_linear_vel'])).values
            if len(actual) != 1 or actual[0].type != 3 or abs(actual[0].double_value - args.cruise) > 1e-6:
                raise RuntimeError('Cruise readback mismatch')
        guard.ready()
        guard.spin(.5)
        event('configuration', execute=args.execute, cruise_request=args.cruise,
              staging_waypoint=None, observation_source='live lidar; camera/model after confirmed stop')
        bridge.deadline = time.monotonic() + 600.
        if not args.execute:
            sample = bridge.fresh_stationary_sample()
            event('current_openings', frame=sample['frame'], observations=sample['rows'])
            if sample['eligible']:
                success = bool(bridge.inspect(False))
            else:
                event('plan_only', scope='no nearby eligible opening; execute would monitor during C transit')
                success = True
        else:
            node.passage_alignment.poll = bridge.poll
            initial = bridge.fresh_stationary_sample()
            if initial['eligible']:
                success = bool(bridge.inspect(True))
            else:
                success = transit_then_inspect(node, bridge)
            success = success and bridge.crossed
    except (Exception, KeyboardInterrupt) as error:
        event('abort', reason=str(error) or 'Interrupted')
    finally:
        stopped = bool(guard.cleanup())
        stopped = bool(guard.stopped()) and stopped
        node.destroy_node()
        rclpy.try_shutdown()
        summary = dict(success=success and stopped, execute=args.execute,
                       vision_crossing_confirmed=bridge.crossed, stop_confirmed=stopped)
        (out / 'summary.json').write_text(json.dumps(summary, indent=2))
        event('finish', **summary)
    return 0 if success and stopped else 1


if __name__ == '__main__':
    raise SystemExit(main())
