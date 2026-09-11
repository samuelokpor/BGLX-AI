
import os, uuid
from pathlib import Path as FilePath
run = FilePath.home()/"bglx_navtest/logs"/("four_pillar_"+uuid.uuid4().hex)
run.mkdir(parents=True)
os.environ.update(BGLX_RESULT=str(run/"result.json"),
                  BGLX_RUN_ID=run.name, BGLX_STAGE="forward")
print("Structured result:", run/"result.json", flush=True)
from protocol import Result, StageFailure
report = Result()
'Supervised simulated alignment and passage; TRIKE MOVES.'
import ast
import math
import signal
import time
from pathlib import Path
import numpy as np
import rclpy
import tf2_ros
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from nav2_msgs.msg import Costmap
from nav2_msgs.action import ComputePathToPose, FollowPath
from gazebo_msgs.srv import GetModelList, DeleteEntity, SpawnEntity
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import Log
FP = [(-0.38, -0.52), (1.63, -0.52), (1.63, 0.52), (-0.38, 0.52)]
(GAP, OFFSET, DISTANCE) = (1.6, 0.5, 4.0)
ANGLE = math.radians(15)
MIN_CLEARANCE = 0.15
STOP = False
from geometry import xf, edges, intersects, pt_seg, poly_dist
geo = dict(xf=xf, edges=edges, intersects=intersects, pt_seg=pt_seg, poly_dist=poly_dist)
import argparse
parser = argparse.ArgumentParser(description='Extended recovery audit ONLY: existing 0.6m test boxes; no motion')
args = parser.parse_args()
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

def interrupt(*_):
    global STOP
    STOP = True
signal.signal(signal.SIGINT, interrupt)
n = rclpy.create_node('staged_gap_trial', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
buf = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(buf, n)
state = {}
odom = []
active = result_future = send_future = None

def spin(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.05)

def wait(future, seconds=20, interruptible=True):
    end = time.monotonic() + seconds
    while not future.done() and time.monotonic() < end:
        if interruptible and STOP:
            raise StageFailure('INTERRUPTED', 'Interrupted')
        rclpy.spin_once(n, timeout_sec=0.05)
    if not future.done():
        raise RuntimeError('Request timed out')
    return future.result()

def service(kind, name, request):
    client = n.create_client(kind, name)
    try:
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError('Service unavailable: ' + name)
        return wait(client.call_async(request))
    finally:
        n.destroy_client(client)

def parameters(node, names):
    req = GetParameters.Request()
    req.names = names
    return service(GetParameters, node + '/get_parameters', req).values

def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

def pose():
    tf = buf.lookup_transform('map', 'base_link', Time())
    now = n.get_clock().now().nanoseconds
    age = (now - Time.from_msg(tf.header.stamp).nanoseconds) / 1000000000.0
    if now <= 0 or not -0.05 <= age <= 0.5:
        raise StageFailure('TF_STALE', f'TF not fresh: age={age:.3f}s')
    p = tf.transform.translation
    return (p.x, p.y, yaw(tf.transform.rotation))

def odom_cb(m):
    state['odom_pose'] = (m, time.monotonic())
    v = m.twist.twist
    odom.append((time.monotonic(), v.linear.x, v.linear.y, v.angular.z))
    del odom[:-300]

def stopped():
    recent = [v for v in odom if time.monotonic() - v[0] < 1.0]
    return len(recent) >= 5 and recent[-1][0] - recent[0][0] >= 0.5 and all((abs(x) < 0.01 and abs(y) < 0.01 and (abs(w) < 0.02) for (_, x, y, w) in recent))

def wait_stopped(seconds=8):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        spin(0.1)
        if stopped():
            return True
    return False
n.create_subscription(Odometry, '/tricycle_steering_controller/odometry', odom_cb, qos_profile_sensor_data)
n.create_subscription(Costmap, '/global_costmap/costmap_raw', lambda m: state.update(grid=m), 10)
for (key, topic) in (('NAV', '/etrike/cmd_vel'), ('CM', '/etrike/collision_checked_cmd_vel'), ('LIM', '/tricycle_steering_controller/reference_unstamped')):
    n.create_subscription(Twist, topic, lambda m, k=key: state.update({k: (m.linear.x, m.angular.z, time.monotonic())}), 10)

def joints(m):
    if 'steering_joint' in m.name:
        i = m.name.index('steering_joint')
        if i < len(m.position):
            state['steering'] = (math.degrees(m.position[i]), time.monotonic())
n.create_subscription(JointState, '/joint_states', joints, qos_profile_sensor_data)

def logs(m):
    if m.level >= 30 and m.name in ('controller_server', 'collision_monitor', 'cmd_vel_limiter'):
        print(f'[{m.name}] {m.msg}', flush=True)
n.create_subscription(Log, '/rosout', logs, 100)

def stamped(p):
    m = PoseStamped()
    m.header.frame_id = 'map'
    m.header.stamp = n.get_clock().now().to_msg()
    (m.pose.position.x, m.pose.position.y) = p[:2]
    m.pose.orientation.z = math.sin(p[2] / 2)
    m.pose.orientation.w = math.cos(p[2] / 2)
    return m

def clearance(p):
    body = geo['xf'](FP, *p)
    return min((geo['poly_dist'](body, box) for box in boxes))

def refresh_stationary_pose():
    deadline = time.monotonic() + 3.0
    last_error = 'No fresh stationary pose'
    while time.monotonic() < deadline:
        if STOP:
            raise StageFailure('INTERRUPTED', 'Interrupted')
        spin(0.1)
        try:
            current = pose()
            tf = buf.lookup_transform('map', 'base_link', Time())
            age = (n.get_clock().now().nanoseconds - Time.from_msg(tf.header.stamp).nanoseconds) / 1000000000.0
            if not stopped():
                last_error = 'Robot not stationary or odometry not ready'
                continue
            if -0.05 <= age <= 0.2:
                return current
            last_error = f'TF age remains {age:.3f}s'
        except Exception as exc:
            last_error = str(exc)
    raise RuntimeError('Stationary refresh failed: ' + last_error)

def unchanged(reference):
    p = refresh_stationary_pose()
    if math.dist(p[:2], reference[:2]) > 0.02 or abs(wrap(p[2] - reference[2])) > 0.01:
        raise RuntimeError('Robot/map pose changed during stationary planning')

def _geometry_audit(path, target, passage):
    pts = [(p.pose.position.x, p.pose.position.y, yaw(p.pose.orientation)) for p in path.poses]
    (minimum, length, reverse, hits) = (float('inf'), 0.0, 0.0, 0)
    crossings = []
    for (p, q) in zip(pts, pts[1:]):
        (dx, dy) = (q[0] - p[0], q[1] - p[1])
        (ds, da) = (math.hypot(dx, dy), wrap(q[2] - p[2]))
        if dx * math.cos(p[2]) + dy * math.sin(p[2]) < -1e-05:
            reverse += ds
        u0 = (p[0] - cx) * nx + (p[1] - cy) * ny
        u1 = (q[0] - cx) * nx + (q[1] - cy) * ny
        if min(u0, u1) < 0 <= max(u0, u1):
            t = -u0 / (u1 - u0)
            crossings.append(-(p[0] + t * dx - cx) * ny + (p[1] + t * dy - cy) * nx)
        steps = max(1, math.ceil((ds + 1.72 * abs(da)) / 0.005))
        for i in range(steps + 1):
            t = i / steps
            clr = clearance((p[0] + t * dx, p[1] + t * dy, p[2] + t * da))
            minimum = min(clr, minimum)
            hits += clr <= 0
        length += ds
    endpoint_error = math.dist(pts[-1][:2], target[:2])
    heading_error = abs(wrap(pts[-1][2] - target[2]))
    route_ok = bool(crossings) and all((abs(v) < GAP / 2 for v in crossings)) if passage else not crossings
    print(f'AUDIT: length={length:.3f}m reverse={reverse:.3f}m minimum={minimum:.4f}m intersections={hits} endpoint_error={endpoint_error:.3f}m', flush=True)
    if minimum < PLAN_CLEARANCE or hits or reverse > 0.01 or (not route_ok) or (endpoint_error > 0.05):
        raise RuntimeError(f'Screen failed: min={minimum:.4f}, hits={hits}, reverse={reverse:.3f}, route={route_ok}, endpoint={endpoint_error:.3f}')
    return (pts, minimum, length)

def velocity_text():
    parts = []
    for key in ('NAV', 'CM', 'LIM'):
        if key not in state:
            parts.append(key + ' MISSING')
            continue
        (v, w, stamp) = state[key]
        angle = f'{math.degrees(math.atan(1.2 * w / v)):+.1f}' if abs(v) > 1e-06 else 'N/A'
        parts.append(f'{key} v={v:+.3f} w={w:+.3f} steer={angle}deg age={time.monotonic() - stamp:.2f}s')
    if 'steering' in state:
        (a, stamp) = state['steering']
        parts.append(f'MEASURED={a:+.1f}deg age={time.monotonic() - stamp:.2f}s')
    return ' | '.join(parts)
import subprocess

def world_pose(name):
    reply = subprocess.run(['gz', 'model', '-m', name, '--pose'], capture_output=True, text=True, timeout=15, check=True)
    numbers = list(map(float, reply.stdout.split()))
    if len(numbers) != 6 or not all((math.isfinite(v) for v in numbers)):
        raise RuntimeError('Invalid Gazebo pose: ' + name)
    if abs(numbers[3]) > 0.02 or abs(numbers[4]) > 0.02:
        raise RuntimeError('Unexpected model tilt: ' + name)
    return (numbers[0], numbers[1], numbers[5])

def plan_between(start, target, planner):
    goal = ComputePathToPose.Goal()
    goal.planner_id = planner
    goal.use_start = True
    goal.start = stamped(start)
    goal.goal = stamped(target)
    handle = wait(planner_client.send_goal_async(goal))
    if not handle.accepted:
        raise RuntimeError('Planning rejected')
    future = handle.get_result_async()
    try:
        result = wait(future, 30)
    except Exception:
        wait(handle.cancel_goal_async(), 5, False)
        raise
    path = result.result.path
    if result.status != 4 or len(path.poses) < 2:
        raise RuntimeError(f'Planning failed: status={result.status}')
    if path.header.frame_id != 'map':
        raise RuntimeError('Unexpected path frame')
    unchanged(anchor)
    return path
exit_status = 1
audit_grids = {}
for (key, topic) in (('local', '/local_costmap/costmap_raw'), ('global', '/global_costmap/costmap_raw')):
    n.create_subscription(Costmap, topic, lambda m, k=key: audit_grids.update({k: (m, time.monotonic())}), 10)

def grid_snapshot(moving=False):
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        if STOP:
            raise StageFailure('INTERRUPTED', 'Interrupted')
        if not moving:
            spin(0.2)
        if not moving:
            unchanged(anchor)
        if all((k in audit_grids for k in ('local', 'global'))):
            now = n.get_clock().now().nanoseconds
            if all((-0.05 <= (now - Time.from_msg(m.header.stamp).nanoseconds) / 1000000000.0 <= 4 and time.monotonic() - received <= 4 for (m, received) in audit_grids.values())):
                break
        if moving:
            raise RuntimeError('Fresh local/global raw costmaps unavailable during reverse')
    else:
        raise RuntimeError('Fresh local/global raw costmaps unavailable')
    snapshots = []
    for key in ('local', 'global'):
        (m, _) = audit_grids[key]
        md = m.metadata
        if md.resolution <= 0:
            raise RuntimeError('Invalid costmap resolution')
        (ox, oy) = (md.origin.position.x, md.origin.position.y)
        heading = yaw(md.origin.orientation)
        if m.header.frame_id != 'map':
            tf = buf.lookup_transform('map', m.header.frame_id, Time())
            angle = yaw(tf.transform.rotation)
            (tx, ty) = (tf.transform.translation.x, tf.transform.translation.y)
            (ox, oy) = (tx + math.cos(angle) * ox - math.sin(angle) * oy, ty + math.sin(angle) * ox + math.cos(angle) * oy)
            heading += angle
        grid = np.asarray(m.data, np.uint8).reshape(md.size_y, md.size_x)
        (jj, ii) = np.nonzero(grid >= 254)
        (u, v) = ((ii + 0.5) * md.resolution, (jj + 0.5) * md.resolution)
        wx = ox + math.cos(heading) * u - math.sin(heading) * v
        wy = oy + math.sin(heading) * u + math.cos(heading) * v
        snapshots.append((key, md, ox, oy, heading, wx, wy, grid[jj, ii]))
    return snapshots

def reverse_grid_audit(poses, snapshots):
    passed = True
    for (key, md, ox, oy, heading, wx, wy, costs) in snapshots:
        lethal = unknown = 0
        outside = False
        margin = md.resolution / math.sqrt(2) + 0.011
        for (x, y, a) in poses:
            body = geo['xf'](FP, x, y, a)
            for (bx, by) in body:
                (dx, dy) = (bx - ox, by - oy)
                u = dx * math.cos(heading) + dy * math.sin(heading)
                v = -dx * math.sin(heading) + dy * math.cos(heading)
                if not (0 <= u < md.size_x * md.resolution and 0 <= v < md.size_y * md.resolution):
                    outside = True
            (dx, dy) = (wx - x, wy - y)
            fx = dx * math.cos(a) + dy * math.sin(a)
            fy = -dx * math.sin(a) + dy * math.cos(a)
            touched = (fx >= -0.38 - margin) & (fx <= 1.63 + margin) & (abs(fy) <= 0.52 + margin)
            lethal += int((touched & (costs == 254)).sum())
            unknown += int((touched & (costs == 255)).sum())
        print(f'  {key}: lethal encounters={lethal}; unknown encounters={unknown}; outside grid={outside}', flush=True)
        passed = passed and (not (lethal or unknown or outside))
    return passed

def measure_scene():
    global anchor, boxes, GAP, cx, cy, ch, nx, ny, exit_pose, frame_anchor
    anchor = pose()
    robot_world = world_pose('bglx_etrike')
    pillar_world = [world_pose('pillar_test_left'), world_pose('pillar_test_right')]
    robot_after = world_pose('bglx_etrike')
    if math.dist(robot_world[:2], robot_after[:2]) > 0.01 or abs(wrap(robot_world[2] - robot_after[2])) > 0.005:
        raise RuntimeError('Robot moved during geometry query')
    if not wait_stopped():
        raise RuntimeError('Stopped odometry unavailable')
    unchanged(anchor)
    rotation = anchor[2] - robot_world[2]
    (c, s) = (math.cos(rotation), math.sin(rotation))

    def to_map(p):
        (dx, dy) = (p[0] - robot_world[0], p[1] - robot_world[1])
        return (anchor[0] + c * dx - s * dy, anchor[1] + s * dx + c * dy, wrap(p[2] + rotation))
    (left, right) = [to_map(p) for p in pillar_world]
    ch = left[2]
    if abs(wrap(right[2] - ch)) > 0.01:
        raise RuntimeError('Pillars are not parallel')
    (nx, ny) = (math.cos(ch), math.sin(ch))
    (dx, dy) = (left[0] - right[0], left[1] - right[1])
    separation = -dx * ny + dy * nx
    GAP = separation - 0.6
    if abs(dx * nx + dy * ny) > 0.03 or not 1.37 <= GAP <= 1.63:
        raise RuntimeError(f'Unexpected pillar arrangement: gap={GAP:.3f}m')
    (cx, cy) = ((left[0] + right[0]) / 2, (left[1] + right[1]) / 2)
    square = [(-0.3, -0.3), (0.3, -0.3), (0.3, 0.3), (-0.3, 0.3)]
    boxes = [geo['xf'](square, *p) for p in (left, right)]
    exit_pose = (cx + 2.0 * nx, cy + 2.0 * ny, ch)
    print(f'Existing gap={GAP:.3f}m; current clearance={clearance(anchor):.4f}m', flush=True)
    frame_anchor = frame_pose()
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue
from nav2_msgs.action import BackUp
PLAN_CLEARANCE = 0.17
original_settings = None
setting_names = ['FollowPath.use_velocity_scaled_lookahead_dist', 'FollowPath.lookahead_dist']

def frame_pose():
    t = buf.lookup_transform('map', 'odom', Time())
    return (t.transform.translation.x, t.transform.translation.y, yaw(t.transform.rotation))

def check_frame():
    f = frame_pose()
    if math.dist(f[:2], frame_anchor[:2]) > 0.025 or abs(wrap(f[2] - frame_anchor[2])) > math.radians(0.5):
        raise StageFailure('FRAME_CHANGED', 'Map/odom correction invalidates frozen box audit; stop and re-audit')

def heading_error_at(pts, p):
    best = None
    for (a, b) in zip(pts, pts[1:]):
        (dx, dy) = (b[0] - a[0], b[1] - a[1])
        d = dx * dx + dy * dy
        if d < 1e-12:
            continue
        t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / d))
        e = math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)
        if best is None or e < best[0]:
            best = (e, a[2] + t * wrap(b[2] - a[2]))
    return math.degrees(wrap(p[2] - best[1])) if best else float('nan')

def reverse_poses(start, d):
    count = max(1, math.ceil(d / 0.005))
    return [(start[0] - d * i / count * math.cos(start[2]), start[1] - d * i / count * math.sin(start[2]), start[2]) for i in range(count + 1)]

def odom_pose():
    (m, received) = state.get('odom_pose', (None, 0))
    if m is None or time.monotonic() - received > 0.3:
        raise RuntimeError('Odometry stale')
    if m.header.frame_id != 'odom' or m.child_frame_id not in ('base_link', 'base_footprint'):
        raise RuntimeError('Unexpected odometry frames')
    p = m.pose.pose
    return (p.position.x, p.position.y, yaw(p.orientation))

def escape_screen(poses):
    initial = clearance(poses[0])
    values = [clearance(p) for p in poses]
    return initial >= 0.1 and min(values) >= max(0.1, initial - 0.001) and (values[-1] >= initial + 0.05)

def mixed_metrics(path):
    pts = [(e.pose.position.x, e.pose.position.y, yaw(e.pose.orientation)) for e in path.poses]
    minimum = float('inf')
    length = reverse = 0.0
    hits = 0
    directions = []
    cusps = []
    samples = []
    for (a, b) in zip(pts, pts[1:]):
        (dx, dy) = (b[0] - a[0], b[1] - a[1])
        ds = math.hypot(dx, dy)
        da = wrap(b[2] - a[2])
        projection = dx * math.cos(a[2]) + dy * math.sin(a[2])
        direction = 1 if projection > 1e-05 else -1 if projection < -1e-05 else 0
        if direction:
            if directions and direction != directions[-1]:
                cusps.append((length, *a))
            directions.append(direction)
            if direction < 0:
                reverse += ds
        steps = max(1, math.ceil((ds + 1.72 * abs(da)) / 0.005))
        for j in range(steps + 1):
            f = j / steps
            p = (a[0] + f * dx, a[1] + f * dy, a[2] + f * da)
            clr = clearance(p)
            minimum = min(minimum, clr)
            hits += clr <= 0
            samples.append(p)
        length += ds
    return dict(pts=pts, samples=samples, minimum=minimum, length=length, reverse=reverse, hits=hits, cusps=cusps, initial=directions[0] if directions else 0)

def describe_pose(label, p):
    along = (p[0] - cx) * nx + (p[1] - cy) * ny
    lateral = -(p[0] - cx) * ny + (p[1] - cy) * nx
    print(f'{label}: map=({p[0]:.4f},{p[1]:.4f}); along={along:+.4f}m lateral={lateral:+.4f}m heading_vs_gap={math.degrees(wrap(p[2] - ch)):+.3f}deg box_clearance={clearance(p):.4f}m', flush=True)

def line_samples(a, b):
    (dx, dy) = (b[0] - a[0], b[1] - a[1])
    da = wrap(b[2] - a[2])
    count = max(1, math.ceil((math.hypot(dx, dy) + 1.72 * abs(da)) / 0.005))
    return [(a[0] + i / count * dx, a[1] + i / count * dy, a[2] + i / count * da) for i in range(count + 1)]

def inspect_path(label, path, requested):
    metrics = mixed_metrics(path)
    first = metrics['pts'][0]
    last = metrics['pts'][-1]
    print('\n' + label, flush=True)
    describe_pose('Requested start', requested)
    describe_pose('Returned first', first)
    describe_pose('Returned last', last)
    print(f'Start mismatch: position={math.dist(first[:2], requested[:2]):.4f}m heading={math.degrees(wrap(first[2] - requested[2])):+.3f}deg', flush=True)
    connector = line_samples(requested, first)
    combined = connector + metrics['samples']
    closest = min(combined, key=clearance)
    print(f"Returned path minimum={metrics['minimum']:.4f}m; including start connector={clearance(closest):.4f}m; reverse={metrics['reverse']:.3f}m; path intersection samples={metrics['hits']}", flush=True)
    describe_pose('Closest sampled body pose', closest)
    print('Start connector is a geometry check, not a validated steering manoeuvre.', flush=True)
    print('Observed occupancy for path plus start connector:', flush=True)
    reverse_grid_audit(combined, grid_snapshot())
    return last
from nav_msgs.msg import Path as NavPath

def reverse_curve(start, end, scale):
    distance = math.dist(start[:2], end[:2])
    tangent = scale * distance
    m0 = (-tangent * math.cos(start[2]), -tangent * math.sin(start[2]))
    m1 = (-tangent * math.cos(end[2]), -tangent * math.sin(end[2]))
    count = max(500, math.ceil((distance + 2 * tangent) * 400))
    poses = []
    max_curvature = 0.0
    for i in range(count + 1):
        t = i / count
        (h0, h1) = (2 * t ** 3 - 3 * t * t + 1, -2 * t ** 3 + 3 * t * t)
        (h2, h3) = (t ** 3 - 2 * t * t + t, t ** 3 - t * t)
        (d0, d1) = (6 * t * t - 6 * t, -6 * t * t + 6 * t)
        (d2, d3) = (3 * t * t - 4 * t + 1, 3 * t * t - 2 * t)
        (e0, e1) = (12 * t - 6, -12 * t + 6)
        (e2, e3) = (6 * t - 4, 6 * t - 2)
        x = h0 * start[0] + h1 * end[0] + h2 * m0[0] + h3 * m1[0]
        y = h0 * start[1] + h1 * end[1] + h2 * m0[1] + h3 * m1[1]
        dx = d0 * start[0] + d1 * end[0] + d2 * m0[0] + d3 * m1[0]
        dy = d0 * start[1] + d1 * end[1] + d2 * m0[1] + d3 * m1[1]
        ddx = e0 * start[0] + e1 * end[0] + e2 * m0[0] + e3 * m1[0]
        ddy = e0 * start[1] + e1 * end[1] + e2 * m0[1] + e3 * m1[1]
        speed_squared = dx * dx + dy * dy
        if speed_squared < 1e-10:
            raise RuntimeError('Degenerate curve')
        curvature = abs(dx * ddy - dy * ddx) / speed_squared ** 1.5
        max_curvature = max(max_curvature, curvature)
        heading = math.atan2(-dy, -dx)
        if abs(wrap(heading - ch)) > math.radians(75):
            raise RuntimeError('Curve requires excessive heading excursion')
        poses.append((x, y, heading))
    if max_curvature > 1 / 0.75:
        raise RuntimeError(f'Turning-radius check failed: minimum radius {1 / max_curvature:.3f}m < 0.75m')
    if math.dist(poses[0][:2], start[:2]) > 1e-06 or abs(wrap(poses[0][2] - start[2])) > 1e-06:
        raise RuntimeError('Start continuity failed')
    if math.dist(poses[-1][:2], end[:2]) > 1e-06 or abs(wrap(poses[-1][2] - end[2])) > 1e-06:
        raise RuntimeError('End continuity failed')
    return (poses, max_curvature)

def as_path(poses):
    path = NavPath()
    path.header.frame_id = 'map'
    path.poses = [stamped(p) for p in poses]
    return path

def execute(path, pts, checker, label, timeout):
    global active, result_future, send_future
    if STOP:
        raise StageFailure('INTERRUPTED', 'Interrupted')
    current = refresh_stationary_pose()
    if not stopped() or math.dist(current[:2], pts[0][:2]) > 0.05:
        raise RuntimeError('Robot no longer at audited start')
    goal = FollowPath.Goal()
    goal.path = path
    goal.controller_id = 'FollowPath'
    goal.goal_checker_id = checker
    print(f'\nTRIKE WILL MOVE — {label}', flush=True)
    send_future = follower.send_goal_async(goal)
    active = wait(send_future, 15)
    if not active.accepted:
        active = None
        raise RuntimeError('FollowPath rejected')
    result_future = active.get_result_async()
    start = last_print = time.monotonic()
    minimum = float('inf')
    while not result_future.done():
        if STOP:
            raise StageFailure('INTERRUPTED', 'Interrupted; cancelling')
        if time.monotonic() - start > timeout:
            raise RuntimeError('Stage timeout; cancelling')
        rclpy.spin_once(n, timeout_sec=0.05)
        check_frame()
        p = pose()
        clr = clearance(p)
        minimum = min(minimum, clr)
        tracking_error = min((geo['pt_seg'](p[:2], a[:2], b[:2]) for (a, b) in zip(pts, pts[1:])))
        if tracking_error > 0.1:
            raise StageFailure('TRACKING', f'Tracking guard: {tracking_error:.3f}m; cancelling')
        if clr < MIN_CLEARANCE:
            raise StageFailure('CLEARANCE', f'Clearance guard: {clr:.4f}m; cancelling')
        if time.monotonic() - last_print >= 0.5:
            last_print = time.monotonic()
            xte = min((geo['pt_seg'](p[:2], a[:2], b[:2]) for (a, b) in zip(pts, pts[1:])))
            print(f't={time.monotonic() - start:6.1f} clr={clr:.3f} xte={xte:.3f} heading_error={heading_error_at(pts, p):+.2f}deg | {velocity_text()}', flush=True)
    status = result_future.result().status
    stationary = wait_stopped()
    if stationary:
        active = result_future = send_future = None
    print(f'{label}: status={status}; stopped={stationary}; minimum sampled clearance={minimum:.4f}m', flush=True)
    if status != 4 or not stationary:
        raise RuntimeError('Stage did not complete successfully and stop')
    report.data['metrics'].update(minimum_clearance=minimum)

def finish_motion():
    global active, result_future, send_future
    if active is None and send_future is not None:
        candidate = wait(send_future, 15, False)
        if candidate.accepted:
            active = candidate
            result_future = active.get_result_async()
    if active is not None and active.accepted:
        if result_future is None:
            result_future = active.get_result_async()
        if not result_future.done():
            response = wait(active.cancel_goal_async(), 8, False)
            print('Cancel response:', response.return_code, flush=True)
        status = wait(result_future, 12, False).status
        print('Final action status:', status, flush=True)
        if status not in (4, 5, 6):
            raise RuntimeError('Action termination unconfirmed')
    if not wait_stopped():
        raise RuntimeError('Odometry stop unconfirmed')
    active = result_future = send_future = None
    print('Odometry confirms stopped: True', flush=True)

def update_parameters(values):
    request = SetParameters.Request()
    request.parameters = [ParameterMsg(name=name, value=value) for (name, value) in zip(changed_names, values)]
    response = service(SetParameters, '/controller_server/set_parameters', request)
    if len(response.results) != len(values) or not all((r.successful for r in response.results)):
        raise RuntimeError('Parameter update failed: ' + str(response.results))
from nav2_msgs.action import FollowPath
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue
saved = None
changed_names = ['FollowPath.allow_reversing', 'FollowPath.use_velocity_scaled_lookahead_dist', 'FollowPath.lookahead_dist', 'FollowPath.desired_linear_vel']

def forward_curve(start, target, scale):
    global ch
    original_heading = ch
    try:
        ch = wrap(ch + math.pi)
        backward_start = (*start[:2], wrap(start[2] + math.pi))
        backward_target = (*target[:2], wrap(target[2] + math.pi))
        (points, curvature) = reverse_curve(backward_start, backward_target, scale)
    finally:
        ch = original_heading
    return ([(x, y, wrap(a - math.pi)) for (x, y, a) in points], curvature)
_full_grid_audit = reverse_grid_audit

def reverse_grid_audit(poses, snapshots):
    global_maps = [s for s in snapshots if s[0] == 'global']
    local_maps = [s for s in snapshots if s[0] == 'local']
    if len(global_maps) != 1 or len(local_maps) != 1:
        raise RuntimeError('Both costmap snapshots are required')
    if len(poses) < 2:
        raise RuntimeError('Path too short to audit')
    print('GLOBAL: complete forward route', flush=True)
    if not _full_grid_audit(poses, global_maps):
        return False
    horizon = 0.5
    nearby = [poses[0]]
    distance = 0.0
    for (a, b) in zip(poses, poses[1:]):
        ds = math.dist(a[:2], b[:2])
        if distance + ds >= horizon and ds > 1e-12:
            t = (horizon - distance) / ds
            nearby.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2] + t * wrap(b[2] - a[2])))
            distance = horizon
            break
        nearby.append(b)
        distance += ds
    print(f'LOCAL: next {distance:.3f}m, including the full footprint', flush=True)
    return _full_grid_audit(nearby, local_maps)

def rank_candidates(candidates):
    from protocol import rank_candidates as rank
    return rank(candidates)
from nav_msgs.msg import Path as DisplayPath
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
_original_pose = pose

def pose():
    deadline = time.monotonic() + 0.05
    while True:
        for _ in range(8):
            if time.monotonic() >= deadline:
                break
            rclpy.spin_once(n, timeout_sec=0.0)
        try:
            return _original_pose()
        except RuntimeError as exc:
            if not str(exc).startswith('TF not fresh:') or STOP or time.monotonic() >= deadline:
                raise
            rclpy.spin_once(n, timeout_sec=0.005)
_display_execute = execute

def execute(path, pts, checker, label, timeout):
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = n.create_publisher(DisplayPath, '/supervised_gap/selected_path', qos)

    def publish_path():
        publisher.publish(path)
    publish_path()
    timer = n.create_timer(0.5, publish_path)
    print('RViz path: /supervised_gap/selected_path (nav_msgs/Path; fixed frame map)', flush=True)
    try:
        return _display_execute(path, pts, checker, label, timeout)
    finally:
        n.destroy_timer(timer)
        n.destroy_publisher(publisher)
from audit_display import install
audit = install(n, _geometry_audit)

import contextlib
import io
NAMES = ('pillar_test_left', 'pillar_test_right',
         'pillar_s2_left', 'pillar_s2_right')

def measure_four():
    global anchor, boxes, gates, pillars, frame_anchor
    anchor = refresh_stationary_pose()
    frame_anchor = frame_pose()
    robot = world_pose('bglx_etrike')
    world = [world_pose(name) for name in NAMES]
    after = world_pose('bglx_etrike')
    if (math.dist(robot[:2], after[:2]) > .01 or
            abs(wrap(robot[2]-after[2])) > .005):
        raise RuntimeError('Robot moved during Gazebo queries')
    unchanged(anchor)
    base_frame_check()
    angle = anchor[2]-robot[2]
    c, s = math.cos(angle), math.sin(angle)
    pillars = [
        (anchor[0]+c*(x-robot[0])-s*(y-robot[1]),
         anchor[1]+s*(x-robot[0])+c*(y-robot[1]), wrap(a+angle))
        for x,y,a in world
    ]
    square = [(-.3,-.3),(.3,-.3),(.3,.3),(-.3,.3)]
    boxes = [xf(square,*p) for p in pillars]
    gates = []
    for left,right in (pillars[:2],pillars[2:]):
        a = left[2]
        dx,dy = left[0]-right[0],left[1]-right[1]
        gap = -dx*math.sin(a)+dy*math.cos(a)-.6
        if (abs(wrap(right[2]-a))>.01 or
                abs(dx*math.cos(a)+dy*math.sin(a))>.03 or
                abs(gap-1.6)>.03):
            raise RuntimeError('Expected two parallel 1.6m gate pairs')
        gates.append(((left[0]+right[0])/2,
                      (left[1]+right[1])/2,a))
    print('Measured all four pillars; no spawning or deletion.',flush=True)

def gate_point(g,d):
    return (g[0]+d*math.cos(g[2]),g[1]+d*math.sin(g[2]),g[2])

def bend(a,b,scale):
    global ch
    ch = b[2]
    points,k = forward_curve(a,b,scale)
    print(f'Curve maximum curvature={k:.3f}/m',flush=True)
    return points

def course_audit(points):
    minima = [float('inf')]*4
    crossings = [[],[]]
    length = reverse = 0.
    last_spin = time.monotonic()
    for p,q in zip(points,points[1:]):
        if time.monotonic()-last_spin>.1:
            spin(.001)
            if STOP:
                raise StageFailure('INTERRUPTED','Interrupted')
            last_spin = time.monotonic()
        dx,dy = q[0]-p[0],q[1]-p[1]
        ds = math.hypot(dx,dy)
        if dx*math.cos(p[2])+dy*math.sin(p[2]) < -1e-5:
            reverse += ds
        for j,(x,y,a) in enumerate(gates):
            u = (p[0]-x)*math.cos(a)+(p[1]-y)*math.sin(a)
            v = (q[0]-x)*math.cos(a)+(q[1]-y)*math.sin(a)
            if u<0<=v:
                t = -u/(v-u)
                lateral = (-(p[0]+t*dx-x)*math.sin(a)
                           +(p[1]+t*dy-y)*math.cos(a))
                if abs(lateral)>.28:
                    raise RuntimeError('Off-centre gate crossing')
                crossings[j].append(length+t*ds)
        for sample in line_samples(p,q):
            body = xf(FP,*sample)
            minima = [min(old,poly_dist(body,b))
                      for old,b in zip(minima,boxes)]
        length += ds
    print(f'Four-box minima={minima}; length={length:.3f}m; '
          f'reverse={reverse:.3f}m',flush=True)
    if (min(minima)<.17 or reverse>.01 or
            any(len(v)!=1 for v in crossings) or
            crossings[0][0]>=crossings[1][0]):
        raise RuntimeError('Four-pillar geometry audit rejected')
    report.data['metrics'].update(
        planned_clearances=minima,length=length)

def local_check(p,snapshots):
    distances = np.sum((route_xy-np.asarray(p[:2]))**2,axis=1)
    i = int(np.argmin(distances))
    horizon = [p]
    distance = 0.
    for q in route_points[i:]:
        distance += math.dist(horizon[-1][:2],q[:2])
        horizon.append(q)
        if distance>=.5:
            break
    samples = []
    for a,b in zip(horizon,horizon[1:]):
        samples.extend(line_samples(a,b))
    local = [g for g in snapshots if g[0]=='local']
    with contextlib.redirect_stdout(io.StringIO()):
        okay = _full_grid_audit(samples,local)
    if len(local)!=1 or not okay:
        raise RuntimeError(
            'Local next-0.5m footprint sweep blocked or outside coverage')

def observe_gate(j,snapshots):
    counts = []
    for x,y,a in pillars[2*j:2*j+2]:
        best = 0
        for key,md,ox,oy,heading,wx,wy,costs in snapshots:
            dx,dy = wx-x,wy-y
            u = dx*math.cos(a)+dy*math.sin(a)
            v = -dx*math.sin(a)+dy*math.cos(a)
            best = max(best,int(
                ((abs(u)<.4)&(abs(v)<.4)&(costs==254)).sum()))
        counts.append(best)
    if min(counts)<4:
        raise RuntimeError(
            f'Gate {j+1} not sufficiently observed: {counts}; '
            'stopped before entry')

base_frame_check = check_frame
last_grid_check = 0.
def check_frame():
    global last_grid_check
    base_frame_check()
    if time.monotonic()-last_grid_check<.25:
        return
    p = pose()
    snapshots = grid_snapshot(moving=True)
    local_check(p,snapshots)
    for j,(x,y,a) in enumerate(gates):
        along = (p[0]-x)*math.cos(a)+(p[1]-y)*math.sin(a)
        if -3.5<along<-.9:
            observe_gate(j,snapshots)
    last_grid_check = time.monotonic()

try:
    print('FOUR-PILLAR MOTION: existing scene; 0.25m/s, '
          'no automatic reverse',flush=True)
    deadline = time.monotonic()+25
    while True:
        try:
            refresh_stationary_pose()
            break
        except RuntimeError:
            if STOP or time.monotonic()>deadline:
                raise

    values = parameters('/controller_server',[
        'FollowPath.use_collision_detection',
        'FollowPath.use_rotate_to_heading'])
    if (len(values)!=2 or values[0].type!=1 or
            not values[0].bool_value or values[1].type!=1 or
            values[1].bool_value):
        raise RuntimeError(
            'Require collision detection on and rotate-to-heading off')

    measure_four()
    a1 = gate_point(gates[0],-2.5)
    c1 = gate_point(gates[0],.8)
    a2 = gate_point(gates[1],-2.2)
    target = gate_point(gates[1],2.)
    if ((a1[0]-anchor[0])*math.cos(anchor[2])+
            (a1[1]-anchor[1])*math.sin(anchor[2])<.3):
        raise RuntimeError(
            'This first-run script requires the robot before gate 1 alignment')

    segments = [
        bend(anchor,a1,1.), line_samples(a1,c1),
        bend(c1,a2,1.2), line_samples(a2,target)]
    route_points = []
    for segment in segments:
        if route_points and (
                math.dist(route_points[-1][:2],segment[0][:2])>1e-6 or
                abs(wrap(route_points[-1][2]-segment[0][2]))>1e-6):
            raise RuntimeError('Route junction discontinuity')
        route_points.extend(segment if not route_points else segment[1:])
    route_xy = np.asarray([p[:2] for p in route_points])
    path = as_path(route_points)

    qos = QoSProfile(
        depth=1,reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    display = n.create_publisher(
        DisplayPath,'/supervised_gap/s_course_plan',qos)
    display.publish(path)
    display_timer = n.create_timer(.5,lambda:display.publish(path))

    course_audit(route_points)
    unchanged(anchor)
    snapshots = grid_snapshot()
    if not reverse_grid_audit(route_points,snapshots):
        raise RuntimeError('Preflight costmap audit rejected')
    observe_gate(0,snapshots)
    unchanged(anchor)
    base_frame_check()

    follower = ActionClient(n,FollowPath,'/follow_path')
    if not follower.wait_for_server(timeout_sec=10):
        raise RuntimeError('FollowPath unavailable')
    saved = parameters('/controller_server',changed_names)
    if len(saved)!=4 or [v.type for v in saved]!=[1,1,3,3]:
        saved = None
        raise RuntimeError('Unexpected controller parameter types')
    update_parameters([
        ParameterValue(type=1,bool_value=False),
        ParameterValue(type=1,bool_value=False),
        ParameterValue(type=3,double_value=.6),
        ParameterValue(type=3,double_value=.25)])

    execute(path,route_points,'general_goal_checker',
            'FOUR-PILLAR S-COURSE',180)
    finish_motion()
    final = refresh_stationary_pose()
    base_frame_check()
    error = math.dist(final[:2],target[:2])
    beyond = ((final[0]-gates[1][0])*math.cos(gates[1][2])+
              (final[1]-gates[1][1])*math.sin(gates[1][2]))
    if error>.35 or beyond<1:
        raise RuntimeError('Final course position verification failed')
    print(f'S-COURSE COMPLETE: goal error={error:.3f}m',flush=True)
    report.data['metrics']['goal_error'] = error
    report.data['code'] = 'SUCCESS'
except Exception as exc:
    report.fail(exc)
    print('COURSE STOPPED:',exc,flush=True)
finally:
    try:
        finish_motion()
        STOP = False
        if saved is not None:
            update_parameters(saved)
            print('Original controller parameters restored.',flush=True)
        report.data.update(stopped=True,cleanup_ok=True)
    except Exception as exc:
        print('CLEANUP INCOMPLETE:',exc,
              'Stop navigation if still moving.',flush=True)
        report.data.update(
            code='CLEANUP_FAILED',cleanup_ok=False,detail=str(exc))
    n.destroy_node()
    rclpy.shutdown()
report.finish()
raise SystemExit(
    0 if report.data['code']=='SUCCESS' and report.data['cleanup_ok'] else 10)
