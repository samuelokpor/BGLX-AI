"""Supervised simulated alignment and passage; TRIKE MOVES."""
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
GAP, OFFSET, DISTANCE = (1.6, 0.5, 4.0)
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
            raise RuntimeError('Interrupted')
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
        raise RuntimeError(f'TF not fresh: age={age:.3f}s')
    p = tf.transform.translation
    return (p.x, p.y, yaw(tf.transform.rotation))

def odom_cb(m):
    state['odom_pose'] = (m, time.monotonic())
    v = m.twist.twist
    odom.append((time.monotonic(), v.linear.x, v.linear.y, v.angular.z))
    del odom[:-300]

def stopped():
    recent = [v for v in odom if time.monotonic() - v[0] < 1.0]
    return len(recent) >= 5 and recent[-1][0] - recent[0][0] >= 0.5 and all((abs(x) < 0.01 and abs(y) < 0.01 and (abs(w) < 0.02) for _, x, y, w in recent))

def wait_stopped(seconds=8):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        spin(0.1)
        if stopped():
            return True
    return False
n.create_subscription(Odometry, '/tricycle_steering_controller/odometry', odom_cb, qos_profile_sensor_data)
n.create_subscription(Costmap, '/global_costmap/costmap_raw', lambda m: state.update(grid=m), 10)
for key, topic in (('NAV', '/etrike/cmd_vel'), ('CM', '/etrike/collision_checked_cmd_vel'), ('LIM', '/tricycle_steering_controller/reference_unstamped')):
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
    m.pose.position.x, m.pose.position.y = p[:2]
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
            raise RuntimeError('Interrupted')
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

def audit(path, target, passage):
    pts = [(p.pose.position.x, p.pose.position.y, yaw(p.pose.orientation)) for p in path.poses]
    minimum, length, reverse, hits = (float('inf'), 0.0, 0.0, 0)
    crossings = []
    for p, q in zip(pts, pts[1:]):
        dx, dy = (q[0] - p[0], q[1] - p[1])
        ds, da = (math.hypot(dx, dy), wrap(q[2] - p[2]))
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
        v, w, stamp = state[key]
        angle = f'{math.degrees(math.atan(1.2 * w / v)):+.1f}' if abs(v) > 1e-06 else 'N/A'
        parts.append(f'{key} v={v:+.3f} w={w:+.3f} steer={angle}deg age={time.monotonic() - stamp:.2f}s')
    if 'steering' in state:
        a, stamp = state['steering']
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
for key, topic in (('local', '/local_costmap/costmap_raw'), ('global', '/global_costmap/costmap_raw')):
    n.create_subscription(Costmap, topic, lambda m, k=key: audit_grids.update({k: (m, time.monotonic())}), 10)

def grid_snapshot(moving=False):
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        if STOP:
            raise RuntimeError('Interrupted')
        if not moving:
            spin(0.2)
        if not moving:
            unchanged(anchor)
        if all((k in audit_grids for k in ('local', 'global'))):
            now = n.get_clock().now().nanoseconds
            if all((-0.05 <= (now - Time.from_msg(m.header.stamp).nanoseconds) / 1000000000.0 <= 4 and time.monotonic() - received <= 4 for m, received in audit_grids.values())):
                break
        if moving:
            raise RuntimeError('Fresh local/global raw costmaps unavailable during reverse')
    else:
        raise RuntimeError('Fresh local/global raw costmaps unavailable')
    snapshots = []
    for key in ('local', 'global'):
        m, _ = audit_grids[key]
        md = m.metadata
        if md.resolution <= 0:
            raise RuntimeError('Invalid costmap resolution')
        ox, oy = (md.origin.position.x, md.origin.position.y)
        heading = yaw(md.origin.orientation)
        if m.header.frame_id != 'map':
            tf = buf.lookup_transform('map', m.header.frame_id, Time())
            angle = yaw(tf.transform.rotation)
            tx, ty = (tf.transform.translation.x, tf.transform.translation.y)
            ox, oy = (tx + math.cos(angle) * ox - math.sin(angle) * oy, ty + math.sin(angle) * ox + math.cos(angle) * oy)
            heading += angle
        grid = np.asarray(m.data, np.uint8).reshape(md.size_y, md.size_x)
        jj, ii = np.nonzero(grid >= 254)
        u, v = ((ii + 0.5) * md.resolution, (jj + 0.5) * md.resolution)
        wx = ox + math.cos(heading) * u - math.sin(heading) * v
        wy = oy + math.sin(heading) * u + math.cos(heading) * v
        snapshots.append((key, md, ox, oy, heading, wx, wy, grid[jj, ii]))
    return snapshots

def reverse_grid_audit(poses, snapshots):
    passed = True
    for key, md, ox, oy, heading, wx, wy, costs in snapshots:
        lethal = unknown = 0
        outside = False
        margin = md.resolution / math.sqrt(2) + 0.011
        for x, y, a in poses:
            body = geo['xf'](FP, x, y, a)
            for bx, by in body:
                dx, dy = (bx - ox, by - oy)
                u = dx * math.cos(heading) + dy * math.sin(heading)
                v = -dx * math.sin(heading) + dy * math.cos(heading)
                if not (0 <= u < md.size_x * md.resolution and 0 <= v < md.size_y * md.resolution):
                    outside = True
            dx, dy = (wx - x, wy - y)
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
    c, s = (math.cos(rotation), math.sin(rotation))

    def to_map(p):
        dx, dy = (p[0] - robot_world[0], p[1] - robot_world[1])
        return (anchor[0] + c * dx - s * dy, anchor[1] + s * dx + c * dy, wrap(p[2] + rotation))
    left, right = [to_map(p) for p in pillar_world]
    ch = left[2]
    if abs(wrap(right[2] - ch)) > 0.01:
        raise RuntimeError('Pillars are not parallel')
    nx, ny = (math.cos(ch), math.sin(ch))
    dx, dy = (left[0] - right[0], left[1] - right[1])
    separation = -dx * ny + dy * nx
    GAP = separation - 0.6
    if abs(dx * nx + dy * ny) > 0.03 or not 1.37 <= GAP <= 1.63:
        raise RuntimeError(f'Unexpected pillar arrangement: gap={GAP:.3f}m')
    cx, cy = ((left[0] + right[0]) / 2, (left[1] + right[1]) / 2)
    square = [(-0.3, -0.3), (0.3, -0.3), (0.3, 0.3), (-0.3, 0.3)]
    boxes = [geo['xf'](square, *p) for p in (left, right)]
    exit_pose = (cx + 4 * nx, cy + 4 * ny, ch)
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
        raise RuntimeError('Map/odom correction invalidates frozen box audit; stop and re-audit')

def heading_error_at(pts, p):
    best = None
    for a, b in zip(pts, pts[1:]):
        dx, dy = (b[0] - a[0], b[1] - a[1])
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
    m, received = state.get('odom_pose', (None, 0))
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
    pts=[(e.pose.position.x,e.pose.position.y,yaw(e.pose.orientation)) for e in path.poses]
    minimum=float('inf');length=reverse=0.;hits=0;directions=[];cusps=[];samples=[]
    for a,b in zip(pts,pts[1:]):
        dx,dy=b[0]-a[0],b[1]-a[1]
        ds=math.hypot(dx,dy);da=wrap(b[2]-a[2])
        projection=dx*math.cos(a[2])+dy*math.sin(a[2])
        direction=1 if projection>1e-5 else -1 if projection< -1e-5 else 0
        if direction:
            if directions and direction!=directions[-1]:cusps.append((length,*a))
            directions.append(direction)
            if direction<0:reverse+=ds
        steps=max(1,math.ceil((ds+1.72*abs(da))/.005))
        for j in range(steps+1):
            f=j/steps;p=(a[0]+f*dx,a[1]+f*dy,a[2]+f*da)
            clr=clearance(p);minimum=min(minimum,clr);hits+=clr<=0
            samples.append(p)
        length+=ds
    return dict(pts=pts,samples=samples,minimum=minimum,length=length,reverse=reverse,
                hits=hits,cusps=cusps,initial=directions[0] if directions else 0)



def describe_pose(label, p):
    along = (p[0]-cx)*nx + (p[1]-cy)*ny
    lateral = -(p[0]-cx)*ny + (p[1]-cy)*nx
    print(
        f"{label}: map=({p[0]:.4f},{p[1]:.4f}); "
        f"along={along:+.4f}m lateral={lateral:+.4f}m "
        f"heading_vs_gap={math.degrees(wrap(p[2]-ch)):+.3f}deg "
        f"box_clearance={clearance(p):.4f}m", flush=True)

def line_samples(a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    da = wrap(b[2]-a[2])
    count = max(1, math.ceil((math.hypot(dx,dy)+1.72*abs(da))/.005))
    return [
        (a[0]+i/count*dx, a[1]+i/count*dy, a[2]+i/count*da)
        for i in range(count+1)
    ]

def inspect_path(label, path, requested):
    metrics = mixed_metrics(path)
    first = metrics["pts"][0]
    last = metrics["pts"][-1]
    print("\n" + label, flush=True)
    describe_pose("Requested start", requested)
    describe_pose("Returned first", first)
    describe_pose("Returned last", last)
    print(
        f"Start mismatch: position={math.dist(first[:2],requested[:2]):.4f}m "
        f"heading={math.degrees(wrap(first[2]-requested[2])):+.3f}deg",
        flush=True)

    connector = line_samples(requested, first)
    combined = connector + metrics["samples"]
    closest = min(combined, key=clearance)
    print(
        f"Returned path minimum={metrics['minimum']:.4f}m; "
        f"including start connector={clearance(closest):.4f}m; "
        f"reverse={metrics['reverse']:.3f}m; "
        f"path intersection samples={metrics['hits']}", flush=True)
    describe_pose("Closest sampled body pose", closest)
    print("Start connector is a geometry check, not a validated steering manoeuvre.",
          flush=True)
    print("Observed occupancy for path plus start connector:", flush=True)
    reverse_grid_audit(combined, grid_snapshot())
    return last

try:
    print("CORRIDOR CONSISTENCY AUDIT — NO MOVEMENT / NO CHANGES", flush=True)
    refresh_stationary_pose()
    measure_scene()
    check_frame()
    print(
        f"Known-box gap={GAP:.4f}m; "
        f"ideal aligned side clearance={(GAP-1.04)/2:.4f}m", flush=True)
    describe_pose("Current robot", anchor)

    planner_client = ActionClient(n, ComputePathToPose, "/compute_path_to_pose")
    if not planner_client.wait_for_server(timeout_sec=10):
        raise RuntimeError("Planner unavailable")

    target = (cx-2.5*nx, cy-2.5*ny, ch)
    print("\nREFERENCE CENTRELINE — hypothetical aligned start", flush=True)
    reference = line_samples(target, exit_pose)
    print(
        f"Exact straight reference minimum="
        f"{min(clearance(p) for p in reference):.4f}m", flush=True)
    reverse_grid_audit(reference, grid_snapshot())

    alignment = plan_between(anchor, target, "TightSpace")
    endpoint = inspect_path(
        "ALIGNMENT TO 2.5m SETBACK", alignment, anchor)

    unchanged(anchor)
    check_frame()
    passage = plan_between(endpoint, exit_pose, "GridBased")
    inspect_path(
        "PASSAGE FROM RETURNED ALIGNMENT ENDPOINT", passage, endpoint)

    unchanged(anchor)
    check_frame()
    ideal = plan_between(target, exit_pose, "GridBased")
    inspect_path(
        "PLANNER PASSAGE FROM EXACT CENTRED START", ideal, target)

    print("\nCOMPLETED: no motion, spawning or parameter changes.", flush=True)
    print("Costmap coverage checks are conservative; unknown/outside-grid "
          "does not prove a physical obstacle.", flush=True)
    exit_status = 0
except Exception as exc:
    print("AUDIT STOPPED:", exc, flush=True)
    exit_status = 1
finally:
    n.destroy_node()
    rclpy.shutdown()
raise SystemExit(exit_status)
