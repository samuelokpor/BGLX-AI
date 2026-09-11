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
(GAP, OFFSET, DISTANCE) = (1.6, -0.5, 4.0)
ANGLE = math.radians(-15)
MIN_CLEARANCE = 0.15
STOP = False
source = Path.home() / 'bglx_navtest/drive_trial.py'
wanted = {'xf', 'edges', 'intersects', 'pt_seg', 'poly_dist'}
funcs = [f for f in ast.parse(source.read_text()).body if isinstance(f, ast.FunctionDef) and f.name in wanted]
if {f.name for f in funcs} != wanted:
    raise SystemExit('Geometry helpers missing; nothing changed.')
geo = {'math': math}
exec(compile(ast.Module(body=funcs, type_ignores=[]), str(source), 'exec'), geo)
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

def unchanged(reference):
    p = pose()
    if not stopped() or math.dist(p[:2], reference[:2]) > 0.02 or abs(wrap(p[2] - reference[2])) > 0.01:
        raise RuntimeError('Robot/map pose changed during stationary planning')

def plan(target, planner):
    start = pose()
    goal = ComputePathToPose.Goal()
    goal.planner_id = planner
    goal.use_start = True
    goal.start = stamped(start)
    goal.goal = stamped(target)
    handle = wait(planner_client.send_goal_async(goal))
    if not handle.accepted:
        raise RuntimeError('Planning rejected')
    try:
        result = wait(handle.get_result_async(), 30)
    except Exception:
        wait(handle.cancel_goal_async(), 5, False)
        raise
    unchanged(start)
    path = result.result.path
    if result.status != 4 or len(path.poses) < 2 or path.header.frame_id != 'map':
        raise RuntimeError(f'No valid map path: status={result.status}')
    return path

def audit(path, target, passage):
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
    if minimum < MIN_CLEARANCE or hits or reverse > 0.01 or (not route_ok) or (endpoint_error > 0.05):
        reasons = []
        if minimum < MIN_CLEARANCE:
            reasons.append(f'clearance {minimum:.4f}m < {MIN_CLEARANCE:.2f}m')
        if hits:
            reasons.append(f'{hits} intersection samples')
        if reverse > 0.01:
            reasons.append(f'reverse travel {reverse:.4f}m')
        if not route_ok:
            reasons.append(f'route/crossing check; crossings={crossings}')
        if endpoint_error > 0.05:
            reasons.append(f'endpoint position error {endpoint_error:.4f}m')
        if heading_error > math.radians(1):
            reasons.append(f'endpoint heading error {math.degrees(heading_error):.3f}deg')
        raise RuntimeError('Audit rejected: ' + '; '.join(reasons))
    return pts

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

def execute(path, pts, checker, label, timeout):
    global active, result_future, send_future
    if STOP:
        raise RuntimeError('Interrupted')
    current = pose()
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
            raise RuntimeError('Interrupted; cancelling')
        if time.monotonic() - start > timeout:
            raise RuntimeError('Stage timeout; cancelling')
        rclpy.spin_once(n, timeout_sec=0.05)
        p = pose()
        clr = clearance(p)
        minimum = min(minimum, clr)
        tracking_error = min((geo['pt_seg'](p[:2], a[:2], b[:2]) for (a, b) in zip(pts, pts[1:])))
        if tracking_error > 0.1:
            raise RuntimeError(f'Tracking guard: {tracking_error:.3f}m; cancelling')
        if clr < MIN_CLEARANCE:
            raise RuntimeError(f'Clearance guard: {clr:.4f}m; cancelling')
        if time.monotonic() - last_print >= 0.5:
            last_print = time.monotonic()
            xte = min((geo['pt_seg'](p[:2], a[:2], b[:2]) for (a, b) in zip(pts, pts[1:])))
            print(f't={time.monotonic() - start:6.1f} clr={clr:.3f} xte={xte:.3f} heading_error={path_heading_error(pts, p):+.2f}deg | {velocity_text()}', flush=True)
    status = result_future.result().status
    active = result_future = send_future = None
    stationary = wait_stopped()
    print(f'{label}: status={status}; stopped={stationary}; minimum sampled clearance={minimum:.4f}m', flush=True)
    if status != 4 or not stationary:
        raise RuntimeError('Stage did not complete successfully and stop')
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
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue
lookahead_names = ['FollowPath.use_velocity_scaled_lookahead_dist', 'FollowPath.lookahead_dist']
original_settings = None

def set_lookahead(values):
    request = SetParameters.Request()
    request.parameters = [ParameterMsg(name=name, value=value) for (name, value) in zip(lookahead_names, values)]
    response = service(SetParameters, '/controller_server/set_parameters', request)
    if len(response.results) != 2 or not all((r.successful for r in response.results)):
        raise RuntimeError('Lookahead update failed: ' + str([r.reason for r in response.results]))

def path_heading_error(pts, p):
    best = None
    for (a, b) in zip(pts, pts[1:]):
        (dx, dy) = (b[0] - a[0], b[1] - a[1])
        squared = dx * dx + dy * dy
        if squared < 1e-12:
            continue
        t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / squared))
        distance = math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)
        heading = a[2] + t * wrap(b[2] - a[2])
        if best is None or distance < best[0]:
            best = (distance, heading)
    return float('nan') if best is None else math.degrees(wrap(p[2] - best[1]))
try:
    print('MIRRORED GAP 1.6m — angle -15deg, offset -0.5m.', flush=True)
    deadline = time.monotonic() + 25
    while time.monotonic() < deadline:
        spin(0.2)
        try:
            pose()
            if stopped():
                break
        except Exception:
            pass
    else:
        raise RuntimeError('Fresh TF and stationary odometry not ready')
    values = parameters('/controller_server', ['FollowPath.desired_linear_vel', 'FollowPath.use_collision_detection', 'alignment_goal_checker.xy_goal_tolerance', 'alignment_goal_checker.yaw_goal_tolerance'])
    if not (0 < values[0].double_value <= 0.1001 and values[1].bool_value and (0 < values[2].double_value <= 0.0501) and (0 < values[3].double_value <= math.radians(1.01))):
        raise RuntimeError('Speed, collision detection, or alignment checker mismatch')
    planner_client = ActionClient(n, ComputePathToPose, '/compute_path_to_pose')
    follower = ActionClient(n, FollowPath, '/follow_path')
    if not planner_client.wait_for_server(timeout_sec=10):
        raise RuntimeError('Planner unavailable')
    if not follower.wait_for_server(timeout_sec=10):
        raise RuntimeError('Controller unavailable')
    models = service(GetModelList, '/get_model_list', GetModelList.Request())
    if not models.success or 'bglx_etrike' not in models.model_names:
        raise RuntimeError('Gazebo trike unavailable')
    for name in ('box1', 'pillar_test_left', 'pillar_test_right'):
        if name in models.model_names:
            req = DeleteEntity.Request()
            req.name = name
            reply = service(DeleteEntity, '/delete_entity', req)
            if not reply.success:
                raise RuntimeError(reply.status_message)
            print('Deleted:', name, flush=True)
    spin(4)
    if not stopped():
        raise RuntimeError('Robot not stationary')
    origin = pose()
    (sx, sy, sh) = origin
    (c, s) = (math.cos(sh), math.sin(sh))
    cx = sx + DISTANCE * c - OFFSET * s
    cy = sy + DISTANCE * s + OFFSET * c
    ch = sh + ANGLE
    (nx, ny) = (math.cos(ch), math.sin(ch))
    (boxes, centres) = ([], [])
    for (name, side) in zip(('pillar_test_left', 'pillar_test_right'), (GAP / 2 + 0.3, -GAP / 2 - 0.3)):
        unchanged(origin)
        rx = DISTANCE - side * math.sin(ANGLE)
        ry = OFFSET + side * math.cos(ANGLE)
        req = SpawnEntity.Request()
        req.name = name
        req.reference_frame = 'bglx_etrike::base_link'
        req.initial_pose.position.x = rx
        req.initial_pose.position.y = ry
        req.initial_pose.position.z = 0.5
        req.initial_pose.orientation.z = math.sin(ANGLE / 2)
        req.initial_pose.orientation.w = math.cos(ANGLE / 2)
        req.xml = f'<sdf version="1.6"><model name="{name}">\n<static>true</static><link name="body">\n<collision name="collision"><geometry><box>\n<size>0.6 0.6 1</size></box></geometry></collision>\n<visual name="visual"><geometry><box><size>0.6 0.6 1</size></box></geometry>\n<material><ambient>1 0.4 0 1</ambient><diffuse>1 0.4 0 1</diffuse></material>\n</visual></link></model></sdf>'
        reply = service(SpawnEntity, '/spawn_entity', req)
        if not reply.success:
            raise RuntimeError(reply.status_message)
        (bx, by) = (cx - side * ny, cy + side * nx)
        centres.append((bx, by))
        boxes.append(geo['xf']([(-0.3, -0.3), (0.3, -0.3), (0.3, 0.3), (-0.3, 0.3)], bx, by, ch))
    spawned = n.get_clock().now().nanoseconds
    deadline = time.monotonic() + 25
    while time.monotonic() < deadline:
        spin(0.2)
        unchanged(origin)
        m = state.get('grid')
        if m is None or m.header.frame_id != 'map':
            continue
        stamp = Time.from_msg(m.header.stamp).nanoseconds
        age = (n.get_clock().now().nanoseconds - stamp) / 1000000000.0
        if stamp <= spawned or not -0.05 <= age <= 4:
            continue
        md = m.metadata
        g = np.asarray(m.data, np.uint8).reshape(md.size_y, md.size_x)
        (jj, ii) = np.nonzero(g == 254)
        a = yaw(md.origin.orientation)
        (u, v) = ((ii + 0.5) * md.resolution, (jj + 0.5) * md.resolution)
        x = md.origin.position.x + math.cos(a) * u - math.sin(a) * v
        y = md.origin.position.y + math.sin(a) * u + math.cos(a) * v
        counts = []
        for (bx, by) in centres:
            (dx, dy) = (x - bx, y - by)
            counts.append(int(((abs(dx * nx + dy * ny) < 0.4) & (abs(-dx * ny + dy * nx) < 0.4)).sum()))
        if min(counts) >= 4:
            print('Fresh pillar observations:', counts, flush=True)
            break
    else:
        raise RuntimeError('Both pillars not observed in fresh global costmap')
    anchor = pose()
    unchanged(origin)
    original_settings = parameters('/controller_server', lookahead_names)
    set_lookahead([ParameterValue(type=1, bool_value=False), ParameterValue(type=3, double_value=0.6)])
    print('Temporary fixed lookahead: 0.6m', flush=True)
    exit_pose = (cx + 4 * nx, cy + 4 * ny, ch)
    selected = selected_pts = None
    for setback in (None, 2.5, 3.0):
        if STOP:
            raise RuntimeError('Interrupted')
        label = 'DIRECT' if setback is None else f'CONTINUOUS VIA {setback:.1f}m'
        print('\nCandidate:', label, flush=True)
        try:
            if setback is None:
                candidate = plan_between(anchor, exit_pose, 'GridBased')
            else:
                waypoint = (cx - setback * nx, cy - setback * ny, ch)
                first = plan_between(anchor, waypoint, 'TightSpace')
                audit(first, waypoint, False)
                last = first.poses[-1].pose
                endpoint = (last.position.x, last.position.y, yaw(last.orientation))
                second = plan_between(endpoint, exit_pose, 'GridBased')
                beginning = second.poses[0].pose
                if math.hypot(beginning.position.x - endpoint[0], beginning.position.y - endpoint[1]) > 0.03 or abs(wrap(yaw(beginning.orientation) - endpoint[2])) > math.radians(1):
                    raise RuntimeError('Discontinuous path junction')
                candidate = first
                candidate.poses.extend(second.poses)
            points = audit(candidate, exit_pose, True)
            (selected, selected_pts) = (candidate, points)
            print('Selected:', label, flush=True)
            break
        except RuntimeError as exc:
            if STOP:
                raise
            print('Rejected:', exc, flush=True)
    if selected is None:
        raise RuntimeError('No candidate passed; no driving')
    unchanged(anchor)
    execute(selected, selected_pts, 'general_goal_checker', 'ANGLED PILLAR PASSAGE / LOOKAHEAD 0.6m', 240)
    final = pose()
    distance = math.dist(final[:2], exit_pose[:2])
    beyond = (final[0] - cx) * nx + (final[1] - cy) * ny
    if distance > 0.35 or beyond < 1.0:
        raise RuntimeError('Passage completion not verified')
    print(f'\nPASSAGE VERIFIED: goal distance={distance:.3f}m; final heading error={math.degrees(wrap(final[2] - ch)):+.2f}deg', flush=True)
except Exception as exc:
    print('\nTRIAL STOPPED:', exc, flush=True)
finally:
    try:
        if active is None and send_future is not None:
            try:
                candidate = wait(send_future, 10, False)
                if candidate.accepted:
                    active = candidate
                    result_future = candidate.get_result_async()
            except Exception:
                print('Goal acknowledgement unresolved; stop navigation if moving.', flush=True)
        if active is not None:
            if result_future is None:
                result_future = active.get_result_async()
            if not result_future.done():
                reply = wait(active.cancel_goal_async(), 8, False)
                print('Cancel response:', reply.return_code, flush=True)
            result = wait(result_future, 10, False)
            print('Final action status:', result.status, flush=True)
        confirmed = wait_stopped()
        print('Odometry confirms stopped:', confirmed, flush=True)
        if not confirmed:
            print('STOP THE NAVIGATION LAUNCH if the trike is still moving.', flush=True)
    except Exception as exc:
        print('Termination unconfirmed:', exc, '— stop navigation if the trike is still moving.', flush=True)
    if original_settings is not None:
        if (send_future is None or (send_future.done() and active is not None and (result_future is not None) and result_future.done())) and stopped():
            try:
                STOP = False
                set_lookahead(original_settings)
                print('Original lookahead settings restored.', flush=True)
            except Exception as exc:
                print('Lookahead restoration failed:', exc, flush=True)
        else:
            print('Lookahead not restored: termination/stop unconfirmed. Stop navigation and verify parameters.', flush=True)
    n.destroy_node()
    rclpy.shutdown()
