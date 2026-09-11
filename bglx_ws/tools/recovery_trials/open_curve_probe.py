#!/usr/bin/env python3
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

FP = [(-.38, -.52), (1.63, -.52), (1.63, .52), (-.38, .52)]
GAP, OFFSET, DISTANCE = 1.6, .5, 4.
ANGLE = math.radians(15)
MIN_CLEARANCE = .15
STOP = False

source = Path.home() / "bglx_navtest/drive_trial.py"
wanted = {"xf", "edges", "intersects", "pt_seg", "poly_dist"}
funcs = [f for f in ast.parse(source.read_text()).body
         if isinstance(f, ast.FunctionDef) and f.name in wanted]
if {f.name for f in funcs} != wanted:
    raise SystemExit("Geometry helpers missing; nothing changed.")
geo = {"math": math}
exec(compile(ast.Module(body=funcs, type_ignores=[]),
             str(source), "exec"), geo)

rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
def interrupt(*_):
    global STOP
    STOP = True
signal.signal(signal.SIGINT, interrupt)

n = rclpy.create_node("staged_gap_trial", parameter_overrides=[
    Parameter("use_sim_time", Parameter.Type.BOOL, True)])
buf = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(buf, n)
state = {}
odom = []
active = result_future = send_future = None

def spin(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=.05)

def wait(future, seconds=20, interruptible=True):
    end = time.monotonic() + seconds
    while not future.done() and time.monotonic() < end:
        if interruptible and STOP:
            raise RuntimeError("Interrupted")
        rclpy.spin_once(n, timeout_sec=.05)
    if not future.done():
        raise RuntimeError("Request timed out")
    return future.result()

def service(kind, name, request):
    client = n.create_client(kind, name)
    try:
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError("Service unavailable: " + name)
        return wait(client.call_async(request))
    finally:
        n.destroy_client(client)

def parameters(node, names):
    req = GetParameters.Request()
    req.names = names
    return service(GetParameters, node + "/get_parameters", req).values

def yaw(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y),
                      1 - 2*(q.y*q.y + q.z*q.z))

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

def pose():
    tf = buf.lookup_transform("map", "base_link", Time())
    now = n.get_clock().now().nanoseconds
    age = (now - Time.from_msg(tf.header.stamp).nanoseconds)/1e9
    if now <= 0 or not -.05 <= age <= .5:
        raise RuntimeError(f"TF not fresh: age={age:.3f}s")
    p = tf.transform.translation
    return p.x, p.y, yaw(tf.transform.rotation)

def odom_cb(m):
    v = m.twist.twist
    odom.append((time.monotonic(), v.linear.x, v.linear.y, v.angular.z))
    del odom[:-300]

def stopped():
    recent = [v for v in odom if time.monotonic()-v[0] < 1.]
    return (len(recent) >= 5 and recent[-1][0]-recent[0][0] >= .5
            and all(abs(x) < .01 and abs(y) < .01 and abs(w) < .02
                    for _, x, y, w in recent))

def wait_stopped(seconds=8):
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        spin(.1)
        if stopped():
            return True
    return False

n.create_subscription(Odometry, "/tricycle_steering_controller/odometry",
                      odom_cb, qos_profile_sensor_data)
n.create_subscription(Costmap, "/global_costmap/costmap_raw",
                      lambda m: state.update(grid=m), 10)

for key, topic in (
    ("NAV", "/etrike/cmd_vel"),
    ("CM", "/etrike/collision_checked_cmd_vel"),
    ("LIM", "/tricycle_steering_controller/reference_unstamped"),
):
    n.create_subscription(
        Twist, topic,
        lambda m, k=key: state.update(
            {k: (m.linear.x, m.angular.z, time.monotonic())}), 10)

def joints(m):
    if "steering_joint" in m.name:
        i = m.name.index("steering_joint")
        if i < len(m.position):
            state["steering"] = (math.degrees(m.position[i]), time.monotonic())
n.create_subscription(JointState, "/joint_states", joints,
                      qos_profile_sensor_data)

def logs(m):
    if m.level >= 30 and m.name in (
        "controller_server", "collision_monitor", "cmd_vel_limiter"
    ):
        print(f"[{m.name}] {m.msg}", flush=True)
n.create_subscription(Log, "/rosout", logs, 100)

def stamped(p):
    m = PoseStamped()
    m.header.frame_id = "map"
    m.header.stamp = n.get_clock().now().to_msg()
    m.pose.position.x, m.pose.position.y = p[:2]
    m.pose.orientation.z = math.sin(p[2]/2)
    m.pose.orientation.w = math.cos(p[2]/2)
    return m

def clearance(p):
    body = geo["xf"](FP, *p)
    return min(geo["poly_dist"](body, box) for box in boxes)

def unchanged(reference):
    p = pose()
    if (not stopped() or math.dist(p[:2], reference[:2]) > .02
            or abs(wrap(p[2]-reference[2])) > .01):
        raise RuntimeError("Robot/map pose changed during stationary planning")

def plan(target, planner):
    start = pose()
    goal = ComputePathToPose.Goal()
    goal.planner_id = planner
    goal.use_start = True
    goal.start = stamped(start)
    goal.goal = stamped(target)
    handle = wait(planner_client.send_goal_async(goal))
    if not handle.accepted:
        raise RuntimeError("Planning rejected")
    try:
        result = wait(handle.get_result_async(), 30)
    except Exception:
        wait(handle.cancel_goal_async(), 5, False)
        raise
    unchanged(start)
    path = result.result.path
    if result.status != 4 or len(path.poses) < 2 or path.header.frame_id != "map":
        raise RuntimeError(f"No valid map path: status={result.status}")
    return path

def audit(path, target, passage):
    pts = [(p.pose.position.x, p.pose.position.y, yaw(p.pose.orientation))
           for p in path.poses]
    minimum, length, reverse, hits = float("inf"), 0., 0., 0
    crossings = []
    for p, q in zip(pts, pts[1:]):
        dx, dy = q[0]-p[0], q[1]-p[1]
        ds, da = math.hypot(dx, dy), wrap(q[2]-p[2])
        if dx*math.cos(p[2])+dy*math.sin(p[2]) < -1e-5:
            reverse += ds
        u0 = (p[0]-cx)*nx + (p[1]-cy)*ny
        u1 = (q[0]-cx)*nx + (q[1]-cy)*ny
        if min(u0, u1) < 0 <= max(u0, u1):
            t = -u0/(u1-u0)
            crossings.append(-(p[0]+t*dx-cx)*ny
                             +(p[1]+t*dy-cy)*nx)
        steps = max(1, math.ceil((ds+1.72*abs(da))/.005))
        for i in range(steps+1):
            t = i/steps
            clr = clearance((p[0]+t*dx, p[1]+t*dy, p[2]+t*da))
            minimum = min(clr, minimum)
            hits += clr <= 0
        length += ds
    endpoint_error = math.dist(pts[-1][:2], target[:2])
    heading_error = abs(wrap(pts[-1][2]-target[2]))
    route_ok = (bool(crossings) and all(abs(v) < GAP/2 for v in crossings)
                if passage else not crossings)
    print(f"AUDIT: length={length:.3f}m reverse={reverse:.3f}m "
          f"minimum={minimum:.4f}m intersections={hits} "
          f"endpoint_error={endpoint_error:.3f}m", flush=True)
    if (minimum < MIN_CLEARANCE or hits or reverse > .01 or not route_ok
            or endpoint_error > .05 or heading_error > math.radians(1)):
        raise RuntimeError("Path failed screening; no execution of this stage")
    return pts

def velocity_text():
    parts = []
    for key in ("NAV", "CM", "LIM"):
        if key not in state:
            parts.append(key+" MISSING")
            continue
        v, w, stamp = state[key]
        angle = f"{math.degrees(math.atan(1.2*w/v)):+.1f}" if abs(v) > 1e-6 else "N/A"
        parts.append(f"{key} v={v:+.3f} w={w:+.3f} steer={angle}deg "
                     f"age={time.monotonic()-stamp:.2f}s")
    if "steering" in state:
        a, stamp = state["steering"]
        parts.append(f"MEASURED={a:+.1f}deg age={time.monotonic()-stamp:.2f}s")
    return " | ".join(parts)

def execute(path, pts, checker, label, timeout):
    global active, result_future, send_future
    if STOP:
        raise RuntimeError("Interrupted")
    current = pose()
    if not stopped() or math.dist(current[:2], pts[0][:2]) > .05:
        raise RuntimeError("Robot no longer at audited start")
    goal = FollowPath.Goal()
    goal.path = path
    goal.controller_id = "FollowPath"
    goal.goal_checker_id = checker
    print(f"\nTRIKE WILL MOVE — {label}", flush=True)
    send_future = follower.send_goal_async(goal)
    active = wait(send_future, 15)
    if not active.accepted:
        active = None
        raise RuntimeError("FollowPath rejected")
    result_future = active.get_result_async()
    start = last_print = time.monotonic()
    minimum = float("inf")
    while not result_future.done():
        if STOP:
            raise RuntimeError("Interrupted; cancelling")
        if time.monotonic()-start > timeout:
            raise RuntimeError("Stage timeout; cancelling")
        rclpy.spin_once(n, timeout_sec=.05)
        p = pose()
        clr = clearance(p)
        minimum = min(minimum, clr)
        tracking_error = min(
            geo["pt_seg"](p[:2], a[:2], b[:2])
            for a, b in zip(pts, pts[1:]))
        if tracking_error > .10:
            raise RuntimeError(
                f"Tracking guard: {tracking_error:.3f}m; cancelling")
        if clr < MIN_CLEARANCE:
            raise RuntimeError(f"Clearance guard: {clr:.4f}m; cancelling")
        if time.monotonic()-last_print >= .5:
            last_print = time.monotonic()
            xte = min(geo["pt_seg"](p[:2], a[:2], b[:2])
                      for a, b in zip(pts, pts[1:]))
            print(f"t={time.monotonic()-start:6.1f} "
                  f"clr={clr:.3f} xte={xte:.3f} | {velocity_text()}",
                  flush=True)
    status = result_future.result().status
    active = result_future = send_future = None
    stationary = wait_stopped()
    print(f"{label}: status={status}; stopped={stationary}; "
          f"minimum sampled clearance={minimum:.4f}m", flush=True)
    if status != 4 or not stationary:
        raise RuntimeError("Stage did not complete successfully and stop")


import subprocess

def world_pose(name):
    reply = subprocess.run(
        ["gz", "model", "-m", name, "--pose"],
        capture_output=True, text=True, timeout=15, check=True)
    numbers = list(map(float, reply.stdout.split()))
    if len(numbers) != 6 or not all(math.isfinite(v) for v in numbers):
        raise RuntimeError("Invalid Gazebo pose: " + name)
    if abs(numbers[3]) > .02 or abs(numbers[4]) > .02:
        raise RuntimeError("Unexpected model tilt: " + name)
    return numbers[0], numbers[1], numbers[5]

def plan_between(start, target, planner):
    goal = ComputePathToPose.Goal()
    goal.planner_id = planner
    goal.use_start = True
    goal.start = stamped(start)
    goal.goal = stamped(target)
    handle = wait(planner_client.send_goal_async(goal))
    if not handle.accepted:
        raise RuntimeError("Planning rejected")
    future = handle.get_result_async()
    try:
        result = wait(future, 30)
    except Exception:
        wait(handle.cancel_goal_async(), 5, False)
        raise
    path = result.result.path
    if result.status != 4 or len(path.poses) < 2:
        raise RuntimeError(f"Planning failed: status={result.status}")
    if path.header.frame_id != "map":
        raise RuntimeError("Unexpected path frame")
    unchanged(anchor)
    return path

exit_status = 1


from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue

names = [
    "FollowPath.use_velocity_scaled_lookahead_dist",
    "FollowPath.lookahead_dist",
]
original = None

def set_values(values):
    request = SetParameters.Request()
    request.parameters = [
        ParameterMsg(name=name, value=value)
        for name, value in zip(names, values)]
    response = service(
        SetParameters, "/controller_server/set_parameters", request)
    if len(response.results) != len(names) or not all(
            result.successful for result in response.results):
        raise RuntimeError("Parameter update failed: " +
                           str([r.reason for r in response.results]))

try:
    print("OPEN CURVE PROBE — maximum 12 seconds of motion.", flush=True)
    deadline = time.monotonic()+25
    while time.monotonic() < deadline:
        spin(.2)
        try:
            pose()
            if stopped():
                break
        except Exception:
            pass
    else:
        raise RuntimeError("Stationary robot/fresh TF unavailable")

    config = parameters("/controller_server", [
        "FollowPath.desired_linear_vel",
        "FollowPath.use_collision_detection"])
    if not (0 < config[0].double_value <= .1001 and config[1].bool_value):
        raise RuntimeError("Require speed <=0.1m/s and collision detection")

    original = parameters("/controller_server", names)
    set_values([
        ParameterValue(type=1, bool_value=False),
        ParameterValue(type=3, double_value=.6)])
    print("Temporary fixed lookahead: 0.6m", flush=True)

    models = service(GetModelList, "/get_model_list", GetModelList.Request())
    if not models.success:
        raise RuntimeError("Cannot read Gazebo models")
    for name in ("pillar_test_left", "pillar_test_right"):
        if name in models.model_names:
            request = DeleteEntity.Request()
            request.name = name
            reply = service(DeleteEntity, "/delete_entity", request)
            if not reply.success:
                raise RuntimeError(reply.status_message)
            print("Deleted test pillar:", name, flush=True)

    spin(4)
    if not wait_stopped():
        raise RuntimeError("Robot not stopped")
    anchor = pose()

    # Forward-left curved route, expressed relative to the current robot.
    h = anchor[2]
    goal_pose = (
        anchor[0]+4*math.cos(h)-1.5*math.sin(h),
        anchor[1]+4*math.sin(h)+1.5*math.cos(h),
        h+math.radians(30))

    planner_client = ActionClient(n, ComputePathToPose, "/compute_path_to_pose")
    follower = ActionClient(n, FollowPath, "/follow_path")
    if not planner_client.wait_for_server(timeout_sec=10):
        raise RuntimeError("Planner unavailable")
    if not follower.wait_for_server(timeout_sec=10):
        raise RuntimeError("Controller unavailable")
    path = plan_between(anchor, goal_pose, "GridBased")
    pts = [(p.pose.position.x, p.pose.position.y, yaw(p.pose.orientation))
           for p in path.poses]

    length = reverse = 0.
    for a, b in zip(pts, pts[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        ds = math.hypot(dx, dy)
        length += ds
        if dx*math.cos(a[2])+dy*math.sin(a[2]) < -1e-5:
            reverse += ds
    if reverse > .01 or length < 2 or length > 7:
        raise RuntimeError("Unexpected route; motion withheld")
    print(f"Planned length={length:.3f}m; reverse={reverse:.3f}m", flush=True)
    unchanged(anchor)

    goal = FollowPath.Goal()
    goal.path = path
    goal.controller_id = "FollowPath"
    goal.goal_checker_id = "general_goal_checker"
    print("\nTRIKE WILL MOVE — 12-second tracking probe", flush=True)
    send_future = follower.send_goal_async(goal)
    active = wait(send_future, 15)
    if not active.accepted:
        active = send_future = None
        raise RuntimeError("Motion rejected")
    result_future = active.get_result_async()
    started = last_print = time.monotonic()
    max_xte = max_heading = 0.

    while not result_future.done():
        if STOP:
            print("Operator cancellation requested.", flush=True)
            break
        if time.monotonic()-started >= 12:
            print("Probe duration reached; cancelling normally.", flush=True)
            break
        rclpy.spin_once(n, timeout_sec=.05)
        p = pose()

        best = None
        for a, b in zip(pts, pts[1:]):
            dx, dy = b[0]-a[0], b[1]-a[1]
            squared = dx*dx+dy*dy
            if squared < 1e-12:
                continue
            t = max(0., min(1., ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/squared))
            error = math.hypot(p[0]-a[0]-t*dx, p[1]-a[1]-t*dy)
            planned_heading = a[2]+t*wrap(b[2]-a[2])
            if best is None or error < best[0]:
                best = error, planned_heading
        if best is None:
            raise RuntimeError("Degenerate path")
        xte, planned_heading = best
        heading_error = math.degrees(wrap(p[2]-planned_heading))
        max_xte = max(max_xte, xte)
        max_heading = max(max_heading, abs(heading_error))
        if xte > .10:
            raise RuntimeError(f"Tracking guard: XTE={xte:.3f}m")
        if time.monotonic()-last_print >= .5:
            last_print = time.monotonic()
            print(f"t={last_print-started:5.1f} xte={xte:.3f}m "
                  f"heading_error={heading_error:+.2f}deg | "
                  f"{velocity_text()}", flush=True)
    print(f"Peak XTE={max_xte:.4f}m; "
          f"peak heading error={max_heading:.2f}deg", flush=True)

except Exception as exc:
    print("PROBE STOPPED:", exc, flush=True)
finally:
    termination_confirmed = False
    try:
        if active is None and send_future is not None:
            candidate = wait(send_future, 10, False)
            if candidate.accepted:
                active = candidate
                result_future = candidate.get_result_async()
        if active is not None:
            if result_future is None:
                result_future = active.get_result_async()
            if not result_future.done():
                reply = wait(active.cancel_goal_async(), 8, False)
                print("Cancel response:", reply.return_code, flush=True)
            result = wait(result_future, 10, False)
            print("Final action status:", result.status, flush=True)
        termination_confirmed = True
        stationary = wait_stopped()
        print("Odometry confirms stopped:", stationary, flush=True)
        if original is not None and stationary:
            # Restoration must still work following Ctrl+C.
            STOP = False
            set_values(original)
            print("Original lookahead settings restored.", flush=True)
        elif original is not None:
            print("Lookahead remains 0.6m; stop navigation before restoring.",
                  flush=True)
        if not stationary:
            print("Stop navigation if the trike is still moving.", flush=True)
    except Exception as exc:
        print("Cleanup incomplete:", exc,
              "Stop navigation if moving; verify lookahead settings.",
              flush=True)
    n.destroy_node()
    rclpy.shutdown()
