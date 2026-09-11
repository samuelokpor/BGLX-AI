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

audit_grids = {}
for key, topic in (
    ("local", "/local_costmap/costmap_raw"),
    ("global", "/global_costmap/costmap_raw"),
):
    n.create_subscription(
        Costmap, topic,
        lambda m, k=key: audit_grids.update(
            {k: (m, time.monotonic())}), 10)

def grid_snapshot():
    deadline = time.monotonic()+15
    while time.monotonic() < deadline:
        if STOP:
            raise RuntimeError("Interrupted")
        spin(.2)
        unchanged(anchor)
        if all(k in audit_grids for k in ("local", "global")):
            now = n.get_clock().now().nanoseconds
            if all(
                -.05 <= (now-Time.from_msg(m.header.stamp).nanoseconds)/1e9 <= 4
                and time.monotonic()-received <= 4
                for m, received in audit_grids.values()
            ):
                break
    else:
        raise RuntimeError("Fresh local/global raw costmaps unavailable")

    snapshots = []
    for key in ("local", "global"):
        m, _ = audit_grids[key]
        md = m.metadata
        if md.resolution <= 0:
            raise RuntimeError("Invalid costmap resolution")
        ox, oy = md.origin.position.x, md.origin.position.y
        heading = yaw(md.origin.orientation)
        if m.header.frame_id != "map":
            tf = buf.lookup_transform("map", m.header.frame_id, Time())
            angle = yaw(tf.transform.rotation)
            tx, ty = tf.transform.translation.x, tf.transform.translation.y
            ox, oy = (tx+math.cos(angle)*ox-math.sin(angle)*oy,
                      ty+math.sin(angle)*ox+math.cos(angle)*oy)
            heading += angle
        grid = np.asarray(m.data, np.uint8).reshape(md.size_y, md.size_x)
        jj, ii = np.nonzero(grid >= 254)
        u, v = (ii+.5)*md.resolution, (jj+.5)*md.resolution
        wx = ox+math.cos(heading)*u-math.sin(heading)*v
        wy = oy+math.sin(heading)*u+math.cos(heading)*v
        snapshots.append((key, md, ox, oy, heading, wx, wy, grid[jj, ii]))
    return snapshots

def reverse_grid_audit(poses, snapshots):
    passed = True
    for key, md, ox, oy, heading, wx, wy, costs in snapshots:
        lethal = unknown = 0
        outside = False
        # Conservative cell coverage plus allowance between reverse samples.
        margin = md.resolution/math.sqrt(2)+.011
        for x, y, a in poses:
            body = geo["xf"](FP, x, y, a)
            for bx, by in body:
                dx, dy = bx-ox, by-oy
                u = dx*math.cos(heading)+dy*math.sin(heading)
                v = -dx*math.sin(heading)+dy*math.cos(heading)
                if not (0 <= u < md.size_x*md.resolution
                        and 0 <= v < md.size_y*md.resolution):
                    outside = True
            dx, dy = wx-x, wy-y
            fx = dx*math.cos(a)+dy*math.sin(a)
            fy = -dx*math.sin(a)+dy*math.cos(a)
            touched = ((fx >= -.38-margin) & (fx <= 1.63+margin)
                       & (abs(fy) <= .52+margin))
            lethal += int((touched & (costs == 254)).sum())
            unknown += int((touched & (costs == 255)).sum())
        print(f"  {key}: lethal encounters={lethal}; "
              f"unknown encounters={unknown}; outside grid={outside}",
              flush=True)
        passed = passed and not (lethal or unknown or outside)
    return passed

try:
    print("RESUME EXISTING GAP — no spawning or deletion.", flush=True)
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
        raise RuntimeError("Fresh TF and stationary odometry not ready")

    values = parameters("/controller_server", [
        "FollowPath.desired_linear_vel",
        "FollowPath.use_collision_detection",
        "FollowPath.allow_reversing"])
    if not (0 < values[0].double_value <= .1001 and values[1].bool_value):
        raise RuntimeError("Require speed <=0.1m/s and collision detection enabled")

    anchor = pose()
    robot_world = world_pose("bglx_etrike")
    pillar_world = [
        world_pose("pillar_test_left"),
        world_pose("pillar_test_right")]
    robot_after = world_pose("bglx_etrike")
    if (math.dist(robot_world[:2], robot_after[:2]) > .01
            or abs(wrap(robot_world[2]-robot_after[2])) > .005):
        raise RuntimeError("Robot moved during geometry query")
    if not wait_stopped():
        raise RuntimeError("Stopped odometry unavailable")
    unchanged(anchor)

    # World -> map, anchored by the stationary robot.
    rotation = anchor[2]-robot_world[2]
    c, s = math.cos(rotation), math.sin(rotation)
    def to_map(p):
        dx, dy = p[0]-robot_world[0], p[1]-robot_world[1]
        return (anchor[0]+c*dx-s*dy, anchor[1]+s*dx+c*dy,
                wrap(p[2]+rotation))

    left, right = [to_map(p) for p in pillar_world]
    ch = left[2]
    if abs(wrap(right[2]-ch)) > .01:
        raise RuntimeError("Pillars are not parallel")
    nx, ny = math.cos(ch), math.sin(ch)
    dx, dy = left[0]-right[0], left[1]-right[1]
    separation = -dx*ny+dy*nx
    GAP = separation-.6
    if abs(dx*nx+dy*ny) > .03 or abs(GAP-1.6) > .03:
        raise RuntimeError(f"Unexpected pillar arrangement: gap={GAP:.3f}m")
    cx, cy = (left[0]+right[0])/2, (left[1]+right[1])/2
    square = [(-.3,-.3),(.3,-.3),(.3,.3),(-.3,.3)]
    boxes = [geo["xf"](square, *p) for p in (left, right)]
    exit_pose = (cx+4*nx, cy+4*ny, ch)
    print(f"Existing gap={GAP:.3f}m; current clearance="
          f"{clearance(anchor):.4f}m", flush=True)
    if clearance(anchor) < MIN_CLEARANCE:
        raise RuntimeError(
            "Current clearance is below 0.15m; forward execution withheld. "
            "A separately audited retreat is required.")

    planner_client = ActionClient(n, ComputePathToPose, "/compute_path_to_pose")
    follower = ActionClient(n, FollowPath, "/follow_path")
    if not planner_client.wait_for_server(timeout_sec=10):
        raise RuntimeError("Planner unavailable")
    if not follower.wait_for_server(timeout_sec=10):
        raise RuntimeError("Controller unavailable")


    snapshots = grid_snapshot()
    passed_distances = []
    print("\nRETREAT → FORWARD PASSAGE AUDIT — NO MOVEMENT", flush=True)
    for distance in (.3, .5, .7):
        if STOP:
            raise RuntimeError("Interrupted")
        unchanged(anchor)
        print(f"\n===== STRAIGHT REVERSE {distance:.1f}m =====", flush=True)
        steps = max(1, math.ceil(distance/.02))
        reverse_poses = [
            (anchor[0]-distance*i/steps*math.cos(anchor[2]),
             anchor[1]-distance*i/steps*math.sin(anchor[2]),
             anchor[2])
            for i in range(steps+1)]
        # Finer sampling for clearance to complete box polygons.
        count = max(1, math.ceil(distance/.005))
        reverse_min = min(clearance((
            anchor[0]-distance*i/count*math.cos(anchor[2]),
            anchor[1]-distance*i/count*math.sin(anchor[2]),
            anchor[2])) for i in range(count+1))
        print(f"Reverse minimum full-box clearance={reverse_min:.4f}m",
              flush=True)
        grids_ok = reverse_grid_audit(reverse_poses, snapshots)
        if reverse_min < MIN_CLEARANCE or not grids_ok:
            print("REVERSE REJECTED", flush=True)
            continue

        retreat_end = reverse_poses[-1]
        accepted = False
        for setback in (None, 2.0, 2.5, 3.0):
            label = "DIRECT" if setback is None else f"VIA {setback:.1f}m"
            print("Forward candidate:", label, flush=True)
            try:
                if setback is None:
                    candidate = plan_between(
                        retreat_end, exit_pose, "GridBased")
                else:
                    waypoint = (cx-setback*nx, cy-setback*ny, ch)
                    first = plan_between(
                        retreat_end, waypoint, "TightSpace")
                    audit(first, waypoint, False)
                    last = first.poses[-1].pose
                    endpoint = (last.position.x, last.position.y,
                                yaw(last.orientation))
                    second = plan_between(endpoint, exit_pose, "GridBased")
                    beginning = second.poses[0].pose
                    if (math.hypot(beginning.position.x-endpoint[0],
                                   beginning.position.y-endpoint[1]) > .03
                            or abs(wrap(yaw(beginning.orientation)-endpoint[2]))
                            > math.radians(1)):
                        raise RuntimeError("Discontinuous junction")
                    candidate = first
                    candidate.poses.extend(second.poses)
                audit(candidate, exit_pose, True)
                print(f"PASS SCREEN: reverse {distance:.1f}m → {label}",
                      flush=True)
                passed_distances.append((distance, label))
                accepted = True
                break
            except RuntimeError as exc:
                if STOP:
                    raise
                print("Rejected:", exc, flush=True)
        if not accepted:
            print("No forward continuation passed from this retreat.", flush=True)

    print("\n===== SUMMARY =====", flush=True)
    for distance, label in passed_distances:
        print(f"PASS SCREEN: {distance:.1f}m straight reverse → {label}")
    if not passed_distances:
        print("No complete candidate passed.")
    print("No motion or entity changes. Robot and pillars remain in place.")
    print("Rear checks use observed costmaps; they do not prove rear visibility.")
    print("Planning clearance does not establish execution tracking performance.")
    exit_status = 0

except Exception as exc:
    print("\nTRIAL STOPPED:", exc, flush=True)
finally:
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
        confirmed = wait_stopped()
        print("Odometry confirms stopped:", confirmed, flush=True)
        if not confirmed:
            exit_status = 1
            print("Stop the navigation launch if the trike is moving.", flush=True)
    except Exception as exc:
        exit_status = 1
        print("Termination unconfirmed:", exc,
              "— stop navigation if moving.", flush=True)
    n.destroy_node()
    rclpy.shutdown()
raise SystemExit(exit_status)
