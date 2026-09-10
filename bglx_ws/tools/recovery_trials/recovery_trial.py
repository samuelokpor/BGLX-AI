#!/usr/bin/env python3
"""Supervised Gazebo recovery stages; not an automatic Nav2 recovery plugin."""
import argparse
import math
import signal
import subprocess
import time

import rclpy
import tf2_ros
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from nav2_msgs.action import BackUp, ComputePathToPose, FollowPath
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import Log
from geometry import xf, poly_dist

FP = [(-.38,-.52),(1.63,-.52),(1.63,.52),(-.38,.52)]

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

def gz_pose(name):
    result = subprocess.run(
        ["gz","model","-m",name,"--pose"],
        capture_output=True, text=True, timeout=15, check=True)
    for line in reversed(result.stdout.splitlines()):
        try:
            v = [float(x) for x in line.split()]
        except ValueError:
            continue
        if len(v) == 6 and all(math.isfinite(x) for x in v):
            if abs(v[3]) > .02 or abs(v[4]) > .02:
                raise RuntimeError("Model not level: " + name)
            return v[0], v[1], v[5]
    raise RuntimeError("Gazebo pose unavailable: " + name)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=("reverse", "passage"))
    parser.add_argument("--execute", action="store_true",
                        help="Explicitly permit simulated motion")
    parser.add_argument("--rear-space-checked", action="store_true",
                        help="Confirm a fresh rear-space audit before reversing")
    args = parser.parse_args()
    if not args.execute:
        parser.exit(message="No motion: --execute is required. Read README.md.\n")
    if args.stage == "reverse" and not args.rear_space_checked:
        parser.error("Reverse requires --rear-space-checked after a fresh rear audit")

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop = {"requested": False}
    previous_handler = signal.signal(
        signal.SIGINT, lambda *_: stop.update(requested=True))
    node = rclpy.create_node("supervised_recovery_trial", parameter_overrides=[
        Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer, node)
    odometry, state, subscriptions = [], {}, []
    handle = result_future = send_future = None
    exit_code = 1

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)

    def wait(future, seconds=20):
        rclpy.spin_until_future_complete(node, future, timeout_sec=seconds)
        if not future.done():
            raise RuntimeError("Request timed out")
        return future.result()

    def pose():
        tf = buffer.lookup_transform("map", "base_link", Time())
        now = node.get_clock().now().nanoseconds / 1e9
        age = now - Time.from_msg(tf.header.stamp).nanoseconds / 1e9
        if now <= 0 or not -.05 <= age <= .5:
            raise RuntimeError(f"TF timing error: {age:.3f}s")
        return (tf.transform.translation.x, tf.transform.translation.y,
                yaw(tf.transform.rotation))

    def odom_cb(m):
        v = m.twist.twist
        odometry.append((time.monotonic(), v.linear.x, v.linear.y, v.angular.z))
        del odometry[:-100]

    subscriptions.append(node.create_subscription(
        Odometry, "/tricycle_steering_controller/odometry",
        odom_cb, qos_profile_sensor_data))

    def stopped():
        recent = [v for v in odometry if time.monotonic()-v[0] < 1]
        return (len(recent) >= 5 and recent[-1][0]-recent[0][0] >= .5
                and all(abs(x)<.01 and abs(y)<.01 and abs(w)<.02
                        for _,x,y,w in recent))

    def ready():
        deadline = time.monotonic() + 20
        good = 0
        while time.monotonic() < deadline and not stop["requested"]:
            spin(.2)
            try:
                pose()
                good = good+1 if stopped() else 0
            except Exception:
                good = 0
            if good >= 3:
                return
        raise RuntimeError("Cancelled or stationary robot/TF unavailable")

    for key, topic in (
        ("NAV", "/etrike/cmd_vel"),
        ("CM", "/etrike/collision_checked_cmd_vel"),
        ("LIM", "/tricycle_steering_controller/reference_unstamped")):
        subscriptions.append(node.create_subscription(
            Twist, topic,
            lambda m,k=key: state.update(
                {k: (m.linear.x,m.angular.z,time.monotonic())}), 10))

    def joints(m):
        if "steering_joint" in m.name:
            i = m.name.index("steering_joint")
            if i < len(m.position):
                state["steer"] = (math.degrees(m.position[i]),time.monotonic())

    subscriptions.append(node.create_subscription(
        JointState, "/joint_states", joints, qos_profile_sensor_data))

    def logs(m):
        if m.level >= 30 and m.name in (
            "controller_server","behavior_server","collision_monitor","cmd_vel_limiter"):
            print(f"\n[{m.name}] {m.msg}", flush=True)

    subscriptions.append(node.create_subscription(Log, "/rosout", logs, 100))

    def stamped(p):
        m = PoseStamped()
        m.header.frame_id = "map"
        m.header.stamp = node.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y = p[0], p[1]
        m.pose.orientation.z = math.sin(p[2]/2)
        m.pose.orientation.w = math.cos(p[2]/2)
        return m

    try:
        ready()
        before = gz_pose("bglx_etrike")
        wl, wr = gz_pose("pillar_test_left"), gz_pose("pillar_test_right")
        robot = gz_pose("bglx_etrike")
        if math.hypot(robot[0]-before[0],robot[1]-before[1])>.01 or \
           abs(wrap(robot[2]-before[2]))>.005:
            raise RuntimeError("Robot moved during Gazebo queries")
        ready()
        start = pose()
        rotation = wrap(start[2]-robot[2])
        c, s = math.cos(rotation), math.sin(rotation)

        def to_map(p):
            dx, dy = p[0]-robot[0], p[1]-robot[1]
            return (start[0]+c*dx-s*dy, start[1]+s*dx+c*dy,
                    wrap(p[2]+rotation))

        left, right = to_map(wl), to_map(wr)
        h = left[2]
        nx, ny = math.cos(h), math.sin(h)
        cx, cy = (left[0]+right[0])/2, (left[1]+right[1])/2
        gap = abs(-(left[0]-right[0])*ny+(left[1]-right[1])*nx)-.6
        along = abs((left[0]-right[0])*nx+(left[1]-right[1])*ny)
        if abs(gap-1.6)>.05 or along>.05 or abs(wrap(left[2]-right[2]))>.02:
            raise RuntimeError("Expected parallel 0.6m boxes with a 1.6m gap")
        boxes = [xf([(-.3,-.3),(.3,-.3),(.3,.3),(-.3,.3)],*p)
                 for p in (left,right)]

        def clearance(p):
            return min(poly_dist(xf(FP,*p),b) for b in boxes)

        if args.stage == "reverse":
            planned = min(
                clearance((start[0]-i*.005*math.cos(start[2]),
                           start[1]-i*.005*math.sin(start[2]),start[2]))
                for i in range(141))
            action = ActionClient(node, BackUp, "/backup")
            goal = BackUp.Goal()
            goal.target.x = -.7
            goal.speed = .1
            goal.time_allowance.sec = 30
            timeout = 35
        else:
            pc = node.create_client(GetParameters,"/controller_server/get_parameters")
            if not pc.wait_for_service(timeout_sec=10):
                raise RuntimeError("Controller parameters unavailable")
            request = GetParameters.Request()
            request.names = ["FollowPath.desired_linear_vel",
                             "FollowPath.use_collision_detection"]
            values = wait(pc.call_async(request)).values
            if len(values)!=2 or values[0].type!=3 or \
               not 0<values[0].double_value<=.100001:
                raise RuntimeError("Expected controller speed at or below 0.1m/s")
            if values[1].type!=1 or not values[1].bool_value:
                raise RuntimeError("Controller collision detection disabled")
            target = (cx+4*nx,cy+4*ny,h)
            planner = ActionClient(node,ComputePathToPose,"/compute_path_to_pose")
            if not planner.wait_for_server(timeout_sec=10):
                raise RuntimeError("Planner unavailable")
            request = ComputePathToPose.Goal()
            request.planner_id = "GridBased"
            request.use_start = True
            request.start, request.goal = stamped(start), stamped(target)
            ph = wait(planner.send_goal_async(request))
            if not ph.accepted:
                raise RuntimeError("Planning rejected")
            try:
                result = wait(ph.get_result_async())
            except Exception:
                wait(ph.cancel_goal_async(),5)
                raise
            path = result.result.path
            if result.status!=4 or len(path.poses)<2 or path.header.frame_id!="map":
                raise RuntimeError("No valid passage path")
            poses = [(p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation))
                     for p in path.poses]
            planned, reverse, crossings = float("inf"), 0., []
            for (x,y,a),(xx,yy,aa) in zip(poses,poses[1:]):
                dx,dy = xx-x,yy-y
                distance,change = math.hypot(dx,dy),wrap(aa-a)
                if dx*math.cos(a)+dy*math.sin(a)<-1e-5:
                    reverse += distance
                f0,f1 = (x-cx)*nx+(y-cy)*ny,(xx-cx)*nx+(yy-cy)*ny
                if min(f0,f1)<0<=max(f0,f1):
                    t = -f0/(f1-f0)
                    crossings.append(-(x+t*dx-cx)*ny+(y+t*dy-cy)*nx)
                steps = max(1,math.ceil((distance+1.72*abs(change))/.005))
                for i in range(steps+1):
                    t = i/steps
                    planned = min(planned,clearance((x+t*dx,y+t*dy,a+t*change)))
            if reverse>.01 or not crossings or any(abs(y)>=gap/2 for y in crossings):
                raise RuntimeError("Path is not forward through the gap")
            action = ActionClient(node,FollowPath,"/follow_path")
            goal = FollowPath.Goal()
            goal.path = path
            goal.controller_id = "FollowPath"
            goal.goal_checker_id = "general_goal_checker"
            timeout = 180

        print(f"Sampled box clearance: {planned:.4f}m",flush=True)
        if planned < .15:
            raise RuntimeError("Clearance audit failed; no motion")
        if not action.wait_for_server(timeout_sec=10):
            raise RuntimeError("Motion action unavailable")
        ready()
        current = pose()
        if math.hypot(current[0]-start[0],current[1]-start[1])>.02 or \
           abs(wrap(current[2]-start[2]))>.01:
            raise RuntimeError("Start pose changed")
        if stop["requested"]:
            raise RuntimeError("Cancelled before motion")
        print("TRIKE WILL MOVE: "+args.stage,flush=True)
        send_future = action.send_goal_async(goal)
        handle = wait(send_future)
        if not handle.accepted:
            raise RuntimeError("Motion goal rejected")
        result_future = handle.get_result_async()
        started = time.monotonic()
        next_print = started
        observed = float("inf")
        reason = "action completed"
        while not result_future.done():
            rclpy.spin_once(node,timeout_sec=.05)
            now = time.monotonic()
            if stop["requested"] or now-started > timeout:
                reason = "Ctrl+C" if stop["requested"] else "timeout"
                break
            current = pose()
            clr = clearance(current)
            observed = min(observed,clr)
            if clr < .15:
                reason = f"clearance guard: {clr:.4f}m"
                break
            if args.stage == "reverse":
                dx,dy = current[0]-start[0],current[1]-start[1]
                lateral = -dx*math.sin(start[2])+dy*math.cos(start[2])
                if abs(lateral)>.03 or abs(wrap(current[2]-start[2]))>math.radians(2):
                    reason = "straight-reverse tracking guard"
                    break
            if now >= next_print:
                next_print = now+.5
                fields = []
                for key in ("NAV","CM","LIM"):
                    item = state.get(key)
                    if item is None:
                        fields.append(key+" MISSING")
                    else:
                        v,w,stamp = item
                        angle = (f"{math.degrees(math.atan(1.2*w/v)):+.1f}"
                                 if abs(v)>1e-6 else "N/A")
                        fields.append(f"{key} v={v:+.3f} w={w:+.3f} "
                                      f"steer={angle} age={now-stamp:.2f}s")
                joint = state.get("steer")
                measured = f"{joint[0]:+.2f}deg" if joint else "MISSING"
                print(f"t={now-started:.1f} clearance={clr:.4f} | "+
                      " | ".join(fields)+" | MEASURED="+measured,flush=True)
        print(f"Trial ended: {reason}; minimum observed clearance={observed:.4f}m",
              flush=True)
    except Exception as exc:
        print("TRIAL ERROR:",exc,flush=True)
    finally:
        confirmed = send_future is None
        status = None
        try:
            if handle is None and send_future is not None:
                handle = wait(send_future,10)
            if handle is not None:
                if not handle.accepted:
                    confirmed = True
                else:
                    if result_future is None:
                        result_future = handle.get_result_async()
                    if not result_future.done():
                        response = wait(handle.cancel_goal_async(),5)
                        print("Cancel response:",response.return_code,flush=True)
                    status = wait(result_future,10).status
                    confirmed = status in (4,5,6)
                    print("Final action status:",status,flush=True)
            end = time.monotonic()+5
            physical_stop = False
            while time.monotonic()<end:
                spin(.2)
                if stopped():
                    physical_stop = True
                    break
            print("Odometry confirms stopped:",physical_stop,flush=True)
            if confirmed and physical_stop and status == 4:
                exit_code = 0
                if args.stage == "passage":
                    current = pose()
                    print(f"Final goal distance: "
                          f"{math.hypot(current[0]-target[0],current[1]-target[1]):.3f}m")
            if not confirmed or not physical_stop:
                print("STOP NAVIGATION LAUNCH: action/stop unconfirmed.",flush=True)
        except Exception as exc:
            print("STOP CHECK FAILED:",exc,flush=True)
            print("Stop navigation if the trike is moving.",flush=True)
        node.destroy_node()
        rclpy.shutdown()
        signal.signal(signal.SIGINT,previous_handler)
    return exit_code

if __name__ == "__main__":
    raise SystemExit(main())
