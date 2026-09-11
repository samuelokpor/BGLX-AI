import math
import time
import rclpy
import tf2_ros
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import ComputePathToPose
from gazebo_msgs.srv import GetModelList, DeleteEntity, SpawnEntity
from geometry import xf, poly_dist

FP = [(-.38,-.52),(1.63,-.52),(1.63,.52),(-.38,.52)]
GAP = 1.6
SCENE = [(5., .75, math.radians(15)),
         (10., -.75, math.radians(-15))]
NAMES = [
    ("pillar_test_left", "pillar_test_right"),
    ("pillar_s2_left", "pillar_s2_right"),
]

rclpy.init()
n = rclpy.create_node("four_pillar_audit", parameter_overrides=[
    Parameter("use_sim_time", Parameter.Type.BOOL, True)])
buf = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(buf, n)
odometry = []

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),
                      1-2*(q.y*q.y+q.z*q.z))

def odom_cb(m):
    v = m.twist.twist
    odometry.append((time.monotonic(), v.linear.x, v.linear.y, v.angular.z))
    del odometry[:-300]

n.create_subscription(
    Odometry, "/tricycle_steering_controller/odometry",
    odom_cb, qos_profile_sensor_data)

def spin(seconds):
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=.05)

def stationary():
    recent = [p for p in odometry if time.monotonic()-p[0] < 1]
    return (
        len(recent) >= 5 and recent[-1][0]-recent[0][0] >= .5
        and all(abs(x)<.01 and abs(y)<.01 and abs(w)<.02
                for _,x,y,w in recent)
    )

def pose():
    t = buf.lookup_transform("map", "base_link", Time())
    now = n.get_clock().now().nanoseconds
    age = (now-Time.from_msg(t.header.stamp).nanoseconds)/1e9
    if now <= 0 or not -.05 <= age <= .5:
        raise RuntimeError(f"TF not fresh: {age:.3f}s")
    return (t.transform.translation.x, t.transform.translation.y,
            yaw(t.transform.rotation))

def unchanged():
    spin(.2)
    p = pose()
    if not stationary() or math.dist(p[:2], start[:2]) > .02 or \
            abs(wrap(p[2]-start[2])) > .01:
        raise RuntimeError("Robot/map pose changed; audit invalid")

def wait(f, seconds=30):
    end = time.monotonic()+seconds
    while not f.done() and time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=.05)
    if not f.done():
        raise RuntimeError("Request timed out")
    return f.result()

def service(kind, name, request):
    client = n.create_client(kind, name)
    try:
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError("Unavailable: "+name)
        return wait(client.call_async(request))
    finally:
        n.destroy_client(client)

def stamped(p):
    m = PoseStamped()
    m.header.frame_id = "map"
    m.header.stamp = n.get_clock().now().to_msg()
    m.pose.position.x, m.pose.position.y = p[:2]
    m.pose.orientation.z = math.sin(p[2]/2)
    m.pose.orientation.w = math.cos(p[2]/2)
    return m

def point(p):
    return (p.pose.position.x, p.pose.position.y, yaw(p.pose.orientation))

def plan(a, b):
    goal = ComputePathToPose.Goal()
    goal.planner_id = "GridBased"
    goal.use_start = True
    goal.start, goal.goal = stamped(a), stamped(b)
    handle = wait(planner.send_goal_async(goal))
    if not handle.accepted:
        raise RuntimeError("Planning rejected")
    try:
        result = wait(handle.get_result_async())
    except Exception:
        wait(handle.cancel_goal_async(), 5)
        raise
    unchanged()
    path = result.result.path
    if result.status != 4 or path.header.frame_id != "map" or len(path.poses)<2:
        raise RuntimeError(f"Planning failed: status={result.status}")
    return path

try:
    print("FOUR-PILLAR S-COURSE — PLANNING ONLY", flush=True)
    deadline = time.monotonic()+25
    while time.monotonic() < deadline:
        spin(.2)
        try:
            start = pose()
            if stationary():
                break
        except Exception:
            pass
    else:
        raise RuntimeError("Stationary robot/TF unavailable")

    models = service(GetModelList, "/get_model_list", GetModelList.Request())
    if not models.success or "bglx_etrike" not in models.model_names:
        raise RuntimeError("Trike model unavailable")
    for name in ("box1",)+tuple(x for pair in NAMES for x in pair):
        if name in models.model_names:
            req = DeleteEntity.Request()
            req.name = name
            reply = service(DeleteEntity, "/delete_entity", req)
            if not reply.success:
                raise RuntimeError(reply.status_message)

    spin(2)
    unchanged()
    c, s = math.cos(start[2]), math.sin(start[2])
    boxes, gates = [], []

    for (gx, gy, angle), names in zip(SCENE, NAMES):
        centre = (
            start[0]+gx*c-gy*s, start[1]+gx*s+gy*c,
            wrap(start[2]+angle))
        gates.append(centre)
        for name, side in zip(names, (GAP/2+.3, -GAP/2-.3)):
            unchanged()
            x = gx-side*math.sin(angle)
            y = gy+side*math.cos(angle)
            req = SpawnEntity.Request()
            req.name = name
            req.reference_frame = "bglx_etrike::base_link"
            req.initial_pose.position.x = x
            req.initial_pose.position.y = y
            req.initial_pose.position.z = .5
            req.initial_pose.orientation.z = math.sin(angle/2)
            req.initial_pose.orientation.w = math.cos(angle/2)
            req.xml = f"""<sdf version="1.6"><model name="{name}">
<static>true</static><link name="body">
<collision name="collision"><geometry><box>
<size>0.6 0.6 1</size></box></geometry></collision>
<visual name="visual"><geometry><box>
<size>0.6 0.6 1</size></box></geometry>
<material><ambient>1 0.4 0 1</ambient>
<diffuse>1 0.4 0 1</diffuse></material></visual>
</link></model></sdf>"""
            reply = service(SpawnEntity, "/spawn_entity", req)
            if not reply.success:
                raise RuntimeError(reply.status_message)
            boxes.append(xf(
                [(-.3,-.3),(.3,-.3),(.3,.3),(-.3,.3)],
                start[0]+x*c-y*s, start[1]+x*s+y*c,
                start[2]+angle))

    print("Spawned two 1.6m gaps at 5m/+0.75m/+15deg "
          "and 10m/-0.75m/-15deg", flush=True)
    spin(3)
    unchanged()
    planner = ActionClient(n, ComputePathToPose, "/compute_path_to_pose")
    if not planner.wait_for_server(timeout_sec=10):
        raise RuntimeError("Planner unavailable")

    # Plan through the first gate's exit, then through the second gate.
    # Intermediate points constrain the audit; no driving actions are sent.
    route = Path()
    route.header = stamped(start).header
    current = start
    for gate, exit_distance in zip(gates, (1.0, 2.0)):
        gx, gy, a = gate
        target = (gx+exit_distance*math.cos(a),
                  gy+exit_distance*math.sin(a), a)
        leg = plan(current, target)
        first = point(leg.poses[0])
        if math.dist(first[:2], current[:2]) > .05 or \
                abs(wrap(first[2]-current[2])) > math.radians(3):
            raise RuntimeError("Planner start/junction mismatch")
        route.poses.extend(leg.poses)
        current = point(leg.poses[-1])

    # Include the true start in the geometric screen.
    pts = [start]+[point(p) for p in route.poses]
    minima = [float("inf")]*4
    crossings = [[], []]
    length = reverse = 0.
    hits = 0
    for p, q in zip(pts, pts[1:]):
        dx, dy = q[0]-p[0], q[1]-p[1]
        ds, da = math.hypot(dx,dy), wrap(q[2]-p[2])
        if dx*math.cos(p[2])+dy*math.sin(p[2]) < -1e-5:
            reverse += ds
        for j, (gx,gy,a) in enumerate(gates):
            nx, ny = math.cos(a), math.sin(a)
            u = (p[0]-gx)*nx+(p[1]-gy)*ny
            v = (q[0]-gx)*nx+(q[1]-gy)*ny
            if u < 0 <= v:
                t = -u/(v-u)
                lateral = -(p[0]+t*dx-gx)*ny+(p[1]+t*dy-gy)*nx
                crossings[j].append((length+t*ds, lateral))
        count = max(1, math.ceil((ds+1.72*abs(da))/.005))
        for i in range(count+1):
            t = i/count
            body = xf(FP, p[0]+t*dx, p[1]+t*dy, p[2]+t*da)
            distances = [poly_dist(body,b) for b in boxes]
            minima = [min(a,b) for a,b in zip(minima,distances)]
            hits += min(distances) <= 0
        length += ds

    valid = [[arc for arc,lat in group if abs(lat)<GAP/2]
             for group in crossings]
    ordered = bool(valid[0] and valid[1]) and min(valid[0])<min(valid[1])
    passed = ordered and reverse < .01 and not hits and min(minima) >= .17
    unchanged()
    print(f"Length={length:.3f}m; reverse={reverse:.3f}m")
    print("Minimum clearances [gate1 L/R, gate2 L/R]:",
          [round(x,4) for x in minima])
    print("Gate crossings [arc distance, lateral offset]:", crossings)
    print("Intersections:", hits, "| Both gates in order:", ordered)
    print("GEOMETRIC SCREEN:", "PASS" if passed else "REJECTED", flush=True)

    publisher = n.create_publisher(Path, "/supervised_gap/s_course_plan", 10)
    print("Publishing /supervised_gap/s_course_plan for 15 seconds.",
          flush=True)
    end = time.monotonic()+15
    while time.monotonic()<end:
        publisher.publish(route)
        spin(.2)
    print("No driving performed. Pillars remain in place.")
    print("Audit uses requested box placement and fixed starting map pose.")
    print("It does not establish sensor coverage of the distant second gate.")
finally:
    n.destroy_node()
    rclpy.shutdown()
