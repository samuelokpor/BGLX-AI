from protocol import Result
report = Result()
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
FP = [(-0.38, -0.52), (1.63, -0.52), (1.63, 0.52), (-0.38, 0.52)]
GAP = 1.6
SCENE = [(5.0, -0.75, math.radians(-15)), (10.0, 0.75, math.radians(15))]
NAMES = [('pillar_test_left', 'pillar_test_right'), ('pillar_s2_left', 'pillar_s2_right')]
rclpy.init()
n = rclpy.create_node('four_pillar_audit', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
buf = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(buf, n)
odometry = []

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

def odom_cb(m):
    v = m.twist.twist
    odometry.append((time.monotonic(), v.linear.x, v.linear.y, v.angular.z))
    del odometry[:-300]
n.create_subscription(Odometry, '/tricycle_steering_controller/odometry', odom_cb, qos_profile_sensor_data)

def spin(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.05)

def stationary():
    recent = [p for p in odometry if time.monotonic() - p[0] < 1]
    return len(recent) >= 5 and recent[-1][0] - recent[0][0] >= 0.5 and all((abs(x) < 0.01 and abs(y) < 0.01 and (abs(w) < 0.02) for (_, x, y, w) in recent))

def pose():
    t = buf.lookup_transform('map', 'base_link', Time())
    now = n.get_clock().now().nanoseconds
    age = (now - Time.from_msg(t.header.stamp).nanoseconds) / 1000000000.0
    if now <= 0 or not -0.05 <= age <= 0.5:
        raise RuntimeError(f'TF not fresh: {age:.3f}s')
    return (t.transform.translation.x, t.transform.translation.y, yaw(t.transform.rotation))

def unchanged():
    deadline = time.monotonic() + 3.0
    last_reason = 'Waiting for fresh stationary odometry'
    while time.monotonic() < deadline:
        spin(0.1)
        if not stationary():
            last_reason = 'Fresh stopped-odometry window not established'
            continue
        try:
            current = pose()
        except RuntimeError as exc:
            last_reason = str(exc)
            continue
        displacement = math.dist(current[:2], start[:2])
        heading_change = abs(wrap(current[2] - start[2]))
        if displacement > 0.02 or heading_change > 0.01:
            raise RuntimeError(f'Actual robot/map pose change: position={displacement:.4f}m, heading={math.degrees(heading_change):.3f}deg; audit invalid')
        return
    raise RuntimeError('Stationary refresh failed: ' + last_reason)

def wait(f, seconds=30):
    end = time.monotonic() + seconds
    while not f.done() and time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    if not f.done():
        raise RuntimeError('Request timed out')
    return f.result()

def service(kind, name, request):
    client = n.create_client(kind, name)
    try:
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError('Unavailable: ' + name)
        return wait(client.call_async(request))
    finally:
        n.destroy_client(client)

def stamped(p):
    m = PoseStamped()
    m.header.frame_id = 'map'
    m.header.stamp = n.get_clock().now().to_msg()
    (m.pose.position.x, m.pose.position.y) = p[:2]
    m.pose.orientation.z = math.sin(p[2] / 2)
    m.pose.orientation.w = math.cos(p[2] / 2)
    return m

def point(p):
    return (p.pose.position.x, p.pose.position.y, yaw(p.pose.orientation))

def plan(a, b):
    goal = ComputePathToPose.Goal()
    goal.planner_id = 'GridBased'
    goal.use_start = True
    (goal.start, goal.goal) = (stamped(a), stamped(b))
    handle = wait(planner.send_goal_async(goal))
    if not handle.accepted:
        raise RuntimeError('Planning rejected')
    try:
        result = wait(handle.get_result_async())
    except Exception:
        wait(handle.cancel_goal_async(), 5)
        raise
    unchanged()
    path = result.result.path
    if result.status != 4 or path.header.frame_id != 'map' or len(path.poses) < 2:
        raise RuntimeError(f'Planning failed: status={result.status}')
    return path
try:
    print('FOUR-PILLAR S-COURSE — PLANNING ONLY', flush=True)
    deadline = time.monotonic() + 25
    while time.monotonic() < deadline:
        spin(0.2)
        try:
            start = pose()
            if stationary():
                break
        except Exception:
            pass
    else:
        raise RuntimeError('Stationary robot/TF unavailable')
    models = service(GetModelList, '/get_model_list', GetModelList.Request())
    if not models.success or 'bglx_etrike' not in models.model_names:
        raise RuntimeError('Trike model unavailable')
    for name in ('box1',) + tuple((x for pair in NAMES for x in pair)):
        if name in models.model_names:
            req = DeleteEntity.Request()
            req.name = name
            reply = service(DeleteEntity, '/delete_entity', req)
            if not reply.success:
                raise RuntimeError(reply.status_message)
    spin(2)
    unchanged()
    (c, s) = (math.cos(start[2]), math.sin(start[2]))
    (boxes, gates) = ([], [])
    for ((gx, gy, angle), names) in zip(SCENE, NAMES):
        centre = (start[0] + gx * c - gy * s, start[1] + gx * s + gy * c, wrap(start[2] + angle))
        gates.append(centre)
        for (name, side) in zip(names, (GAP / 2 + 0.3, -GAP / 2 - 0.3)):
            unchanged()
            x = gx - side * math.sin(angle)
            y = gy + side * math.cos(angle)
            req = SpawnEntity.Request()
            req.name = name
            req.reference_frame = 'bglx_etrike::base_link'
            req.initial_pose.position.x = x
            req.initial_pose.position.y = y
            req.initial_pose.position.z = 0.5
            req.initial_pose.orientation.z = math.sin(angle / 2)
            req.initial_pose.orientation.w = math.cos(angle / 2)
            req.xml = f'<sdf version="1.6"><model name="{name}">\n<static>true</static><link name="body">\n<collision name="collision"><geometry><box>\n<size>0.6 0.6 1</size></box></geometry></collision>\n<visual name="visual"><geometry><box>\n<size>0.6 0.6 1</size></box></geometry>\n<material><ambient>1 0.4 0 1</ambient>\n<diffuse>1 0.4 0 1</diffuse></material></visual>\n</link></model></sdf>'
            reply = service(SpawnEntity, '/spawn_entity', req)
            if not reply.success:
                raise RuntimeError(reply.status_message)
            boxes.append(xf([(-0.3, -0.3), (0.3, -0.3), (0.3, 0.3), (-0.3, 0.3)], start[0] + x * c - y * s, start[1] + x * s + y * c, start[2] + angle))
    print('MIRROR: two 1.6m gaps at 5m/-0.75m/-15deg and 10m/+0.75m/+15deg', flush=True)
    spin(3)
    unchanged()
    report.data['code'] = 'SUCCESS'
except BaseException as exc:
    report.fail(exc)
    print('SETUP STOPPED:', exc, flush=True)
finally:
    try:
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            spin(0.1)
            if stationary():
                break
        if not stationary():
            raise RuntimeError('Setup stopped odometry unavailable')
        report.data.update(stopped=True, cleanup_ok=True)
    except BaseException as exc:
        report.data.update(code='CLEANUP_FAILED', cleanup_ok=False, detail=str(exc))
    n.destroy_node()
    rclpy.shutdown()
    report.finish()
raise SystemExit(0 if report.data['code'] == 'SUCCESS' and report.data['cleanup_ok'] else 10)
