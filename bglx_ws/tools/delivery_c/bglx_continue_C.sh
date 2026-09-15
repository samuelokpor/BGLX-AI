#!/usr/bin/env bash
# Verify/persist the existing floor extension, then DRIVE from the stopped outside pose to C.
# SIMULATION MOTION at 0.4 m/s. No pickup, unloading, or return-HOME leg.
set -eo pipefail
cd "$HOME/projects/BGLX/bglx_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tools/prototype_resize/stationary_check.py
python3 -u - <<'PY'
import ast
import difflib
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
import traceback
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import GetModelList, SpawnEntity
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

NAME = 'bglx_campus_ground_visual_v1'
ANCHOR = '<include><uri>model://ground_plane</uri></include>'
BEGIN = '<!-- BGLX_CAMPUS_GROUND_VISUAL_V1_BEGIN -->'
END = '<!-- BGLX_CAMPUS_GROUND_VISUAL_V1_END -->'
ROOT = Path.cwd()
OUT = Path.home() / 'bglx_navtest/logs' / (
    'ground_extension_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
OUT.mkdir(parents=True)

def model_xml():
    # Four tiles fill [-200,200]^2 excluding the original [-50,50]^2.
    # Coplanar tiles meet at edges only: no stacked visuals / z-fighting.
    tiles = [('north', 0, 125, 100, 150), ('south', 0, -125, 100, 150),
             ('east', 125, 0, 150, 400), ('west', -125, 0, 150, 400)]
    visuals = []
    for name, x, y, sx, sy in tiles:
        visuals.append(f'''      <visual name="{name}">
        <pose>{x} {y} 0 0 0 0</pose>
        <cast_shadows>false</cast_shadows>
        <geometry><plane><normal>0 0 1</normal><size>{sx} {sy}</size></plane></geometry>
        <material><script>
          <uri>file://media/materials/scripts/gazebo.material</uri>
          <name>Gazebo/Grey</name>
        </script></material>
      </visual>''')
    return f'''<model name="{NAME}">
  <static>true</static>
  <pose>0 0 0 0 0 0</pose>
  <link name="ground_visual_extension">
{chr(10).join(visuals)}
  </link>
</model>'''

MODEL = model_xml()
BLOCK = BEGIN + '\n' + MODEL + '\n' + END
SDF = '<sdf version="1.6">\n' + MODEL + '\n</sdf>\n'
(OUT / 'ground_visual_extension.sdf').write_text(SDF)

def patched(text):
    if BEGIN in text or END in text:
        if text.count(BLOCK) != 1:
            raise RuntimeError('An existing ground-extension block differs; refusing to overwrite it.')
        return text
    if text.count(ANCHOR) != 1:
        raise RuntimeError('Expected exactly one standard ground_plane include.')
    return text.replace(ANCHOR, ANCHOR + '\n\n' + BLOCK, 1)

def prepare_files():
    src = ROOT / 'src/etrike_description/worlds'
    installed = Path(get_package_share_directory('etrike_description')) / 'worlds'
    paths = [src / 'oxford_college.world', src / 'gen_oxford.py',
             installed / 'oxford_college.world']
    if (installed / 'gen_oxford.py').is_file():
        paths.append(installed / 'gen_oxford.py')
    staged = []
    seen = set()
    for path in paths:
        path = path.resolve(strict=True)
        if path in seen:
            continue
        seen.add(path)
        old = path.read_text()
        new = patched(old)
        if path.suffix == '.py':
            ast.parse(new, feature_version=(3, 10))
        else:
            world = ET.fromstring(new).find('world')
            if world is None or world.get('name') != 'oxford_college':
                raise RuntimeError('Unexpected world: ' + str(path))
            if len(world.findall(f"model[@name='{NAME}']")) != 1:
                raise RuntimeError('Ground extension is not a world model.')
        tag = f'{len(staged):02d}_{path.name}'
        (OUT / (tag + '.before')).write_bytes(path.read_bytes())
        (OUT / (tag + '.after')).write_text(new)
        (OUT / (tag + '.diff')).write_text(''.join(difflib.unified_diff(
            old.splitlines(True), new.splitlines(True), fromfile=str(path), tofile=str(path))))
        staged.append((path, old, new))
    return staged

def persist(staged):
    for path, old, new in staged:
        if path.read_text() != old:
            raise RuntimeError('File changed during the check: ' + str(path))
    changed = []
    try:
        for path, old, new in staged:
            if old == new:
                continue
            fd, tmp = tempfile.mkstemp(prefix=path.name + '.ground_', dir=path.parent)
            try:
                with os.fdopen(fd, 'w') as stream:
                    stream.write(new)
                shutil.copymode(path, tmp)
                os.replace(tmp, path)
                changed.append((path, old))
                print('Saved:', path, flush=True)
            finally:
                if os.path.exists(tmp):
                    os.unlink(tmp)
    except BaseException:
        for path, old in reversed(changed):
            path.write_text(old)
        raise

print('Preparing ground visibility correction. No robot movement will be requested.')
print('Backup and results:', OUT, flush=True)
staged = prepare_files()

# Read the actual active model, not a potentially different model cache.
info = subprocess.run(['gz', 'model', '-m', 'ground_plane', '-i'],
                      capture_output=True, text=True, timeout=15, check=True).stdout
(OUT / 'ground_model_before.txt').write_text(info)
planes = re.findall(r'plane\s*\{\s*normal\s*\{([^{}]+)\}\s*size\s*\{([^{}]+)\}', info)
if len(planes) < 2 or 'is_static: true' not in info:
    raise RuntimeError('Cannot verify the live standard ground plane; no model was spawned.')
for normal, size in planes:
    nums = lambda s: {k: float(v) for k, v in re.findall(r'([xyz]):\s*([-+0-9.eE]+)', s)}
    if nums(normal) != dict(x=0., y=0., z=1.) or nums(size) != dict(x=100., y=100.):
        raise RuntimeError('Live ground plane is not the expected 100 x 100 m horizontal plane.')
poses = re.findall(r'pose\s*\{\s*position\s*\{([^{}]+)\}\s*orientation\s*\{([^{}]+)\}', info)
for position, rotation in poses:
    p = {k: float(v) for k, v in re.findall(r'([xyzw]):\s*([-+0-9.eE]+)', position)}
    q = {k: float(v) for k, v in re.findall(r'([xyzw]):\s*([-+0-9.eE]+)', rotation)}
    if p != dict(x=0., y=0., z=0.) or q != dict(x=0., y=0., z=0., w=1.):
        raise RuntimeError('Live ground plane is transformed; this patch is for the origin-centred campus.')
if not poses:
    raise RuntimeError('Cannot verify the live ground pose.')

rclpy.init()
node = rclpy.create_node('bglx_ground_visual_repair', parameter_overrides=[
    Parameter('use_sim_time', Parameter.Type.BOOL, True)])
buffer = Buffer()
listener = TransformListener(buffer, node)
state = {}
events = []
subscriptions = []
phase = 'before'
result = {'passed': False, 'files_saved': False, 'live_model': NAME}

def put(key, value):
    state[key] = (time.monotonic(), value)
    if key != 'odom':
        events.append({'phase': phase, 'wall_time': time.time(),
                       'sim_time': node.get_clock().now().nanoseconds * 1e-9,
                       'topic': key, 'data': value})

def on_depth(msg):
    if msg.encoding.upper() != '32FC1':
        put('depth', {'encoding': msg.encoding, 'usable_points': 0})
        return
    d = np.frombuffer(msg.data, dtype='>f4' if msg.is_bigendian else '<f4')
    d = d.reshape(msg.height, msg.step // 4)[:, :msg.width]
    good = d[np.isfinite(d) & (d > .15) & (d < 3.999)]
    put('depth', {'stamp': [msg.header.stamp.sec, msg.header.stamp.nanosec],
                  'usable_points': int(good.size), 'total_points': int(d.size),
                  'spread_m': float(np.ptp(good)) if good.size else 0.,
                  'saturated_points': int(np.count_nonzero(d >= 3.999))})
    key = f'{phase}_depth_saved'
    if key not in state and (phase == 'before' or good.size >= 500):
        (OUT / (phase + '_depth.bin')).write_bytes(bytes(msg.data))
        (OUT / (phase + '_depth.json')).write_text(json.dumps({
            'width': msg.width, 'height': msg.height, 'step': msg.step,
            'encoding': msg.encoding, 'is_bigendian': msg.is_bigendian,
            'frame_id': msg.header.frame_id, **state['depth'][1]}, indent=2))
        state[key] = True

def on_status(msg):
    data = json.loads(msg.data)
    prior = state.get('terrain', (0, {}))[1]
    if (data.get('state'), data.get('hard_stop')) != (prior.get('state'), prior.get('hard_stop')):
        print('TERRAIN:', json.dumps(data), flush=True)
    put('terrain', data)

def subscribe(cls, topic, cb):
    subscriptions.append(node.create_subscription(cls, topic, cb, qos_profile_sensor_data))

subscribe(Image, '/etrike/front_depth/depth/image_raw', on_depth)
subscribe(String, '/etrike/terrain/status', on_status)
subscribe(Bool, '/etrike/terrain/hard_stop', lambda m: put('hard_stop', m.data))
subscribe(Odometry, '/tricycle_steering_controller/odometry',
          lambda m: put('odom', [m.twist.twist.linear.x, m.twist.twist.angular.z]))

def fresh(key, max_age=.7):
    row = state.get(key)
    return row is not None and time.monotonic() - row[0] < max_age

def stopped():
    if not fresh('odom'):
        return False
    v, w = state['odom'][1]
    return math.isfinite(v) and math.isfinite(w) and abs(v) < .02 and abs(w) < .03

def wait(future, seconds):
    end = time.monotonic() + seconds
    while rclpy.ok() and not future.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.1)
    if not future.done():
        raise RuntimeError('Gazebo service timed out; inspect model list before retrying.')
    return future.result()

def call(cls, topic, request, seconds=12):
    client = node.create_client(cls, topic)
    try:
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError('Service unavailable: ' + topic)
        return wait(client.call_async(request), seconds)
    finally:
        node.destroy_client(client)

try:
    start = time.monotonic()
    while time.monotonic() - start < 4:
        rclpy.spin_once(node, timeout_sec=.1)
    if not stopped() or not fresh('terrain') or not fresh('depth'):
        raise RuntimeError('Fresh stopped odometry, terrain state and depth are required.')
    pose = buffer.lookup_transform('map', 'base_footprint', Time()).transform.translation
    if math.hypot(pose.x - 28.0927, pose.y - 48.7199) > 2.0:
        raise RuntimeError('Robot moved away from the recorded floor-edge test pose; no live change requested.')
    result['before'] = {k: state[k][1] for k in ('terrain', 'depth', 'hard_stop') if k in state}
    listing = call(GetModelList, '/get_model_list', GetModelList.Request())
    if not listing.success or 'ground_plane' not in listing.model_names:
        raise RuntimeError('Original ground model is unavailable.')
    if NAME not in listing.model_names:
        if state['terrain'][1].get('state') != 'GROUND_FIT_FAILED' or state['depth'][1]['usable_points'] != 0:
            raise RuntimeError('The baseline no longer matches the captured floor-visibility failure.')
        req = SpawnEntity.Request()
        req.name = NAME
        req.xml = SDF
        req.reference_frame = 'world'
        req.initial_pose.orientation.w = 1.0
        print('Adding the missing floor visual beyond x/y = +/-50 m...', flush=True)
        response = call(SpawnEntity, '/spawn_entity', req)
        result['spawn'] = {'success': response.success, 'message': response.status_message}
        if not response.success:
            raise RuntimeError('Spawn failed: ' + response.status_message)
    else:
        print('Ground visual extension already exists; verifying current sensing.', flush=True)

    phase = 'after'
    print('Checking restored depth and terrain state for up to 15 seconds...', flush=True)
    deadline = time.monotonic() + 15
    clear_start = None
    last_depth_stamp = None
    clear_frames = 0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
        if not stopped():
            raise RuntimeError('Robot motion or odometry loss during stationary verification.')
        terrain = state.get('terrain', (0, {}))[1]
        depth = state.get('depth', (0, {}))[1]
        # Validate restored ground sensing, not the absence of all planning obstacles.
        # A raised feature outside the stop envelope remains Nav2's responsibility.
        # Neither the detector nor the limiter is reconfigured here.
        ordinary_ground = (terrain.get('state') in ('CLEAR', 'SLOPE')
                           and terrain.get('hazard') is False)
        distance = terrain.get('hazard_distance_m')
        stop_distance = terrain.get('hard_stop_distance_m')
        distant_positive = (terrain.get('state') == 'POSITIVE_OBSTACLE'
                            and isinstance(distance, (int, float))
                            and isinstance(stop_distance, (int, float))
                            and math.isfinite(distance) and math.isfinite(stop_distance)
                            and stop_distance > 0 and distance > stop_distance + .5)
        good = (fresh('terrain') and fresh('depth') and fresh('hard_stop')
                and state['hard_stop'][1] is False
                and (ordinary_ground or distant_positive)
                and terrain.get('hard_stop') is False
                and terrain.get('ground_points', 0) >= 300
                and depth.get('usable_points', 0) >= 500 and depth.get('spread_m', 0) > .05)
        if not good:
            clear_start, clear_frames, last_depth_stamp = None, 0, None
            continue
        if clear_start is None:
            clear_start = time.monotonic()
        if depth['stamp'] != last_depth_stamp:
            clear_frames += 1
            last_depth_stamp = depth['stamp']
        if time.monotonic() - clear_start >= 2 and clear_frames >= 10:
            result['passed'] = True
            break
    result['after'] = {k: state[k][1] for k in ('terrain', 'depth', 'hard_stop') if k in state}
    if not result['passed']:
        raise RuntimeError('Ground sensing did not pass. No world files were changed; do not resume the mission.')

    persist(staged)
    result['files_saved'] = True
    result['paths'] = [str(p) for p, _, _ in staged]
    print('\nGROUND CHECK PASS: usable depth returned; terrain hard_stop=false for 2 seconds.')
    print('World and generator saved with backups. Gazebo and the robot pose are preserved.')
    print('Ground check complete. Preparing the continuation from the current outside position.')
except KeyboardInterrupt:
    result['error'] = 'Interrupted; no navigation goal was sent. A requested model insertion may still finish.'
    print(result['error'])
except Exception as exc:
    result['error'] = str(exc)
    (OUT / 'error.txt').write_text(traceback.format_exc())
    print('GROUND CHECK FAILED:', exc, flush=True)
    print('The visual extension, if spawned, is left in place for inspection.')
finally:
    (OUT / 'summary.json').write_text(json.dumps(result, indent=2) + '\n')
    with (OUT / 'events.jsonl').open('w') as stream:
        for row in events:
            stream.write(json.dumps(row) + '\n')
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    print('Summary:', OUT / 'summary.json')
sys.exit(0 if result['passed'] and result['files_saved'] else 1)
PY

# Ground sensing and persistence passed. Now execute the authorized C continuation.
# Continue from the stopped OUTSIDE pose straight to the remaining C destination.
# This is a simulation motion command. No parcel/history or HOME leg is run.
set -eo pipefail
cd "$HOME/projects/BGLX/bglx_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tools/prototype_resize/stationary_check.py
ros2 param set /controller_server FollowPath.desired_linear_vel 0.4
ros2 param set /controller_server FollowPath.use_velocity_scaled_lookahead_dist true
mkdir -p "$HOME/bglx_navtest/logs"
run_log="$HOME/bglx_navtest/logs/continue_C_outside_$(date +%Y%m%d_%H%M%S).log"
echo "Log: $run_log"
python3 -u - <<'PY' 2>&1 | tee -i "$run_log"
import json
import math
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, String
from rclpy.signals import SignalHandlerOptions
from bglx_agentic.mission_recovery import RecoveryGuard

TREE = '''<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="FreshRouteThenNavigate">
      <ComputePathThroughPoses goals="{goals}" path="{path}" planner_id="GridBased"/>
      <RecoveryNode number_of_retries="2" name="BoundedRouteRecovery">
        <PipelineSequence name="KeepRouteAndFollow">
          <RateController hz="2.0">
            <Sequence name="UpdateRemainingPassagePoints">
              <RemovePassedGoals input_goals="{goals}" output_goals="{goals}"
                                 radius="0.6" global_frame="map"
                                 robot_base_frame="base_footprint"/>
              <Fallback name="KeepValidPath">
                <IsPathValid path="{path}" server_timeout="100"/>
                <ComputePathThroughPoses goals="{goals}" path="{path}"
                                         planner_id="GridBased"/>
              </Fallback>
            </Sequence>
          </RateController>
          <FollowPath path="{path}" controller_id="FollowPath"
                      goal_checker_id="general_goal_checker"/>
        </PipelineSequence>
        <Sequence name="WaitThenReplanRemainingRoute">
          <Wait wait_duration="1.0"/>
          <RemovePassedGoals input_goals="{goals}" output_goals="{goals}"
                             radius="0.6" global_frame="map"
                             robot_base_frame="base_footprint"/>
          <ComputePathThroughPoses goals="{goals}" path="{path}" planner_id="GridBased"/>
        </Sequence>
      </RecoveryNode>
    </Sequence>
  </BehaviorTree>
</root>
'''

run_dir = (Path.home() / 'bglx_navtest/logs' /
           ('continue_C_outside_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f')))
run_dir.mkdir(parents=True, exist_ok=False)
tree_path = run_dir / 'navigate_C_remaining.xml'
tree_path.write_text(TREE)

# Preserve the ROS context until owned actions have been cancelled/settled.
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
interrupted = False
def interrupt(signum, frame):
    global interrupted
    interrupted = True
signal.signal(signal.SIGINT, interrupt)
signal.signal(signal.SIGTERM, interrupt)

node = rclpy.create_node('bglx_continue_c_outside', parameter_overrides=[
    Parameter('use_sim_time', Parameter.Type.BOOL, True)])
node.active_goal_handle = None
node.tf_buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(node.tf_buffer, node)
map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
guard = RecoveryGuard(node, map_qos)
client = ActionClient(node, NavigateThroughPoses, '/navigate_through_poses')
report = {'destination': 'DELIVERY_C', 'action_status': None,
          'navigation_requested': False, 'success': False,
          'route': [[28.0, 82.0, 74.65]],
          'behavior_tree': str(tree_path)}
last_feedback = 0.0
last_remaining = None

terrain_state = {}
def terrain_status(msg):
    try:
        data = json.loads(msg.data)
    except ValueError:
        return
    old = terrain_state.get('status', (0, {}))[1]
    if (old.get('state'), old.get('hard_stop')) != (data.get('state'), data.get('hard_stop')):
        print('[terrain] ' + json.dumps(data), flush=True)
    terrain_state['status'] = (time.monotonic(), data)
    report['latest_terrain'] = data

def terrain_stop(msg):
    terrain_state['stop'] = (time.monotonic(), msg.data)

terrain_subscriptions = [
    node.create_subscription(String, '/etrike/terrain/status', terrain_status, qos_profile_sensor_data),
    node.create_subscription(Bool, '/etrike/terrain/hard_stop', terrain_stop, qos_profile_sensor_data),
]

def check_terrain_before_driving():
    limit = time.monotonic() + 5.0
    clear_since = None
    while time.monotonic() < limit:
        if interrupted:
            raise RuntimeError('Interrupted before navigation')
        rclpy.spin_once(node, timeout_sec=.05)
        now = time.monotonic()
        st, data = terrain_state.get('status', (0, {}))
        ht, stop = terrain_state.get('stop', (0, True))
        good = (now-st < .6 and now-ht < .6 and stop is False
                and data.get('hard_stop') is False
                and data.get('ground_points', 0) >= 300
                and data.get('state') in ('CLEAR', 'SLOPE', 'POSITIVE_OBSTACLE'))
        if not good:
            clear_since = None
        else:
            clear_since = clear_since if clear_since is not None else now
            if now-clear_since >= .6:
                print('Fresh terrain check: ground fitted; hard_stop=false.', flush=True)
                return
    raise RuntimeError('Terrain sensing/stop check did not clear; no navigation requested')


def feedback(message):
    global last_feedback, last_remaining
    f = message.feedback
    count = f.number_of_poses_remaining
    if count != last_remaining:
        if count == 1:
            print('Following the remaining route to DELIVERY_C.', flush=True)
        last_remaining = count
    if time.monotonic() - last_feedback >= 2.0:
        last_feedback = time.monotonic()
        p = f.current_pose.pose.position
        print('pose=(%.2f, %.2f) remaining=%.2fm poses_left=%d recoveries=%d' %
              (p.x, p.y, f.distance_remaining, count, f.number_of_recoveries),
              flush=True)

def own_response(future):
    # Track late acceptance too, so cleanup can await the terminal result.
    try:
        handle = future.result()
    except Exception as exc:
        report['goal_response_error'] = str(exc)
        return
    if handle is not None and handle.accepted and node.active_goal_handle is None:
        node.active_goal_handle = handle
        guard.nav_result = handle.get_result_async()

try:
    guard.ready()
    pose = guard.transform('map', 'base_footprint')
    report['start_pose_xy_yaw_rad'] = list(pose)
    if (math.hypot(pose[0]-28.0927, pose[1]-48.7199) > 2.0 or
            abs(math.atan2(math.sin(pose[2]-1.4872), math.cos(pose[2]-1.4872))) > 0.35):
        raise RuntimeError('Robot has moved from the outside terrain-check pose; no navigation requested.')

    params = node.create_client(GetParameters, '/controller_server/get_parameters')
    if not params.wait_for_service(timeout_sec=5.0):
        raise RuntimeError('Controller parameter service unavailable')
    request = GetParameters.Request()
    request.names = ['FollowPath.desired_linear_vel', 'FollowPath.allow_reversing',
                     'FollowPath.use_collision_detection']
    checked = params.call_async(request)
    rclpy.spin_until_future_complete(node, checked, timeout_sec=5.0)
    if not checked.done() or checked.result() is None:
        raise RuntimeError('Controller parameter verification timed out')
    values = checked.result().values
    if (len(values) != 3 or values[0].type != 3 or values[1].type != 1 or
            values[2].type != 1 or abs(values[0].double_value-0.4) > 1e-6 or
            values[1].bool_value or not values[2].bool_value):
        raise RuntimeError('Expected 0.4 m/s, forward-only controller and collision checking enabled')


    safety_params = node.create_client(GetParameters, '/cmd_vel_limiter/get_parameters')
    if not safety_params.wait_for_service(timeout_sec=5.0):
        raise RuntimeError('Limiter parameter service unavailable')
    safety_request = GetParameters.Request()
    safety_request.names = ['fail_closed_on_terrain_loss', 'terrain_hazard_topic']
    safety_future = safety_params.call_async(safety_request)
    rclpy.spin_until_future_complete(node, safety_future, timeout_sec=5.0)
    if not safety_future.done() or safety_future.result() is None:
        raise RuntimeError('Limiter terrain-protection verification timed out')
    sv = safety_future.result().values
    if (len(sv) != 2 or sv[0].type != 1 or not sv[0].bool_value or sv[1].type != 4
            or sv[1].string_value != '/etrike/terrain/hard_stop'):
        raise RuntimeError('Expected active terrain protection on /etrike/terrain/hard_stop')
    check_terrain_before_driving()

    if not client.wait_for_server(timeout_sec=8.0):
        raise RuntimeError('NavigateThroughPoses action unavailable')
    if interrupted:
        raise RuntimeError('Interrupted before navigation was requested')
    goal = NavigateThroughPoses.Goal()
    goal.behavior_tree = str(tree_path)
    for x, y, degrees in report['route']:
        point = PoseStamped()
        point.header.frame_id = 'map'  # Zero stamp requests latest available TF.
        point.pose.position.x, point.pose.position.y = x, y
        angle = math.radians(degrees)
        point.pose.orientation.z = math.sin(angle/2)
        point.pose.orientation.w = math.cos(angle/2)
        goal.poses.append(point)

    print('TRIKE WILL MOVE: current outside pose -> DELIVERY_C (28, 82); 0.4 m/s.', flush=True)
    print('Ctrl-C requests cancellation and waits for a confirmed stop.', flush=True)
    report['navigation_requested'] = True
    pending = client.send_goal_async(goal, feedback_callback=feedback)
    pending.add_done_callback(own_response)
    handle = guard.resolve_send(pending, 'C outside continuation', 10.0)
    if handle is None or not handle.accepted:
        raise RuntimeError('Nav2 rejected the route')
    own_response(pending)
    deadline = time.monotonic() + 600.0
    print('Goal accepted by Nav2.', flush=True)
    while rclpy.ok() and not guard.nav_result.done():
        if interrupted:
            raise RuntimeError('Operator requested cancellation')
        if time.monotonic() >= deadline:
            raise RuntimeError('C navigation exceeded 600 seconds; cancelling')
        rclpy.spin_once(node, timeout_sec=0.05)
    if not guard.nav_result.done() or guard.nav_result.result() is None:
        raise RuntimeError('Navigation ended without a terminal action result')
    report['action_status'] = guard.nav_result.result().status
    if not guard.cancel_navigation():
        raise RuntimeError('Navigation terminal state / stop not confirmed')
    if report['action_status'] != 4:
        raise RuntimeError('Nav2 finished with status %s' % report['action_status'])
    end = guard.transform('map', 'base_footprint')
    report['final_pose_xy_yaw_rad'] = list(end)
    report['goal_error_m'] = math.hypot(end[0]-28.0, end[1]-82.0)
    if report['goal_error_m'] > 0.85:
        raise RuntimeError('Final position exceeds combined planner/controller goal tolerance')
    report['success'] = True
except Exception as exc:
    report['error'] = str(exc)
    print('C NAVIGATION STOPPED:', exc, flush=True)
finally:
    settled = guard.cleanup()
    # A pending response can be accepted during RecoveryGuard.cleanup's spin.
    if guard.pending is not None and guard.pending.done():
        own_response(guard.pending)
        guard.pending = None
    if node.active_goal_handle is not None:
        settled = guard.cancel_navigation() and settled
    stopped = guard.stopped()
    report['cleanup_settled'], report['stopped_confirmed'] = settled, stopped
    report['success'] = report['success'] and settled and stopped
    if not settled or not stopped:
        print('STOP/CANCELLATION UNRESOLVED: stop the navigation launch before another motion request.', flush=True)
    (run_dir / 'summary.json').write_text(json.dumps(report, indent=2))
    print('Summary:', run_dir / 'summary.json', flush=True)
    if report['success']:
        print('ARRIVED AT C; STOP CONFIRMED. Goal error=%.3fm' % report['goal_error_m'], flush=True)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
sys.exit(0 if report['success'] else 1)
PY
