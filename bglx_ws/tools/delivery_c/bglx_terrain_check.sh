#!/usr/bin/env bash
# Stationary terrain investigation. No motion, cancellation, or parameter writes.
set -eo pipefail
cd "$HOME/projects/BGLX/bglx_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -u - <<'PY'
import hashlib
import importlib.util
import json
import shutil
import subprocess
import time
import traceback
from collections import Counter
from datetime import datetime
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import GetModelList
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Bool, Float32, String
from tf2_ros import Buffer, TransformListener

stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
out = Path.home() / 'bglx_navtest/logs' / ('terrain_check_' + stamp)
out.mkdir(parents=True)
manifest = {'read_only': True, 'files': {}, 'errors': []}
params = {}

def save_json(name, value):
    (out / name).write_text(json.dumps(value, indent=2, default=str) + '\n')

def copy_source(path, name):
    path = Path(path)
    if path.is_file():
        shutil.copyfile(path, out / name)
        manifest['files'][name] = {
            'source': str(path.resolve()),
            'sha256': hashlib.sha256((out / name).read_bytes()).hexdigest()}

print('READ-ONLY TERRAIN CAPTURE. Leave Gazebo running and the trike at its stopped pose.')
print('Output:', out)
for node_name in ('/terrain_detector', '/cmd_vel_limiter'):
    print('Reading parameters:', node_name, flush=True)
    try:
        result = subprocess.run(['ros2', 'param', 'dump', node_name],
                                capture_output=True, text=True, timeout=15)
        (out / (node_name.strip('/') + '_params.txt')).write_text(
            result.stdout + '\n' + result.stderr)
        if result.returncode:
            raise RuntimeError(result.stderr or result.stdout)
        data = yaml.safe_load(result.stdout)
        params[node_name] = next(iter(data.values()))['ros__parameters']
    except Exception as exc:
        manifest['errors'].append(f'{node_name}: {exc}')
        print('Parameter capture unavailable:', exc)

for module in ('terrain_detector', 'cmd_vel_limiter'):
    try:
        spec = importlib.util.find_spec('bglx_navigation.' + module)
        if spec and spec.origin:
            copy_source(spec.origin, 'installed_' + module + '.py')
    except Exception as exc:
        manifest['errors'].append(str(exc))
for package, rel in (
    ('bglx_navigation', 'config/terrain_detector.yaml'),
    ('etrike_description', 'urdf/etrike.urdf.xacro'),
    ('etrike_description', 'worlds/oxford_college.world'),
):
    try:
        copy_source(Path(get_package_share_directory(package)) / rel,
                    'installed_' + Path(rel).name)
        copy_source(Path('src') / package / rel, 'source_' + Path(rel).name)
    except Exception as exc:
        manifest['errors'].append(str(exc))

tp = params.get('/terrain_detector', {})
lp = params.get('/cmd_vel_limiter', {})
depth_topic = tp.get('depth_topic', '/etrike/front_depth/depth/image_raw')
info_topic = tp.get('camera_info_topic', '/etrike/front_depth/depth/camera_info')
base_frame = tp.get('base_frame', 'base_link')
guard_topic = lp.get('terrain_hazard_topic', '/etrike/terrain/hard_stop')
manifest['topics'] = dict(depth=depth_topic, camera_info=info_topic,
                          limiter_terrain_input=guard_topic)
manifest['base_frame'] = base_frame
records, latest, images = [], {}, []
counts = Counter()
odom_max = {'abs_linear_x': 0.0, 'abs_angular_z': 0.0}
last_depth_wall = -float('inf')
last_status_wall = -float('inf')
last_state = None
rclpy.init()
node = rclpy.create_node('bglx_stationary_terrain_check', parameter_overrides=[
    Parameter('use_sim_time', Parameter.Type.BOOL, True)])
buffer = Buffer()
listener = TransformListener(buffer, node)
subscriptions = []

def record(topic, data):
    counts[topic] += 1
    latest[topic] = data
    records.append({'topic': topic, 'wall_time': time.time(),
                    'sim_time': node.get_clock().now().nanoseconds * 1e-9,
                    'data': data})

def status(msg):
    global last_status_wall, last_state
    try:
        data = json.loads(msg.data)
    except ValueError:
        data = {'unparsed': msg.data}
    record('/etrike/terrain/status', data)
    now = time.monotonic()
    state = (data.get('state'), data.get('hard_stop'))
    if state != last_state or now - last_status_wall >= 3.0:
        print('TERRAIN:', json.dumps(data), flush=True)
        last_status_wall, last_state = now, state

def depth(msg):
    global last_depth_wall
    counts[depth_topic] += 1
    if len(images) >= 5 or time.monotonic() - last_depth_wall < 1.0:
        return
    last_depth_wall = time.monotonic()
    images.append((msg, node.get_clock().now().nanoseconds * 1e-9))

def odom(msg):
    odom_max['abs_linear_x'] = max(odom_max['abs_linear_x'], abs(msg.twist.twist.linear.x))
    odom_max['abs_angular_z'] = max(odom_max['abs_angular_z'], abs(msg.twist.twist.angular.z))
    record('/tricycle_steering_controller/odometry', message_to_ordereddict(msg))

def rosout(msg):
    if msg.level >= 20 and any(s in msg.name for s in ('terrain', 'cmd_vel_limiter')):
        record('/rosout', message_to_ordereddict(msg))

def sub(kind, topic, callback):
    subscriptions.append(node.create_subscription(kind, topic, callback,
                                                  qos_profile_sensor_data))

sub(String, '/etrike/terrain/status', status)
for topic in set(('/etrike/terrain/hard_stop', '/etrike/terrain/hazard', guard_topic)):
    sub(Bool, topic, lambda msg, t=topic: record(t, msg.data))
sub(Float32, '/etrike/terrain/hazard_distance',
    lambda msg: record('/etrike/terrain/hazard_distance', msg.data))
sub(Image, depth_topic, depth)
sub(CameraInfo, info_topic, lambda msg: record(info_topic, message_to_ordereddict(msg)))
sub(LaserScan, '/etrike/terrain/scan',
    lambda msg: record('/etrike/terrain/scan', message_to_ordereddict(msg)))
sub(Odometry, '/tricycle_steering_controller/odometry', odom)
sub(Log, '/rosout', rosout)
models_client = node.create_client(GetModelList, '/get_model_list')
models_future = None

try:
    print('Sampling terrain decisions, depth and odometry for 12 wall seconds...', flush=True)
    until = time.monotonic() + 12.0
    while rclpy.ok() and time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=0.1)
        if models_future is None and models_client.service_is_ready():
            models_future = models_client.call_async(GetModelList.Request())
    if models_future is not None and models_future.done():
        save_json('gazebo_models.json', message_to_ordereddict(models_future.result()))
    else:
        manifest['errors'].append('Gazebo model list unavailable')

    for i, (msg, receive_sim) in enumerate(images):
        name = 'depth_%02d' % i
        # Preserve exact bytes, endian flag, row padding and encoding for replay.
        (out / (name + '.bin')).write_bytes(bytes(msg.data))
        metadata = {field: getattr(msg, field) for field in
                    ('height', 'width', 'encoding', 'is_bigendian', 'step')}
        metadata.update(header=message_to_ordereddict(msg.header),
                        receive_sim_time=receive_sim)
        try:
            frame = msg.header.frame_id or 'depth_camera_optical'
            transform = buffer.lookup_transform(base_frame, frame, Time.from_msg(msg.header.stamp))
            metadata['camera_to_base_at_image_stamp'] = message_to_ordereddict(transform)
        except Exception as exc:
            metadata['transform_error'] = str(exc)
        if msg.encoding.upper() == '32FC1':
            raw = np.frombuffer(bytes(msg.data), dtype='>f4' if msg.is_bigendian else '<f4')
            raw = raw.reshape(msg.height, msg.step // 4)[:, :msg.width]
            finite = raw[np.isfinite(raw)]
            metadata['depth_summary'] = {
                'pixels': int(raw.size), 'finite_pixels': int(finite.size),
                'within_detector_015_to_6m': int(np.count_nonzero(np.isfinite(raw) & (raw >= .15) & (raw <= 6))),
                'finite_percentiles_m': np.percentile(finite, [0, 25, 50, 75, 100]).tolist() if finite.size else []}
        save_json(name + '.json', metadata)

    transforms = {}
    for target, source in [('map', 'base_footprint'), ('odom', base_frame),
                           (base_frame, 'depth_camera_optical')]:
        try:
            transforms[target + '<-' + source] = message_to_ordereddict(
                buffer.lookup_transform(target, source, Time()))
        except Exception as exc:
            transforms[target + '<-' + source] = {'error': str(exc)}
    save_json('transforms.json', transforms)
    manifest['publishers'] = {
        topic: [{'name': p.node_name, 'namespace': p.node_namespace,
                 'type': p.topic_type} for p in node.get_publishers_info_by_topic(topic)]
        for topic in (depth_topic, info_topic, guard_topic, '/etrike/terrain/status')}
except KeyboardInterrupt:
    manifest['errors'].append('Capture interrupted; partial data saved')
except Exception:
    manifest['errors'].append(traceback.format_exc())
finally:
    manifest['received_counts'] = dict(counts)
    manifest['depth_frames_saved'] = len(list(out.glob('depth_*.bin')))
    manifest['odometry_max_reported_speeds'] = odom_max
    if counts['/tricycle_steering_controller/odometry'] == 0:
        manifest['errors'].append('No odometry received; stationary state unverified')
    elif odom_max['abs_linear_x'] > .02 or odom_max['abs_angular_z'] > .03:
        manifest['errors'].append('Odometry reported motion during capture')
    save_json('manifest.json', manifest)
    save_json('latest.json', latest)
    with (out / 'events.jsonl').open('w') as stream:
        for row in records:
            stream.write(json.dumps(row, default=str) + '\n')
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    archive = Path.home() / ('BGLX_terrain_check_' + stamp + '.zip')
    with ZipFile(archive, 'w', ZIP_DEFLATED) as z:
        for p in sorted(out.iterdir()):
            if p.is_file():
                z.write(p, p.name)
    latest_archive = Path.home() / 'BGLX_terrain_check_latest.zip'
    shutil.copyfile(archive, latest_archive)
    print('\nLast terrain decision:', json.dumps(latest.get('/etrike/terrain/status', 'NOT RECEIVED')))
    print('Depth frames saved:', manifest['depth_frames_saved'])
    print('Odometry maximum reported speeds:', odom_max)
    for error in manifest['errors']:
        print('CAPTURE NOTE:', error)
    print('No motion requested; no parameters changed.')
    print('UPLOAD:', archive)
    print('Latest copy:', latest_archive)
PY
