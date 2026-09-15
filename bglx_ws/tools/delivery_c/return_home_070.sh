#!/usr/bin/env bash
# Return from DELIVERY_C to HOME using the current live map. Requested cruise speed: 0.7 m/s.
# This is a simulation motion command. No parcel/history or HOME leg is run.
set -eo pipefail
cd "$HOME/projects/BGLX/bglx_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tools/prototype_resize/stationary_check.py
ros2 param set /controller_server FollowPath.desired_linear_vel 0.7
ros2 param set /controller_server FollowPath.use_velocity_scaled_lookahead_dist true
mkdir -p "$HOME/bglx_navtest/logs"
run_log="$HOME/bglx_navtest/logs/return_HOME_from_C_070_$(date +%Y%m%d_%H%M%S).log"
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
           ('return_HOME_from_C_070_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f')))
run_dir.mkdir(parents=True, exist_ok=False)
tree_path = run_dir / 'navigate_HOME_from_C.xml'
tree_path.write_text(TREE)

# Preserve the ROS context until owned actions have been cancelled/settled.
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
interrupted = False
def interrupt(signum, frame):
    global interrupted
    interrupted = True
signal.signal(signal.SIGINT, interrupt)
signal.signal(signal.SIGTERM, interrupt)

node = rclpy.create_node('bglx_return_home_from_c', parameter_overrides=[
    Parameter('use_sim_time', Parameter.Type.BOOL, True)])
node.active_goal_handle = None
node.tf_buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(node.tf_buffer, node)
map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
guard = RecoveryGuard(node, map_qos)
client = ActionClient(node, NavigateThroughPoses, '/navigate_through_poses')
report = {'destination': 'HOME', 'action_status': None,
          'navigation_requested': False, 'success': False,
          'route': [[0.0, 0.0, -108.85]],
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
            print('Following the map-planned route to HOME.', flush=True)
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
    if math.hypot(pose[0]-28.0, pose[1]-82.0) > 1.5:
        raise RuntimeError('This return trial starts at C; the robot is not near the recorded C arrival.')
    from bglx_agentic.mission_waypoints import get_location
    hx, hy = get_location('HOME')
    if math.hypot(hx, hy) > .01:
        raise RuntimeError('The live HOME waypoint differs from the established map origin.')
    report['requested_cruise_m_s'] = .7
    report['route_policy'] = 'HOME-only goal; Nav2 chooses the route using the live map'
    print('RETURN ROUTE: C -> HOME (0, 0); Nav2 selects the passage from the live map.', flush=True)

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
            values[2].type != 1 or abs(values[0].double_value-0.7) > 1e-6 or
            values[1].bool_value or not values[2].bool_value):
        raise RuntimeError('Expected 0.7 m/s, forward-only controller and collision checking enabled')


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

    print('TRIKE WILL MOVE: C -> HOME (0, 0); requested cruise 0.7 m/s, regulated in turns/near obstacles.', flush=True)
    print('Ctrl-C requests cancellation and waits for a confirmed stop.', flush=True)
    report['navigation_requested'] = True
    pending = client.send_goal_async(goal, feedback_callback=feedback)
    pending.add_done_callback(own_response)
    handle = guard.resolve_send(pending, 'HOME return from C', 10.0)
    if handle is None or not handle.accepted:
        raise RuntimeError('Nav2 rejected the route')
    own_response(pending)
    deadline = time.monotonic() + 600.0
    print('Goal accepted by Nav2.', flush=True)
    while rclpy.ok() and not guard.nav_result.done():
        if interrupted:
            raise RuntimeError('Operator requested cancellation')
        if time.monotonic() >= deadline:
            raise RuntimeError('HOME return exceeded 600 seconds; cancelling')
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
    report['goal_error_m'] = math.hypot(end[0], end[1])
    if report['goal_error_m'] > 0.85:
        raise RuntimeError('Final position exceeds combined planner/controller goal tolerance')
    report['success'] = True
except Exception as exc:
    report['error'] = str(exc)
    print('HOME RETURN STOPPED:', exc, flush=True)
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
        print('ARRIVED HOME; STOP CONFIRMED. Goal error=%.3fm' % report['goal_error_m'], flush=True)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
sys.exit(0 if report['success'] else 1)
PY
