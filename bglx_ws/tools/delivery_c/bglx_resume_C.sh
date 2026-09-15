#!/usr/bin/env bash
# Resume the approved pillar route from the current stopped pose to C.
# This is a simulation motion command. No parcel/history or HOME leg is run.
set -eo pipefail
cd "$HOME/projects/BGLX/bglx_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tools/prototype_resize/stationary_check.py
ros2 param set /controller_server FollowPath.desired_linear_vel 0.4
ros2 param set /controller_server FollowPath.use_velocity_scaled_lookahead_dist true
mkdir -p "$HOME/bglx_navtest/logs"
run_log="$HOME/bglx_navtest/logs/resume_C_pillars_$(date +%Y%m%d_%H%M%S).log"
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
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
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
           ('resume_C_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f')))
run_dir.mkdir(parents=True, exist_ok=False)
tree_path = run_dir / 'navigate_C_via_pillars.xml'
tree_path.write_text(TREE)

# Preserve the ROS context until owned actions have been cancelled/settled.
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
interrupted = False
def interrupt(signum, frame):
    global interrupted
    interrupted = True
signal.signal(signal.SIGINT, interrupt)
signal.signal(signal.SIGTERM, interrupt)

node = rclpy.create_node('bglx_resume_c_via_pillars', parameter_overrides=[
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
          'route': [[2.0, 34.675, 0.0], [6.5, 34.675, 0.0],
                    [28.0, 82.0, 74.65]],
          'behavior_tree': str(tree_path)}
last_feedback = 0.0
last_remaining = None

def feedback(message):
    global last_feedback, last_remaining
    f = message.feedback
    count = f.number_of_poses_remaining
    if count != last_remaining:
        if count == 2:
            print('PASSAGE ENTRY PASSED; continuing through the gap.', flush=True)
        elif count == 1:
            print('PILLAR EXIT PASSED; continuing toward C.', flush=True)
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
    if (math.hypot(pose[0]-1.31, pose[1]-46.697) > 1.0 or
            abs(math.atan2(math.sin(pose[2]-1.28), math.cos(pose[2]-1.28))) > 0.35):
        raise RuntimeError('Robot has moved from the reviewed stopped pose; recheck this route first.')

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

    print('TRIKE WILL MOVE: current pose -> pillar opening -> DELIVERY_C; 0.4 m/s.', flush=True)
    print('Ctrl-C requests cancellation and waits for a confirmed stop.', flush=True)
    report['navigation_requested'] = True
    pending = client.send_goal_async(goal, feedback_callback=feedback)
    pending.add_done_callback(own_response)
    handle = guard.resolve_send(pending, 'C passage navigation', 10.0)
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
