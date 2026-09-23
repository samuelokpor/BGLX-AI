"""Stationary vision selection using the existing passage controller."""
from bglx_tf_sync import grid_context as bglx_grid_context
import json
import math
import sys
import time

def finish_after_crossing(node, candidate, deadline, event):
    if not node.passage_alignment.perform(candidate, deadline):
        raise RuntimeError('Existing alignment failed; C goal NOT sent')
    (gate, frame) = candidate
    guard = node.recovery_guard
    guard.ready()
    (x, y, h) = guard.transform(frame, 'base_footprint')
    along = (x - gate.x) * math.cos(gate.heading) + (y - gate.y) * math.sin(gate.heading)
    if along < 0.5:
        raise RuntimeError('Crossing not confirmed; C goal NOT sent')
    event('vision_crossing_complete', along_m=along)
    return node.navigate_with_exploration('DELIVERY_C', (28.0, 82.0, 1.3028507292104001))

def run(node, process, event, out, here, topics, model, execute=False):
    import rclpy
    from rcl_interfaces.srv import GetParameters
    from bglx_agentic.passage_geometry import Grid, aligned_path, xf, wrap
    guard = node.recovery_guard
    if node.active_goal_handle is not None:
        raise RuntimeError('Cancel the running mission first')
    guard.ready()
    client = node.create_client(GetParameters, '/controller_server/get_parameters')
    if not client.wait_for_service(timeout_sec=5):
        raise RuntimeError('Controller parameters unavailable')
    future = client.call_async(GetParameters.Request(names=['use_sim_time', 'FollowPath.desired_linear_vel']))
    rclpy.spin_until_future_complete(node, future, timeout_sec=10)
    if not future.done() or future.result() is None:
        raise RuntimeError('Controller parameter timeout')
    values = future.result().values
    if len(values) != 2 or values[0].type != 1 or (not values[0].bool_value) or (values[1].type != 3) or (not 0 < values[1].double_value <= 1.5):
        raise RuntimeError('Requires simulation and cruise <=1.5m/s')
    before = guard.transform('map', 'base_footprint')
    capture = out / 'selection_capture'
    folder = out / 'selection_analysis'
    event('vision_capture', execute=execute, pose=before)
    process(['ros2', 'bag', 'record', '-o', str(capture), *topics], 'selection_record', 8)
    process([sys.executable, str(here / 'analyze.py'), str(capture), '--output', str(folder)], 'selection_analyze')
    process([sys.executable, str(here / 'rank_model.py'), str(folder), '--model', model], 'selection_model')
    guard.ready()
    current = guard.transform('map', 'base_footprint')
    report = json.loads((folder / 'candidates.json').read_text())
    if math.dist(current[:2], before[:2]) > 0.03 or abs(wrap(current[2] - before[2])) > 0.03 or math.dist(current[:2], report['robot_map_xy']) > 0.15 or (abs(wrap(current[2] - report['robot_yaw'])) > 0.1):
        raise RuntimeError('Capture and current pose disagree; recapture required')
    ranking = json.loads((folder / 'model_rank.json').read_text())
    candidates = {c['id']: c for c in report['candidates']}
    ids = ranking.get('ranking', [])
    if not isinstance(ids, list) or not ids or any((not isinstance(i, str) or i not in candidates for i in ids)) or (len(set(ids)) != len(ids)):
        raise RuntimeError('No valid vision choice; no motion or C goal')
    event('vision_ranking', ranking=ranking)
    guard.spin(0.2)
    (ok, reason) = guard.reverse_audit(0.0)
    if not ok:
        raise RuntimeError('Existing sensor/footprint checks: ' + str(reason))
    chosen = None
    for cid in ids:
        candidate = candidates[cid]
        (msg, robot, map_to_grid) = bglx_grid_context(node, event=event)
        frame = msg.header.frame_id
        grid = Grid(msg)
        mouth = xf((*candidate['mouth_map_xy'], candidate['approach_yaw']), map_to_grid)
        gate = grid.section(mouth)
        points = None
        reason = None
        backup = False
        if gate is None or not 1.0 <= gate.width <= 2.1:
            reason = 'Opening not confirmed by existing local geometry'
        elif math.dist((gate.x, gate.y), mouth[:2]) > 0.3 or abs(gate.width - candidate['width_at_scan_height_m']) > 0.4:
            reason = 'Camera/lidar and local geometry disagree'
        elif (robot[0] - gate.x) * math.cos(gate.heading) + (robot[1] - gate.y) * math.sin(gate.heading) >= -0.5:
            reason = 'Robot is not on the approach side'
        else:
            points = aligned_path(grid, robot, gate)
            if points is None:
                speed = node.unstuck_backup_speed
                distance = node.unstuck_backup_distance
                (ok, why) = guard.reverse_audit(distance + speed * 1.15 + speed * speed + 0.05)
                if ok:
                    backed = (robot[0] - distance * math.cos(robot[2]), robot[1] - distance * math.sin(robot[2]), robot[2])
                    points = aligned_path(grid, backed, gate)
                    backup = points is not None
                if points is None:
                    reason = 'No bounded connector; reverse audit: ' + str(why)
        event('vision_candidate_audit', candidate_id=cid, passed=points is not None, reason=reason, backup_needed=backup)
        if points is not None:
            chosen = (gate, frame)
            (out / 'selected_alignment_path.json').write_text(json.dumps({'frame': frame, 'points': points}))
            event('vision_selected', candidate_id=cid, gate=[gate.x, gate.y, gate.heading, gate.width], frame=frame)
            break
    if chosen is None:
        raise RuntimeError('No ranked opening has an audited connector; C goal NOT sent')
    if not execute:
        event('vision_plan_only_complete', scope='Crossing checked; no motion; C leg not checked')
        return True
    return finish_after_crossing(node, chosen, time.monotonic() + 180, event)
