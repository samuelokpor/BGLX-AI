#!/usr/bin/env python3
"""Simulation adapter to installed BGLX mission; default is read-only."""
import argparse
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from datetime import datetime
from wiring import attach_advisory, execute_leg

HERE = Path(__file__).resolve().parent
TOPICS = ['/clock','/tf','/tf_static','/map','/global_costmap/costmap',
 '/local_costmap/published_footprint','/etrike/front/image_raw','/etrike/front/camera_info',
 '/etrike/front_camera/depth/image_raw','/etrike/front_camera/depth/camera_info',
 '/etrike/front_depth/depth/image_raw','/etrike/front_depth/depth/camera_info',
 '/etrike/scan','/tricycle_steering_controller/odometry']


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode',choices=['return-home','home-to-c','vision-to-c'],default='return-home')
    p.add_argument('--execute',action='store_true')
    p.add_argument('--recover-first',action='store_true',help='Existing guarded backup before HOME only')
    p.add_argument('--vision-shadow',action='store_true',help='Log camera/model advice at original alignment handoff; no model motion authority')
    p.add_argument('--model',default='qwen2.5vl:7b')
    a = p.parse_args()
    if a.recover_first and a.mode != 'return-home':p.error('--recover-first is only for return-home')
    out=Path.home()/'bglx_navtest/logs'/('existing_mission_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    out.mkdir(parents=True)
    def event(name,**data):
        row=dict(event=name,wall_time=time.time(),**data)
        print('[existing-mission]',json.dumps(row),flush=True)
        with (out/'events.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
    expected=json.loads((HERE/'source_hashes.json').read_text())
    for name,sha in expected.items():
        module=importlib.import_module('bglx_agentic.'+name)
        path=Path(module.__file__).resolve();actual=hashlib.sha256(path.read_bytes()).hexdigest()
        event('dependency',module=name,path=str(path),sha256=actual,matches_uploaded_source=actual==sha)
        if actual!=sha:raise RuntimeError('Installed '+name+' differs from reviewed source; no motion. Share dependency log.')
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from bglx_agentic.delivery_mission import DeliveryMission
    from bglx_agentic.passage_geometry import xf
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true','-p','passage_alignment_enabled:=true',
        '-p','turnaround_assist_enabled:=true','-p','early_unstuck_enabled:=true','-p','leg_timeout:=600.0'],
        signal_handler_options=SignalHandlerOptions.NO)
    def interrupted(*_):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupted)
    node=None;success=False;stop=False
    try:
        node=DeliveryMission();guard=node.recovery_guard
        guard.ready()
        if not node.nav_client.wait_for_server(timeout_sec=5):raise RuntimeError('NavigateToPose missing')
        pose=guard.transform('map','base_footprint')
        if a.mode=='home-to-c' and math.hypot(*pose[:2])>1.:raise RuntimeError('HOME-to-C requires starting within 1m of HOME')
        event('configuration',execute=a.execute,mode=a.mode,pose=pose,
              alignment_enabled=node.passage_alignment.enabled,
              turnaround_enabled=node.turnaround_assist_enabled,
              early_unstuck_enabled=node.early_unstuck_enabled,vision_shadow=a.vision_shadow)
        if a.recover_first:
            speed=node.unstuck_backup_speed
            distance=node.unstuck_backup_distance+speed*1.15+speed*speed+.05
            # Allow startup subscriptions to receive fresh sensor data.
            audit_deadline = time.monotonic() + 5.0
            while True:
                guard.spin(.1)
                ok,detail=guard.reverse_audit(distance)
                if ok or time.monotonic() >= audit_deadline:
                    break
                if not any(word in str(detail).lower()
                           for word in ('missing', 'stale')):
                    break
            event('existing_reverse_preflight',passed=ok,detail=detail,distance_with_reserve_m=distance)
            if not ok:raise RuntimeError('Existing reverse preflight refused; no navigation')
        def process(cmd,label,duration=None):
            with (out/(label+'.log')).open('w') as f:
                proc=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
                deadline=time.monotonic()+(duration or 110)
                try:
                    while proc.poll() is None and time.monotonic()<deadline:guard.spin(.05)
                    if proc.poll() is None:
                        if duration is None:raise RuntimeError(label+' timed out')
                        os.killpg(proc.pid,signal.SIGINT)
                        end=time.monotonic()+15
                        while proc.poll() is None and time.monotonic()<end:guard.spin(.05)
                        if proc.poll() is None:raise RuntimeError('Recorder did not settle')
                    if proc.returncode not in (0,-signal.SIGINT):raise RuntimeError(label+' failed')
                finally:
                    if proc.poll() is None:
                        os.killpg(proc.pid,signal.SIGTERM)
                        try:proc.wait(timeout=3)
                        except subprocess.TimeoutExpired:
                            os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=3)
        count=0
        def observe(candidate,deadline):
            nonlocal count
            count+=1;label='vision_%02d'%count
            before=guard.transform('map','base_footprint')
            gate,frame=candidate
            gate_map=xf((gate.x,gate.y,gate.heading),guard.transform('map',frame))
            bag=out/(label+'_capture');analysis=out/(label+'_analysis')
            process(['ros2','bag','record','-o',str(bag),*TOPICS],label+'_record',8)
            process([sys.executable,str(HERE/'analyze.py'),str(bag),'--output',str(analysis)],label+'_analyze')
            process([sys.executable,str(HERE/'rank_model.py'),str(analysis),'--model',a.model],label+'_model')
            guard.ready();after=guard.transform('map','base_footprint')
            if math.dist(before[:2],after[:2])>.03 or abs(math.atan2(math.sin(before[2]-after[2]),math.cos(before[2]-after[2])))>.03:
                raise RuntimeError('Pose changed during stationary vision observation')
            if time.monotonic()>=deadline:raise RuntimeError('Leg deadline expired during vision')
            report=json.loads((analysis/'candidates.json').read_text())
            ranking=json.loads((analysis/'model_rank.json').read_text())
            matches=[c['id'] for c in report['candidates'] if math.dist(c['mouth_map_xy'],gate_map[:2])<.8]
            event('vision_advisory',existing_gate_map=gate_map,matching_camera_ids=matches,
                  ranking=ranking,policy='shadow only; original alignment owns geometry and motion')
        if a.vision_shadow:attach_advisory(node,observe)
        if a.mode=='vision-to-c':
            from vision_handoff import run
            success=bool(run(node,process,event,out,HERE,TOPICS,a.model,a.execute))
        elif not a.execute:
            event('check_complete',scope='dependencies, lifecycle, pose, flags and requested reverse audit; no forward path approval')
            success=True
        else:
            # User-requested simulation cruise ceiling: 0.90m/s.
            from rcl_interfaces.srv import GetParameters
            client=node.create_client(GetParameters,'/controller_server/get_parameters')
            if not client.wait_for_service(timeout_sec=5):raise RuntimeError('Controller parameter service missing')
            future=client.call_async(GetParameters.Request(names=['use_sim_time','FollowPath.desired_linear_vel']))
            rclpy.spin_until_future_complete(node,future,timeout_sec=10)
            if not future.done() or future.result() is None:raise RuntimeError('Controller parameter timeout')
            values=future.result().values
            if len(values)!=2 or values[0].type!=1 or not values[0].bool_value or values[1].type!=3 or not 0<values[1].double_value<=1.5:
                raise RuntimeError('Requires simulation controller with cruise speed <=1.5m/s; no settings changed')
            success=bool(execute_leg(node,a.mode,a.recover_first))
            event('mission_result',success=success)
    except (Exception,KeyboardInterrupt) as e:
        event('abort',reason=str(e) or 'Interrupted')
    finally:
        if node is not None:
            stop=bool(node.recovery_guard.cleanup())
            stop=bool(node.recovery_guard.stopped()) and stop
            node.destroy_node()
        rclpy.try_shutdown()
        (out/'summary.json').write_text(json.dumps(dict(success=success and stop,stop_confirmed=stop,execute=a.execute),indent=2))
        print('RESULTS:',out,flush=True)
    return 0 if success and stop else 1

if __name__=='__main__':sys.exit(main())
