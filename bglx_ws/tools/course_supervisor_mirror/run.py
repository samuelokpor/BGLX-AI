#!/usr/bin/env python3
"""Supervised Gazebo four-pillar experiment. Default: audit existing scene."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

HERE=Path(__file__).resolve().parent

def decision(data, code, token, stage, execute):
    if not isinstance(data,dict) or data.get('schema')!=1 or data.get('run_id')!=token or data.get('stage')!=stage:
        return 'STOP'
    if data.get('stopped') is not True or data.get('cleanup_ok') is not True:
        return 'STOP'
    expected='SUCCESS' if stage=='setup' or execute else 'AUDIT_PASS'
    if code==0 and data.get('code')==expected:return 'PASS'
    if code==10 and stage=='course' and data.get('code')=='FRAME_CHANGED':return 'REPLAN'
    return 'STOP'

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--setup',action='store_true',help='Replace named test pillars relative to the stopped starting pose')
    parser.add_argument('--execute',action='store_true',help='Enable 0.25 m/s forward motion after screening')
    parser.add_argument('--max-replans',type=int,choices=range(3),default=2,help='Map-correction retry limit (0..2)')
    args=parser.parse_args()
    logs=Path.home()/'bglx_navtest/logs';logs.mkdir(parents=True,exist_ok=True)
    lock=(logs/'course_supervisor.lock').open('a')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise SystemExit('Another course supervisor is running.')
    run=logs/('course_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8]);run.mkdir()
    summary=dict(schema=1,execute=args.execute,setup=args.setup,stages=[],outcome='STOPPED')
    cancelled=False
    proc=None
    def interrupt(*_):
        nonlocal cancelled
        cancelled=True
        if proc is not None and proc.poll() is None:
            os.killpg(proc.pid,signal.SIGINT)
    signal.signal(signal.SIGINT,interrupt)
    signal.signal(signal.SIGTERM,interrupt)
    def stage(name):
        nonlocal proc
        token=uuid.uuid4().hex
        result=run/(name+'_'+token+'.json');log=run/(name+'_'+token+'.log')
        env=os.environ.copy()
        env.update(BGLX_RESULT=str(result),BGLX_RUN_ID=token,BGLX_STAGE=name,COURSE_EXECUTE='1' if args.execute else '0')
        file='setup.py' if name=='setup' else 'worker.py'
        print(f'\nSTATE {name.upper()} | log={log}',flush=True)
        timed_out=False;deadline=time.monotonic()+360;cancel_time=None
        with log.open('w') as output,log.open() as reader:
            proc=subprocess.Popen([sys.executable,'-u',str(HERE/file)],env=env,stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
            while proc.poll() is None:
                chunk=reader.read()
                if chunk:print(chunk,end='',flush=True)
                if time.monotonic()>deadline and not timed_out:
                    timed_out=True;os.killpg(proc.pid,signal.SIGINT)
                if cancelled or timed_out:
                    if cancel_time is None:cancel_time=time.monotonic()
                    if time.monotonic()-cancel_time>35:
                        raise RuntimeError('Cancellation not confirmed. Stop navigation and verify robot stopped; no retry.')
                time.sleep(.1)
            print(reader.read(),end='',flush=True)
            code=proc.returncode
        try:data=json.loads(result.read_text())
        except (OSError,ValueError):data={}
        action=decision(data,code,token,name,args.execute)
        if cancelled or timed_out:action='STOP'
        summary['stages'].append(dict(stage=name,returncode=code,decision=action,result=data,log=log.name))
        return action
    try:
        print('COURSE SUPERVISOR — '+('MOTION ENABLED' if args.execute else 'AUDIT ONLY'),flush=True)
        print('Results:',run,flush=True)
        if args.setup and stage('setup')!='PASS':raise RuntimeError('Scene setup did not complete cleanly')
        for attempt in range(args.max_replans+1):
            if cancelled:raise RuntimeError('Interrupted')
            action=stage('course')
            if action=='PASS':
                summary['outcome']='SUCCESS' if args.execute else 'AUDIT_PASS'
                print(summary['outcome'],flush=True)
                return 0
            if action!='REPLAN' or attempt==args.max_replans:
                raise RuntimeError('Stage stopped or map-replan budget exhausted; see stage log')
            print('Confirmed stopped cleanup. Remeasuring all four pillars and rebuilding remaining path.',flush=True)
        return 10
    except Exception as exc:
        summary['detail']=str(exc);print('SUPERVISOR STOPPED:',exc,flush=True);return 10
    finally:
        target=run/'summary.json';tmp=target.with_suffix('.tmp')
        tmp.write_text(json.dumps(summary,indent=2)+'\n');tmp.replace(target)
        print('Summary:',target,flush=True)
        lock.close()

if __name__=='__main__':raise SystemExit(main())
