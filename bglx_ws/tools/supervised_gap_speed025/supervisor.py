_map_replans = 0
import argparse
import json
import os
import signal
import subprocess
import sys
import uuid
from pathlib import Path
from protocol import decision
parser = argparse.ArgumentParser()
parser.add_argument('--execute', action='store_true', help='Authorize simulated motion')
parser.add_argument('--scene', choices=('existing', 'speed025', 'speed025_angle35', 'speed025_mirror35'), default='existing')
args = parser.parse_args()
if not args.execute:
    parser.error('Use --execute for motion; run test_protocol.py for offline tests')
folder = Path(__file__).resolve().parent
run_id = uuid.uuid4().hex
logs = Path.home() / 'bglx_navtest/logs' / ('v2_' + run_id)
logs.mkdir(parents=True)
print('Structured run directory:', logs, flush=True)

def run(stage, filename, recovered=False):
    global _map_replans
    token = uuid.uuid4().hex
    result_file = logs / (stage + '_' + token + '.json')
    env = dict(os.environ, BGLX_RESULT=str(result_file), BGLX_RUN_ID=token, BGLX_STAGE=stage, BGLX_SCENE=args.scene)
    with (logs / (stage + '_' + token + '.log')).open('w') as log:
        child = subprocess.Popen([sys.executable, '-u', str(folder / filename)], env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
        try:
            for line in child.stdout:
                print(line, end='', flush=True)
                log.write(line)
                log.flush()
            status = child.wait()
        except KeyboardInterrupt:
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=30)
            except subprocess.TimeoutExpired:
                print('Cleanup unconfirmed; verify stopped robot. No next stage.', flush=True)
            raise SystemExit(130)
    try:
        data = json.loads(result_file.read_text())
        if data.get('run_id') != token:
            raise ValueError('Wrong result identity')
    except (OSError, ValueError) as exc:
        raise SystemExit('Missing/invalid structured result; stopped: ' + str(exc))
    step = decision(data, stage, status, recovered)
    print('DECISION:', stage, data['code'], '->', step, flush=True)
    if step == 'REPLAN':
        if _map_replans >= 1:
            return 'STOP'
        _map_replans += 1
        print('REMEASURE: one fresh forward audit from stopped pose', flush=True)
        return run(stage, filename, recovered)
    return step
if args.scene != 'existing' and run('scene', 'scene.py') != 'NEXT':
    raise SystemExit('Setup failed; no motion stage started')
step = run('forward', 'forward.py')
if step == 'RECOVER':
    if run('reverse', 'reverse.py') != 'NEXT':
        raise SystemExit('Recovery failed; stopped')
    step = run('forward', 'forward.py', recovered=True)
if step != 'NEXT':
    raise SystemExit('Sequence stopped; no further retry')
print('V2 SEQUENCE COMPLETE', flush=True)
