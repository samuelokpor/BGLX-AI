import json
import math
import os
from pathlib import Path

class StageFailure(RuntimeError):

    def __init__(self, code, message):
        self.code = code
        super().__init__(message)

class Result:

    def __init__(self):
        self.path = Path(os.environ['BGLX_RESULT'])
        self.data = dict(schema=1, run_id=os.environ['BGLX_RUN_ID'], stage=os.environ['BGLX_STAGE'], code='ERROR', stopped=False, cleanup_ok=False, metrics={})

    def fail(self, exc):
        self.data.update(code=getattr(exc, 'code', 'ERROR'), detail=str(exc))

    def finish(self):
        temp = self.path.with_suffix('.tmp')
        temp.write_text(json.dumps(self.data, indent=2, allow_nan=False) + '\n')
        temp.replace(self.path)

def decision(data, stage, exit_code, recovered=False):
    if data.get('schema') != 1 or data.get('stage') != stage or data.get('stopped') is not True or (data.get('cleanup_ok') is not True):
        return 'STOP'
    code = data.get('code')
    if code == 'SUCCESS' and exit_code == 0:
        return 'NEXT'
    if exit_code != 10:
        return 'STOP'
    if stage == 'forward' and code == 'FRAME_CHANGED':
        return 'REPLAN'
    if stage == 'forward' and (not recovered) and (code in ('CLEARANCE', 'TRACKING', 'NO_ROUTE')):
        return 'RECOVER'
    if stage == 'reverse' and code == 'REVERSE_ENDPOINT':
        value = data.get('metrics', {}).get('endpoint_distance')
        if isinstance(value, (int, float)) and (not isinstance(value, bool)) and math.isfinite(value) and (0 <= value <= 0.08):
            return 'NEXT'
    return 'STOP'

def rank_candidates(candidates):
    if not candidates:
        return []
    best = max((c[0] for c in candidates))
    early = [c for c in candidates if c[0] >= best - 0.005]
    rest = [c for c in candidates if c[0] < best - 0.005]
    return sorted(early, key=lambda c: (c[2], c[0], c[1]), reverse=True) + sorted(rest, key=lambda c: (c[0], c[2], c[1]), reverse=True)
