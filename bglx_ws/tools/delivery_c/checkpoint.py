#!/usr/bin/env python3
"""Checkpoint the current ROS checkout and successful C-arrival evidence, then push.

No reset, stash, branch switch, rebase, force-push, or ROS motion is performed.
"""
import ast
import json
import os
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path

WS = Path.home() / 'projects/BGLX/bglx_ws'
BUNDLE = Path(__file__).resolve().parent
TAG = 'sim-delivery-c-arrival-20260915'
DOC = Path('docs/simulation_cases/delivery-c-arrival-20260915')
TOOL = Path('tools/delivery_c')
TITLE = 'feat(sim): checkpoint supervised Delivery C arrival and campus ground coverage'

def run(args, *, capture=False, check=True, timeout=60, cwd=None):
    return subprocess.run(args, cwd=cwd or WS, text=True, capture_output=capture,
                          check=check, timeout=timeout)

def git(*args, capture=False, check=True):
    return run(['git', *args], capture=capture, check=check, timeout=120)

def output(*args):
    return git(*args, capture=True).stdout.strip()

def paths_from_git(*args):
    return [Path(s) for s in git(*args, capture=True).stdout.split('\0') if s]

def relevant(p):
    """Worktree paths are relative to bglx_ws; scope to the BGLX stack and tools."""
    parts = p.parts
    if not parts or '..' in parts:
        return False
    allowed = (len(parts) >= 2 and parts[0] == 'src'
               and parts[1] in ('bglx_agentic', 'bglx_navigation', 'etrike_description'))
    allowed = allowed or parts[0] == 'tools' or p.is_relative_to(DOC)
    if not allowed:
        return False
    junk = ('__pycache__', 'node_modules', '.venv', 'build', 'install', 'log', 'logs',
            'validated_runs', '.git')
    if (any(s in junk or s.startswith('.') for s in parts)
            or any(s.lower() in ('backup', 'backups') or s.lower().endswith('_backups')
                   for s in parts[:-1])):
        return False
    name = p.name.lower()
    if ('.before_' in name or '.before.' in name or name.endswith(('~', '.bak', '.pyc', '.pyo'))
            or name.startswith(('before_', 'pre_', 'copy_of_'))):
        return False
    if p.is_relative_to(DOC):
        return True
    return (p.suffix.lower() in ('.py', '.sh', '.yaml', '.yml', '.json', '.xml', '.xacro',
                                '.urdf', '.sdf', '.world', '.rviz', '.md', '.rst', '.txt',
                                '.stl', '.dae', '.obj', '.mtl', '.png', '.jpg', '.jpeg',
                                '.config', '.cfg', '.cpp', '.hpp', '.h', '.c', '.launch',
                                '.bt', '.npz', '.npy')
            or p.name in ('COLCON_IGNORE', 'CMakeLists.txt'))

def candidates():
    # Commands run with cwd=bglx_ws. --relative avoids sibling projects in the parent repo.
    modified = paths_from_git('diff', '--relative', '--name-only', '-z', 'HEAD', '--', '.')
    indexed = paths_from_git('diff', '--cached', '--relative', '--name-only', '-z', '--', '.')
    untracked = paths_from_git('ls-files', '--others', '--exclude-standard', '-z', '--', '.')
    return sorted({p for p in modified + indexed + untracked if relevant(p)}, key=str)

def latest_success(glob, predicate):
    for path in sorted((Path.home() / 'bglx_navtest/logs').glob(glob), reverse=True):
        try:
            data = json.loads(path.read_text())
            if predicate(data):
                return path, data
        except (OSError, ValueError, TypeError):
            pass
    raise RuntimeError('Required successful run evidence not found: ' + glob)

def syntax_check(paths):
    import yaml
    for p in paths:
        file = WS / p
        if not file.is_file():
            continue  # A tracked deletion can be part of this checkpoint.
        if file.suffix == '.py':
            ast.parse(file.read_text(), filename=str(p), feature_version=(3, 10))
        elif file.suffix in ('.xml', '.xacro', '.urdf', '.sdf', '.world'):
            ET.parse(file)
        elif file.suffix in ('.yaml', '.yml', '.rviz'):
            yaml.safe_load(file.read_text())
        elif file.suffix == '.json':
            json.loads(file.read_text())
        elif file.suffix == '.sh':
            run(['bash', '-n', str(file)], timeout=15)

def push(branch):
    print(f'Pushing branch {branch} and annotated tag {TAG} (no force)...', flush=True)
    git('push', '--atomic', 'origin', f'HEAD:refs/heads/{branch}', f'refs/tags/{TAG}')
    sha = output('rev-parse', 'HEAD')
    refs = git('ls-remote', 'origin', f'refs/heads/{branch}', f'refs/tags/{TAG}^{{}}',
               capture=True).stdout
    remote = dict(line.split()[::-1] for line in refs.splitlines() if line.strip())
    if remote.get(f'refs/heads/{branch}') != sha or remote.get(f'refs/tags/{TAG}^{{}}') != sha:
        raise RuntimeError('Push returned, but remote branch/tag verification did not match.')
    print('CHECKPOINT PUSHED:', sha, flush=True)
    print('TAG:', TAG, flush=True)

def main():
    if Path.cwd().resolve() != WS.resolve():
        os.chdir(WS)
    if not output('rev-parse', '--is-inside-work-tree') == 'true':
        raise RuntimeError('Not a Git worktree')
    branch = output('symbolic-ref', '--short', 'HEAD')
    if output('ls-files', '-u'):
        raise RuntimeError('Git has unresolved conflicts; checkpoint was not created.')
    origin = output('remote', 'get-url', 'origin')
    if not re.search(r'github\.com[:/]samuelokpor/BGLX-AI(?:\.git)?/?$', origin, re.I):
        raise RuntimeError('Origin is not the expected samuelokpor/BGLX-AI repository; no push attempted.')
    print('Branch:', branch, flush=True)
    git('status', '--short', '--branch', '--', 'src/bglx_agentic', 'src/bglx_navigation',
        'src/etrike_description', 'tools', str(DOC))
    git('fetch', 'origin')
    remote_branch = f'refs/remotes/origin/{branch}'
    if git('rev-parse', '--verify', remote_branch, capture=True, check=False).returncode == 0:
        if git('merge-base', '--is-ancestor', remote_branch, 'HEAD', check=False).returncode:
            raise RuntimeError('Remote branch has commits not in this checkout. No merge/force-push attempted.')
    if git('rev-parse', '--verify', f'refs/tags/{TAG}', capture=True, check=False).returncode == 0:
        if output('rev-parse', f'{TAG}^{{}}') != output('rev-parse', 'HEAD'):
            raise RuntimeError('Checkpoint tag already exists on another commit; it will not be moved.')
        if candidates():
            raise RuntimeError('Checkpoint tag already exists and new code changes remain; tag was not overwritten.')
        if not (WS / DOC / 'milestone.json').is_file():
            raise RuntimeError('Existing tag is missing the expected milestone evidence.')
        push(branch)
        return

    # Protect unrelated staged work, including siblings above bglx_ws.
    repo = Path(output('rev-parse', '--show-toplevel')).resolve()
    all_staged = run(['git', '-C', str(repo), 'diff', '--cached', '--name-only', '-z'],
                     capture=True).stdout.split('\0')
    for p in filter(None, all_staged):
        absolute = repo / p
        if not absolute.is_relative_to(WS) or not relevant(absolute.relative_to(WS)):
            raise RuntimeError('Unrelated staged changes are present; index was left untouched.')

    arrival, a = latest_success('continue_C_outside_*/summary.json', lambda d:
        d.get('success') is True and d.get('action_status') == 4
        and d.get('stopped_confirmed') is True and d.get('goal_error_m', 999) <= .85
        and d.get('route') == [[28.0, 82.0, 74.65]])
    ground, g = latest_success('ground_extension_*/summary.json', lambda d:
        d.get('passed') is True and d.get('files_saved') is True)
    world = WS / 'src/etrike_description/worlds/oxford_college.world'
    gen = WS / 'src/etrike_description/worlds/gen_oxford.py'
    marker = 'BGLX_CAMPUS_GROUND_VISUAL_V1_BEGIN'
    if marker not in world.read_text() or marker not in gen.read_text():
        raise RuntimeError('Floor extension has not been saved in both world and generator.')

    # Local restore material for the index and selected tracked edits; never clears the worktree.
    backup = Path.home() / 'bglx_navtest/backups' / ('git_C_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    backup.mkdir(parents=True)
    (backup / 'head.txt').write_text(output('rev-parse', 'HEAD') + '\n')
    (backup / 'worktree.patch').write_text(git('diff', '--binary', 'HEAD', '--', '.', capture=True).stdout)
    (backup / 'index.patch').write_text(git('diff', '--cached', '--binary', '--', '.', capture=True).stdout)
    for p in candidates():
        src = WS / p
        if src.is_file():
            dst = backup / 'files' / p
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dst)
    print('Checkpoint backup:', backup, flush=True)

    docs = WS / DOC
    scripts = WS / TOOL
    docs.mkdir(parents=True, exist_ok=True)
    scripts.mkdir(parents=True, exist_ok=True)
    for name in ('bglx_resume_C.sh', 'bglx_continue_C.sh', 'bglx_terrain_check.sh'):
        src = Path.home() / name
        if not src.is_file():
            raise RuntimeError('Missing proven mission helper: ' + str(src))
        target = scripts / name
        if target.exists() and target.read_bytes() != src.read_bytes():
            raise RuntimeError('A different tracked helper already exists: ' + str(target))
        shutil.copy2(src, target)
    for name in ('return_home_070.sh', 'checkpoint.py'):
        target = scripts / name
        if target.exists() and target.read_bytes() != (BUNDLE / name).read_bytes():
            raise RuntimeError('A different checkpoint helper already exists: ' + str(target))
        shutil.copy2(BUNDLE / name, target)
    shutil.copy2(arrival, docs / 'C_arrival_summary.json')
    shutil.copy2(ground, docs / 'ground_extension_summary.json')
    for label, path in [('arrival', arrival), ('ground_extension', ground)]:
        for xml in path.parent.glob('*.xml'):
            shutil.copy2(xml, docs / (label + '_' + xml.name))
    # Save the exact successful continuation stdout when it can be paired by timestamp.
    match = re.search(r'(\d{8}_\d{6})_', arrival.parent.name)
    if match:
        log = arrival.parent.parent / ('continue_C_outside_' + match.group(1) + '.log')
        if log.is_file():
            shutil.copy2(log, docs / 'C_arrival.log')
    for node in ('controller_server', 'planner_server', 'terrain_detector', 'cmd_vel_limiter'):
        print('Capturing live configuration:', node, flush=True)
        try:
            r = run(['ros2', 'param', 'dump', '/' + node], capture=True, check=False, timeout=15)
            (docs / ('live_' + node + '.txt')).write_text(r.stdout + r.stderr)
        except subprocess.TimeoutExpired:
            (docs / ('live_' + node + '.txt')).write_text('Parameter capture timed out.\n')
    from bglx_agentic.mission_waypoints import get_location
    waypoints = {name: get_location(name) for name in ('HOME', 'PICKUP_A', 'DELIVERY_A', 'BUILDING_B', 'DELIVERY_C')}
    milestone = {'checkpoint': TAG, 'destination': 'DELIVERY_C', 'C_arrival': 'succeeded',
                 'goal_error_m': a['goal_error_m'], 'continuation_cruise_m_s': .4,
                 'stopped_confirmed': True, 'ground_extension_persisted': True,
                 'return_home_070': 'not run at this checkpoint',
                 'parcel_unloading_at_C': 'not performed by standalone continuation',
                 'waypoints_xy': waypoints, 'source_arrival_summary': str(arrival),
                 'source_ground_summary': str(ground)}
    (docs / 'milestone.json').write_text(json.dumps(milestone, indent=2) + '\n')
    (docs / 'README.md').write_text(f'''# Delivery C arrival — 15 September 2026

The resized trike reached DELIVERY_C (28, 82) after the guided pillar passage
and a stationary repair to the Gazebo floor visual. The final continuation
ran at a requested 0.4 m/s and finished with Nav2 status 4, zero recoveries in
the supplied run log, goal error {a['goal_error_m']:.3f} m, and a confirmed stop.

This is a staged navigation milestone. The standalone continuation does not
perform parcel unloading or the return-HOME mission. The 0.7 m/s HOME return
is a new test and has not passed at this checkpoint.

## Floor correction

The standard ground visual ended at y=50. The front depth frames at the stop
were entirely 4.0 m, and terrain reported GROUND_FIT_FAILED. Four visual tiles
now extend the rendered ground to +/-200 m without overlapping the existing
floor or replacing its collision geometry. The world and gen_oxford.py both
contain the correction. Terrain stop and collision protections remain active.
A POSITIVE_OBSTACLE report beyond the terrain stop envelope remained after
ground fitting recovered; its origin is not resolved by this checkpoint.

## Replay tools

- tools/delivery_c/bglx_resume_C.sh: guided outbound passage from the recorded indoor pose.
- tools/delivery_c/bglx_continue_C.sh: ground verification/persistence and continuation from the recorded outside pose.
- tools/delivery_c/return_home_070.sh: HOME-only goal from C using the live map; requested speed 0.7 m/s.
- tools/delivery_c/bglx_terrain_check.sh: stationary terrain/depth capture.

These helpers retain simulation pose checks, existing recovery ownership,
Nav2 collision handling, and cancellation followed by stop confirmation.
The working SLAM map is still live in the running stack; this Git checkpoint
does not serialize the SLAM session. Do not restart the stack before the return test.
''')

    selected = candidates()
    if not selected:
        raise RuntimeError('No scoped changes to checkpoint.')
    syntax_check(selected)
    # Structured argv preserves spaces and avoids shell interpolation of paths.
    for begin in range(0, len(selected), 100):
        git('add', '-A', '--', *map(str, selected[begin:begin+100]))
    git('diff', '--cached', '--check')
    print('\nSelected checkpoint changes:', flush=True)
    git('diff', '--cached', '--stat')
    body = backup / 'commit-message.txt'
    body.write_text(TITLE + '''

Preserve the current prototype, mission recovery and navigation changes with
the successful staged Delivery C arrival. Add reproducible C test helpers,
the campus ground-visual extension in both world and generator, and run evidence.

The final 0.4 m/s continuation reached C and confirmed stopped odometry.
The requested 0.7 m/s return-HOME test is included as an unvalidated next trial.
''')
    git('commit', '--file', str(body))
    git('tag', '-a', TAG, '-m', 'Supervised Delivery C arrival; floor visibility restored; return test pending')
    push(branch)

if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print('\nCHECKPOINT STOPPED:', exc, file=sys.stderr, flush=True)
        print('No force-push/reset was attempted. The return test has not started.', file=sys.stderr)
        sys.exit(1)
