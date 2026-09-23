"""Patch only the stationary handoff snapshot; retain installed motion modules."""
import ast
import shutil
from pathlib import Path
import time


def patch_handoff(source):
    tree = ast.parse(source)
    run = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'run')
    if any(isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
           and n.func.id == 'bglx_grid_context' for n in ast.walk(run)):
        return source
    changes = {'grid': 0, 'robot': 0, 'map': 0}

    def transform_call(value, frame):
        return (isinstance(value, ast.Call) and isinstance(value.func, ast.Attribute)
                and isinstance(value.func.value, ast.Name) and value.func.value.id == 'guard'
                and value.func.attr == 'transform' and len(value.args) == 2
                and isinstance(value.args[0], ast.Name) and value.args[0].id == 'frame'
                and isinstance(value.args[1], ast.Constant) and value.args[1].value == frame)

    class Update(ast.NodeTransformer):
        def visit_Assign(self, node):
            if len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
                name = node.targets[0].id
                if name == 'msg' and ast.unparse(node.value) == "guard.fresh('grid', 0.75)":
                    changes['grid'] += 1
                    return ast.parse('msg, robot, map_to_grid = bglx_grid_context(node, event=event)').body[0]
                if name == 'robot' and transform_call(node.value, 'base_footprint'):
                    changes['robot'] += 1
                    return None
            return self.generic_visit(node)

        def visit_Call(self, node):
            if transform_call(node, 'map'):
                changes['map'] += 1
                return ast.Name(id='map_to_grid', ctx=ast.Load())
            return self.generic_visit(node)

    Update().visit(run)
    if changes != {'grid': 1, 'robot': 1, 'map': 1}:
        raise RuntimeError('Unexpected handoff layout; no changes: ' + str(changes))
    insert_at = 1 if isinstance(tree.body[0], ast.Expr) and isinstance(tree.body[0].value, ast.Constant) else 0
    tree.body.insert(insert_at, ast.parse('from bglx_tf_sync import grid_context as bglx_grid_context').body[0])
    ast.fix_missing_locations(tree)
    result = ast.unparse(tree) + '\n'
    compile(result, 'vision_handoff.py', 'exec')
    return result


def main():
    here = Path(__file__).resolve().parent
    installed = Path.home() / 'BGLX_Existing_Mission_v01'
    path = installed / 'vision_handoff.py'
    source = path.read_text()
    updated = patch_handoff(source)
    for name in ('analyze.py', 'look_at_opening.py', 'resume.py', 'rank_model.py', 'source_hashes.json'):
        if not (installed / name).is_file():
            raise RuntimeError('Missing existing dependency: ' + name)
    if '# BGLX synchronized-frame selection v1' not in (installed / 'analyze.py').read_text():
        raise RuntimeError('Install the earlier recorded-frame synchronization fix first')
    look_api = {n.name for n in ast.parse((installed / 'look_at_opening.py').read_text()).body
                if isinstance(n, ast.FunctionDef)}
    if not {'choose', 'project'} <= look_api:
        raise RuntimeError('Existing viewing helper API differs; no installation performed')
    helper = installed / 'bglx_tf_sync.py'
    if helper.exists() and helper.read_bytes() != (here / 'bglx_tf_sync.py').read_bytes():
        helper.with_name(helper.name + '.before_update_' + str(time.time_ns())).write_bytes(helper.read_bytes())
    shutil.copy2(here / 'bglx_tf_sync.py', helper)
    if updated != source:
        backup = path.with_name(path.name + '.before_exact_tf_wait_' + str(time.time_ns()))
        backup.write_text(source)
        temporary = path.with_name(path.name + '.pending')
        temporary.write_text(updated)
        temporary.chmod(path.stat().st_mode & 0o777)
        temporary.replace(path)
        print('Handoff TF wait installed. Backup:', backup)
    retired = installed / 'repeat_home_vision_c.py'
    notice = "raise SystemExit('Fixed-waypoint runner retired. Use ~/BGLX_Live_Vision_v01/run.py --execute --cruise 1.5')\n"
    if retired.exists() and retired.read_text() != notice:
        backup = retired.with_name(retired.name + '.before_live_vision_' + str(time.time_ns()))
        backup.write_bytes(retired.read_bytes())
        retired.write_text(notice)
        print('Retired fixed observation route. Backup:', backup)
    print('Ready: dynamic sensor-triggered inspection; existing recovery/alignment sources unchanged.')


if __name__ == '__main__':
    main()
