# Executed reverse and passage checkpoint — 2026-09-10

Tag: recovery-passage-validated-2026-09-10

## Executed result
- A 0.7 m straight reverse succeeded; odometry confirmed stopped.
- Reverse minimum observed box clearance: 0.1511 m.
- Clearance increased during reversing with negligible recorded lateral/heading drift.
- A direct passage was replanned from the actual backed-up pose and executed.
- Passage action status: 4 (succeeded).
- Passage minimum observed full-box clearance: 0.2658 m.
- Final goal distance: 0.292 m; odometry confirmed stopped.
- User reported smooth passage.

## What was not executed
The intermediate alignment target generated a 4.866 m loop, but its motion action
immediately succeeded because the robot was already inside the goal tolerance.
That loop was not executed. The successful sequence was reverse, then direct
replanning and passage.

## Scope and remaining work
The reusable implementation reconstructs the terminal experiments, using their
geometry helpers. Syntax validation passed; packaged execution regression is
pending. The terminal experiments validate the maneuver, not this new package.

This is supervised simulation tooling, not installed automatic recovery.
Runtime inflation changes are not automatically persisted by this checkpoint.
The forward planner/full-box collision discrepancy remains unresolved.

Next session:
1. Regress the packaged scripts in the same controlled setup.
2. Implement automatic stop, rear-space validation, bounded reverse selection,
   reverse execution, measured-pose replanning, and passage validation.
3. Repeat the original 4 m offset scenario.
4. Test blocked rear space and rotated entrances.
