# Validated simulation case: 1.4 m angled gap

Recorded: 2026-09-11

## Scenario
- ROS 2 Humble / Gazebo simulation; new trike model.
- Rectangular audit footprint: x=-0.38..1.63 m, y=-0.52..0.52 m.
- Two 0.6 m square pillars; clear gap 1.4 m.
- Pillars 6 m ahead of the starting robot, offset -0.5 m, angle -15 degrees.
- Selected continuous route via a point 3.0 m before the pillar centre.
- Forward command speed: 0.1 m/s.
- Temporary fixed RPP lookahead: 0.6 m; original settings restored afterward.
- Inflation radius used for the experiment: 0.7 m.
- Planning clearance threshold: 0.17 m.
- Execution clearance guard: 0.15 m; tracking-error guard: 0.10 m.

## Executed result
- Planned path length: 10.101 m; no planned reverse.
- Planned minimum full-box clearance: 0.1773 m.
- Minimum sampled execution clearance: 0.1622 m.
- Largest printed cross-track error: 0.033 m.
- Final goal distance: 0.295 m.
- Final heading error: +0.09 degrees.
- Action status: 4 (succeeded); odometry confirmed stopped.
- Evidence: successful_trial.log.

## Scope and limitations
This validates one supervised simulation execution.
It does not certify a production-safe minimum gap or repeatability.
Clearance uses known simulated box geometry and sampled robot poses.
The margin above the execution guard was 0.0122 m.
The earlier 4 m approach failed; this case used more approach space.
Runtime settings listed above are experimental, not necessarily startup defaults.
Other tools committed alongside this case include exploratory and failed trials;
their inclusion does not imply validation.

## Reproduction
Launch the simulation stack and start from home in a suitable clear area.
Apply the recorded experimental settings, then run:
python3 -u tools/recovery_trials/early_align_gap_14_trial.py

This script spawns test obstacles and can move the trike.

## Next development
Build a supervised sequence that evaluates direct passage, early alignment,
and audited retreat candidates. Verify stops and achieved poses between
separate manoeuvres, replan from measured poses, preserve collision checks,
and bound retries. This automatic sequence is not implemented by this checkpoint.
