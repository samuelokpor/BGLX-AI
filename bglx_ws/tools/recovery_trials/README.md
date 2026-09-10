# Supervised recovery trials

Reusable reconstruction of the successful 2026-09-10 experiments.
This is not an automatic recovery plugin. Packaged-code regression is pending.

## Environment
Source ROS 2 Humble and bglx_ws/install/setup.bash.
Gazebo, SLAM/localization, Nav2, collision monitor and limiter must be running.
Models: bglx_etrike, pillar_test_left, pillar_test_right.
The two static pillars must each be 0.6 x 0.6 x 1.0 m, parallel, with a 1.6 m gap.
The trike model origin must coincide with base_link, as verified in the test.
Tested speed: 0.1 m/s. Tested inflation radius: 0.7 m in both costmaps.
No runtime parameter changes are persisted by this tool.

## Commands
From this directory, after freshly auditing available rear space:

    python3 recovery_trial.py reverse --execute --rear-space-checked

This commands another 0.7 m reverse every time it is run.
The flag is an operator confirmation, NOT an automated rear-space check.

After verifying the reverse stopped, run the separately selected passage stage:

    python3 recovery_trial.py passage --execute

This reads current Gazebo poses, replans directly through the gap, audits the
complete boxes, and follows the exact audited path. It does not execute the
previous intermediate alignment loop.

Without --execute, the program exits without sending movement commands.
Ctrl+C requests cancellation. Monitor terminal warnings and the simulator.
Save a log by using Bash pipefail and tee if desired.

## Limits
- Geometry uses Gazebo model poses and a stationary world-to-map alignment.
- Audits measure the two complete boxes; they do not cover every world obstacle.
- Existing Nav2/sensor protections remain in the motion chain.
- The reverse stage requires a separate fresh rear-space audit.
- The 0.15 m clearance trigger requests cancellation; it is not a braking guarantee.
- The normal permissive goal checker remains in use.
- No automatic recovery triggering, retry policy or planner selection is installed.
- Packaged scripts have been syntax-checked, not yet execution-regression-tested.
