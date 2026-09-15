# Supervised four-pillar simulation course

This installs a separate snapshot-based supervisor in `tools/course_supervisor`.
The source is the supplied September 14 snapshot following checkpoint
`10adadb`, with the validated full-course and direct-first continuation logic.
Existing experiments and configuration files are not modified.

## Commands

From the ROS workspace after sourcing ROS Humble and install/setup.bash:

- `python3 tools/course_supervisor/run.py --setup`: replace the named test pillars relative to the stopped robot, then audit. No driving or velocity-parameter changes.
- `python3 tools/course_supervisor/run.py --execute`: audit and drive the existing course. Does not spawn/delete anything.
- `python3 tools/course_supervisor/run.py --setup --execute`: setup, audit, and drive from a suitable clear starting pose in one invocation.
- `python3 tools/course_supervisor/run.py`: audit the existing course without motion.

Scene: two 1.6 m clear gaps, centres at (5, +0.75), angle +15 degrees;
(10, -0.75), angle -15 degrees, relative to the setup pose.
Named models: pillar_test_left/right, pillar_s2_left/right. Setup also removes box1 if present.
Forward target 0.25 m/s, temporary fixed lookahead 0.6 m. Parameters are restored after confirmed stopping.
Plan clearance 0.17 m; sampled execution clearance guard 0.15 m; tracking guard 0.10 m; minimum turning radius 0.75 m.

## State and recovery

Setup and course workers return atomic JSON with per-stage identity, code,
stopped and cleanup status. The supervisor never interprets console phrases
as authorization to move. Each course worker remeasures all four Gazebo pillar
poses and the robot pose. It chooses the full-course construction while gate 1
is not fully behind the footprint, otherwise the validated direct-first gate 2
continuation with alignment alternatives.

Only FRAME_CHANGED with terminal failure status, confirmed stopping and clean
cleanup may retry. At most two retries, configurable downwards with --max-replans.
Clearance, tracking, stale TF, observations, or cleanup failures stop the sequence.
No automatic reverse is implemented for this S-course. Some intermediate poses
remain unsupported: a retry can correctly stop if no candidate fits its guards.
In particular, the existing continuation excludes starts too close to/past gate 2.
No guaranteed recovery from every possible course pose is claimed.

Ctrl+C is forwarded to the active worker for cancellation. No new worker starts
after interruption. A lock prevents simultaneous instances of this supervisor;
it does not lock unrelated navigation clients.

## Evidence and display

Each run writes stage logs, JSON results and summary.json under
~/bglx_navtest/logs/course_*. Paths publish on
/supervised_gap/s_course_plan and /supervised_gap/selected_path (map frame).
Topics are available while workers are running; they are not a persistent replay.
Fresh observed costmaps are checked globally before execution and locally along
the next 0.5 m during motion, including the footprint. Pillar observation checks
are not proof of visibility of every obstacle or rear sensor coverage.

## Validation

Nine ROS-independent tests cover result identity, state decisions, audit/action
separation and setup's lack of action requests. Mock subprocess runs verified
map-correction retries in both audit and execution modes. Runtime files compile.
No ROS/Gazebo motion was run in the development environment. The earlier manual
course completion is evidence for the inherited components, not a completed
regression run of this supervisor.

Reference configurations in the distribution record the uploaded working files.
They are not applied by the installer; the runtime stack must still provide the
same working navigation settings, sensors, frames and services.
