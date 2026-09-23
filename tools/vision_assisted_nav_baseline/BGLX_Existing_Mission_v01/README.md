# BGLX existing mission adapter v01 — simulation

Uses the installed bglx_agentic DeliveryMission, RecoveryGuard and PassageAlignment.
Does not install, replace or patch workspace source or Nav2 parameters. Requires
reviewed source hashes to match; dependencies and results are logged. Do not run
another mission process alongside this adapter.

## Immediate recovery
Run from a sourced ROS2 Humble workspace:

    python3 ~/BGLX_Existing_Mission_v01/resume.py --mode return-home --recover-first

Default is read-only: dependency hashes, original guard readiness, pose and the
original reverse audit including stopping reserve. This is NOT forward path approval.
If it passes:

    python3 ~/BGLX_Existing_Mission_v01/resume.py --mode return-home --recover-first --execute

Execution requests ONE original guarded backup before navigating HOME. A refused
or failed backup blocks HOME. Backup distance/speed come from the original mission
(default 0.90m and 0.10m/s), with its live rear/side scans, footprint, costmap,
stopping reserve and continuous monitoring. Existing automatic recoveries can
still occur later under the original mission limits. Cruise must already be
<=0.30m/s; the adapter refuses a faster profile, without changing it.

## HOME-to-C, after successful recovery

    python3 ~/BGLX_Existing_Mission_v01/resume.py --mode home-to-c --vision-shadow
    python3 ~/BGLX_Existing_Mission_v01/resume.py --mode home-to-c --vision-shadow --execute

HOME-to-C requires starting within 1m of HOME. It targets (28,82,1.30285) in the
CURRENT map frame via the existing navigation method (the reviewed version dispatches a direct goal); it does not
perform pickup/unloading or automatic return. Confirm these saved destination
coordinates still belong to the running SLAM session.

The original alignment poll selects geometric openings on Nav2's route. When it
requests a manoeuvre, DeliveryMission cancels and settles navigation first. The
adapter then records fresh camera/depth/scan data and logs Qwen advice before
calling the SAME original alignment.perform method. Original manoeuvre geometry,
audited backup, controller settings restoration and command ownership remain.

Vision is SHADOW advice in this version: its ranking neither chooses a new route
nor certifies clearance. No camera-visible matching opening is not motion approval.
This explicitly restores the original geometric safety policy. The experimental
Vision-C wrapper's all-121-depth-samples gate and extra global raster margin are
not part of this adapter; do not label this as depth-certified navigation. Existing
terrain/collision layers must stay active. The original alignment only triggers
when its geometric conditions are met (including sufficient remaining goal range,
stable opening detection, attempt limits and actual need for alignment). It may
not trigger during a particular run; inspect logs rather than claiming success.

## Validation
Six offline ownership/dispatch tests cover original navigation delegation, backup
failure blocking, stationary handoff, and camera failure blocking. ROS/Gazebo live
execution has NOT been tested here. Check output on the robot before continuing.
All original source is imported from the user's installed workspace; no stale copy
is bundled. If a hash differs, share the printed dependency log before running.
