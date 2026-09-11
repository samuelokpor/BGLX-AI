# Supervised mirrored recovery: validated simulation case

Scene: 1.4 m gap, pillars 6 m ahead, +0.5 m lateral offset,
+15 degree corridor orientation. Desired speed: 0.1 m/s.

The mirror supervisor spawned the scene and attempted forward passage.
A clearance guard cancelled the first attempt near 0.1500 m.
After confirmed stopping, it automatically ran one reverse recovery.
The reverse-only guard cancelled a forward correction request.
After confirmed stopping and parameter restoration, the supervisor
freshly audited the actual pose and executed forward passage.

Results:
- Final forward action: status 4; stopped odometry confirmed.
- Planned passage clearance: 0.1800 m.
- Minimum sampled execution clearance: 0.1698 m.
- Final goal error: 0.305 m.
- Sequence completed without a manual recovery handoff.

Test settings include inflation radius 0.7 m in both costmaps,
temporary fixed RPP lookahead 0.6 m, and collision detection enabled.
Planning clearance: 0.17 m. Forward execution guard: 0.15 m.
Reverse escape uses its separately audited clearance rule.
Temporary controller parameters were restored after execution.

Scope:
One full mirrored simulation sequence, using known Gazebo pillar
geometry. Not a production safety claim or a general success-rate claim.
The scripts include experimental helpers; this result validates this
executed sequence, not every tool in the directory.

Known follow-up work:
- Replace log-string orchestration with structured stage results.
- Validate steeper approaches and larger offsets separately.
- Improve terminal reverse heading control.
- Establish repeatability and test blocked-rear recovery refusal.
