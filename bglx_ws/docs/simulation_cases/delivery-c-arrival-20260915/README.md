# Delivery C arrival — 15 September 2026

The resized trike reached DELIVERY_C (28, 82) after the guided pillar passage
and a stationary repair to the Gazebo floor visual. The final continuation
ran at a requested 0.4 m/s and finished with Nav2 status 4, zero recoveries in
the supplied run log, goal error 0.308 m, and a confirmed stop.

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
