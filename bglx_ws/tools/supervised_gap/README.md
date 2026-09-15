# Supervised simulated gap sequence — v1

Built from source supplied at efb4599. This is a new, unvalidated orchestration
layer around the tested geometry/planning/action patterns. It is not an installed
Nav2 recovery plugin or a production safety system. Do not run concurrent goals.

## Commands (from bglx_ws with ROS Humble and install/setup.bash sourced)

- Local tests: `python3 tools/supervised_gap/test_screening.py`
- Spawn and audit at stationary home:
  `python3 -u tools/supervised_gap/runner.py --spawn --gap 1.4 --distance 6 --angle -15 --offset -0.5 --allow-retreat`
- Execute against those unchanged entities:
  `python3 -u tools/supervised_gap/runner.py --execute --allow-retreat`

Without --execute, no driving action is sent and no parameters are changed.
--spawn explicitly deletes box1/pillar_test_left/pillar_test_right and replaces
the pillars. Never use --spawn to resume a failed motion test. Existing-entity
mode measures actual Gazebo poses and accepts gaps approximately 1.4–1.6m.
Both pillars MUST be the known static 0.6 x 0.6 x 1m test boxes; dimensions are
assumed, not inferred from Gazebo. Robot base_link is assumed identical to model
frame, as established for this trike. No human detection or arbitrary gap
perception is implemented.

## Sequence

1. Require stationary odometry/fresh TF and controller collision checking;
   desired controller speed must be positive and <=0.1m/s.
2. Measure existing box poses; construct complete polygons in map coordinates.
3. Audit direct and continuous-alignment paths via setbacks 2,2.5,3,3.5,4m.
   Reject reverse paths, box intersections, insufficient clearance, wrong route,
   or endpoint mismatch. Rank accepted paths by clearance then path length.
4. Only if none pass and --allow-retreat is set: check 0.3,0.5,0.7m straight
   retreats in BOTH observed costmaps and against boxes, and audit continuations.
   Rank by forward clearance, then shorter retreat. Unknown/outside-grid reject.
5. Audit-only exits. Execution temporarily uses fixed lookahead0.6m.
6. If selected, execute at most ONE reverse at0.1m/s via Nav2 BackUp. Measure
   distance/tracking in odom. Refresh rear occupancy during the reverse.
7. Confirm stopped, remeasure scene and replan all forward candidates from the
   achieved pose. No blind reuse of the predicted continuation.
8. Execute the continuous forward path with general_goal_checker; monitor full
   box clearance, XTE, TF and map/odom frame correction. Cancel on failure.
9. Verify terminal result and stopped odometry; check goal position <=0.35m and
   beyond gap. Report final heading separately. Restore prior lookahead only
   after action termination and stop confirmation. Exit nonzero on failure.

## Limits

Planning minimum0.17m, execution guard0.15m, tracking guard0.10m are experimental
values, not calibrated production margins. Footprint x=-.38..1.63, y=+/- .52m.
Sample spacing5mm of translation plus1.72*heading change. Costmap reverse checks
use conservative cell coverage. No full-world ground-truth sweep is available;
other obstacles rely on Nav2's costmaps and live collision-checking chain.
Observed free space does not establish rear visibility. --allow-retreat requires
supervised clear rear space; it is not an autonomous visibility assurance.

Live forward safety uses Nav2 collision detection, collision monitor and limiter;
the extra script checks only the known boxes. A cancellation threshold is not
a guaranteed minimum braking clearance. Map/odom changes >2.5cm or0.5deg cancel
because frozen box coordinates may be invalid. This may deliberately stop on
SLAM corrections. Do not disable this guard to ignore coordinate disagreement.

No automatic retry after a forward clearance/tracking guard. Rerun audit from the
stopped scene to assess a recovery, or fix the underlying issue. No boundless
retry loop. Exact final heading alignment is NOT validated or enforced here.

Baseline experiments used inflation0.7m, limiter speed cap enabled, margin0.15,
floor0.10 and terrain positive hard stop disabled IN SIMULATION. Runner does not
change these startup settings; do not assume they persisted after a restart.
Validated trial files and configuration files are not overwritten by installation.

## Verification performed locally

Python compilation and seven ROS-independent tests on the actual runner's
screening functions passed. ROS2/Gazebo execution cannot be tested in this
workspace and remains required. The original efb4599 tag does NOT validate this
new runner. Keep the first audit and execution logs before committing a new tag.
