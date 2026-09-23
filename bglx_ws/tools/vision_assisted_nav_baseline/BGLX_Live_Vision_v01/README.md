# BGLX live vision-assisted C trial

This supersedes the fixed observation waypoint in `repeat_home_vision_c.py`.
Only the delivery destination C is a site coordinate. Inspection locations are
computed from current lidar measurements; no saved observation pose is loaded.

## What changes

The runner monitors nearby scan-supported openings during the existing Nav2 C
mission. Eligible openings must be 1.0–2.1 m wide, within 2–7 m, on the approach
side with room before the mouth, and oriented toward the destination. The
nearest eligible opening must persist across two distinct scans during transit.
The log lists all detected openings and eligibility/rejection reasons. Width and
distance thresholds are simulation policy, not learned environmental knowledge.

When an inspection is triggered, the current mission call unwinds and the
existing recovery guard cancels Nav2 and confirms stopped odometry. The runner
then recomputes candidates. If necessary, it calls the existing viewing-curve
generator and existing audited `follow_checked` controller. Qwen ranks a fresh
stationary capture. The installed `vision_handoff` performs the original
footprint/connector checks, backup and passage alignment. It dispatches C only
after confirming that crossing. Normal original alignment and recovery remain
available on the final C leg.

There is no independent velocity publisher or replacement driving controller.
Requested cruise is 1.5 m/s by default, subject to the existing controller and
limiter. Viewing and passage manoeuvres retain the existing 0.25 m/s setting.
The original backup speed, geometry margins and sensor checks are retained.

The TF repair pins a fresh costmap timestamp and spins callbacks for at most
1.5 seconds to obtain both required transforms at that exact timestamp. It
replaces a snapshot if its existing 0.75-second freshness bound expires. It
does not fall back to latest TF or extrapolate. This directly covers the
reported 17 ms map/TF arrival race. Moving-state inspection uses nonblocking
lookups of recent buffered scans; a monitor data outage lasting 2.5 seconds
aborts the trial.

## Installation (existing ROS stack can stay running)

Extract the archive under `$HOME`, then run:

```bash
python3 ~/BGLX_Live_Vision_v01/install.py
```

The installer backs up and patches only the installed vision handoff, adds the
TF helper, and retires the old fixed-waypoint entry point with a clear message.
It does not modify the source-hashed BGLX mission, recovery or alignment modules.
It requires the existing helpers and recorded-frame synchronization patch from
this session. It preserves the installed 1.5 m/s handoff speed limit.

## Check current position without movement

```bash
cd ~/projects/BGLX/bglx_ws &&
source /opt/ros/humble/setup.bash && source install/setup.bash &&
python3 ~/BGLX_Live_Vision_v01/run.py --cruise 1.5
```

If a viewing movement is needed, the check audits and saves that curve only;
vision/crossing still require fresh inspection after executing the movement.
At HOME with no nearby eligible opening, it reports that transit monitoring
would be required. Neither case certifies a complete C route.

## Repeat from HOME using the existing mission

```bash
cd ~/projects/BGLX/bglx_ws &&
source /opt/ros/humble/setup.bash && source install/setup.bash &&
python3 ~/BGLX_Existing_Mission_v01/resume.py --mode return-home --execute &&
python3 ~/BGLX_Live_Vision_v01/run.py --execute --cruise 1.5 --model qwen2.5vl:7b
```

The live runner can also execute from a stopped intermediate pose. Testing from
HOME is necessary to assess whether earlier openings are considered; running
from the old viewing position naturally cannot reproduce that earlier travel.

Expected events include `live_openings`, `inspection_trigger`,
`handoff_cancel_requested`, `handoff_stopped`, `view_plan` or `opening_in_view`,
`vision_selected`, `vision_crossing_complete`, and the C arrival result.
Logs and summaries are under `~/bglx_navtest/logs/live_vision_*`.
Reaching C without a confirmed vision crossing is explicitly not counted as a
successful vision trial. If viewing/vision/geometry refuses, the robot stays
stopped; this version does not silently drive on to a stored fallback point.

RViz's manoeuvre Path display should subscribe to `/bglx/alignment_path` before
execution. The ordinary `/plan` is a different path topic.

## Validation and limits

Offline tests cover recovery from the 17 ms TF lag at an unchanged timestamp,
permanent TF failure, movement refusal during stationary synchronization,
selection of earlier nearby openings, translation invariance of scene geometry,
Nav2 cancellation before inspection, cancellation failure, preservation of
original alignment polling, and rejecting C-only arrival as a vision success.
They also check that installation preserves the geometric and cruise guards.

```bash
cd ~/BGLX_Live_Vision_v01 && python3 -m unittest -v test_offline.py
```

This package has not been driven in the user's live simulator by the assistant.
It reuses the existing 2D geometry and sensor protections; camera/model ranking
is not ground-clearance, overhead or full-route certification. Seeing an
opening does not guarantee an audited viewing curve or crossing is feasible.
