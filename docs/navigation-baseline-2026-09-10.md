# New-trike navigation simulation baseline — 2026-09-10

Tag: stable-nav-new-trike-2026-09-10

## Configuration and scope
- New trike footprint: x -0.38 to 1.63 m; y +/-0.52 m.
- GridBased: Smac Hybrid with DUBIN motion model.
- TightSpace: Smac Hybrid with REEDS_SHEPP motion model.
- Both planners load at startup; no live motion-model switching required.
- Ordinary navigation uses the plan-once behavior tree.
- Tight-space trial explicitly selected TightSpace and followed its path.
- Controller reversing was temporarily enabled and restored to false.
- Automatic planner selection is not implemented by this checkpoint.

## Validation
- Forward turning and 0.6 m box avoidance were observed to run smoothly.
- Complete 0.2 m nose-gap box trial succeeded with action status 4.
- Minimum observed box clearance: 0.199 m.
- Final distance to goal: 0.274 m.
- User reported smooth execution through reverse and forward travel.
- Clearance estimates use the requested box geometry and robot map pose,
  not independent Gazebo ground-truth tracking.

## Remaining work
- Forward-only 0.2 m planning produced an intersecting full-box audit.
  That collision-validity discrepancy remains unresolved.
- Validate pillar threading and offset approaches.
- Implement and test automatic forward/tight-space planner selection.
- Plan-once navigation does not provide automatic replanning when blocked.
- Precise final heading and real hardware steering transitions need validation.
- Speed-cap margin 0.15 m and hard floor 0.10 m are simulation test settings;
  braking and latency values remain uncalibrated for hardware.
- Shell/heredoc trial programs outside this repository are not preserved
  by this commit.
