# Recovery and alignment checkpoint — 2026-09-10

Tag: recovery-alignment-audited-2026-09-10

## Validation status
- Straight reverse recovery EXECUTED successfully.
- Alignment and subsequent passage PLANNED AND AUDITED only.
- Alignment execution, stage transitions and autonomous triggering remain pending.
- This checkpoint is not a claim of completed automatic recovery.

## Test configuration
- Trike footprint: x -0.38 to 1.63 m; y +/-0.52 m.
- Two static 0.6 x 0.6 x 1.0 m pillars.
- Clear gap 1.6 m; gap centre offset 0.5 m left.
- Pillars initially placed 6 m ahead of the starting trike.
- Tested inflation radius: 0.7 m in both costmaps.
- Tested speed: 0.1 m/s.
- Runtime settings are not persisted merely by creating this note.
- GridBased uses DUBIN; TightSpace uses REEDS_SHEPP.

## Finding
An offset approach triggered the diagnostic clearance guard at approximately
0.1494 m. Small cross-track error did not ensure adequate nose clearance.
Planning both motion models produced the same forward path in earlier tests.

Direct recovery alignment candidates included paths intersecting the complete
pillar geometry. External full-box audits rejected those candidates.

## Recovery sequence tested
1. Audit straight reverse distances of 0.3, 0.5 and 0.7 m.
2. Select 0.7 m for its larger subsequent alignment clearance.
3. Execute only the 0.7 m straight reverse using Nav2 BackUp.
4. Confirm the robot stopped.
5. Replan alignment from the actual stopping pose.
6. Audit passage from the planned alignment endpoint.

## Executed reverse result
- BackUp action status: 4 (succeeded).
- Odometry confirmed stopped.
- Minimum observed full-box clearance: 0.1511 m.
- Clearance increased during the recorded reverse.
- Negligible recorded lateral and heading change.
- Final Gazebo trike pose: x 3.31541 m, y 0.534989 m, yaw 0.046024 rad.

## Post-reverse planning result
- Current clearance: 0.7980 m.
- Alignment target in map: x 3.5012 m, y 0.5150 m, yaw 0.0001 rad.
- Alignment length: 4.866 m; entirely forward.
- Alignment minimum full-box clearance: 0.3762 m.
- Alignment intersections: 0.
- Planned endpoint position error: 0.015 m; heading error: 0.00 deg.
- Passage length: 6.500 m; entirely forward.
- Passage minimum full-box clearance: 0.2774 m.
- Passage intersections: 0; route passes through gap.
- Gap centre crossing offset: -0.003 m.

## Audit method and limits
Gazebo pillar and robot poses were queried read-only. The model's base_link
pose was confirmed identity relative to the model. World geometry was
transformed into the navigation map using the stationary robot pose.
Full rectangular footprints were sampled against complete pillar polygons.
Straight reverse was also screened against local and global costmaps.

Sampled audits are not continuous collision guarantees. Unknown physical
obstacles and localization/tracking error remain relevant. The planner's
collision-validity discrepancy against full-box geometry remains unresolved.

## Next work
- Execute the audited alignment with monitoring.
- Verify actual position and heading before passage; a permissive goal checker
  must not be treated as proof of accurate alignment.
- Re-audit passage from the measured alignment endpoint.
- Implement reusable recovery orchestration and automatic triggering.
- Preserve executable trial tools and logs in the repository.
- Validate rear sensing, rotated entrances, tighter approaches and people-aware
  navigation separately.

The experiments were run through terminal heredocs. This note preserves their
results and method; it does not itself install the recovery implementation.
