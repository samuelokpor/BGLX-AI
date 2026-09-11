# Four-pillar S-course simulation checkpoint

Completed a supervised Gazebo course with two 1.6 m gaps:
- Gate 1: 5 m ahead, +0.75 m offset, +15 degrees.
- Gate 2: 10 m ahead, -0.75 m offset, -15 degrees.
- Forward target speed: 0.25 m/s.
- Full-footprint geometry checked against all four pillars.
- Execution clearance guard: 0.15 m.
- Tracking guard: 0.10 m.
- Local forward occupancy and gate observations checked.

The course required controlled stops after map/odom corrections,
followed by fresh scene measurements and continuation audits.
The final continuation used a direct route through gate 2.

Final continuation evidence:
- Action status: 4; stopped odometry confirmed.
- Minimum sampled clearance: 0.2106271321456892 m.
- Final goal error: 0.27816675987858913 m.
- Original controller parameters restored.

This records one completed simulated course with supervised resumptions.
It does not establish uninterrupted passage, repeatability, or hardware readiness.
The retry launcher was executed from a terminal heredoc; this checkpoint
preserves the motion and resume scripts, not an installed end-to-end launcher.

Configuration YAML changes are outside this commit.
