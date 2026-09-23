# Vision-assisted navigation baseline

Helper snapshots from a successful home-to-C simulation trial.
Vision crossing confirmed; continuous-forward path 5.16 m; no backup; C reached with zero recoveries.

Deployment: copy the two helper directories and bglx_clean_log.py into the user's home directory. Their existing paths are retained.
Geometry remains in the bglx_agentic package.

Trial cruise: 1.5 m/s; passage controller: <=0.25 m/s.
Runtime planner max_iterations override: 5000000.
Runtime parameter overrides are documented here, not persisted into Nav2 configuration by this commit.
