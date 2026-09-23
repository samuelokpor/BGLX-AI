# Vision Assisted Nav

ROS 2 Humble / Gazebo simulation baseline.

Validated twice: HOME to C with live lidar-triggered inspection, stationary
Qwen2.5-VL 7B ranking, original guarded backup/alignment, confirmed pillar
crossing, C arrival and confirmed stop. Cruise request: 1.5 m/s; measured
approximately 1.498 m/s. Viewing/crossing manoeuvres: <=0.25 m/s.
Both reported successful runs had one Nav2 recovery after crossing.
Evidence is recorded under evidence/. No saved observation waypoint is used.

## Restore helpers after checking out this tag

From bglx_ws, copy both BGLX_* folders in this directory into your home
folder, preserving any existing versions as backups first. The scripts
currently expect those home-folder paths. Do not re-extract older ZIPs over
them. Build/source the ROS workspace and start Gazebo, SLAM and Nav2.
Then run:

    python3 ~/BGLX_Live_Vision_v01/install.py
    python3 ~/BGLX_Live_Vision_v01/run.py --execute --cruise 1.5 --model qwen2.5vl:7b

RViz: /bglx/alignment_path is the planned manoeuvre Path (purple, volatile).
Optional vision_rviz.py publishes /bglx/vision/status_markers (transient local).
RViz colour/display settings must be configured or loaded separately.
Ollama, the model weights and live SLAM maps are separate runtime dependencies.

## Known limitations

The conversational agent is not connected to this live-vision runner.
Its landmark store did not contain DELIVERY_C in the reported chat test;
the model incorrectly substituted entrance and drove there. That is a failed
agent instruction, not a C mission success. Do not treat the conversational
agent as validated by this tag. Camera rankings remain advisory; geometry,
freshness and original controller checks decide whether passage can execute.
