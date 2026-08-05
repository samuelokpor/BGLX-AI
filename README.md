# BGLX — Autonomous Last-Mile Delivery E-Trike

**A delivery tricycle you operate in plain language.** Tell it what to do; it
plans, drives, and — when it fails — explains why in terms a person or an
agent can act on.

An autonomy stack for a steered electric cargo trike, developed in simulation
and on hardware in parallel. The target is campus and institutional
logistics: contained, low-speed, geofenced environments where last-mile
autonomy is tractable today rather than aspirational.

![BGLX autonomous e-trike in the campus world](docs/trike.png)

## What's different

**Natural-language operation.** Operators describe outcomes, not coordinates.
BGLX takes the task in language and works out the goals itself.

**Machine-legible failure.** Nav2 reports `ABORTED` — a status code no agent
can act on. BGLX translates robot state into a diagnosis an operator or an
agent can reason about, so the fleet gets itself unstuck instead of waiting
for a human.

**Inference runs locally.** The agent runs on the vehicle's own compute, with
cloud backends available but not required.

## Why a tricycle

A cargo trike cannot rotate in place. With a 1.33 m wheelbase and a limited
steering angle the minimum turning radius is large, and yaw rate is
proportional to forward speed — at standstill, steering does nothing. That
single constraint propagates through the whole stack: the global planner must
produce kinematically feasible paths, the local controller must never command
in-place rotation, Nav2's default `spin` recovery is useless, and the vehicle
recovers from a stuck pose by **reversing**, not pivoting.

## Status

| Capability | Simulation | Hardware |
|---|---|---|
| Steered-tricycle kinematics (`ros2_control`) | Working | Controller written, untested on vehicle |
| SLAM (`slam_toolbox`) — full campus mapped & saved | Working | First map captured |
| AMCL localization on the saved map | Working | Not yet |
| Nav2 navigation — Smac Hybrid (Dubins, forward-only) + Regulated Pure Pursuit | Working | Not yet |
| Full-perimeter costmap — 360° LiDAR + front/left/right/rear depth | Working | Depth mounts modelled |
| Reverse recovery (no-spin BT + `BackUp`) | Working | Not yet |
| IMU odometry (MPU6050) | — | Publishing to `/imu` |
| Frontier exploration | Working (mapping complete) | Not yet |
| Natural-language task control | Working | Not yet |

## Stack

**Planning.** Nav2 `SmacPlannerHybrid` with a bounded minimum turning radius
and **Dubins (forward-only)** motion, so global paths are drivable by the
actual vehicle. **Regulated Pure Pursuit** for local control with
`use_rotate_to_heading: false` and velocity scaling through tight curvature.

**Recovery.** A steered trike can't execute Nav2's default `Spin`, so
navigation runs a **no-spin behavior tree**: on failure it clears the
costmaps, reverses a short distance (`BackUp`), and re-plans. Rear depth
sensing makes that reverse safe.

**Sensing.** A 360° LiDAR (masked within 0.45 m of the mast to reject
self-hits) plus **four depth cameras — front, left, right, rear**. The
side/rear depth clouds feed the **local** (reactive) costmap voxel layer,
closing the LiDAR's close-range blind ring around the vehicle body — the
failure mode where the trike scraped obstacles alongside itself. The
**global** costmap plans over the saved map + LiDAR only, to stay light on a
large map. A `cmd_vel_limiter` node bridges Nav2's Twist output to the
tricycle steering controller and enforces a speed-dependent steering limit.

**Localization.** `slam_toolbox` builds the map. For operations the campus map
is saved (serialized pose graph + occupancy grid) and localized against with
**AMCL** (OmniMotionModel tuned for the trike) — lighter than continuous SLAM
and, crucially, it publishes a stable `map→odom` with no gaps.

**Exploration.** An info-gain / path-cost frontier explorer selects goals;
Nav2 does the planning and driving. Used to build the campus map, now
complete.

**The agentic layer.** `bglx_agentic` exposes tools over Nav2, TF and the
LiDAR, and turns robot state into text a model can reason about — sectorised
LiDAR, width checks, landmark navigation, and failure translation.

## Packages

| Package | Role |
|---|---|
| `etrike_description` | URDF (modular xacro, incl. `perimeter_sensors.xacro`), Gazebo worlds, sensor models, `ros2_control` tricycle interface, RViz views |
| `bglx_navigation` | Nav2 bringup + params, no-spin recovery BT, AMCL localization launch, `cmd_vel_limiter` |
| `bglx_autonomy` | Waypoint follower with LiDAR front-cone obstacle stop |
| `bglx_exploration` | Info-gain / path-cost frontier explorer |
| `bglx_agentic` | Sensor and failure translation, LLM tool layer and agent loop |

## Run it

ROS 2 Humble, Gazebo Classic, plus `navigation2`, `nav2-bringup`,
`slam-toolbox`, `ros2-controllers`, `gazebo-ros2-control`.

```bash
cd bglx_ws
colcon build --symlink-install
source install/setup.bash
```

### A. Map the campus (only needed once)

```bash
ros2 launch etrike_description gazebo.launch.py     # sim + controllers
ros2 launch etrike_description slam.launch.py       # slam_toolbox (map->odom) + RViz
ros2 launch bglx_navigation navigation.launch.py    # Nav2
ros2 launch bglx_exploration explore.launch.py      # autonomous frontier exploration
```

Save the finished map (serialized graph = continue/localize, plus occupancy grid):

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$HOME/projects/BGLX/bglx_ws/maps/campus_full'}"
ros2 run nav2_map_server map_saver_cli -f "$HOME/projects/BGLX/bglx_ws/maps/campus_full"
```

### B. Operate on the saved map (normal run)

```bash
ros2 launch etrike_description gazebo.launch.py            # sim + controllers
ros2 launch bglx_navigation amcl_localization.launch.py   # map_server + AMCL (no slam_toolbox)
ros2 launch bglx_navigation navigation.launch.py          # Nav2
# then: natural-language control, or RViz 2D Goal Pose
ros2 run bglx_agentic agent
```

RViz (until it's folded into the localization launch):

```bash
ros2 run rviz2 rviz2 -d src/etrike_description/rviz/slam.rviz --ros-args -p use_sim_time:=true
```

## Known limitations

- **Reverse is recovery-only.** The planner is Dubins forward-only; the trike
  gets out of stuck poses via a straight `BackUp` behavior, not arcing
  reverse. Reeds-Shepp planning (arcing reverse) is future work now that rear
  depth sensing exists.
- **Perimeter depth on the local costmap only.** The global costmap plans over
  the saved map + LiDAR to stay light on an 80×136 m campus. Four depth
  cameras plus a large map is compute-heavy; sensor rates are tuned down.
- **`track_unknown_space` is disabled** on the global costmap so the planner
  will route through unexplored ground. Fine in simulation.
- **Failure diagnoses are validated in simulation only** and will be noisier
  with real wheel slip and outdoor LiDAR dropout.

## Open threads (next)

- **Speed/agility pass** — `desired_linear_vel`, lookahead, and collision
  horizon tuned together; check the tricycle controller's
  `reduce_wheel_speed_until_steering_reached` as a possible speed throttle.
- **Vision `look` tool** returns nothing — VLM backend needs fixing.
- **Fold RViz into `amcl_localization.launch.py`**; verify AMCL start pose.
- **Landmark/map coverage** — some landmarks may fall outside mapped ground;
  audit and re-map thin areas.

## Roadmap

- Hardware bring-up: throttle-by-wire, the `ros2_control` tricycle controller
  on the physical vehicle, IMU/GPS odometry fusion.
- Reeds-Shepp planning for arcing reverse, now that rear depth sensing exists.
- Validating the failure-translation layer against real-world noise.
- Measuring how far local inference gets before cloud models are needed.
