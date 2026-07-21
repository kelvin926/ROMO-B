# Operations

## Build overlays

```bash
./scripts/fetch_dependencies.sh
./scripts/build_all.sh
source vendor_ws/install/setup.bash
source robot_ws/install/setup.bash
```

Use `--project-only` while developing without Livox or localization sources.

## Profiles

- `bench`: 0.1 m/s, +/-5 degrees, Nav2 disabled.
- `navigation`: signed speed up to +/-0.5 m/s, +/-22 degrees in 2WIS,
  +/-18 degrees in counter-phase 4WIS, plus Pivot heading alignment capped at
  0.15 m/s tangential wheel speed. Nav2 remains forward-preferred; signed
  reverse is available to the browser deadman control and compatible planners.

The bridge starts inactive and disarmed. Activating the lifecycle node does not
arm motion. Use `/romo_b/arm` only after `doctor.sh --hardware` is green.

The navigation profile adds a second gate: `/romo_b/arm` is rejected unless
`lidar.calibrated: true` was loaded from the hardware-local YAML. Bench profile
tests remain possible without LiDAR calibration, within their lower limits.

Nav2 uses a custom `NavigateThroughPoses` behavior tree containing only repeated
planning and path following. Rotation Shim may Pivot for path and final-goal
heading alignment, but there is no unconditional spin, reverse, or backup
recovery action.
The waypoint manager publishes the YAML `default_speed_mps` as an absolute Nav2
speed limit before submitting each route.

## Live field navigation

After the measured-speed calibration passes, start the complete stack with:

```bash
./scripts/run_field_navigation.sh
```

This starts the Nav2 baseline, not the full Autoware graph. It defaults to
0.5 m/s and accepts `MAX_SPEED_MPS=...` only within the bridge's 0.5 m/s hard
ceiling. Set the pose with `2D Pose Estimate`, then use `Nav2 Goal` for a single
goal or `Publish Point` plus the waypoint services for a continuous route.
The laptop operator console is installed as a user service and remains at
`http://127.0.0.1:8765/` even when the field stack is stopped. Install or repair
that login-time service with:

```bash
./scripts/install_operator_ui_service.sh
```

The **Main** tab mirrors the manual's HLV test program while separating the HLV
software arm request from PCU-confirmed Auto feedback. It shows every Auto-entry
condition, all wheel speeds and steering angles, Alive counters, and signed
forward/reverse deadman control for 2WIS, 4WIS, and Pivot. Motion is sent only
while the on-screen button is held, at 20 Hz through `/cmd_vel_teleop`; releasing
it sends zero while the selected steering-mode request continues at 20 Hz.
Selecting 4WIS or Pivot therefore stays selected across forward/reverse holds;
if the browser control node disappears, the bridge timeout still returns to
2WIS.
**ZERO + MANUAL** is an explicit operator action. The UI never
exposes or transmits a software E-stop command. The **Navigation** tab sets an
initial map pose, submits/cancels a direct Nav2 goal, and manages waypoint routes.
The **System control** tab starts/stops the full field stack and shows the command
pipeline, sensors, node graph, and log path. **Diagnostics** includes all ROS
diagnostic key/value pairs. **Operations** exposes allow-listed buttons for live
mapping, map save, bag recording, Nav2/Autoware launch, localization replay,
software checks, builds, and map preparation, with artifact selectors, conflict
handling, PID, exit status, and live logs. Privileged host setup remains
terminal-only.
`source_env.sh` selects Cyclone DDS on configured hosts; set
`ROMO_B_RMW_IMPLEMENTATION=rmw_fastrtps_cpp` only for an intentional fallback.

The indoor localization profile deliberately matches only a 15 m map crop
around the RViz seed and uses LiDAR returns within 12 m. It also rejects any
single NDT correction larger than 1.5 m or 15 degrees, even when a repetitive
corridor produces a deceptively good fitness score. Place `2D Pose Estimate`
within roughly 1 m of the robot, drag its arrow in the actual forward direction,
and keep the robot still for the first two or three scans. If localization does
not settle, click again near a doorway, corner, pillar, or other asymmetric
feature; the localizer will retain the clicked/odometry seed instead of jumping
to a distant look-alike corridor.

In RViz, set the initial pose, add points with `Publish Point`, save them, then
execute only after the platform status is healthy:

```bash
ros2 service call /romo_b/waypoints/save std_srvs/srv/Trigger '{}'
ros2 service call /romo_b/arm std_srvs/srv/SetBool '{data: true}'
ros2 service call /romo_b/waypoints/execute std_srvs/srv/Trigger '{}'
```

Disarm after the route:

```bash
ros2 service call /romo_b/arm std_srvs/srv/SetBool '{data: false}'
```

There is intentionally no `/romo_b/software_estop` service. Command loss,
feedback loss, an Auto transition failure, and normal shutdown hold zero while
the software arm state is retained; they never set the HLV E-stop bit. Use the
physical/RC E-stop for an operator-requested emergency stop.

The command path is fixed as Nav2, the Autoware trajectory follower, or teleop
→ `twist_mux` → velocity smoother → Collision Monitor → `/cmd_vel_safe` →
serial bridge. The optional Autoware procedure is kept separate in
[`autoware.md`](autoware.md) and uses the same final safety chain.

## Data

Generated data belongs in `data/local/`. Create a manifest with
`scripts/register_artifact.sh`; it calculates size and SHA-256. `uri: pending`
is accepted during software development. Compact files at or below 10 MiB are
tracked for handoff. Recording databases remain ignored and their URI must be
replaced before another host needs the original data.

With a pose graph argument, `pcd_to_occupancy` compares obstacle height to the
nearest recorded base pose, raycasts only pose-to-return lines as free, and
leaves every other cell unknown. Obstacle cells override free rays. The legacy
mode without a pose graph marks obstacles but deliberately leaves all other
cells unknown. Inspect every generated map before navigation.

Generate and validate a map without contacting the robot:

```bash
MAP_RUN=data/local/maps/mapping-20260717-195653
BAG=data/local/bags/mapping-20260717-195653
mkdir -p "$MAP_RUN/nav2-raycast"
ros2 run romo_b_perception pcd_to_occupancy \
  "$MAP_RUN/map.pcd" "$MAP_RUN/nav2-raycast/map" \
  0.05 0.10 1.80 "$MAP_RUN/pose_graph.g2o" 15.0 0.171
./scripts/run_localization_replay.sh "$BAG" "$MAP_RUN"
```

The replay forces an isolated ROS domain, launches no serial bridge or velocity
publisher, records its localization outputs under `data/local/validation/`, and
returns nonzero when any acceptance threshold fails.

For offline mapping, keep the launch running until RKO-LIO has consumed the bag,
then call `/map_save`. A successful run must leave non-empty `map.pcd` and
`pose_graph.g2o` files in the selected `save_dir`. The upstream offline node can
remain alive while draining its tail buffer after the final scan; once `/map_save`
has succeeded and the two files are stable, stop the launch with Ctrl+C.

For live mapping while an operator drives in RC Manual, connect both hardware
links and run:

```bash
cd /home/hyunseo/ROMO-B
./scripts/run_live_mapping.sh
```

Keep the robot motionless until `/rko_lio/odometry` starts. The launch opens the
PCU in receive-only mode, starts the Mid-360, RKO-LIO, graph SLAM, RViz, and a
raw recovery bag. It cannot arm the robot or send a motion command. The yellow
`Live local map` in RViz updates during driving; the white optimized map is
updated by graph optimization and final save.

In a second terminal, validate the currently connected sensors. Keep still for
phase 1, then make gentle left and right turns in RC Manual during phase 2:

```bash
cd /home/hyunseo/ROMO-B
source scripts/source_env.sh
python3 scripts/check_mapping_calibration.py --require-lio
```

This checks acquisition rates, frames, point timestamp support, LiDAR/IMU time
alignment, stationary gyro bias and gravity scale, wheel zero speed, IMU/wheel
yaw sign/correlation/scale, RKO-LIO output, and agreement between the measured
hardware transform and the RKO-LIO transform. It only subscribes; it never
publishes a command.

Before stopping the mapping terminal, save and generate the Nav2 occupancy map:

```bash
./scripts/save_live_mapping.sh
```

The save helper selects the newest `data/local/maps/mapping-*` directory unless
the exact path printed by `run_live_mapping.sh` is passed as its argument.

Mapping does not indiscriminately mix all three motion estimates. RKO-LIO
tightly combines the raw LiDAR and the Mid-360 IMU and owns `odom -> base_link`.
Graph SLAM consumes that fused odometry plus deskewed clouds. Wheel odometry is
recorded and used as an independent yaw/scale cross-check. The navigation EKF
uses wheel odometry; raw Mid-360 acceleration remains excluded there because
past direct integration caused odometry divergence. This separation prevents
duplicate TF ownership and double-counting one motion estimate.

The tracked mapping RViz profile uses `map` as its fixed frame and displays
`/rko_lio/frame`, `/rko_lio/odometry`, `/modified_map`, and `/modified_path`.
The identity `map -> odom` transform in that launch exists only for offline map
visualization; runtime localization remains the sole owner of `map -> odom`.

Two point-cloud products are intentional. The obstacle topic
`/sensing/lidar/top/pointcloud_filtered` is transformed to `base_footprint` and
height-sliced to 0.15--1.80 m, range-limited to 8 m, and voxelized at 0.10 m.
Localization consumes
`/sensing/lidar/top/pointcloud_localization`, transformed to `base_link` with
full vertical structure retained. Both preserve Livox intensity and omit its
vendor nanosecond timestamp field. Reusing the obstacle slice for 6-DoF NDT can
make the solution degenerate and is prohibited.
