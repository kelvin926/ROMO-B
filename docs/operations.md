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
- `navigation`: 0.2 m/s, +/-22 degrees, forward-only 2WIS.

The bridge starts inactive and disarmed. Activating the lifecycle node does not
arm motion. Use `/romo_b/arm` only after `doctor.sh --hardware` is green.

The navigation profile adds a second gate: `/romo_b/arm` is rejected unless
`lidar.calibrated: true` was loaded from the hardware-local YAML. Bench profile
tests remain possible without LiDAR calibration, within their lower limits.

Nav2 uses a custom `NavigateThroughPoses` behavior tree containing only repeated
planning and path following. It has no spin, reverse, or backup recovery action.
The waypoint manager publishes the YAML `default_speed_mps` as an absolute Nav2
speed limit before submitting each route.

## Live field navigation

After the measured-speed calibration passes, start the complete stack with:

```bash
./scripts/run_field_navigation.sh
```

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

The tracked mapping RViz profile uses `map` as its fixed frame and displays
`/rko_lio/frame`, `/rko_lio/odometry`, `/modified_map`, and `/modified_path`.
The identity `map -> odom` transform in that launch exists only for offline map
visualization; runtime localization remains the sole owner of `map -> odom`.

Two point-cloud products are intentional. The obstacle topic
`/sensing/lidar/top/pointcloud_filtered` is transformed to `base_footprint` and
height-sliced to 0.10--1.80 m. Localization consumes
`/sensing/lidar/top/pointcloud_localization`, transformed to `base_link` with
full vertical structure retained. Both preserve Livox intensity and omit its
vendor nanosecond timestamp field. Reusing the obstacle slice for 6-DoF NDT can
make the solution degenerate and is prohibited.
