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
The command path is fixed as Nav2/teleop → twist_mux → velocity smoother →
Collision Monitor → serial bridge.

## Data

Generated data belongs in `data/local/`. Create a manifest with
`scripts/register_artifact.sh`; it calculates size and SHA-256. `uri: pending`
is accepted during software development but must be replaced before the first
real-data acceptance run.

`pcd_to_occupancy` marks the selected-height PCD points as occupied and assumes
the padded PCD bounding box is free elsewhere. Inspect and edit the generated map
before navigation; a PCD alone does not prove that every unobserved cell is free.

For offline mapping, keep the launch running until RKO-LIO has consumed the bag,
then call `/map_save`. A successful run must leave non-empty `map.pcd` and
`pose_graph.g2o` files in the selected `save_dir`. The upstream offline node can
remain alive while draining its tail buffer after the final scan; once `/map_save`
has succeeded and the two files are stable, stop the launch with Ctrl+C.

The tracked mapping RViz profile uses `map` as its fixed frame and displays
`/rko_lio/frame`, `/rko_lio/odometry`, `/modified_map`, and `/modified_path`.
The identity `map -> odom` transform in that launch exists only for offline map
visualization; runtime localization remains the sole owner of `map -> odom`.
