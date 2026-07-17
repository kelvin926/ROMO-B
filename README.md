# ROMO-B waypoint navigation

ROS 2 Humble software for controlling the ROMO-B platform through USB-to-RS232,
mapping and localizing with one Livox Mid-360, and following YAML/RViz waypoints
with Nav2 or Autoware Universe 1.8.0 local obstacle avoidance.

The protocol source of truth is the tracked
[verified manual extraction](ROMO-B_manual_verified_complete.md).

The supported host is **Ubuntu 22.04 x86_64 with ROS 2 Humble**. The V1 runtime
uses forward-only 2WIS Ackermann motion. Pivot, 4WIS, reverse recovery, and
rotate-in-place are deliberately disabled.

> **Safety:** This software and a single LiDAR are not safety-rated. Keep the
> physical/RC E-stop available, lift the wheels for first motion tests, and do
> not arm navigation until the sensor transform and serial protocol have been
> verified on the real hardware.

## Quick start (no hardware)

```bash
./scripts/doctor.sh --preflight
./scripts/setup_host.sh
./scripts/fetch_dependencies.sh
./scripts/build_all.sh --project-only
source scripts/source_env.sh
colcon test --base-paths robot_ws/src --event-handlers console_direct+
colcon test-result --test-result-base robot_ws/build --verbose
python3 scripts/test_pty_bridge.py
ros2 launch romo_b_sim simulation.launch.py
```

`setup_host.sh` requires sudo for apt packages and adding the current user to
`dialout`. Log out and back in (or reboot) after group membership changes.

The simulator creates `/tmp/romo_b_pcu`; it never opens the physical USB adapter.
The bridge comes up active but disarmed, so call `/romo_b/arm` explicitly for a
motion test.

## Hardware workflow

1. Connect the USB-to-RS232 adapter without selecting PCU Auto mode.
2. Run `./scripts/onboard_hardware.sh --discover`, then `--generate` (neither
   command opens the port).
3. Complete `config/local/hardware.yaml`, including the measured LiDAR pose,
   then run `./scripts/onboard_hardware.sh --apply` for the udev/network setup.
4. After the required reboot/replug, run
   `./scripts/onboard_hardware.sh --receive-only`; it reads and validates PCU
   feedback but never writes serial bytes.
5. Run `./scripts/doctor.sh --hardware` until all mandatory checks pass.
6. Follow [hardware acceptance](docs/hardware_acceptance.md), transmitting a
   zero-speed frame only after receive-only validation passes.
7. Map by driving in RC Manual mode, create the 2D occupancy map, localize, and
   only then execute waypoints.

See [architecture](docs/architecture.md), [status](docs/status.md), and
[operations](docs/operations.md) before connecting the platform. The Autoware
research runtime has a separate [installation and field runbook](docs/autoware.md)
and a concise [corridor field checklist](docs/autoware_field_checklist.md).

## Runtime launches

After completing `config/local/hardware.yaml`:

```bash
source scripts/source_env.sh

# Safe receive-only serial validation (Livox disabled)
ros2 launch romo_b_bringup hardware.launch.py

# Sensors and wheel/IMU EKF; still receive-only by default
ros2 launch romo_b_bringup hardware.launch.py use_livox:=true

# PCD localization (in another terminal after hardware bringup)
ros2 launch romo_b_bringup localization.launch.py \
  pcd_map:=data/local/maps/mapping_run/map.pcd

# Forward-only waypoint navigation
ros2 launch romo_b_navigation navigation.launch.py \
  map:=data/local/maps/map.yaml
```

The operator must separately select PCU Auto, verify diagnostics, disable
`receive_only`, and explicitly call `/romo_b/arm`. No launch file arms motion.

## Autoware corridor runtime

Autoware Universe 1.8.0 is now integrated for Lanelet2 route planning and
local obstacle avoidance. Its low-speed trajectory is still passed through the
same velocity limits and independent Collision Monitor used by Nav2.

```bash
./scripts/prepare_autoware_map.sh
./scripts/doctor.sh --autoware
./scripts/run_autoware_validation.sh
USE_RVIZ=true ./scripts/run_autoware_planning_sim.sh

# Live inspection only: no serial writes
RECEIVE_ONLY=true ./scripts/run_autoware_field.sh
```

See [Autoware operations](docs/autoware.md) before enabling serial TX. A live
Mid-360 cluster is always treated as an obstacle, including people, even when
its semantic class is unknown. The software-only acceptance requires a feasible
lateral avoidance response and, separately, a zero-speed response to a blocking
obstacle; it launches no PCU bridge.

Use RViz `Publish Point` or copy the tracked waypoint example:

```bash
mkdir -p ~/.ros
cp robot_ws/src/romo_b_navigation/config/waypoints/example.yaml \
  ~/.ros/romo_b_waypoints.yaml
ros2 service call /romo_b/waypoints/reload std_srvs/srv/Trigger '{}'
ros2 service call /romo_b/waypoints/execute std_srvs/srv/Trigger '{}'
```

For mapping, record live topics while RC-driving in Manual, then run the pinned
offline SLAM and occupancy conversion:

```bash
./scripts/record_mapping_bag.sh
ros2 launch romo_b_bringup mapping_offline.launch.py \
  bag_path:=data/local/bags/MAPPING_BAG \
  save_dir:=data/local/maps/mapping_run
ros2 service call /map_save std_srvs/srv/Empty '{}'
ros2 run romo_b_perception pcd_to_occupancy \
  data/local/maps/mapping_run/map.pcd \
  data/local/maps/mapping_run/nav2-raycast/map \
  0.05 0.10 1.80 \
  data/local/maps/mapping_run/pose_graph.g2o 15.0 0.171
./scripts/run_localization_replay.sh \
  data/local/bags/MAPPING_BAG data/local/maps/mapping_run
./scripts/run_nav2_preflight.sh \
  data/local/maps/mapping_run/nav2-raycast/map.yaml \
  data/local/maps/mapping_run/pose_graph.g2o
```

The pose-graph converter leaves unobserved cells unknown, raycasts free space
only between recorded poses and obstacle returns, and gives obstacle cells
precedence. The replay runs in isolated ROS domain 97 without serial or velocity
nodes and fails unless localization health, fitness, rejection, trajectory, and
loop-closure checks pass. Still review every generated map before motion.

## Repository policy

Project code, configuration, scripts, documentation, and dependency locks are
tracked. Vendor checkouts, Autoware, build products, hardware-local settings,
rosbags, PCDs, and generated maps live below this repository but are ignored.
Large artifacts are recorded under `data/manifests/` with a SHA-256 and later an
external URI.
