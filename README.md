# ROMO-B waypoint navigation

ROS 2 Humble software for controlling the ROMO-B platform through USB-to-RS232,
mapping and localizing with one Livox Mid-360, and following YAML/RViz waypoints
with Nav2 local obstacle avoidance.

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
source robot_ws/install/setup.bash
colcon test --base-paths robot_ws/src --event-handlers console_direct+
colcon test-result --test-result-base robot_ws/build --verbose
```

`setup_host.sh` requires sudo for apt packages and adding the current user to
`dialout`. Log out and back in (or reboot) after group membership changes.

## Hardware workflow

1. Connect the USB-to-RS232 adapter without selecting PCU Auto mode.
2. Run `./scripts/onboard_hardware.sh --discover` (read-only discovery).
3. Complete `config/local/hardware.yaml`, including the measured LiDAR pose.
4. Run `./scripts/doctor.sh --hardware` until all mandatory checks pass.
5. Follow [hardware acceptance](docs/hardware_acceptance.md), beginning with a
   receive-only test and then a zero-speed frame.
6. Map by driving in RC Manual mode, create the 2D occupancy map, localize, and
   only then execute waypoints.

See [architecture](docs/architecture.md), [status](docs/status.md), and
[operations](docs/operations.md) before connecting the platform.

## Repository policy

Project code, configuration, scripts, documentation, and dependency locks are
tracked. Vendor checkouts, Autoware, build products, hardware-local settings,
rosbags, PCDs, and generated maps live below this repository but are ignored.
Large artifacts are recorded under `data/manifests/` with a SHA-256 and later an
external URI.
