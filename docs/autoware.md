# Autoware Universe 1.8.0

Autoware is the route/behavior-planning UI for the corridor research setup. The
ROMO-B field stack keeps the already validated serial bridge, NDT localization,
and independent Nav2 Collision Monitor. Autoware supplies the Lanelet2 route,
trajectory, and local avoidance decisions.

The command path is:

`Autoware trajectory -> ROMO-B low-speed follower -> /cmd_vel_nav -> twist_mux
-> velocity_smoother -> Collision Monitor -> /cmd_vel_safe -> PCU bridge`

The Mid-360 cloud is clustered without semantic classification. Every cluster
remains `UNKNOWN`, so a person, cart, box, or other return is an obstacle even
when no neural classifier is available. Autoware first attempts a lateral path
shift inside the 2.4 m corridor. If no safe path exists, its velocity planner
slows/stops; the independent Collision Monitor also slows at 1.5 m and stops at
0.9 m. This is research software, not a safety-rated person detector.

Both moving and stationary avoidance are retuned for ROMO-B's 0.05--0.20 m/s
range. The tracked override enables UNKNOWN safety checks, removes the default
classification waiting period, caps lateral shift to 0.65 m, and uses reliable
stop/deceleration behavior when the requested clearance does not fit.

## Reproduce the installation

```bash
cd /home/hyunseo/ROMO-B
./scripts/setup_autoware.sh
./scripts/build_all.sh --project-only
./scripts/prepare_autoware_map.sh
./scripts/doctor.sh --autoware
```

`setup_autoware.sh` checks out the exact Autoware 1.8.0 meta revision, imports
its pinned repositories, installs dependencies without changing the NVIDIA
driver, applies the tracked ROMO-B presets, and builds with two workers.

The generated local map bundle contains:

- `lanelet2_map.osm`: a 2.4 m bidirectional corridor derived from the SLAM pose
  graph, split into short connected lanelets;
- `pointcloud_map.pcd`: a relative link to the validated Mid-360 PCD;
- `map_projector_info.yaml`: Autoware's local-coordinate projector;
- `generation_report.json`: route endpoints and generation inputs.

## Planning simulation and obstacle test

```bash
USE_RVIZ=true ./scripts/run_autoware_planning_sim.sh
```

In the Autoware RViz window:

1. Use `2D Pose Estimate` on a lane and drag the arrow along the lane.
2. Use `RouteTool` (preferred) or `2D Goal Pose` farther along the same
   corridor.
3. Confirm a route and `/planning/trajectory` appear before enabling motion.
4. Use `PedestrianInitialPoseTool` to place a stationary dummy object on the
   trajectory. The planned path must shift around it when corridor clearance is
   available, or reach zero velocity before it when clearance is unavailable.
5. Use `DeleteAllObjectsTool` to clear the simulation object.

The simulator uses Autoware's dummy vehicle only. It neither opens
`/dev/romo_b_pcu` nor launches the ROMO-B serial bridge.

## Field startup

Inspect the full live stack without permitting PCU writes:

```bash
RECEIVE_ONLY=true ./scripts/run_autoware_field.sh
```

After map, localization, live objects, planned trajectory, and Collision
Monitor zones have all been checked in RViz, stop that launch and restart with
transmission enabled:

```bash
RECEIVE_ONLY=false ./scripts/run_autoware_field.sh
```

This still starts disarmed and publishes zero motion. In RViz, initialize the
pose and set the route. Only with an operator holding the physical/RC E-stop,
clear space around the robot, healthy diagnostics, and PCU Auto selected:

```bash
ros2 service call /romo_b/arm std_srvs/srv/SetBool '{data: true}'
```

Disarm at route completion or on any unexpected behavior:

```bash
ros2 service call /romo_b/arm std_srvs/srv/SetBool '{data: false}'
ros2 service call /romo_b/software_estop std_srvs/srv/SetBool '{data: true}'
```

No launch or script arms the robot automatically. The first live autonomous
run remains a supervised hardware acceptance step.
