# Autoware Universe 1.8.0

Autoware is the route/behavior-planning UI for the corridor research setup. The
ROMO-B field stack keeps the already validated serial bridge, NDT localization,
and independent Nav2 Collision Monitor. Autoware supplies the Lanelet2 route,
trajectory, and local avoidance decisions.

The localization interface forwards RViz/API initial poses to the pinned NDT
localizer and translates its alignment diagnostics into Autoware's
`INITIALIZING/INITIALIZED` state. Autoware therefore cannot expose autonomous
mode while NDT is unhealthy or has requested reinitialization.

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
An always-on 0.14 m/s planning candidate leaves smoothing margin below the
physical 0.20 m/s limit. The ROMO-B follower independently clamps to 0.20 m/s
and also applies the currently selected YAML/Autoware speed limit. The velocity
smoother's own `max_vel` is fixed at 0.20 m/s as a third guard against a cold
startup profile being calculated before an external limit update is consumed.
The launch relays the identical Lanelet2 sample every two seconds for a bounded
30-second startup window after an initial settling delay. This lets late
volatile planner subscriptions receive the map during composable-node loading;
it never alters the map, stops automatically, and route submission waits for
the relay-complete readiness latch.

## Reproduce the installation

```bash
cd /home/hyunseo/ROMO-B
./scripts/setup_autoware.sh
./scripts/build_all.sh --project-only
./scripts/prepare_autoware_map.sh
./scripts/run_autoware_validation.sh
```

`setup_autoware.sh` checks out the exact Autoware 1.8.0 meta revision, imports
its pinned repositories, installs dependencies without changing the NVIDIA
driver, applies the tracked ROMO-B presets, and builds with two workers. The
upstream `acados` v0.5.3 dependency is built below ignored `autoware/deps/`
instead of modifying `/opt` or requiring permanent shell-profile edits.

The perception check runs in an isolated ROS domain with a synthetic point
cluster. It must verify cluster detection, UNKNOWN classification, map-frame
tracking, and a predicted path. It launches no serial or velocity node.

The validation starts only the disengaged dummy simulator. It verifies the
localization bridge and fail-closed motion gates, then clusters a synthetic
UNKNOWN object. Two independent planning cases are mandatory: a 0.40 m object
offset 0.30 m from the baseline must be avoided, and a centered 0.60 m object
must make the planned speed reach zero. The validated corridor run shifted to
0.835 m clearance in the first case and reached 0.0 m/s in the second. Reports
and launch logs are saved below `data/local/validation/`.

The same suite submits a generated two-pose YAML route through the real ADAPI
`SetRoutePoints` endpoint and checks the intermediate waypoint, forward-only
trajectory, and selected 0.10 m/s limit. Every wrapper uses a separate ROS
domain, launches no serial bridge, and removes its full process group on exit.

Use the short [field checklist](autoware_field_checklist.md) at the robot. It
keeps receive-only inspection, TX startup, route creation, and supervised arm
as separate operator decisions.

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

The generated ROMO-B layout is LiDAR-first. Expand `ROMO-B Field` in the
Displays panel to inspect the PCD map, Lanelet2 road, filtered Mid-360 cloud,
recognized obstacles, active route, final goal, and planned trajectory. The
camera-only image docks and polar range rings are intentionally omitted. The
left `AutowareStatePanel` remains the authoritative localization, routing,
motion, MRM, and emergency-state view.

1. Use `2D Pose Estimate` on a lane and drag the arrow along the lane.
2. Use `RouteTool` (preferred) or `2D Goal Pose` farther along the same
   corridor.
3. Confirm a route and `/planning/trajectory` appear before enabling motion.
4. Use `PedestrianInitialPoseTool` to place a stationary dummy object on the
   trajectory. The planned path must shift around it when corridor clearance is
   available, or reach zero velocity before it when clearance is unavailable.
5. Use `DeleteAllObjectsTool` to clear the simulation object.

The same YAML used by the Nav2 waypoint editor can be sent to Autoware without
arming or engaging the platform:

```bash
python3 scripts/send_autoware_waypoints.py config/local/waypoints.yaml
```

All entries except the final pose become Autoware intermediate route points;
the final pose is the goal, and `default_speed_mps` is published as Autoware's
maximum-velocity candidate. The physical follower also consumes the selected
limit, so a 0.10 m/s YAML route cannot be raised by a smoothing transient. An
existing route is left untouched unless the operator explicitly adds
`--replace` while the platform is stopped. A route change is rejected while
Autoware is actively controlling in autonomous mode or while the PCU reports
armed.

The simulator uses Autoware's dummy vehicle only. It neither opens
`/dev/romo_b_pcu` nor launches the ROMO-B serial bridge. It starts disengaged,
so creating a route alone does not move even the dummy vehicle.

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

Then use the `AutowareStatePanel` in RViz to select `Auto` and turn on
`Autoware Control`. These are a second, independent motion gate. If the panel
shows a start request, accept it only after rechecking the live obstacle view.
The ROMO-B follower outputs zero unless the PCU is armed **and** both Autoware
states are active.

Disarm at route completion or on any unexpected behavior:

```bash
ros2 service call /romo_b/arm std_srvs/srv/SetBool '{data: false}'
```

No launch or script arms the robot automatically, and the bridge never asserts
the HLV E-stop byte. The operator retains emergency-stop authority through the
physical/RC control. The first live autonomous run remains a supervised
hardware acceptance step.
