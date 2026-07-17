# Autoware corridor field checklist

This is the short operator sequence for the mapped corridor. Keep the RC/physical
E-stop in hand and stop at the first unexpected state. A single Mid-360 and this
research software are not safety-rated.

## 1. Prove the laptop configuration

```bash
cd /home/hyunseo/ROMO-B
./scripts/doctor.sh --hardware
./scripts/run_autoware_validation.sh
```

Both commands must end in `READY`/`PASS`. The validation uses isolated ROS
domains and starts no serial or LiDAR node. Synthetic velocity messages cannot
reach a PCU bridge because none is launched in those domains.

## 2. Inspect the live stack without serial writes

Put the PCU in Manual, keep the area clear, then run:

```bash
RECEIVE_ONLY=true USE_RVIZ=true ./scripts/run_autoware_field.sh
```

In Autoware RViz, require all of the following before continuing:

- the Lanelet2 corridor and PCD map overlap;
- the live obstacle point cloud follows the robot, without a second displaced
  copy;
- `2D Pose Estimate` produces a stable map pose and localization remains
  initialized;
- people/boxes in front appear as UNKNOWN objects;
- `RouteTool` creates a forward trajectory on the corridor;
- the robot is still physically stationary and `/cmd_vel_safe` is zero.

Stop the launch with Ctrl-C. Do not switch a running receive-only bridge into
TX mode.

## 3. Start TX, still disarmed

Select PCU Auto and 2WIS, confirm the RC throttle is neutral, and run:

```bash
RECEIVE_ONLY=false USE_RVIZ=true ./scripts/run_autoware_field.sh
```

Set the initial pose and route again. Confirm the PCU diagnostics say connected
and disarmed, localization is healthy, the trajectory is at or below 0.20 m/s,
and `/cmd_vel_safe` remains zero.

The saved YAML route can be submitted instead of drawing a new route:

```bash
python3 scripts/send_autoware_waypoints.py config/local/waypoints.yaml
```

This sets a route only; it never arms or engages the robot. Use `--replace`
only while stopped if an Autoware route is already set.

## 4. Supervised motion

With one operator holding the E-stop and another watching RViz:

```bash
ros2 service call /romo_b/arm std_srvs/srv/SetBool '{data: true}'
```

Only after PCU Auto is confirmed, select `Auto` and enable `Autoware Control` in
the RViz state panel. Start with a short straight route. Then test a box, and
finally a walking person, entering the path. Accept only these responses:

- enough lateral clearance: the trajectory shifts inside the mapped lane;
- insufficient clearance: the planned speed reaches zero before the object;
- near-field backup: Collision Monitor reduces `/cmd_vel_safe` and stops by
  its stop zone even if planning has not reacted.

## 5. Stop and leave safe

```bash
ros2 service call /romo_b/arm std_srvs/srv/SetBool '{data: false}'
ros2 service call /romo_b/software_estop std_srvs/srv/SetBool '{data: true}'
```

Confirm zero wheel feedback, then return the PCU to Manual before stopping the
launch. Never leave the platform armed or in autonomous control unattended.
