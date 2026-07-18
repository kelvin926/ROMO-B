# Architecture

## Runtime command path

```text
Autoware /planning/trajectory -> ROMO-B trajectory follower --\
Nav2 /cmd_vel_nav --------------------------------------------+-> twist_mux
Teleop /cmd_vel_teleop --------------------------------------/       |
                                                     velocity_smoother
                                                              |
                                                     collision_monitor
                                                              |
                                                       /cmd_vel_safe
                                                              |
                                                  romo_b_serial_bridge
                                                              |
                                                 USB-RS232, 20 Hz PCU
```

The serial bridge never consumes an unfiltered Nav2, Autoware, or teleop
command. V1 uses 2WIS only, maps `Twist` to an equivalent steering angle,
rejects pure rotation and reverse commands, and clamps the selected safety
profile.

## Frames

```text
map -- lidar localization --> odom -- robot_localization --> base_link
                                                            |
                                                   robot_state_publisher
                                                     /              \
                                          base_footprint          livox_frame
```

Only the listed component owns each transform. Raw wheel odometry does not
publish TF. The static `base_link -> livox_frame` transform is invalid until its
measured values are placed in `config/local/hardware.yaml`. The approved
hardware configuration is tracked for handoff to users of this same platform.

The pinned Livox driver publishes Mid-360 acceleration in `g`. The hardware
launch keeps that vendor stream private on `/sensing/imu/livox_raw` and
`romo_b_imu_normalizer` converts it by 9.80665 to the public SI-unit
`/sensing/imu/imu_raw` topic used by SLAM, localization, and the EKF.

## Mapping and navigation

- Livox SDK2/driver publishes PointCloud2 and IMU.
- Point clouds split after transform/self-filtering: a height-sliced 0.05 m
  obstacle cloud feeds Nav2, while a full-height 0.10 m cloud feeds localization.
- `lidar_slam_ros2` authors the PCD while the platform is RC-driven in Manual.
- A 0.05 m occupancy map is derived from the PCD for the Nav2 static layer.
- `lidar_localization_ros2` owns global localization.
- Smac Hybrid-A* plans forward-only Dubins paths with a 0.80 m minimum radius.
- MPPI uses the Ackermann motion model; Collision Monitor provides an
  independent slowdown/stop layer from PointCloud2.
- The optional Autoware corridor runtime uses the generated Lanelet2 map,
  clusters live Mid-360 returns as UNKNOWN objects, and publishes a
  forward-only low-speed trajectory through the same Collision Monitor path.

Nav2 remains the V1 baseline runtime and does not require Lanelet2. Autoware
Universe 1.8.0 is also integrated as an optional corridor research runtime in a
separate ignored source workspace; it uses the tracked compact Lanelet2/PCD map
bundle and shares ROMO-B's tracked adapters and final safety pipeline.
