# Architecture

## Runtime command path

```text
Nav2 /cmd_vel_nav ----\
                       twist_mux -> velocity_smoother -> collision_monitor
Teleop /cmd_vel_teleop/                                      |
                                                     /cmd_vel_safe
                                                            |
                                                romo_b_serial_bridge
                                                            |
                                               USB-RS232, 20 Hz PCU
```

The serial bridge never consumes an unfiltered Nav2 command. V1 uses 2WIS only,
maps `Twist` to an equivalent steering angle, rejects pure rotation and reverse
commands, and clamps the selected safety profile.

## Frames

```text
map -- lidar localization --> odom -- robot_localization --> base_link
                                                            |
                                                   robot_state_publisher
                                                            |
                                                        livox_frame
```

Only the listed component owns each transform. Raw wheel odometry does not
publish TF. The static `base_link -> livox_frame` transform is invalid until its
measured values are placed in the ignored hardware-local configuration.

The pinned Livox driver publishes Mid-360 acceleration in `g`. The hardware
launch keeps that vendor stream private on `/sensing/imu/livox_raw` and
`romo_b_imu_normalizer` converts it by 9.80665 to the public SI-unit
`/sensing/imu/imu_raw` topic used by SLAM, localization, and the EKF.

## Mapping and navigation

- Livox SDK2/driver publishes PointCloud2 and IMU.
- Point clouds are cropped, self-filtered, height-filtered, and voxelized.
- `lidar_slam_ros2` authors the PCD while the platform is RC-driven in Manual.
- A 0.05 m occupancy map is derived from the PCD for the Nav2 static layer.
- `lidar_localization_ros2` owns global localization.
- Smac Hybrid-A* plans forward-only Dubins paths with a 0.80 m minimum radius.
- MPPI uses the Ackermann motion model; Collision Monitor provides an
  independent slowdown/stop layer from PointCloud2.

Autoware Universe 1.8.0 is prepared as a separate, ignored research workspace.
It is not part of the V1 runtime and no Lanelet2 map is required. Its complete
environment/build remains independent from the ROMO-B overlays.
