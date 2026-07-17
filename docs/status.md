# Project status

Last updated: 2026-07-17

Protocol source: `ROMO-B_manual_verified_complete.md`, SHA-256
`4384934fefb2b96a3cacc2a07ad52010c9e15cc1f24ae51c05c5b7bcb6bbee7e`.

## Verified on this laptop

- Ubuntu 22.04.5 x86_64 and ROS 2 Humble are installed.
- ROS 2 doctor and basic DDS communication pass.
- GitHub CLI is authenticated as `kelvin926` with admin access to the private,
  initially empty repository.
- USB adapter detected as FTDI FT232R, serial `A5069RR4`, at `/dev/ttyUSB0`.
- Manual protocol examples, fragmented/corrupt parsing, Ackermann conversion,
  steering sign, route YAML, yaw inference, PCD conversion, Livox IMU unit
  normalization, and odometry have automated coverage.
- The tracked PTY PCU integration test passes lifecycle activation, explicit
  arm, simulated motion feedback, odometry, independent command/feedback
  timeouts, PCU ALIVE stagnation, AorM 0-to-1 handshake, preserved E-stop
  latch, and explicit reset. CI runs this test.
- Eight project packages build locally: messages, base bridge, description,
  simulator, perception, waypoint manager, navigation, and bringup.
- The latest pushed `main` revision passes the Ubuntu 22.04/ROS 2 Humble
  GitHub Actions build and all eighteen tests.
- Exact external revisions have been fetched for Livox SDK2,
  `livox_ros_driver2`, LiDAR SLAM, and LiDAR localization. Livox SDK2 v1.3.1
  builds and installs successfully in the ignored local vendor prefix.
- Autoware 1.8.0 sources are fetched below the ignored `autoware/` workspace;
  the meta repository is at `5b27e88` and Autoware Universe at `d4d2609`.
- `vcstool` 0.3.0 and `pyserial` 3.5 are available through the tracked,
  sudo-free bootstrap fallback.
- User `hyunseo` has active `dialout` access. The FTDI `A5069RR4` udev link is
  `/dev/romo_b_pcu`; receive-only validation decoded 199 valid PCU frames at
  19.88 Hz over 10 seconds with no Alive gaps.
- The physical bridge completed its zero-speed Manual/Auto handshake and
  disarmed cleanly. A first 0.05 m/s ground command moved forward. The observed
  `Hi_E-ST` was the expected HLV E-stop after the command publisher stopped and
  the 0.15-second watchdog expired, not an overspeed error. Command encoding is
  confirmed as Big Endian `m/s * 100`; a guarded measured-speed run remains.
- The Mid-360 is reachable at `192.168.1.113` through USB Ethernet
  `enxc84d44208014` with host `192.168.1.5`. Raw and filtered point clouds are
  stable at 10 Hz and the IMU at 200 Hz.
- Livox acceleration is converted from the vendor's `g` units to ROS SI units
  on `/sensing/imu/imu_raw`; its unit test passes.
- The measured Mid-360 transform is xyz `(0.270, 0.000, 0.258)` m and rpy
  `(0, 0, 0)` rad. RViz confirmed a level floor, vertical structures, and the
  sensor's forward direction; the local LiDAR calibration gate is approved.
- The first RC-Manual mapping bag (`mapping-20260717-195653`) passed SQLite and
  topic-count checks: 381.83 seconds, 3,818 clouds, 76,359 normalized IMU
  samples, and 7,448 wheel-odometry samples.
- Offline RKO-LIO plus graph SLAM produced a 77,365-point PCD and a graph with
  143 poses, 702 constraints, and two nonlocal loop edges. The optimized path
  is 235.87 m long and closes to 0.479 m. Top-down geometry review is coherent.
- The graph backend's XYZ-only cloud conversion no longer emits one missing
  `intensity` warning per conversion, and `pose_graph.g2o` is now written into
  the requested map output directory. The mapping RViz profile uses the actual
  RKO-LIO and graph topics and an explicit mapping-only `map -> odom` transform.
- The recorded mapping bag completed a full 381.83-second isolated localization
  replay. All 3,794 alignment states were healthy, with zero rejected updates or
  reinitialization requests, fitness median 0.0145, p95 0.1763, maximum 3.1204,
  and 0.464 m start-to-end closure. The automated result is `PASS`.
- A 0.05 m pose-graph-raycast Nav2 map was generated with 192,064 observed-free,
  13,168 occupied, and 1,527,188 unknown cells after adding the continuous
  measured swept footprint. Unknown traversal is disabled. Isolated Nav2
  preflight planned 87.73 m and 106.71 m forward-only paths with zero reverse
  segments, clamped a 0.5 m/s input to 0.2 m/s, and stopped for three obstacle
  points. All automated checks pass.
- `scripts/run_field_navigation.sh` now starts the complete live Mid-360, EKF,
  PCD localization, Nav2, collision-monitor, waypoint, and RViz stack. The
  bridge remains disarmed until the explicit arm service is called.

## Host actions still required

- Complete the isolated Autoware environment/build only when that research
  workspace is needed; it is not part of V1 waypoint navigation.
- Re-run a clean clone-to-simulation reproduction after the current hardware
  acceptance changes are committed.

## Waiting for hardware validation

- Run the guarded 0.01 m/s measured-speed calibration and verify steering raw
  scale and left/right sign.
- Verify RViz initial-pose recovery at several locations on the live platform.
- Repeat planning and collision checks with live localization and PointCloud
  before any autonomous ground run.
- Actual PCU response to invalid fields and Alive/feedback timeout.
- External storage URI for bags, PCDs, and maps.

Until these items pass `docs/hardware_acceptance.md`, autonomous motion is not
approved.
