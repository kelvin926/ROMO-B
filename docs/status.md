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
  GitHub Actions build and all seventeen tests.
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
  disarmed cleanly. A first ground command moved forward but faster than the
  requested 0.05 m/s and ended in PCU `Hi_E-ST`; physical command scaling is
  therefore not accepted for navigation yet.
- The Mid-360 is reachable at `192.168.1.113` through USB Ethernet
  `enxc84d44208014` with host `192.168.1.5`. Raw and filtered point clouds are
  stable at 10 Hz and the IMU at 200 Hz.
- Livox acceleration is converted from the vendor's `g` units to ROS SI units
  on `/sensing/imu/imu_raw`; its unit test passes.
- The measured Mid-360 transform is xyz `(0.270, 0.000, 0.258)` m and rpy
  `(0, 0, 0)` rad. RViz confirmed a level floor, vertical structures, and the
  sensor's forward direction; the local LiDAR calibration gate is approved.

## Host actions still required

- Complete the isolated Autoware environment/build only when that research
  workspace is needed; it is not part of V1 waypoint navigation.
- Re-run a clean clone-to-simulation reproduction after the current hardware
  acceptance changes are committed.

## Waiting for hardware validation

- Resolve the physical velocity scaling that caused the unexpectedly fast
  0.05 m/s ground test, then verify steering raw scale and left/right sign.
- Actual PCU response to invalid fields and Alive/feedback timeout.
- External storage URI for bags, PCDs, and maps.

Until these items pass `docs/hardware_acceptance.md`, autonomous motion is not
approved.
