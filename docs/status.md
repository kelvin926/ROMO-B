# Project status

Last updated: 2026-07-18

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
- Ten project packages build locally, including the Autoware adapters and the
  standard `romo_b_launch` vehicle-model contract. The current project suite
  contains 25 passing tests plus the PTY PCU state-machine test.
- The latest pushed `main` revision passes the clean Ubuntu 22.04/ROS 2 Humble
  GitHub Actions build, project test suite, and PTY safety-state-machine check.
- Exact external revisions have been fetched for Livox SDK2,
  `livox_ros_driver2`, LiDAR SLAM, and LiDAR localization. Livox SDK2 v1.3.1
  builds and installs successfully in the ignored local vendor prefix.
- Autoware 1.8.0 sources are fetched below the ignored `autoware/` workspace;
  the meta repository is at `5b27e88` and Autoware Universe at `d4d2609`. The
  full 481-package source build completed successfully. Optional CUDA/TensorRT
  packages are intentionally inactive on this CPU perception path.
- `vcstool` 0.3.0 and `pyserial` 3.5 are available through the tracked,
  sudo-free bootstrap fallback.
- User `hyunseo` has active `dialout` access. The FTDI `A5069RR4` udev link is
  `/dev/romo_b_pcu`; receive-only validation decoded 199 valid PCU frames at
  19.88 Hz over 10 seconds with no Alive gaps.
- The physical bridge completed its zero-speed Manual/Auto handshake. The
  manual's Big Endian command claim does not match this PCU: raw speed `1` sent
  as `00 01` produced feedback above 0.29 m/s and rising toward the platform
  maximum before the guarded test latched HLV E-stop. This matches Little
  Endian interpretation as raw `0x0100`. Transmit mode now requires an explicit
  byte-order setting and this unit is configured `little`; low-speed
  revalidation passed: a 0.05 m/s command produced 0.045 m/s median rear-wheel
  feedback. Raw `1` remains below the platform's useful-motion deadband.
- Physical E-stop feedback uses a bitmask on this PCU (`0x05` was observed),
  despite the manual documenting only 0/1. The parser now treats any nonzero
  value as E-stop active and exposes the raw byte in diagnostics.
- Guarded physical calibration now passes in both directions: straight
  0.05 m/s produced 0.045 m/s median rear feedback, left 3 degrees produced
  0.055 m/s and +3.0 degrees, and right 3 degrees produced 0.045 m/s and
  -3.0 degrees. One immediate back-to-back right run tripped on a transient
  0.13 m/s sample; a clean restart plus an added 0.5-second Auto zero-settle
  produced the repeatable passing result.
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
  preflight planned 87.22 m and 106.61 m forward-only paths with zero reverse
  segments, clamped a 0.5 m/s input to 0.2 m/s, and stopped for three obstacle
  points. All automated checks pass.
- `scripts/run_field_navigation.sh` now starts the complete live Mid-360, EKF,
  PCD localization, Nav2, collision-monitor, waypoint, and RViz stack. The
  bridge remains disarmed until the explicit arm service is called.
- The first live full-stack attempt exposed a duplicate bench bridge and raw
  Mid-360 acceleration integration. The launcher now refuses to start beside an
  existing bridge, EKF uses wheel odometry only until IMU bias/covariance are
  calibrated, and RViz requests Best Effort QoS for the filtered PointCloud2.
- The recorded corridor now has a valid 14-lanelet Autoware map, local
  projector, and linked 77,365-point PCD. Runtime map load and ADAPI route
  planning pass in the pinned Autoware 1.8.0 stack.
- The ROMO-B Autoware field stack includes NDT initialization health bridging,
  UNKNOWN-object clustering/tracking/prediction, a forward-only low-speed
  follower, a persistent 0.14 m/s planning guard, YAML intermediate route
  points, and the independent Nav2 Collision Monitor. It defaults to serial
  receive-only and never arms automatically.
- The field and simulation launches relay the unchanged Lanelet2 map during a
  bounded 30-second startup window after an initial delay, preventing a
  verified DDS discovery race from leaving late volatile planning components
  without `/map/vector_map`. Route submission waits for its readiness latch.
- Isolated Autoware acceptance passes nine localization-interface checks,
  seven fail-closed motion-gate checks, six point-cloud/perception checks,
  eight YAML route/speed-limit checks, and both route-planning outcomes. A
  feasible offset obstacle produced 0.835 m lateral clearance while remaining
  forward-moving; a centered blocking obstacle produced a 0.0 m/s planned
  stop. Baseline planning stayed forward-only with a measured maximum of
  0.200000003 m/s. The velocity smoother, persistent planning candidate, and
  physical follower independently enforce the low-speed limits.
- The Autoware RViz configuration loads the Lanelet2/PCD map, live Mid-360
  cloud, UNKNOWN objects, route and trajectory, localization tools,
  `RouteTool`, dummy-object tools for simulation, and the Autoware state panel.

## Reproduction still required

- Run clone-to-build-to-simulation once on a second clean Ubuntu 22.04/Humble
  host. This laptop's complete local build and isolated validations pass.
- Replace the pending external artifact URIs before another researcher needs to
  retrieve the bag, PCD, or generated maps off this laptop.

## Waiting for hardware validation

- Verify RViz initial-pose recovery at several locations on the live platform.
- Run the receive-only Autoware field checklist, then repeat planning,
  Collision Monitor, and UNKNOWN-object checks with live localization and the
  Mid-360 before any autonomous ground run.
- Perform one supervised 0.10 m/s short route, then a box avoidance/stop test,
  then a walking-person intrusion test with an operator holding the E-stop.
- Actual PCU response to invalid fields and Alive/feedback timeout.
- External storage URI for bags, PCDs, and maps.

Until these items pass `docs/hardware_acceptance.md`, autonomous motion is not
approved.
