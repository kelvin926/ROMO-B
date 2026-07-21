# Project status

Last updated: 2026-07-18

Protocol source: `ROMO-B_manual_verified_complete.md`, SHA-256
`4384934fefb2b96a3cacc2a07ad52010c9e15cc1f24ae51c05c5b7bcb6bbee7e`.

## Verified on this laptop

- Ubuntu 22.04.5 x86_64 and ROS 2 Humble are installed.
- ROS 2 doctor and basic DDS communication pass.
- GitHub CLI is authenticated as `kelvin926` with admin access to the private,
  initially empty repository.
- Active GitHub ruleset `Protect main history` (ID `19129588`) blocks deletion
  and non-fast-forward updates of the default branch while retaining the
  agreed atomic direct-to-`main` workflow.
- USB adapter detected as FTDI FT232R, serial `A5069RR4`, at `/dev/ttyUSB0`.
- Manual protocol examples, fragmented/corrupt parsing, Ackermann conversion,
  steering sign, route YAML, yaw inference, PCD conversion, Livox IMU unit
  normalization, and odometry have automated coverage.
- The tracked PTY PCU integration test passes lifecycle activation, explicit
  arm, simulated motion feedback, odometry, independent command/feedback
  timeouts, PCU ALIVE stagnation, and the AorM 0-to-1 handshake. It verifies
  that feedback/ALIVE faults retain arm at zero, recover Auto without another
  arm request, and never transmit E-stop.
  CI runs this test.
- Ten project packages build locally, including the Autoware adapters and the
  standard `romo_b_launch` vehicle-model contract. The current project suite
  contains 25 passing tests plus the PTY PCU state-machine test. The headless
  full-launch smoke test also reaches an active, connected, disarmed simulated
  bridge through a unique temporary serial path.
- The latest pushed `main` revision passes the clean Ubuntu 22.04/ROS 2 Humble
  GitHub Actions build, static script/YAML validation, project test suite, PTY
  safety-state-machine check, and headless simulation launch.
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
- Real mapping-bag yaw validation paired 1,802 moving samples from the
  Mid-360 IMU and wheel odometry. Their yaw rates have 97.61% sign agreement
  and 0.974 correlation, so the current zero-yaw LiDAR mounting and odometry
  yaw sign are consistent and must not be inverted.
- A repeat analysis on 2026-07-21 paired 1,799 moving samples from that same
  physical bag: yaw correlation was 0.9741 and sign agreement was 97.78%.
  `check_mapping_calibration.py` now repeats the live rate, time, stationary
  bias, gravity, yaw sign/scale, and extrinsic checks without sending commands.
- `run_live_mapping.sh` now provides receive-only live mapping while the
  operator drives in RC Manual. It combines Mid-360 LiDAR+IMU in RKO-LIO,
  records wheel odometry as an independent check, shows the live local map in
  RViz, and records a raw recovery bag. `save_live_mapping.sh` optimizes and
  saves both PCD/pose graph and the Nav2 occupancy map.
- The first RC-Manual mapping bag (`mapping-20260717-195653`) passed SQLite and
  topic-count checks: 381.83 seconds, 3,818 clouds, 76,359 normalized IMU
  samples, and 7,448 wheel-odometry samples.
- The replacement RC-Manual map (`mapping-20260721-134953`) records 176.93 s,
  1,741 raw clouds, 34,950 normalized IMU samples, 3,429 wheel-odometry
  samples, and 1,708 RKO-LIO odometry frames. It produced a 13,470-point PCD,
  a graph with 53 poses and 250 edges, and a 0.05 m Nav2 raycast map. Compact
  products and manifests are tracked; the 1.4 GB bag database is excluded.
  `run_field_navigation.sh` now selects this map by default.
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
- The generated Autoware RViz configuration is a LiDAR-first field layout. Its
  expanded `ROMO-B Field` group contains the Lanelet2/PCD map, filtered
  Mid-360 cloud, UNKNOWN objects, active route, final goal, and trajectory;
  camera-only docks and polar rings are removed while localization/route tools,
  dummy-object simulation tools, and the Autoware state panel remain. A fresh
  visual simulation repeated the 0.835 m UNKNOWN-object avoidance pass.
- The 2026-07-18 live logs were audited after the first goal-driving runs. No
  kernel OOM kill or NVIDIA Xid explains the Autoware exits; most component
  faults occurred during shutdown, while the runtime middleware failures were
  in Fast DDS/shared memory. This laptop now uses Cyclone DDS by default, with
  a UDP-only Fast DDS fallback retained for fresh hosts until setup completes.
- Command-source timeout is a recoverable zero-speed hold that preserves the
  20 Hz PCU Auto heartbeat. Stale PCU feedback/ALIVE and failed Auto transitions
  retain the software arm latch at zero and automatically retry Auto; shutdown
  does not manufacture an Auto falling edge. Only an explicit arm=false request
  performs a Manual handoff. Software E-stop TX and its ROS service are removed;
  only physical/PCU E-stop feedback is reported.
- LiDAR localization republishes `map -> odom` at 20 Hz between 10 Hz scans,
  removing the future-extrapolation gap seen by Nav2. The timer now preserves
  the last scan-validated correction instead of recomputing it against current
  odometry, which previously counter-rotated RViz and disturbed localization
  during physical turns. The corridor controller now uses a 0.5 m/s Ackermann
  MPPI profile with a 4-second prediction horizon and full-footprint cost
  scoring. Its reduced path-alignment weight permits local lateral detours
  around people while retaining filtered/rate-limited 2WIS steering and a
  120-second progress allowance. Rotation Shim now invokes the PCU Pivot mode
  for large initial-path heading errors and final goal yaw alignment.
- Repetitive-corridor localization is now bounded to a 15 m target-map crop
  using 12 m LiDAR returns. Previous-NDT-delta extrapolation is disabled in
  favor of the wheel-odometry seed, and a vendor-patched correction gate rejects
  a one-scan registration jump beyond 1.5 m or 15 degrees even if NDT reports a
  good fitness score. This directly addresses RViz `2D Pose Estimate` snapping
  to a visually similar but incorrect section of hallway.
- The obstacle pipeline now rejects floor/near/far noise and small isolated
  groups. A transient local-costmap plugin rebuilds dynamic obstacles at 5 Hz
  from only the latest 0.35 seconds of Mid-360 data, so departed people do not
  leave persistent lethal costs. The global costmap remains static-map-only so
  a pedestrian cannot cancel the route before MPPI makes a local detour.
  Close-stop/slowdown zones require 20/10 points and never disarm. RViz shows
  both costmaps,
  collision zones, footprint, lookahead point, tracked boxes, and predicted
  paths; rendering is PRIME-offloaded to the RTX GPU.
- An isolated synthetic FollowPath test placed a 0.45 m obstacle 1.10 m ahead
  of a straight path. Ackermann MPPI produced 13 moving detour commands in 33
  settled samples (0.258 m/s maximum forward command and 0.111 rad/s maximum
  absolute yaw rate) without launching the serial bridge.
- A separate isolated Rotation Shim test produced a zero-linear CCW Pivot
  command and verified it traversed the velocity smoother, Collision Monitor,
  and `/cmd_vel_safe`. Protocol tests cover both Pivot signs and the PTY PCU
  test covers live 2WIS-to-Pivot mode switching plus signed yaw odometry.
- The corrected nine-package subset builds locally. All 30 recorded package
  tests pass, and the PTY state-machine test passes under Cyclone DDS,
  including 2WIS motion, both signed Pivot directions/yaw odometry, command
  soft-stop/recovery, and feedback/ALIVE arm-retaining Auto recovery without
  software E-stop transmission.

## Reproduction still required

- Run clone-to-build-to-simulation once on a second clean Ubuntu 22.04/Humble
  host. This laptop's complete local build and isolated validations pass.
- Replace the pending external artifact URIs before another researcher needs to
  retrieve the large bag or localization databases. Compact PCD/Nav2/Lanelet2
  maps and validation evidence are now tracked in Git.

## Waiting for hardware validation

- Repeat one direct `Nav2 Goal` run to verify live TF freshness, smooth steering,
  Pivot path/final-yaw alignment, and absence of nuisance `Hi_E-ST` after command gaps.
- Place one box and then one walking person in the corridor to validate the
  0.70 m stop / 1.25 m slowdown polygons and 0.35-second cost expiry.
- Verify RViz initial-pose recovery at several locations on the live platform.
- Run the receive-only Autoware field checklist, then repeat planning,
  Collision Monitor, and UNKNOWN-object checks with live localization and the
  Mid-360 before any autonomous ground run.
- Perform one supervised 0.10 m/s short route, then a box avoidance/stop test,
  then a walking-person intrusion test with an operator holding the E-stop.
- Actual PCU response to invalid fields and Alive/feedback timeout.
- External storage URI for the ignored bag and localization databases.

Until these items pass `docs/hardware_acceptance.md`, autonomous motion is not
approved.
