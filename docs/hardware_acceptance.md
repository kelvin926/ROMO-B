# Hardware acceptance procedure

Perform these stages in order. Stop immediately on an unexpected status,
direction, steering sign, or movement.

## Preconditions

- The physical and RC E-stop functions have been independently verified.
- The robot cannot fall and all wheels are clear of the ground for first motion.
- The USB adapter is a true RS232-level device, not a TTL UART adapter.
- DB9 pinout and straight/null-modem wiring have been confirmed.
- Mid-360 uses fused, regulated 9-27 V power; do not use raw 27.2 V battery power
  and do not use PoE.

## Serial stages

1. Keep PCU in Manual and run
   `./scripts/onboard_hardware.sh --receive-only`. Confirm at least three
   25-byte `STX...CRLF` frames and little-endian feedback. The command never
   writes to the port.
2. Confirm the configured data bits (8N1 is only an initial assumption).
   Confirm `serial.command_endian: little` for this physical PCU. Transmit mode
   refuses to start if byte order is omitted or `unverified`.
3. With wheels raised, select the documented PCU Auto prerequisites and transmit
   only Auto, E-stop off, 2WIS, speed 0, steer 0 at 20 Hz.
4. Arm the bench profile and command at most 0.1 m/s and +/-5 degrees.
   For the first measured ground run, keep RC Auto and 2WIS selected, start the
   bench hardware launch, and run:

   ```bash
   python3 scripts/bench_speed_calibration.py --execute --speed 0.01
   ```

   The tool ramps up/down, keeps zero commands alive while disarming, aborts
   above its configurable feedback threshold (default 0.10 m/s), and stores a JSON report under
   `data/local/validation`.
5. Verify positive speed direction and the manual's negative-left steering
   convention against physical wheel movement.
   On this unit, the guarded 0.05 m/s straight and +/-3 degree tests passed on
   2026-07-17 with median feedback 0.045 m/s, +3.0 degrees, and -3.0 degrees.
6. Disconnect commands and feedback independently; both must result in a full
   stop and latched software E-stop before ground testing.

## LiDAR and navigation stages

1. Check point cloud and IMU rates, timestamps, frame IDs, and packet loss.
2. Measure and record the LiDAR xyz/rpy transform; clear the calibration gate.
3. RC-drive a slow mapping loop while recording a rosbag.
4. Inspect loop closure, PCD alignment, occupancy-map obstacles, and free space.
5. Run `scripts/run_localization_replay.sh` and require every automated check to
   pass, then verify repeatable live localization after several manual initial
   poses.
6. Execute a 0.2 m/s waypoint path in a controlled area.
7. Place a static obstacle and verify slowdown, stop, and a feasible local detour.

The complete live stack is started with `./scripts/run_field_navigation.sh`.
Set the initial pose in RViz before arming or executing waypoints.
