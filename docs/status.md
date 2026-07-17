# Project status

Last updated: 2026-07-17

## Verified on this laptop

- Ubuntu 22.04.5 x86_64 and ROS 2 Humble are installed.
- ROS 2 doctor and basic DDS communication pass.
- GitHub CLI is authenticated as `kelvin926` with admin access to the private,
  initially empty repository.
- USB adapter detected as FTDI FT232R, serial `A5069RR4`, at `/dev/ttyUSB0`.

## Host actions still required

- Reboot is required; NVIDIA 535 kernel/userspace versions currently mismatch.
- User `hyunseo` is not yet in `dialout`; run `scripts/setup_host.sh`, then log
  out/in or reboot.
- Nav2, robot_localization, twist_mux, Xacro, vcstool, and pyserial are not yet
  all installed; `scripts/setup_host.sh` owns this installation.

## Waiting for hardware validation

- PCU data bits, DB9 pinout, and straight/null-modem cable choice.
- Command steering raw scale and physical left/right sign.
- Actual PCU response to invalid fields and Alive/feedback timeout.
- Mid-360 Ethernet adapter, sensor IP, packet rate, timestamps, and measured pose.
- External storage URI for bags, PCDs, and maps.

Until these items pass `docs/hardware_acceptance.md`, autonomous motion is not
approved.
