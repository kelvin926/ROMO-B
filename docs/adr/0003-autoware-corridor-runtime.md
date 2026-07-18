# ADR 0003: Autoware is an optional corridor runtime

- Status: accepted
- Date: 2026-07-18

Nav2 remains the supported V1 baseline for forward-only waypoint navigation.
Autoware Universe 1.8.0 is additionally supported as an optional research
runtime for the mapped corridor, where its Lanelet2 planner can make local
avoidance or stop decisions for UNKNOWN objects produced from the Mid-360.

Autoware does not bypass the ROMO-B command safety architecture. Its trajectory
is converted by the fail-closed ROMO-B follower to `/cmd_vel_nav`, then passes
through the same `twist_mux`, velocity smoother, Collision Monitor,
`/cmd_vel_safe`, and serial bridge used by Nav2. The field launch starts in
receive-only mode and no launch file arms or engages the platform.

The large Autoware source/build tree and generated map remain ignored local
artifacts. Exact source revisions, tracked presets, adapters, validation tools,
and artifact manifests make the runtime reproducible without committing those
large products.
