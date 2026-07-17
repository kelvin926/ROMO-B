# ADR 0001: Nav2 is the V1 runtime

- Status: accepted
- Date: 2026-07-17

The initial objective is waypoint following with local obstacle avoidance, not
road-network behavior planning. ROS 2 Humble Nav2 is therefore the operational
runtime. Autoware Universe 1.8.0 remains a separately reproducible research
workspace and does not introduce Lanelet2 or CUDA into V1 operation.
