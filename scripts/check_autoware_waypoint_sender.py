#!/usr/bin/env python3
"""Validate the real YAML-to-Autoware route sender without robot hardware."""

import argparse
import json
import math
import pathlib
import subprocess
import sys
import time

import rclpy
import yaml
from autoware_adapi_v1_msgs.msg import RouteState
from autoware_internal_planning_msgs.msg import VelocityLimit
from autoware_planning_msgs.msg import Trajectory
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from check_autoware_planning import lane_centerline, yaw_quaternion


class WaypointProbe(Node):
    def __init__(self) -> None:
        super().__init__("romo_b_autoware_waypoint_probe")
        transient = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.initial_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.route_state = None
        self.trajectory = None
        self.selected_velocity_limits = []
        self.create_subscription(
            RouteState,
            "/api/routing/state",
            lambda message: setattr(self, "route_state", message.state),
            transient,
        )
        self.create_subscription(
            Trajectory,
            "/planning/trajectory",
            lambda message: setattr(self, "trajectory", message),
            10,
        )
        self.create_subscription(
            VelocityLimit,
            "/planning/scenario_planning/max_velocity",
            lambda message: self.selected_velocity_limits.append(message.max_velocity),
            10,
        )

    def publish_initial(self, point, following) -> None:
        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.pose.position.x = point[0]
        message.pose.pose.position.y = point[1]
        message.pose.pose.orientation = yaw_quaternion(
            math.atan2(following[1] - point[1], following[0] - point[0])
        )
        message.pose.covariance[0] = 0.04
        message.pose.covariance[7] = 0.04
        message.pose.covariance[35] = 0.01
        self.initial_publisher.publish(message)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-path", required=True, type=pathlib.Path)
    parser.add_argument("--sender", required=True, type=pathlib.Path)
    parser.add_argument("--output-dir", required=True, type=pathlib.Path)
    parser.add_argument("--startup-timeout", type=float, default=90.0)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    centerline = lane_centerline(args.map_path / "lanelet2_map.osm")
    initial_index = min(4, len(centerline) - 2)
    # Exercise a true intermediate point on one forward lane sequence.  The
    # recorded survey centerline contains later out-and-back reversals; using a
    # sparse point across one of those reversals would infer the opposite yaw
    # and correctly be rejected by the forward-only route planner.
    intermediate_index = min(30, len(centerline) - 3)
    goal_index = min(60, len(centerline) - 2)
    waypoint_file = args.output_dir / "test_waypoints.yaml"
    waypoint_file.write_text(
        yaml.safe_dump(
            {
                "schema_version": 1,
                "frame_id": "map",
                "mode": "continuous",
                "default_speed_mps": 0.1,
                "waypoints": [
                    {
                        "x": centerline[intermediate_index][0],
                        "y": centerline[intermediate_index][1],
                    },
                    {
                        "x": centerline[goal_index][0],
                        "y": centerline[goal_index][1],
                    },
                ],
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    rclpy.init()
    node = WaypointProbe()
    report = {}
    try:
        required_nodes = {
            "autoware_operation_mode_transition_manager",
            "behavior_path_planner",
            "motion_velocity_planner",
        }
        deadline = time.monotonic() + args.startup_timeout
        last_initial = 0.0
        while time.monotonic() < deadline:
            visible = {
                name for name, _namespace in node.get_node_names_and_namespaces()
            }
            ready = required_nodes <= visible
            now = time.monotonic()
            if ready and now - last_initial >= 1.0:
                node.publish_initial(
                    centerline[initial_index], centerline[initial_index + 1]
                )
                last_initial = now
            if ready and node.route_state == RouteState.UNSET:
                break
            rclpy.spin_once(node, timeout_sec=0.2)
        else:
            raise RuntimeError("Autoware did not reach an unset, routable state")

        # The full planning simulator exposes component nodes before every
        # transient map subscription has received its sample.  Let the graph
        # settle so this check measures the route sender, not launch CPU load.
        discovery_deadline = time.monotonic() + 8.0
        while time.monotonic() < discovery_deadline:
            rclpy.spin_once(node, timeout_sec=0.2)

        sender_process = subprocess.Popen(
            [sys.executable, str(args.sender), str(waypoint_file), "--timeout", "60"],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        sender_deadline = time.monotonic() + 75.0
        while sender_process.poll() is None and time.monotonic() < sender_deadline:
            # Keep this executor alive while the short-lived CLI publisher
            # sends its transient-local speed limit.
            rclpy.spin_once(node, timeout_sec=0.10)
        if sender_process.poll() is None:
            sender_process.terminate()
            sender_process.wait(timeout=5.0)
            raise RuntimeError("waypoint sender process timed out")
        sender_stdout, sender_stderr = sender_process.communicate()
        (args.output_dir / "sender.stdout").write_text(
            sender_stdout, encoding="utf-8"
        )
        (args.output_dir / "sender.stderr").write_text(
            sender_stderr, encoding="utf-8"
        )
        try:
            sender_report = json.loads(sender_stdout)
        except json.JSONDecodeError as error:
            raise RuntimeError(f"waypoint sender returned invalid JSON: {error}") from error

        response_deadline = time.monotonic() + 45.0
        while time.monotonic() < response_deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            current_velocities = (
                [
                    point.longitudinal_velocity_mps
                    for point in node.trajectory.points
                ]
                if node.trajectory
                else []
            )
            if (
                node.route_state == RouteState.SET
                and len(current_velocities) >= 5
                and min(current_velocities) >= -1.0e-4
                and max(current_velocities) <= 0.2005
                and any(limit <= 0.100001 for limit in node.selected_velocity_limits)
            ):
                break

        trajectory_points = node.trajectory.points if node.trajectory else []
        velocities = [point.longitudinal_velocity_mps for point in trajectory_points]
        checks = {
            "sender_exit_success": sender_process.returncode == 0,
            "sender_report_pass": sender_report.get("result") == "PASS",
            "one_intermediate_waypoint": sender_report.get("intermediate_waypoints") == 1,
            "route_state_set": node.route_state == RouteState.SET,
            "trajectory_generated": len(trajectory_points) >= 5,
            "trajectory_forward_only": bool(velocities) and min(velocities) >= -1.0e-4,
            "planned_speed_at_or_below_0_2_mps": bool(velocities)
            and max(velocities) <= 0.2005,
            "yaml_speed_limit_selected": any(
                limit <= 0.100001 for limit in node.selected_velocity_limits
            ),
        }
        report = {
            "result": "PASS" if all(checks.values()) else "FAIL",
            "checks": checks,
            "sender_report": sender_report,
            "trajectory_points": len(trajectory_points),
            "planned_speed_range_mps": (
                [min(velocities), max(velocities)] if velocities else None
            ),
            "selected_speed_limit_min_mps": (
                min(node.selected_velocity_limits)
                if node.selected_velocity_limits
                else None
            ),
            "waypoint_file": str(waypoint_file),
        }
    except Exception as error:
        report = {"result": "FAIL", "error": str(error)}
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    rendered = json.dumps(report, indent=2, sort_keys=True) + "\n"
    print(rendered, end="")
    (args.output_dir / "result.json").write_text(rendered, encoding="utf-8")
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
