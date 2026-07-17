#!/usr/bin/env python3
"""Verify that the ROMO-B trajectory follower fails closed without hardware."""

import argparse
import json
import pathlib
import time

import rclpy
from autoware_adapi_v1_msgs.msg import (
    LocalizationInitializationState,
    OperationModeState,
)
from autoware_internal_planning_msgs.msg import VelocityLimit
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from romo_b_msgs.msg import PlatformStatus


class SafetyGateProbe(Node):
    def __init__(self) -> None:
        super().__init__("romo_b_autoware_safety_gate_probe")
        transient = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.trajectory_publisher = self.create_publisher(
            Trajectory, "/planning/trajectory", 10
        )
        self.odometry_publisher = self.create_publisher(
            Odometry, "/localization/kinematic_state", 10
        )
        self.platform_publisher = self.create_publisher(
            PlatformStatus, "/romo_b/platform_status", 10
        )
        self.mode_publisher = self.create_publisher(
            OperationModeState, "/api/operation_mode/state", 10
        )
        self.localization_publisher = self.create_publisher(
            LocalizationInitializationState,
            "/localization/initialization_state",
            transient,
        )
        self.velocity_limit_publisher = self.create_publisher(
            VelocityLimit, "/planning/scenario_planning/max_velocity", 10
        )
        self.command = None
        self.create_subscription(Twist, "/cmd_vel_nav", self._on_command, 10)

        self.trajectory = Trajectory()
        self.trajectory.header.frame_id = "map"
        for index in range(30):
            point = TrajectoryPoint()
            point.pose.position.x = 0.20 * index
            point.pose.orientation.w = 1.0
            point.longitudinal_velocity_mps = 0.20
            self.trajectory.points.append(point)

        self.odometry = Odometry()
        self.odometry.header.frame_id = "map"
        self.odometry.child_frame_id = "base_link"
        self.odometry.pose.pose.orientation.w = 1.0

        self.platform = PlatformStatus()
        self.platform.connected = True
        self.platform.auto_mode = True
        self.platform.estop = False
        self.platform.command_timed_out = False
        self.platform.feedback_timed_out = False

        self.mode = OperationModeState()
        self.mode.mode = OperationModeState.AUTONOMOUS
        self.mode.is_autoware_control_enabled = True

        self.localization = LocalizationInitializationState()
        self.velocity_limit = None

    def _on_command(self, message: Twist) -> None:
        self.command = message

    def publish_inputs(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self.trajectory.header.stamp = stamp
        self.odometry.header.stamp = stamp
        self.trajectory_publisher.publish(self.trajectory)
        self.odometry_publisher.publish(self.odometry)
        self.platform_publisher.publish(self.platform)
        self.mode_publisher.publish(self.mode)
        self.localization.stamp = stamp
        self.localization_publisher.publish(self.localization)
        if self.velocity_limit is not None:
            self.velocity_limit.stamp = stamp
            self.velocity_limit_publisher.publish(self.velocity_limit)

    def sample(self, seconds: float = 0.45) -> Twist:
        self.command = None
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.publish_inputs()
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.command is None:
            raise RuntimeError("trajectory follower did not publish /cmd_vel_nav")
        return self.command


def stopped(command: Twist) -> bool:
    return abs(command.linear.x) < 1.0e-6 and abs(command.angular.z) < 1.0e-6


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=pathlib.Path)
    args = parser.parse_args()
    rclpy.init()
    node = SafetyGateProbe()
    try:
        discovery_deadline = time.monotonic() + 8.0
        publishers = (
            node.trajectory_publisher,
            node.odometry_publisher,
            node.platform_publisher,
            node.mode_publisher,
            node.localization_publisher,
            node.velocity_limit_publisher,
        )
        while time.monotonic() < discovery_deadline:
            if all(publisher.get_subscription_count() for publisher in publishers):
                break
            rclpy.spin_once(node, timeout_sec=0.1)
        else:
            raise RuntimeError("trajectory follower subscriptions did not appear")

        node.platform.state = PlatformStatus.STATE_CONNECTED_SAFE
        node.localization.state = LocalizationInitializationState.INITIALIZED
        disarmed = node.sample()

        node.platform.state = PlatformStatus.STATE_ARMED_AUTO
        node.localization.state = LocalizationInitializationState.UNINITIALIZED
        unlocalized = node.sample()

        node.localization.state = LocalizationInitializationState.INITIALIZED
        node.mode.is_autoware_control_enabled = False
        control_disabled = node.sample()

        node.mode.is_autoware_control_enabled = True
        enabled = node.sample()

        node.velocity_limit = VelocityLimit()
        node.velocity_limit.sender = "romo_b_waypoints"
        node.velocity_limit.max_velocity = 0.10
        yaml_limited = node.sample()

        node.localization.state = LocalizationInitializationState.INITIALIZING
        localization_lost = node.sample()

        node.localization.state = LocalizationInitializationState.INITIALIZED
        node.platform.feedback_timed_out = True
        feedback_timeout = node.sample()

        checks = {
            "disarmed_is_zero": stopped(disarmed),
            "unlocalized_is_zero": stopped(unlocalized),
            "autoware_control_disabled_is_zero": stopped(control_disabled),
            "fully_enabled_moves_forward": 0.0 < enabled.linear.x <= 0.20,
            "selected_yaml_limit_clamps_to_0_1_mps": abs(
                yaml_limited.linear.x - 0.10
            )
            <= 1.0e-6,
            "localization_loss_is_zero": stopped(localization_lost),
            "feedback_timeout_is_zero": stopped(feedback_timeout),
        }
        report = {
            "result": "PASS" if all(checks.values()) else "FAIL",
            "checks": checks,
            "enabled_command": {
                "linear_x": enabled.linear.x,
                "angular_z": enabled.angular.z,
            },
            "yaml_limited_command_mps": yaml_limited.linear.x,
        }
    except Exception as error:
        report = {"result": "FAIL", "error": str(error)}
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    rendered = json.dumps(report, indent=2, sort_keys=True) + "\n"
    print(rendered, end="")
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
