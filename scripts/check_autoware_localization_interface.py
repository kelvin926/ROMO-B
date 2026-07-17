#!/usr/bin/env python3
"""Exercise the ROMO-B localization/Autoware state bridge in isolation."""

import argparse
import json
import pathlib
import time

import rclpy
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState
from autoware_localization_msgs.srv import InitializeLocalization
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class LocalizationProbe(Node):
    def __init__(self) -> None:
        super().__init__("romo_b_autoware_localization_probe")
        transient = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.initial_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.alignment_publisher = self.create_publisher(
            DiagnosticArray, "/localization/alignment_status", transient
        )
        self.reinitialization_publisher = self.create_publisher(
            Bool, "/localization/reinitialization_requested", transient
        )
        self.states = []
        self.direct_poses = []
        self.create_subscription(
            LocalizationInitializationState,
            "/localization/initialization_state",
            lambda message: self.states.append(message.state),
            transient,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/localization/initialpose_direct",
            self.direct_poses.append,
            10,
        )
        self.client = self.create_client(
            InitializeLocalization, "/localization/initialize"
        )

    def spin_until(self, predicate, timeout: float = 4.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def pose(self) -> PoseWithCovarianceStamped:
        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = 1.25
        message.pose.pose.position.y = -0.50
        message.pose.pose.orientation.w = 1.0
        return message

    def alignment(self, healthy: bool) -> DiagnosticArray:
        output = DiagnosticArray()
        output.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "lidar_localization_ros2/alignment"
        status.level = DiagnosticStatus.OK if healthy else DiagnosticStatus.ERROR
        status.values = [
            KeyValue(
                key="failure_category",
                value="healthy" if healthy else "registration_rejected",
            )
        ]
        output.status = [status]
        return output

    def call_initialize(self, with_pose: bool):
        request = InitializeLocalization.Request()
        request.method = InitializeLocalization.Request.DIRECT
        if with_pose:
            request.pose_with_covariance.append(self.pose())
        future = self.client.call_async(request)
        if not self.spin_until(future.done):
            raise RuntimeError("/localization/initialize response timed out")
        return future.result()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=pathlib.Path)
    args = parser.parse_args()
    rclpy.init()
    node = LocalizationProbe()
    try:
        if not node.client.wait_for_service(timeout_sec=8.0):
            raise RuntimeError("/localization/initialize did not appear")
        if not node.spin_until(lambda: bool(node.states)):
            raise RuntimeError("initialization state was not published")
        initial_uninitialized = (
            node.states[-1] == LocalizationInitializationState.UNINITIALIZED
        )

        initial_pose = node.pose()
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline and not node.direct_poses:
            node.initial_publisher.publish(initial_pose)
            rclpy.spin_once(node, timeout_sec=0.05)
        forwarded = bool(
            node.direct_poses and node.direct_poses[-1].header.frame_id == "map"
        )
        initializing = node.spin_until(
            lambda: node.states
            and node.states[-1] == LocalizationInitializationState.INITIALIZING
        )

        node.alignment_publisher.publish(node.alignment(False))
        node.spin_until(lambda: False, timeout=0.25)
        unhealthy_not_initialized = (
            node.states[-1] == LocalizationInitializationState.INITIALIZING
        )

        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline and (
            not node.states
            or node.states[-1] != LocalizationInitializationState.INITIALIZED
        ):
            node.alignment_publisher.publish(node.alignment(True))
            rclpy.spin_once(node, timeout_sec=0.05)
        healthy_initialized = (
            node.states[-1] == LocalizationInitializationState.INITIALIZED
        )

        node.reinitialization_publisher.publish(Bool(data=True))
        reinitialized = node.spin_until(
            lambda: node.states
            and node.states[-1] == LocalizationInitializationState.UNINITIALIZED
        )

        empty_response = node.call_initialize(False)
        direct_pose_count = len(node.direct_poses)
        direct_response = node.call_initialize(True)
        direct_service_forwarded = node.spin_until(
            lambda: len(node.direct_poses) > direct_pose_count
            and node.states
            and node.states[-1] == LocalizationInitializationState.INITIALIZING
        )
        checks = {
            "starts_uninitialized": initial_uninitialized,
            "rviz_pose_forwarded_in_map": forwarded,
            "pose_enters_initializing": initializing,
            "unhealthy_alignment_not_initialized": unhealthy_not_initialized,
            "healthy_alignment_initializes": healthy_initialized,
            "reinitialization_revokes_state": reinitialized,
            "empty_initialize_rejected": not empty_response.status.success,
            "direct_initialize_accepted": direct_response.status.success,
            "direct_initialize_forwarded": direct_service_forwarded,
        }
        report = {"result": "PASS" if all(checks.values()) else "FAIL", "checks": checks}
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
