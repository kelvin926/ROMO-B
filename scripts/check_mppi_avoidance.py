#!/usr/bin/env python3
"""Verify MPPI produces a moving detour around a synthetic obstacle."""

import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class AvoidanceProbe(Node):
    def __init__(self):
        super().__init__("romo_b_mppi_avoidance_probe")
        self.client = ActionClient(self, FollowPath, "/follow_path")
        self.cloud_pub = self.create_publisher(
            PointCloud2,
            "/sensing/lidar/top/pointcloud_filtered",
            qos_profile_sensor_data,
        )
        self.odom_pub = self.create_publisher(
            Odometry, "/odometry/filtered", qos_profile_sensor_data
        )
        self.commands = []
        self.create_subscription(Twist, "/cmd_vel_nav", self._on_command, 20)

    def _on_command(self, message):
        self.commands.append((time.monotonic(), message.linear.x, message.angular.z))

    def publish_inputs(self):
        now = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)

        # A 0.45 m obstacle centered 1.10 m ahead, slightly left to make the
        # deterministic preferred detour pass on its right. It is beyond the
        # independent 0.70 m last-resort stop polygon.
        points = []
        for ix in range(-4, 5):
            for iy in range(-4, 5):
                x = 1.10 + ix * 0.05
                y = 0.05 + iy * 0.05
                if math.hypot(x - 1.10, y - 0.05) <= 0.225:
                    points.append((x, y, 0.50))
        header = Header(stamp=now, frame_id="base_link")
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points))


def spin_until(node, predicate, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


def make_path(node):
    path = Path()
    path.header.stamp = node.get_clock().now().to_msg()
    path.header.frame_id = "map"
    for index in range(71):
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = index * 0.05
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    return path


def main():
    rclpy.init()
    node = AvoidanceProbe()
    try:
        if not node.client.wait_for_server(timeout_sec=8.0):
            raise RuntimeError("/follow_path action server is unavailable")

        # Pre-fill the transient costmap before requesting control.
        for _ in range(12):
            node.publish_inputs()
            rclpy.spin_once(node, timeout_sec=0.05)

        goal = FollowPath.Goal()
        goal.path = make_path(node)
        goal.controller_id = "FollowPath"
        goal.goal_checker_id = "goal_checker"
        send_future = node.client.send_goal_async(goal)
        if not spin_until(node, send_future.done, 5.0):
            raise RuntimeError("FollowPath goal response timed out")
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("FollowPath goal was rejected")

        start = time.monotonic()
        while time.monotonic() - start < 4.0:
            node.publish_inputs()
            rclpy.spin_once(node, timeout_sec=0.05)

        cancel_future = handle.cancel_goal_async()
        spin_until(node, cancel_future.done, 2.0)

        settled = [sample for sample in node.commands if sample[0] - start > 0.8]
        moving_detours = [
            sample for sample in settled
            if sample[1] > 0.03 and abs(sample[2]) > 0.05
        ]
        report = {
            "result": "PASS" if moving_detours else "FAIL",
            "controller": "Ackermann MPPI",
            "speed_limit_mps": 0.5,
            "samples": len(settled),
            "moving_detour_samples": len(moving_detours),
            "max_forward_mps": max((sample[1] for sample in settled), default=0.0),
            "max_abs_yaw_rate_radps": max(
                (abs(sample[2]) for sample in settled), default=0.0
            ),
        }
        print(json.dumps(report, indent=2))
        if not moving_detours:
            raise RuntimeError("MPPI did not produce a moving avoidance turn")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
