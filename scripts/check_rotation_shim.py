#!/usr/bin/env python3
"""Verify Nav2 requests Pivot and the complete safe command pipeline preserves it."""

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


class RotationProbe(Node):
    def __init__(self):
        super().__init__("romo_b_rotation_shim_probe")
        self.client = ActionClient(self, FollowPath, "/follow_path")
        self.odom_pub = self.create_publisher(Odometry, "/odometry/filtered", 10)
        self.cloud_pub = self.create_publisher(
            PointCloud2,
            "/sensing/lidar/top/pointcloud_filtered",
            qos_profile_sensor_data,
        )
        self.nav_commands = []
        self.safe_commands = []
        self.create_subscription(Twist, "/cmd_vel_nav", self.nav_commands.append, 20)
        self.create_subscription(Twist, "/cmd_vel_safe", self.safe_commands.append, 20)

    def publish_inputs(self):
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.orientation.w = 1.0
        if self.safe_commands:
            # Feed the closed-loop shim its last measured angular velocity so
            # the acceleration limiter can ramp beyond its first 0.08 rad/s.
            odom.twist.twist.angular.z = self.safe_commands[-1].angular.z
        self.odom_pub.publish(odom)
        self.cloud_pub.publish(
            point_cloud2.create_cloud_xyz32(
                Header(stamp=stamp, frame_id="base_link"), []
            )
        )


def spin_until(node, predicate, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        node.publish_inputs()
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


def left_facing_path(node):
    path = Path()
    path.header.stamp = node.get_clock().now().to_msg()
    path.header.frame_id = "map"
    for index in range(41):
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.y = index * 0.05
        pose.pose.orientation.z = math.sin(math.pi / 4.0)
        pose.pose.orientation.w = math.cos(math.pi / 4.0)
        path.poses.append(pose)
    return path


def is_counter_clockwise_pivot(command):
    return abs(command.linear.x) < 0.01 and command.angular.z > 0.20


def main():
    rclpy.init()
    node = RotationProbe()
    try:
        if not node.client.wait_for_server(timeout_sec=8.0):
            raise RuntimeError("/follow_path action server is unavailable")
        for _ in range(10):
            node.publish_inputs()
            rclpy.spin_once(node, timeout_sec=0.05)

        goal = FollowPath.Goal()
        goal.path = left_facing_path(node)
        goal.controller_id = "FollowPath"
        goal.goal_checker_id = "goal_checker"
        send_future = node.client.send_goal_async(goal)
        if not spin_until(node, send_future.done, 5.0):
            raise RuntimeError("FollowPath goal response timed out")
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("FollowPath goal was rejected")

        raw_ok = spin_until(
            node,
            lambda: any(is_counter_clockwise_pivot(cmd) for cmd in node.nav_commands),
            3.0,
        )
        safe_ok = spin_until(
            node,
            lambda: any(is_counter_clockwise_pivot(cmd) for cmd in node.safe_commands),
            3.0,
        )
        cancel_future = handle.cancel_goal_async()
        spin_until(node, cancel_future.done, 2.0)

        report = {
            "result": "PASS" if raw_ok and safe_ok else "FAIL",
            "controller": "Rotation Shim -> Ackermann MPPI",
            "requested_turn": "counter_clockwise",
            "nav_pivot_seen": raw_ok,
            "safe_pipeline_pivot_seen": safe_ok,
            "max_nav_angular_radps": max(
                (cmd.angular.z for cmd in node.nav_commands), default=0.0
            ),
            "max_safe_angular_radps": max(
                (cmd.angular.z for cmd in node.safe_commands), default=0.0
            ),
        }
        print(json.dumps(report, indent=2))
        if report["result"] != "PASS":
            raise RuntimeError("Pivot command did not cross the complete command pipeline")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
