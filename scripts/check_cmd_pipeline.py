#!/usr/bin/env python3
import json
import math
import struct
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


class PipelineCheck(Node):
    def __init__(self):
        super().__init__("romo_b_command_pipeline_check")
        self.cloud_pub = self.create_publisher(
            PointCloud2,
            "/sensing/lidar/top/pointcloud_filtered",
            qos_profile_sensor_data,
        )
        self.command_pub = self.create_publisher(Twist, "/cmd_vel_teleop", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odometry/filtered", 10)
        self.safe_samples = []
        self.create_subscription(Twist, "/cmd_vel_safe", self._on_safe, 10)

    def _on_safe(self, message):
        self.safe_samples.append((message.linear.x, message.angular.z))

    def publish_cloud(self, points):
        message = PointCloud2()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_footprint"
        message.height = 1
        message.width = len(points)
        message.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = 12 * len(points)
        message.data = b"".join(struct.pack("<fff", *point) for point in points)
        message.is_dense = True
        self.cloud_pub.publish(message)

    def publish_odom(self):
        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "odom"
        message.child_frame_id = "base_link"
        message.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(message)

    def run_phase(self, points, speed, duration):
        self.safe_samples.clear()
        command = Twist()
        command.linear.x = speed
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.publish_cloud(points)
            self.publish_odom()
            self.command_pub.publish(command)
            rclpy.spin_once(self, timeout_sec=0.04)
        return list(self.safe_samples)


def main():
    rclpy.init()
    node = PipelineCheck()
    try:
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if node.cloud_pub.get_subscription_count() >= 1 and node.command_pub.get_subscription_count() >= 1:
                break
            rclpy.spin_once(node, timeout_sec=0.1)
        else:
            raise RuntimeError("Command pipeline subscriptions did not appear")

        clear_samples = node.run_phase([], 0.5, 2.0)
        obstacle = [(0.50, -0.05, 0.50), (0.50, 0.0, 0.50), (0.50, 0.05, 0.50)]
        stop_samples = node.run_phase(obstacle, 0.2, 1.0)
        maximum_clear_speed = max((sample[0] for sample in clear_samples), default=math.nan)
        recent_stop = stop_samples[-5:]
        checks = {
            "clear_output_present": bool(clear_samples),
            "velocity_clamped": 0.15 <= maximum_clear_speed <= 0.2001,
            "obstacle_output_present": bool(stop_samples),
            "obstacle_stop": len(recent_stop) == 5
            and all(abs(linear) < 1e-4 and abs(angular) < 1e-4 for linear, angular in recent_stop),
        }
        summary = {
            "clear_sample_count": len(clear_samples),
            "maximum_clear_speed_mps": maximum_clear_speed,
            "stop_sample_count": len(stop_samples),
            "recent_stop_samples": recent_stop,
            "checks": checks,
            "result": "PASS" if all(checks.values()) else "FAIL",
        }
        print(json.dumps(summary, indent=2, sort_keys=True, allow_nan=False))
        return 0 if summary["result"] == "PASS" else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
