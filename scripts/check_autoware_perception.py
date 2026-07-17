#!/usr/bin/env python3
"""Inject a synthetic LiDAR cluster and verify the Autoware object contract."""

import argparse
import json
import math
import pathlib
import struct
import time

import rclpy
from autoware_perception_msgs.msg import (
    DetectedObjects,
    ObjectClassification,
    PredictedObjects,
)
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class PerceptionProbe(Node):
    def __init__(self) -> None:
        super().__init__("romo_b_autoware_perception_probe")
        self.cloud_publisher = self.create_publisher(
            PointCloud2,
            "/sensing/lidar/top/pointcloud_filtered",
            qos_profile_sensor_data,
        )
        self.detected = None
        self.predicted = None
        self.create_subscription(
            DetectedObjects,
            "/perception/object_recognition/detection/objects",
            self._on_detected,
            10,
        )
        self.create_subscription(
            PredictedObjects,
            "/perception/object_recognition/objects",
            self._on_predicted,
            10,
        )
        broadcaster = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_footprint"
        transform.transform.rotation.w = 1.0
        broadcaster.sendTransform(transform)
        self.broadcaster = broadcaster

    def _on_detected(self, message: DetectedObjects) -> None:
        if message.objects:
            self.detected = message

    def _on_predicted(self, message: PredictedObjects) -> None:
        if message.objects:
            self.predicted = message

    def publish_cluster(self, elapsed: float) -> None:
        # Move slowly enough to stay associated while exercising velocity and
        # future-path generation. The dense 5 cm grid is one PCL cluster.
        center_x = 2.0 + min(elapsed, 1.0) * 0.10
        points = []
        for ix in range(-2, 3):
            for iy in range(-2, 3):
                for iz in range(0, 3):
                    points.append(
                        (center_x + ix * 0.05, iy * 0.05, 0.40 + iz * 0.10)
                    )
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
        message.point_step = 12
        message.row_step = 12 * len(points)
        message.is_dense = True
        message.data = b"".join(struct.pack("<fff", *point) for point in points)
        self.cloud_publisher.publish(message)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=12.0)
    parser.add_argument("--output", type=pathlib.Path)
    args = parser.parse_args()
    rclpy.init()
    node = PerceptionProbe()
    started = time.monotonic()
    try:
        while time.monotonic() - started < args.timeout:
            elapsed = time.monotonic() - started
            node.publish_cluster(elapsed)
            rclpy.spin_once(node, timeout_sec=0.08)
            if elapsed > 1.0 and node.detected and node.predicted:
                break

        detected = node.detected.objects[0] if node.detected else None
        predicted = node.predicted.objects[0] if node.predicted else None
        detected_unknown = bool(
            detected
            and detected.classification
            and detected.classification[0].label == ObjectClassification.UNKNOWN
        )
        predicted_unknown = bool(
            predicted
            and predicted.classification
            and predicted.classification[0].label == ObjectClassification.UNKNOWN
        )
        prediction_points = (
            len(predicted.kinematics.predicted_paths[0].path)
            if predicted and predicted.kinematics.predicted_paths
            else 0
        )
        position = (
            predicted.kinematics.initial_pose_with_covariance.pose.position
            if predicted
            else None
        )
        finite_map_pose = bool(
            position
            and node.predicted.header.frame_id == "map"
            and math.isfinite(position.x)
            and math.isfinite(position.y)
        )
        checks = {
            "cluster_detected": detected is not None,
            "detected_as_unknown": detected_unknown,
            "predicted_object_published": predicted is not None,
            "predicted_as_unknown": predicted_unknown,
            "map_transform_applied": finite_map_pose,
            "future_path_generated": prediction_points >= 2,
        }
        report = {
            "result": "PASS" if all(checks.values()) else "FAIL",
            "checks": checks,
            "prediction_point_count": prediction_points,
            "predicted_position_xy": [position.x, position.y] if position else None,
        }
        rendered = json.dumps(report, indent=2, sort_keys=True) + "\n"
        print(rendered, end="")
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(rendered, encoding="utf-8")
        return 0 if report["result"] == "PASS" else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
