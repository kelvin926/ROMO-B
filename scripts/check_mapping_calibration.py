#!/usr/bin/env python3
"""Non-driving calibration check for ROMO-B mapping sensor alignment."""

import argparse
import json
import math
import pathlib
import statistics
import sys
import time
from collections import deque
from datetime import datetime

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2


def stamp_seconds(message):
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def percentile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        return None
    index = min(len(ordered) - 1, max(0, round((len(ordered) - 1) * fraction)))
    return ordered[index]


def sample_rate(samples, time_index=0):
    if len(samples) < 2:
        return 0.0
    elapsed = samples[-1][time_index] - samples[0][time_index]
    return (len(samples) - 1) / elapsed if elapsed > 0.0 else 0.0


def correlation(pairs):
    left = [pair[0] for pair in pairs]
    right = [pair[1] for pair in pairs]
    left_mean = statistics.fmean(left)
    right_mean = statistics.fmean(right)
    numerator = sum((a - left_mean) * (b - right_mean) for a, b in pairs)
    denominator = math.sqrt(
        sum((value - left_mean) ** 2 for value in left)
        * sum((value - right_mean) ** 2 for value in right)
    )
    return numerator / denominator if denominator > 0.0 else 0.0


def quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def load_extrinsics(hardware_path, rko_path):
    hardware = yaml.safe_load(hardware_path.read_text()) or {}
    rko_document = yaml.safe_load(rko_path.read_text()) or {}
    rko = rko_document.get("/**", {}).get("ros__parameters", {})
    transform = hardware.get("lidar", {}).get("transform", {})
    xyz = [float(transform.get(axis, 0.0)) for axis in ("x", "y", "z")]
    quaternion = quaternion_from_rpy(
        float(transform.get("roll", 0.0)),
        float(transform.get("pitch", 0.0)),
        float(transform.get("yaw", 0.0)),
    )
    expected = list(quaternion) + xyz
    lidar = [float(value) for value in rko.get("extrinsic_lidar2base_quat_xyzw_xyz", [])]
    imu = [float(value) for value in rko.get("extrinsic_imu2base_quat_xyzw_xyz", [])]

    def close(left, right, tolerance=1.0e-6):
        return len(left) == len(right) and all(
            abs(a - b) <= tolerance for a, b in zip(left, right)
        )

    return {
        "hardware_xyz_rpy": xyz
        + [
            float(transform.get("roll", 0.0)),
            float(transform.get("pitch", 0.0)),
            float(transform.get("yaw", 0.0)),
        ],
        "rko_lidar_quat_xyz": lidar,
        "rko_imu_quat_xyz": imu,
        "hardware_matches_rko_lidar": close(expected, lidar),
        "hardware_matches_rko_imu": close(expected, imu),
        "rko_lidar_matches_rko_imu": close(lidar, imu),
    }


class CalibrationProbe(Node):
    def __init__(self):
        super().__init__("romo_b_mapping_calibration_probe")
        self.phase = "waiting"
        self.imu_recent = deque(maxlen=5000)
        self.imu = {"stationary": [], "motion": []}
        self.wheel = {"stationary": [], "motion": []}
        self.cloud = []
        self.cloud_imu_offsets = []
        self.yaw_pairs = []
        self.lio_odom = []
        self.lio_recent = deque(maxlen=1000)
        self.lio_wheel_pairs = []
        self.wheel_seen = False
        self.cloud_fields = set()
        self.cloud_frame = ""
        self.imu_frame = ""
        self.wheel_frame = ""
        self.create_subscription(
            Imu, "/sensing/imu/imu_raw", self.on_imu, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2,
            "/sensing/lidar/top/pointcloud_raw",
            self.on_cloud,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            "/wheel/odometry_raw",
            self.on_wheel,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            "/rko_lio/odometry",
            self.on_lio,
            qos_profile_sensor_data,
        )

    def on_imu(self, message):
        wall = time.monotonic()
        stamp = stamp_seconds(message)
        sample = (
            wall,
            stamp,
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
            message.linear_acceleration.x,
            message.linear_acceleration.y,
            message.linear_acceleration.z,
        )
        self.imu_recent.append(sample)
        self.imu_frame = message.header.frame_id
        if self.phase in self.imu:
            self.imu[self.phase].append(sample)

    def on_cloud(self, message):
        wall = time.monotonic()
        stamp = stamp_seconds(message)
        self.cloud.append((wall, stamp, int(message.width) * int(message.height)))
        self.cloud_fields = {field.name for field in message.fields}
        self.cloud_frame = message.header.frame_id
        if self.imu_recent:
            nearest = min(self.imu_recent, key=lambda sample: abs(sample[1] - stamp))
            self.cloud_imu_offsets.append(abs(nearest[1] - stamp))

    def on_wheel(self, message):
        wall = time.monotonic()
        stamp = stamp_seconds(message)
        yaw = message.twist.twist.angular.z
        sample = (wall, stamp, message.twist.twist.linear.x, yaw)
        self.wheel_frame = message.child_frame_id
        self.wheel_seen = True
        if self.phase in self.wheel:
            self.wheel[self.phase].append(sample)
        if self.phase != "motion" or not self.imu_recent:
            return
        nearest = min(self.imu_recent, key=lambda imu: abs(imu[1] - stamp))
        imu_yaw = nearest[4]
        if abs(nearest[1] - stamp) <= 0.05 and abs(imu_yaw) >= 0.03 and abs(yaw) >= 0.01:
            self.yaw_pairs.append((imu_yaw, yaw, abs(nearest[1] - stamp)))
        if self.lio_recent:
            lio = min(self.lio_recent, key=lambda item: abs(item[1] - stamp))
            if abs(lio[1] - stamp) <= 0.15 and abs(lio[6]) >= 0.01 and abs(yaw) >= 0.01:
                self.lio_wheel_pairs.append(
                    (lio[6], yaw, math.sqrt(sum(value * value for value in lio[3:6])), abs(sample[2]))
                )

    def on_lio(self, message):
        sample = (
            time.monotonic(),
            stamp_seconds(message),
            message.pose.pose.position.x,
            message.twist.twist.linear.x,
            message.twist.twist.linear.y,
            message.twist.twist.linear.z,
            message.twist.twist.angular.z,
        )
        self.lio_odom.append(sample)
        self.lio_recent.append(sample)

    def raw_ready(self):
        return bool(self.imu_recent and self.cloud and self.wheel_seen)


def main():
    parser = argparse.ArgumentParser(
        description="Check live Mid-360, IMU, and wheel-odometry mapping calibration."
    )
    parser.add_argument("--stationary", type=float, default=6.0)
    parser.add_argument("--motion", type=float, default=20.0)
    parser.add_argument("--startup-timeout", type=float, default=15.0)
    parser.add_argument("--require-lio", action="store_true")
    parser.add_argument(
        "--hardware-config", default="config/local/hardware.yaml", type=pathlib.Path
    )
    parser.add_argument(
        "--rko-config", default="config/local/rko_lio_mid360.yaml", type=pathlib.Path
    )
    parser.add_argument("--output", type=pathlib.Path)
    args = parser.parse_args()

    if not args.hardware_config.is_file() or not args.rko_config.is_file():
        parser.error("hardware and RKO-LIO configuration files must exist")

    rclpy.init()
    node = CalibrationProbe()
    deadline = time.monotonic() + args.startup_timeout
    print("Waiting for LiDAR, normalized IMU, and wheel odometry ...", flush=True)
    while rclpy.ok() and not node.raw_ready() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    if not node.raw_ready():
        missing = []
        if not node.cloud:
            missing.append("/sensing/lidar/top/pointcloud_raw")
        if not node.imu_recent:
            missing.append("/sensing/imu/imu_raw")
        if not node.wheel_seen:
            missing.append("/wheel/odometry_raw")
        result = {"result": "FAIL", "error": "missing topics", "missing": missing}
        print(json.dumps(result, indent=2))
        node.destroy_node()
        rclpy.shutdown()
        return 1

    node.phase = "stationary"
    print(
        f"PHASE 1/{2}: keep the robot COMPLETELY STILL for {args.stationary:.0f} seconds.",
        flush=True,
    )
    deadline = time.monotonic() + args.stationary
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)

    node.phase = "motion"
    print(
        f"PHASE 2/2: drive slowly in RC Manual and make gentle LEFT and RIGHT turns for {args.motion:.0f} seconds.",
        flush=True,
    )
    deadline = time.monotonic() + args.motion
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
    node.phase = "done"

    static_imu = node.imu["stationary"]
    static_wheel = node.wheel["stationary"]
    gyro_bias = [statistics.fmean(sample[index] for sample in static_imu) for index in (2, 3, 4)]
    gyro_bias_norm = math.sqrt(sum(value * value for value in gyro_bias))
    accel_norms = [math.sqrt(sum(sample[index] ** 2 for index in (5, 6, 7))) for sample in static_imu]
    accel_norm_median = statistics.median(accel_norms)
    wheel_speed_median = statistics.median(abs(sample[2]) for sample in static_wheel)

    motion_pairs = [(imu, wheel) for imu, wheel, _ in node.yaw_pairs]
    yaw_correlation = correlation(motion_pairs) if len(motion_pairs) >= 20 else None
    sign_agreement = (
        sum(imu * wheel > 0.0 for imu, wheel in motion_pairs) / len(motion_pairs)
        if motion_pairs
        else None
    )
    yaw_scale = (
        statistics.median(abs(wheel) / abs(imu) for imu, wheel in motion_pairs)
        if motion_pairs
        else None
    )
    lio_yaw_pairs = [(lio, wheel) for lio, wheel, _, _ in node.lio_wheel_pairs]
    lio_yaw_correlation = correlation(lio_yaw_pairs) if len(lio_yaw_pairs) >= 10 else None
    lio_sign_agreement = (
        sum(lio * wheel > 0.0 for lio, wheel in lio_yaw_pairs) / len(lio_yaw_pairs)
        if lio_yaw_pairs
        else None
    )
    lio_yaw_scale = (
        statistics.median(abs(wheel) / abs(lio) for lio, wheel in lio_yaw_pairs)
        if lio_yaw_pairs
        else None
    )
    lio_speed_ratios = [
        lio_speed / wheel_speed
        for _, _, lio_speed, wheel_speed in node.lio_wheel_pairs
        if wheel_speed >= 0.03
    ]
    lio_speed_scale = statistics.median(lio_speed_ratios) if lio_speed_ratios else None
    extrinsics = load_extrinsics(args.hardware_config, args.rko_config)

    checks = {
        "lidar_rate_7_to_15_hz": 7.0 <= sample_rate(node.cloud, 1) <= 15.0,
        "imu_rate_at_least_100_hz": sample_rate(static_imu + node.imu["motion"], 1) >= 100.0,
        "wheel_odom_rate_at_least_10_hz": sample_rate(static_wheel + node.wheel["motion"], 1) >= 10.0,
        "lidar_fields_include_xyz_timestamp": {"x", "y", "z", "timestamp"}.issubset(node.cloud_fields),
        "sensor_frames_are_expected": node.cloud_frame == "livox_frame"
        and node.imu_frame == "livox_frame"
        and node.wheel_frame == "base_link",
        "lidar_imu_timestamp_p95_under_30_ms": bool(node.cloud_imu_offsets)
        and percentile(node.cloud_imu_offsets, 0.95) <= 0.03,
        "stationary_gyro_bias_under_0_05_radps": gyro_bias_norm <= 0.05,
        "stationary_acceleration_is_gravity": 9.0 <= accel_norm_median <= 10.6,
        "stationary_wheel_speed_under_0_03_mps": wheel_speed_median <= 0.03,
        "yaw_motion_pairs_at_least_20": len(motion_pairs) >= 20,
        "imu_wheel_yaw_same_sign": sign_agreement is not None and sign_agreement >= 0.80,
        "imu_wheel_yaw_correlated": yaw_correlation is not None and yaw_correlation >= 0.50,
        "imu_wheel_yaw_scale_plausible": yaw_scale is not None and 0.5 <= yaw_scale <= 2.0,
        "hardware_and_rko_extrinsics_match": all(
            extrinsics[key]
            for key in (
                "hardware_matches_rko_lidar",
                "hardware_matches_rko_imu",
                "rko_lidar_matches_rko_imu",
            )
        ),
    }
    if args.require_lio:
        checks["rko_lio_odometry_is_publishing"] = len(node.lio_odom) >= 10
        checks["rko_lio_wheel_yaw_pairs_at_least_10"] = len(lio_yaw_pairs) >= 10
        checks["rko_lio_wheel_yaw_same_sign"] = (
            lio_sign_agreement is not None and lio_sign_agreement >= 0.80
        )
        checks["rko_lio_wheel_yaw_correlated"] = (
            lio_yaw_correlation is not None and lio_yaw_correlation >= 0.50
        )
        checks["rko_lio_wheel_yaw_scale_plausible"] = (
            lio_yaw_scale is not None and 0.4 <= lio_yaw_scale <= 2.5
        )
        checks["rko_lio_wheel_speed_scale_plausible"] = (
            lio_speed_scale is not None and 0.4 <= lio_speed_scale <= 2.5
        )

    report = {
        "result": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "rates_hz": {
            "lidar_sensor_stamp": sample_rate(node.cloud, 1),
            "lidar_arrival": sample_rate(node.cloud),
            "imu_sensor_stamp": sample_rate(static_imu + node.imu["motion"], 1),
            "imu_arrival": sample_rate(static_imu + node.imu["motion"]),
            "wheel_odometry_stamp": sample_rate(static_wheel + node.wheel["motion"], 1),
            "wheel_odometry_arrival": sample_rate(static_wheel + node.wheel["motion"]),
            "rko_lio_odometry_stamp": sample_rate(node.lio_odom, 1),
            "rko_lio_odometry_arrival": sample_rate(node.lio_odom),
        },
        "frames": {
            "lidar": node.cloud_frame,
            "imu": node.imu_frame,
            "wheel_child": node.wheel_frame,
        },
        "lidar_fields": sorted(node.cloud_fields),
        "lidar_imu_timestamp_offset_p95_sec": percentile(node.cloud_imu_offsets, 0.95),
        "stationary": {
            "gyro_bias_xyz_radps": gyro_bias,
            "gyro_bias_norm_radps": gyro_bias_norm,
            "acceleration_norm_median_mps2": accel_norm_median,
            "wheel_speed_median_mps": wheel_speed_median,
        },
        "motion": {
            "paired_samples": len(motion_pairs),
            "yaw_correlation": yaw_correlation,
            "yaw_sign_agreement": sign_agreement,
            "wheel_to_imu_yaw_scale_median": yaw_scale,
            "rko_lio_wheel_paired_samples": len(lio_yaw_pairs),
            "rko_lio_wheel_yaw_correlation": lio_yaw_correlation,
            "rko_lio_wheel_yaw_sign_agreement": lio_sign_agreement,
            "wheel_to_rko_lio_yaw_scale_median": lio_yaw_scale,
            "rko_lio_to_wheel_speed_scale_median": lio_speed_scale,
        },
        "extrinsics": extrinsics,
    }

    output = args.output
    if output is None:
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        output = pathlib.Path("data/local/validation") / f"mapping-calibration-{stamp}.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    print(f"Report: {output}")

    node.destroy_node()
    rclpy.shutdown()
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
