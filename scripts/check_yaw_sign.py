#!/usr/bin/env python3
"""Compare wheel-odometry yaw sign with the Mid-360 IMU yaw sign."""

import argparse
import json
import math
import statistics
import time
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def stamp_seconds(message):
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


class YawSignProbe(Node):
    def __init__(self):
        super().__init__("romo_b_yaw_sign_probe")
        self.imu_samples = deque(maxlen=1000)
        self.pairs = []
        self.create_subscription(
            Imu, "/sensing/imu/imu_raw", self.on_imu, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry,
            "/wheel/odometry_raw",
            self.on_odom,
            qos_profile_sensor_data,
        )

    def on_imu(self, message):
        self.imu_samples.append((stamp_seconds(message), message.angular_velocity.z))

    def on_odom(self, message):
        if not self.imu_samples:
            return
        stamp = stamp_seconds(message)
        imu_stamp, imu_yaw = min(
            self.imu_samples, key=lambda sample: abs(sample[0] - stamp)
        )
        odom_yaw = message.twist.twist.angular.z
        if abs(imu_stamp - stamp) <= 0.05 and abs(imu_yaw) >= 0.03 and abs(odom_yaw) >= 0.01:
            self.pairs.append((imu_yaw, odom_yaw))


def correlation(pairs):
    imu = [pair[0] for pair in pairs]
    odom = [pair[1] for pair in pairs]
    imu_mean = statistics.fmean(imu)
    odom_mean = statistics.fmean(odom)
    numerator = sum((a - imu_mean) * (b - odom_mean) for a, b in pairs)
    denominator = math.sqrt(
        sum((value - imu_mean) ** 2 for value in imu)
        * sum((value - odom_mean) ** 2 for value in odom)
    )
    return numerator / denominator if denominator > 0.0 else 0.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=25.0)
    args = parser.parse_args()

    rclpy.init()
    node = YawSignProbe()
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        pairs = node.pairs
        result = {"pair_count": len(pairs), "result": "INSUFFICIENT_MOTION"}
        if len(pairs) >= 20:
            corr = correlation(pairs)
            agreement = sum(a * b > 0.0 for a, b in pairs) / len(pairs)
            result.update(
                {
                    "correlation": corr,
                    "sign_agreement": agreement,
                    "median_abs_imu_yaw_radps": statistics.median(
                        abs(pair[0]) for pair in pairs
                    ),
                    "median_abs_odom_yaw_radps": statistics.median(
                        abs(pair[1]) for pair in pairs
                    ),
                }
            )
            if corr >= 0.5 and agreement >= 0.7:
                result["result"] = "SAME_SIGN"
            elif corr <= -0.5 and agreement <= 0.3:
                result["result"] = "OPPOSITE_SIGN"
            else:
                result["result"] = "INCONCLUSIVE"
        print(json.dumps(result, indent=2))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
