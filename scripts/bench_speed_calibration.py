#!/usr/bin/env python3
"""Run one guarded, forward-only PCU speed calibration and save the feedback."""

import argparse
import json
import math
import pathlib
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from romo_b_msgs.msg import PlatformStatus
from std_srvs.srv import SetBool


class Calibration(Node):
    def __init__(self, speed, overspeed):
        super().__init__("romo_b_bench_speed_calibration")
        self.speed = speed
        self.overspeed = overspeed
        self.status = None
        self.hold_samples = []
        self.publisher = self.create_publisher(Twist, "/cmd_vel_safe", 10)
        self.create_subscription(
            PlatformStatus, "/romo_b/platform_status", self._on_status, 10
        )
        self.arm = self.create_client(SetBool, "/romo_b/arm")
        self.estop = self.create_client(SetBool, "/romo_b/software_estop")

    def _on_status(self, message):
        self.status = message

    def wait_status(self, timeout=4.0):
        deadline = time.monotonic() + timeout
        while self.status is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.status is None:
            raise RuntimeError("/romo_b/platform_status is unavailable")

    def call(self, client, value, timeout=3.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"service unavailable: {client.srv_name}")
        request = SetBool.Request()
        request.data = value
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError(f"service timeout: {client.srv_name}")
        return future.result()

    def publish_phase(self, duration, start_speed, end_speed, collect=False):
        started = time.monotonic()
        deadline = started + duration
        next_tick = started
        while time.monotonic() < deadline:
            now = time.monotonic()
            ratio = min(1.0, max(0.0, (now - started) / max(duration, 1e-6)))
            command = Twist()
            command.linear.x = start_speed + (end_speed - start_speed) * ratio
            self.publisher.publish(command)
            rclpy.spin_once(self, timeout_sec=0.005)
            if self.status is not None:
                wheel = [float(value) for value in self.status.wheel_speed_mps]
                maximum = max(abs(value) for value in wheel)
                if collect:
                    self.hold_samples.append(
                        {"time_sec": now - started, "wheel_speed_mps": wheel}
                    )
                if maximum > self.overspeed:
                    try:
                        self.call(self.estop, True, timeout=1.0)
                    finally:
                        raise RuntimeError(
                            f"overspeed feedback {maximum:.3f} m/s > {self.overspeed:.3f} m/s"
                        )
                if self.status.estop or self.status.command_timed_out or self.status.feedback_timed_out:
                    raise RuntimeError("bridge reported E-stop or watchdog fault")
            next_tick += 0.05
            time.sleep(max(0.0, next_tick - time.monotonic()))

    def stop_and_disarm(self):
        try:
            self.publish_phase(0.5, 0.0, 0.0)
        except Exception:
            pass
        try:
            self.call(self.arm, False, timeout=1.0)
        except Exception:
            pass
        command = Twist()
        for _ in range(6):
            self.publisher.publish(command)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.03)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true", help="required motion confirmation")
    parser.add_argument("--speed", type=float, default=0.01)
    parser.add_argument("--hold", type=float, default=1.5)
    parser.add_argument("--overspeed", type=float, default=0.10)
    parser.add_argument("--output")
    args = parser.parse_args()
    if not args.execute:
        parser.error("--execute is required; this command moves the robot")
    if not math.isfinite(args.speed) or not 0.005 <= args.speed <= 0.05:
        parser.error("--speed must be between 0.005 and 0.05 m/s")
    if not 0.5 <= args.hold <= 3.0:
        parser.error("--hold must be between 0.5 and 3.0 seconds")
    if not args.speed * 1.5 <= args.overspeed <= 0.15:
        parser.error("--overspeed must be >= 1.5*speed and <= 0.15 m/s")

    root = pathlib.Path(__file__).resolve().parents[1]
    output = pathlib.Path(args.output) if args.output else (
        root / "data/local/validation" /
        time.strftime("speed-calibration-%Y%m%d-%H%M%S.json")
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Calibration(args.speed, args.overspeed)
    result = {"result": "FAIL", "requested_speed_mps": args.speed}
    exit_code = 2
    try:
        node.wait_status()
        status = node.status
        if node.count_publishers("/cmd_vel_safe") != 1:
            raise RuntimeError("another /cmd_vel_safe publisher is active")
        if (
            not status.connected
            or status.state != PlatformStatus.STATE_CONNECTED_SAFE
            or status.estop
            or status.feedback_timed_out
            or status.steer_mode != 0
            or max(abs(float(v)) for v in status.wheel_speed_mps) >= 0.02
        ):
            raise RuntimeError("bridge must be connected, disarmed, 2WIS, fault-free, and stopped")

        node.publish_phase(0.6, 0.0, 0.0)
        response = node.call(node.arm, True)
        if not response.success:
            raise RuntimeError(response.message)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            node.publish_phase(0.1, 0.0, 0.0)
            if node.status.state == PlatformStatus.STATE_ARMED_AUTO and node.status.auto_mode:
                break
        else:
            raise RuntimeError("PCU did not enter armed Auto state")

        node.publish_phase(1.0, 0.0, args.speed)
        node.publish_phase(args.hold, args.speed, args.speed, collect=True)
        node.publish_phase(1.0, args.speed, 0.0)
        node.publish_phase(0.6, 0.0, 0.0)
        response = node.call(node.arm, False)
        if not response.success:
            raise RuntimeError(response.message)

        rear_means = [
            0.5 * (sample["wheel_speed_mps"][2] + sample["wheel_speed_mps"][3])
            for sample in node.hold_samples
            if sample["time_sec"] >= args.hold * 0.5
        ]
        if not rear_means:
            raise RuntimeError("no steady-state feedback samples")
        measured = statistics.median(rear_means)
        tolerance = max(0.01, args.speed * 0.5)
        passed = measured > 0.0 and abs(measured - args.speed) <= tolerance
        result.update(
            {
                "result": "PASS" if passed else "FAIL",
                "median_rear_speed_mps": measured,
                "tolerance_mps": tolerance,
                "samples": node.hold_samples,
            }
        )
        if not passed:
            raise RuntimeError(
                f"requested {args.speed:.3f}, measured median {measured:.3f} m/s"
            )
        exit_code = 0
    except (KeyboardInterrupt, Exception) as error:
        result["error"] = str(error)
    finally:
        node.stop_and_disarm()
        output.write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps({key: value for key, value in result.items() if key != "samples"}, indent=2))
        print(f"Report: {output}")
        node.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
