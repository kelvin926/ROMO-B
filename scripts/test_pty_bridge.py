#!/usr/bin/env python3
"""End-to-end safety test using only a PTY; never opens physical hardware."""

import os
import pathlib
import signal
import subprocess
import tempfile
import time

import rclpy
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from romo_b_msgs.msg import PlatformStatus
from std_srvs.srv import SetBool


class Probe(Node):
    def __init__(self):
        super().__init__("romo_b_pty_test_probe")
        self.status = None
        self.status_count = 0
        self.max_speed = 0.0
        self.create_subscription(
            PlatformStatus, "/romo_b/platform_status", self._on_status, 20
        )
        self.create_subscription(
            Odometry, "/wheel/odometry_raw", self._on_odom, qos_profile_sensor_data
        )
        self.command_pub = self.create_publisher(Twist, "/cmd_vel_safe", 10)
        self.lifecycle = self.create_client(
            ChangeState, "/romo_b_serial_bridge/change_state"
        )
        self.arm = self.create_client(SetBool, "/romo_b/arm")
        self.feedback = self.create_client(SetBool, "/romo_b_sim/feedback")
        self.alive = self.create_client(SetBool, "/romo_b_sim/alive")

    def _on_status(self, message):
        self.status = message
        self.status_count += 1

    def _on_odom(self, message):
        self.max_speed = max(self.max_speed, abs(message.twist.twist.linear.x))


def wait_until(node, predicate, timeout, tick=None, description="condition"):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if tick is not None:
            tick()
        rclpy.spin_once(node, timeout_sec=0.03)
        if predicate():
            return
    raise TimeoutError(f"Timed out waiting for {description}")


def wait_service(node, client, name):
    wait_until(node, client.service_is_ready, 5.0, description=f"{name} service")


def call(node, client, request, name):
    future = client.call_async(request)
    wait_until(node, future.done, 3.0, description=f"{name} response")
    response = future.result()
    if response is None:
        raise RuntimeError(f"{name} returned no response")
    return response


def change_state(node, transition_id, name):
    request = ChangeState.Request()
    request.transition.id = transition_id
    response = call(node, node.lifecycle, request, name)
    if not response.success:
        raise RuntimeError(f"Lifecycle transition failed: {name}")


def set_bool(node, client, value, name):
    request = SetBool.Request()
    request.data = value
    response = call(node, client, request, name)
    if not response.success:
        raise RuntimeError(f"{name} failed: {response.message}")
    return response


def terminate(process):
    if process is None or process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2.0)


def publish_forward(probe):
    message = Twist()
    message.linear.x = 0.08
    message.angular.z = 0.04
    probe.command_pub.publish(message)


def main():
    # Isolate the test unless the operator deliberately chose a ROS domain.
    os.environ.setdefault("ROS_DOMAIN_ID", "93")
    symlink = pathlib.Path(f"/tmp/romo_b_pcu_integration_{os.getpid()}")
    simulator = None
    bridge = None
    probe = None
    with tempfile.TemporaryDirectory(prefix="romo_b_pty_test_") as temp_dir:
        simulator_log_path = pathlib.Path(temp_dir, "simulator.log")
        bridge_log_path = pathlib.Path(temp_dir, "bridge.log")
        with simulator_log_path.open("w") as simulator_log, bridge_log_path.open("w") as bridge_log:
            try:
                simulator = subprocess.Popen(
                    [
                        "ros2", "run", "romo_b_sim", "pcu_simulator", "--ros-args",
                        "-p", f"symlink_path:={symlink}", "-p", "auto_switch:=true",
                    ],
                    stdout=simulator_log,
                    stderr=subprocess.STDOUT,
                    text=True,
                )
                deadline = time.monotonic() + 5.0
                while time.monotonic() < deadline and not symlink.is_symlink():
                    if simulator.poll() is not None:
                        raise RuntimeError("PTY simulator exited before creating its device")
                    time.sleep(0.05)
                if not symlink.is_symlink():
                    raise TimeoutError("PTY simulator did not create its device")

                bridge = subprocess.Popen(
                    [
                        "ros2", "run", "romo_b_base", "romo_b_serial_bridge", "--ros-args",
                        "-p", f"device:={symlink}", "-p", "receive_only:=false",
                        "-p", "safety_profile:=bench", "-p", "command_endian:=big",
                    ],
                    stdout=bridge_log,
                    stderr=subprocess.STDOUT,
                    text=True,
                )

                rclpy.init()
                probe = Probe()
                wait_service(probe, probe.lifecycle, "lifecycle")
                change_state(probe, Transition.TRANSITION_CONFIGURE, "configure")
                for client, name in (
                    (probe.arm, "arm"),
                    (probe.feedback, "simulator feedback"),
                    (probe.alive, "simulator alive"),
                ):
                    wait_service(probe, client, name)
                change_state(probe, Transition.TRANSITION_ACTIVATE, "activate")
                if any(
                    name == "/romo_b/software_estop"
                    for name, _types in probe.get_service_names_and_types()
                ):
                    raise RuntimeError("software E-stop service must not exist")
                wait_until(
                    probe,
                    lambda: probe.status is not None
                    and probe.status.connected
                    and not probe.status.feedback_timed_out,
                    3.0,
                    description="fresh PCU feedback",
                )

                set_bool(probe, probe.arm, True, "arm")
                wait_until(
                    probe,
                    lambda: probe.max_speed > 0.04,
                    2.0,
                    tick=lambda: publish_forward(probe),
                    description="non-zero odometry",
                )

                # A planner/collision-monitor pause must stop motion without
                # latching the PCU's Hi_E-ST; a fresh command resumes in-place.
                wait_until(
                    probe,
                    lambda: probe.status is not None
                    and probe.status.state == PlatformStatus.STATE_ARMED_AUTO
                    and probe.status.command_timed_out,
                    2.0,
                    description="command-timeout soft stop",
                )
                wait_until(
                    probe,
                    lambda: probe.status is not None
                    and max(abs(value) for value in probe.status.wheel_speed_mps) < 0.02,
                    2.0,
                    description="stopped feedback",
                )
                wait_until(
                    probe,
                    lambda: probe.status is not None
                    and not probe.status.command_timed_out
                    and probe.status.state == PlatformStatus.STATE_ARMED_AUTO
                    and max(abs(value) for value in probe.status.wheel_speed_mps) > 0.04,
                    2.0,
                    tick=lambda: publish_forward(probe),
                    description="automatic recovery from soft stop",
                )

                set_bool(probe, probe.arm, False, "disarm after soft stop test")
                wait_until(
                    probe,
                    lambda: probe.status.state == PlatformStatus.STATE_CONNECTED_SAFE
                    and not probe.status.estop
                    and max(abs(value) for value in probe.status.wheel_speed_mps) < 0.02,
                    2.0,
                    description="clean stopped disarm state",
                )

                # Feedback loss disarms with a zero Manual command. Software
                # must never assert the PCU E-stop bit.
                set_bool(probe, probe.arm, True, "re-arm")
                set_bool(probe, probe.feedback, False, "pause feedback")
                wait_until(
                    probe,
                    lambda: probe.status.state == PlatformStatus.STATE_CONNECTED_SAFE
                    and probe.status.feedback_timed_out
                    and not probe.status.estop,
                    2.0,
                    tick=lambda: publish_forward(probe),
                    description="feedback-timeout zero disarm",
                )
                if probe.status.command_timed_out:
                    raise RuntimeError("command timeout tripped during feedback-timeout test")
                alive_before = probe.status.pcu_alive
                set_bool(probe, probe.feedback, True, "resume feedback")
                wait_until(
                    probe,
                    lambda: probe.status.pcu_alive != alive_before
                    and max(abs(value) for value in probe.status.wheel_speed_mps) < 0.02,
                    2.0,
                    description="fresh stopped feedback after resume",
                )
                wait_until(
                    probe,
                    lambda: probe.status.state == PlatformStatus.STATE_CONNECTED_SAFE
                    and not probe.status.estop,
                    2.0,
                    description="feedback recovery without E-stop",
                )

                # Frames that keep arriving with a frozen PCU ALIVE are also stale feedback.
                set_bool(probe, probe.arm, True, "re-arm for ALIVE test")
                set_bool(probe, probe.alive, False, "freeze PCU ALIVE")
                wait_until(
                    probe,
                    lambda: probe.status.state == PlatformStatus.STATE_CONNECTED_SAFE
                    and probe.status.feedback_timed_out
                    and not probe.status.estop,
                    2.0,
                    tick=lambda: publish_forward(probe),
                    description="stale-ALIVE zero disarm",
                )
                if probe.status.command_timed_out:
                    raise RuntimeError("command timeout tripped during stale-ALIVE test")
                alive_before = probe.status.pcu_alive
                set_bool(probe, probe.alive, True, "resume PCU ALIVE")
                wait_until(
                    probe,
                    lambda: probe.status.pcu_alive != alive_before
                    and max(abs(value) for value in probe.status.wheel_speed_mps) < 0.02,
                    2.0,
                    description="fresh ALIVE after resume",
                )
                wait_until(
                    probe,
                    lambda: probe.status.state == PlatformStatus.STATE_CONNECTED_SAFE
                    and not probe.status.estop,
                    2.0,
                    description="ALIVE recovery without E-stop",
                )
                print(
                    "PTY_INTEGRATION_OK: Auto handshake, motion, command soft-stop/recovery, "
                    "feedback/ALIVE zero-disarm, software E-stop disabled"
                )
            except Exception:
                simulator_log.flush()
                bridge_log.flush()
                print("--- simulator log ---")
                print(simulator_log_path.read_text(errors="replace"))
                print("--- bridge log ---")
                print(bridge_log_path.read_text(errors="replace"))
                raise
            finally:
                if probe is not None:
                    probe.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
                terminate(bridge)
                terminate(simulator)
                if symlink.is_symlink():
                    symlink.unlink()


if __name__ == "__main__":
    main()
