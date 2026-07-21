import copy
import json
import logging
import math
import os
import pathlib
import signal
import subprocess
import threading
import time
import webbrowser
from collections import deque

import rclpy
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from flask import Flask, Response, jsonify, request, send_from_directory, stream_with_context
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from romo_b_msgs.msg import PlatformStatus
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import UInt8
from std_srvs.srv import SetBool, Trigger

from .model import (
    ackermann_twist,
    four_wis_twist,
    mode_name,
    pivot_twist,
    state_name,
    uint8_value,
)
from .operations import OperationManager


def _yaw_from_quaternion(quaternion) -> float:
    siny = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny, cosy)


def _quaternion_from_yaw(yaw_rad: float) -> tuple[float, float]:
    return math.sin(yaw_rad * 0.5), math.cos(yaw_rad * 0.5)


class FieldRuntime:
    """Own the optional field-navigation process without using a shell."""

    def __init__(self, repo_root: pathlib.Path):
        self.repo_root = repo_root
        self._lock = threading.RLock()
        self._process = None
        self._log_handle = None
        self._last_log = ""
        self._cache_deadline = 0.0
        self._cached_pids = []

    def _discover_pids(self) -> list[int]:
        now = time.monotonic()
        with self._lock:
            if now < self._cache_deadline:
                return list(self._cached_pids)
        found = []
        for proc in pathlib.Path("/proc").iterdir():
            if not proc.name.isdigit():
                continue
            try:
                command = (proc / "cmdline").read_bytes().replace(b"\0", b" ").decode(
                    errors="replace"
                )
            except (FileNotFoundError, PermissionError, ProcessLookupError):
                continue
            if "field_navigation.launch.py" in command and "romo_b_bringup" in command:
                found.append(int(proc.name))
        with self._lock:
            self._cached_pids = sorted(found)
            self._cache_deadline = now + 0.75
            return list(self._cached_pids)

    def status(self) -> dict:
        pids = self._discover_pids()
        with self._lock:
            owned = self._process is not None and self._process.poll() is None
            return {
                "field_running": bool(pids),
                "field_pids": pids,
                "owned_by_ui": owned,
                "log_path": self._last_log,
            }

    def start(self) -> dict:
        existing = self._discover_pids()
        if existing:
            return {
                "accepted": False,
                "message": f"Field navigation is already running (PID {existing[0]})",
            }
        script = self.repo_root / "scripts" / "run_field_navigation.sh"
        if not script.is_file():
            return {"accepted": False, "message": f"Missing {script}"}
        log_dir = self.repo_root / "data" / "local" / "logs"
        log_dir.mkdir(parents=True, exist_ok=True)
        log_path = log_dir / time.strftime("operator-field-%Y%m%d-%H%M%S.log")
        handle = log_path.open("ab", buffering=0)
        env = os.environ.copy()
        env.update(
            {
                "USE_OPERATOR_UI": "false",
                "OPEN_OPERATOR_UI_BROWSER": "false",
                "USE_RVIZ": "true",
            }
        )
        try:
            process = subprocess.Popen(
                [str(script)],
                cwd=str(self.repo_root),
                env=env,
                stdout=handle,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except Exception:
            handle.close()
            raise
        with self._lock:
            self._process = process
            self._log_handle = handle
            self._last_log = str(log_path)
            self._cache_deadline = 0.0
        return {
            "accepted": True,
            "message": f"Field navigation start requested (PID {process.pid})",
        }

    def stop(self) -> dict:
        pids = self._discover_pids()
        if not pids:
            return {"accepted": True, "message": "Field navigation is already stopped"}
        stopped = []
        for pid in pids:
            try:
                os.kill(pid, signal.SIGINT)
                stopped.append(pid)
            except (ProcessLookupError, PermissionError):
                continue
        with self._lock:
            self._cache_deadline = 0.0
        return {
            "accepted": bool(stopped),
            "message": "Stop requested for field navigation PID(s): "
            + ", ".join(str(pid) for pid in stopped),
        }


class OperatorNode(Node):
    def __init__(self, repo_root: pathlib.Path):
        super().__init__("romo_b_operator_ui")
        self._operations = OperationManager(repo_root)
        self._lock = threading.RLock()
        self._command_deadline = 0.0
        self._last_command_was_active = False
        self._goal_handle = None
        self._topic_times = {
            "platform": deque(maxlen=40),
            "lidar_raw": deque(maxlen=40),
            "lidar_filtered": deque(maxlen=40),
            "imu": deque(maxlen=80),
            "localization": deque(maxlen=40),
            "odometry": deque(maxlen=40),
            "cmd_nav": deque(maxlen=40),
            "cmd_selected": deque(maxlen=40),
            "cmd_smoothed": deque(maxlen=40),
            "cmd_safe": deque(maxlen=40),
        }
        self._state = {
            "version": "0.3.1",
            "platform": {
                "state": 0,
                "state_name": "DISCONNECTED",
                "connected": False,
                "auto_mode": False,
                "estop": False,
                "steer_mode": 0,
                "steer_mode_name": "2WIS",
                "wheel_speed_mps": [0.0, 0.0, 0.0, 0.0],
                "wheel_steer_deg": [0.0, 0.0, 0.0, 0.0],
                "pcu_alive": 0,
                "hlv_alive": 0,
                "command_timed_out": False,
                "feedback_timed_out": False,
            },
            "command": {
                "active": False,
                "mode": "2wis",
                "speed_mps": 0.0,
                "steer_deg": 0.0,
                "pivot_rate_radps": 0.0,
                "safe_linear_mps": 0.0,
                "safe_angular_radps": 0.0,
            },
            "commands": {
                name: {"linear_mps": 0.0, "angular_radps": 0.0}
                for name in ("nav", "selected", "smoothed", "safe")
            },
            "motion": {
                "wheel_odom_speed_mps": 0.0,
                "wheel_odom_yaw_rate_radps": 0.0,
                "odom_x_m": 0.0,
                "odom_y_m": 0.0,
                "odom_yaw_deg": 0.0,
            },
            "localization": {
                "available": False,
                "frame_id": "map",
                "x_m": 0.0,
                "y_m": 0.0,
                "yaw_deg": 0.0,
                "xy_std_m": 0.0,
                "yaw_std_deg": 0.0,
            },
            "sensors": {
                "lidar_raw": {"frame_id": "", "points": 0, "fields": []},
                "lidar_filtered": {"frame_id": "", "points": 0, "fields": []},
                "imu": {
                    "frame_id": "",
                    "angular_velocity_radps": [0.0, 0.0, 0.0],
                    "linear_acceleration_mps2": [0.0, 0.0, 0.0],
                },
            },
            "navigation": {
                "plan_points": 0,
                "plan_length_m": 0.0,
                "waypoint_count": 0,
                "goal_state": "IDLE",
                "goal": None,
                "last_action": "사용자 명령 대기 중",
                "last_action_success": True,
            },
            "diagnostics": {
                "level": 3,
                "summary": "진단 정보 없음",
                "items": [],
                "bridge_values": {},
            },
            "graph": {"node_count": 0, "topic_count": 0, "nodes": []},
            "host": {
                "hostname": os.uname().nodename,
                "load_1m": 0.0,
                "memory_used_gb": 0.0,
                "memory_total_gb": 0.0,
                "uptime_hours": 0.0,
                "gpu": {"available": False},
            },
        }

        self._command_publisher = self.create_publisher(Twist, "/cmd_vel_teleop", 10)
        self._mode_publisher = self.create_publisher(
            UInt8, "/romo_b/steer_mode_request", 10
        )
        self._initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.create_subscription(
            PlatformStatus, "/romo_b/platform_status", self._on_platform, 10
        )
        self.create_subscription(
            Odometry, "/wheel/odometry_raw", self._on_odometry, qos_profile_sensor_data
        )
        for key, topic in (
            ("nav", "/cmd_vel_nav"),
            ("selected", "/cmd_vel_selected"),
            ("smoothed", "/cmd_vel_smoothed"),
            ("safe", "/cmd_vel_safe"),
        ):
            self.create_subscription(Twist, topic, self._twist_callback(key), 10)
        self.create_subscription(
            DiagnosticArray, "/diagnostics", self._on_diagnostics, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/localization/pose_with_covariance",
            self._on_localization,
            10,
        )
        self.create_subscription(
            PointCloud2,
            "/sensing/lidar/top/pointcloud_raw",
            self._pointcloud_callback("lidar_raw"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            "/sensing/lidar/top/pointcloud_filtered",
            self._pointcloud_callback("lidar_filtered"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu, "/sensing/imu/imu_raw", self._on_imu, qos_profile_sensor_data
        )
        self.create_subscription(NavPath, "/plan", self._on_plan, 10)
        self.create_subscription(
            NavPath, "/romo_b/waypoints/path", self._on_waypoints, 10
        )

        self._arm_client = self.create_client(SetBool, "/romo_b/arm")
        self._navigate_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._waypoint_clients = {
            action: self.create_client(Trigger, f"/romo_b/waypoints/{action}")
            for action in ("save", "reload", "clear", "execute", "cancel")
        }
        self.create_timer(0.05, self._publish_teleop)
        self.create_timer(1.0, self._refresh_graph)

    def _mark_topic(self, name: str) -> None:
        self._topic_times[name].append(time.monotonic())

    def _on_platform(self, message: PlatformStatus) -> None:
        with self._lock:
            platform = self._state["platform"]
            platform.update(
                {
                    "state": int(message.state),
                    "state_name": state_name(message.state),
                    "connected": bool(message.connected),
                    "auto_mode": bool(message.auto_mode),
                    "estop": bool(message.estop),
                    "steer_mode": int(message.steer_mode),
                    "steer_mode_name": mode_name(message.steer_mode),
                    "wheel_speed_mps": [round(float(value), 3) for value in message.wheel_speed_mps],
                    "wheel_steer_deg": [
                        round(math.degrees(float(value)), 2)
                        for value in message.wheel_steer_rad
                    ],
                    "pcu_alive": int(message.pcu_alive),
                    "hlv_alive": int(message.hlv_alive),
                    "command_timed_out": bool(message.command_timed_out),
                    "feedback_timed_out": bool(message.feedback_timed_out),
                }
            )
            self._mark_topic("platform")

    def _on_odometry(self, message: Odometry) -> None:
        pose = message.pose.pose
        with self._lock:
            self._state["motion"].update(
                {
                    "wheel_odom_speed_mps": round(float(message.twist.twist.linear.x), 3),
                    "wheel_odom_yaw_rate_radps": round(float(message.twist.twist.angular.z), 3),
                    "odom_x_m": round(float(pose.position.x), 3),
                    "odom_y_m": round(float(pose.position.y), 3),
                    "odom_yaw_deg": round(
                        math.degrees(_yaw_from_quaternion(pose.orientation)), 1
                    ),
                }
            )
            self._mark_topic("odometry")

    def _twist_callback(self, name: str):
        def callback(message: Twist) -> None:
            with self._lock:
                linear = round(float(message.linear.x), 3)
                angular = round(float(message.angular.z), 3)
                self._state["commands"][name] = {
                    "linear_mps": linear,
                    "angular_radps": angular,
                }
                if name == "safe":
                    self._state["command"]["safe_linear_mps"] = linear
                    self._state["command"]["safe_angular_radps"] = angular
                self._mark_topic(f"cmd_{name}")

        return callback

    def _on_diagnostics(self, message: DiagnosticArray) -> None:
        items = sorted(
            (
                {
                    "name": status.name,
                    "level": uint8_value(status.level),
                    "message": status.message,
                    "hardware_id": status.hardware_id,
                    "values": {value.key: value.value for value in status.values},
                }
                for status in message.status
            ),
            key=lambda item: item["level"],
            reverse=True,
        )[:30]
        if items:
            worst = items[0]
            summary = worst["message"] or worst["name"]
            level = worst["level"]
        else:
            summary = "진단 정보가 비어 있습니다"
            level = 3
        with self._lock:
            bridge_values = next(
                (
                    item["values"]
                    for item in items
                    if "serial bridge" in item["name"].lower()
                    or "pcu" in item["name"].lower()
                ),
                self._state["diagnostics"].get("bridge_values", {}),
            )
            self._state["diagnostics"] = {
                "level": level,
                "summary": summary,
                "items": items,
                "bridge_values": bridge_values,
            }

    def _on_localization(self, message: PoseWithCovarianceStamped) -> None:
        pose = message.pose.pose
        covariance = list(message.pose.covariance)
        with self._lock:
            self._state["localization"].update(
                {
                    "available": True,
                    "frame_id": message.header.frame_id or "map",
                    "x_m": round(float(pose.position.x), 3),
                    "y_m": round(float(pose.position.y), 3),
                    "yaw_deg": round(math.degrees(_yaw_from_quaternion(pose.orientation)), 1),
                    "xy_std_m": round(
                        math.sqrt(max(0.0, covariance[0], covariance[7])), 3
                    ),
                    "yaw_std_deg": round(
                        math.degrees(math.sqrt(max(0.0, covariance[35]))), 1
                    ),
                }
            )
            self._mark_topic("localization")

    def _pointcloud_callback(self, name: str):
        def callback(message: PointCloud2) -> None:
            with self._lock:
                self._state["sensors"][name] = {
                    "frame_id": message.header.frame_id,
                    "points": int(message.width) * int(message.height),
                    "fields": [field.name for field in message.fields],
                }
                self._mark_topic(name)

        return callback

    def _on_imu(self, message: Imu) -> None:
        with self._lock:
            self._state["sensors"]["imu"] = {
                "frame_id": message.header.frame_id,
                "angular_velocity_radps": [
                    round(float(message.angular_velocity.x), 4),
                    round(float(message.angular_velocity.y), 4),
                    round(float(message.angular_velocity.z), 4),
                ],
                "linear_acceleration_mps2": [
                    round(float(message.linear_acceleration.x), 4),
                    round(float(message.linear_acceleration.y), 4),
                    round(float(message.linear_acceleration.z), 4),
                ],
            }
            self._mark_topic("imu")

    def _on_plan(self, message: NavPath) -> None:
        length = 0.0
        for current, following in zip(message.poses, message.poses[1:]):
            dx = following.pose.position.x - current.pose.position.x
            dy = following.pose.position.y - current.pose.position.y
            length += math.hypot(dx, dy)
        with self._lock:
            self._state["navigation"]["plan_points"] = len(message.poses)
            self._state["navigation"]["plan_length_m"] = round(length, 2)

    def _on_waypoints(self, message: NavPath) -> None:
        with self._lock:
            self._state["navigation"]["waypoint_count"] = len(message.poses)

    def _refresh_graph(self) -> None:
        try:
            nodes = sorted(
                f"{namespace.rstrip('/')}/{name}" if namespace != "/" else f"/{name}"
                for name, namespace in self.get_node_names_and_namespaces()
            )
            topics = self.get_topic_names_and_types()
        except Exception as error:
            self.get_logger().debug(f"ROS graph refresh failed: {error}")
            return
        host = self._host_status()
        with self._lock:
            self._state["graph"] = {
                "node_count": len(nodes),
                "topic_count": len(topics),
                "nodes": nodes,
            }
            self._state["host"] = host

    @staticmethod
    def _host_status() -> dict:
        meminfo = {}
        try:
            for line in pathlib.Path("/proc/meminfo").read_text().splitlines():
                key, value = line.split(":", 1)
                meminfo[key] = int(value.strip().split()[0])
            uptime_hours = float(pathlib.Path("/proc/uptime").read_text().split()[0]) / 3600.0
        except (OSError, ValueError, IndexError):
            uptime_hours = 0.0
        total_kib = meminfo.get("MemTotal", 0)
        available_kib = meminfo.get("MemAvailable", 0)
        gpu = {"available": False}
        try:
            output = subprocess.run(
                [
                    "nvidia-smi",
                    "--query-gpu=name,utilization.gpu,memory.used,memory.total,temperature.gpu",
                    "--format=csv,noheader,nounits",
                ],
                check=True,
                capture_output=True,
                text=True,
                timeout=1.0,
            ).stdout.splitlines()[0]
            name, utilization, used, total, temperature = [
                value.strip() for value in output.split(",", 4)
            ]
            gpu = {
                "available": True,
                "name": name,
                "utilization_percent": float(utilization),
                "memory_used_mb": float(used),
                "memory_total_mb": float(total),
                "temperature_c": float(temperature),
            }
        except (OSError, subprocess.SubprocessError, ValueError, IndexError):
            pass
        return {
            "hostname": os.uname().nodename,
            "load_1m": round(os.getloadavg()[0], 2),
            "memory_used_gb": round((total_kib - available_kib) / 1048576.0, 2),
            "memory_total_gb": round(total_kib / 1048576.0, 2),
            "uptime_hours": round(uptime_hours, 1),
            "gpu": gpu,
        }

    def _publish_twist(self, linear: float, angular: float) -> None:
        message = Twist()
        message.linear.x = float(linear)
        message.angular.z = float(angular)
        self._command_publisher.publish(message)

    def _publish_mode(self, mode: str) -> None:
        message = UInt8()
        message.data = {"2wis": 0, "4wis": 1, "pivot": 2}[mode]
        self._mode_publisher.publish(message)

    def _publish_teleop(self) -> None:
        with self._lock:
            active = self._command_deadline > time.monotonic()
            command = copy.deepcopy(self._state["command"])
            self._state["command"]["active"] = active
            should_send_zero = self._last_command_was_active and not active
            self._last_command_was_active = active
        # Steering mode is a maintained PCU command, not a one-shot event.  The
        # bridge intentionally expires a missing mode request after 0.6 s, so
        # keep the selected mode alive at this timer's 20 Hz even while the
        # dead-man drive command is released.  If this UI process dies, the
        # bridge timeout still returns the platform to its 2WIS fallback.
        self._publish_mode(command["mode"])
        if active:
            if command["mode"] == "pivot":
                linear, angular = pivot_twist(command["pivot_rate_radps"])
            elif command["mode"] == "4wis":
                linear, angular = four_wis_twist(
                    command["speed_mps"], command["steer_deg"]
                )
            else:
                linear, angular = ackermann_twist(
                    command["speed_mps"], command["steer_deg"]
                )
            self._publish_twist(linear, angular)
        elif should_send_zero:
            self._publish_twist(0.0, 0.0)

    def set_drive_command(self, payload: dict) -> dict:
        mode = str(payload.get("mode", "2wis")).lower()
        if mode not in ("2wis", "4wis", "pivot"):
            raise ValueError("Mode must be 2wis, 4wis, or pivot")
        active = bool(payload.get("active", False))
        speed = float(payload.get("speed_mps", 0.0))
        steer = float(payload.get("steer_deg", 0.0))
        pivot_rate = float(payload.get("pivot_rate_radps", 0.0))
        for value in (speed, steer, pivot_rate):
            if not math.isfinite(value):
                raise ValueError("Drive values must be finite")
        with self._lock:
            self._state["command"].update(
                {
                    "mode": mode,
                    "speed_mps": speed,
                    "steer_deg": steer,
                    "pivot_rate_radps": pivot_rate,
                    "active": active,
                }
            )
            self._command_deadline = time.monotonic() + 0.28 if active else 0.0
        if not active:
            self._publish_mode(mode)
            self._publish_twist(0.0, 0.0)
        return {"accepted": True, "active": active, "mode": mode}

    def _set_operation(self, message: str, success: bool) -> None:
        with self._lock:
            self._state["navigation"]["last_action"] = message
            self._state["navigation"]["last_action_success"] = bool(success)

    def set_arm(self, armed: bool) -> dict:
        if not armed:
            self.stop_motion()
        if not self._arm_client.service_is_ready():
            self._set_operation("Arm service is unavailable", False)
            return {"accepted": False, "message": "Arm service is unavailable"}
        request_message = SetBool.Request()
        request_message.data = bool(armed)
        future = self._arm_client.call_async(request_message)

        def complete(completed):
            try:
                result = completed.result()
                self._set_operation(result.message, result.success)
            except Exception as error:  # service failure should remain visible in the UI
                self._set_operation(f"Arm request failed: {error}", False)

        future.add_done_callback(complete)
        message = "Arm request sent" if armed else "Manual/disarm request sent"
        self._set_operation(message, True)
        return {"accepted": True, "message": message}

    def waypoint_action(self, action: str) -> dict:
        if action not in self._waypoint_clients:
            raise ValueError(f"Unknown waypoint action: {action}")
        client = self._waypoint_clients[action]
        if not client.service_is_ready():
            message = f"Waypoint {action} service is unavailable"
            self._set_operation(message, False)
            return {"accepted": False, "message": message}
        future = client.call_async(Trigger.Request())

        def complete(completed):
            try:
                result = completed.result()
                self._set_operation(result.message, result.success)
            except Exception as error:
                self._set_operation(f"Waypoint {action} failed: {error}", False)

        future.add_done_callback(complete)
        message = f"Waypoint {action} request sent"
        self._set_operation(message, True)
        return {"accepted": True, "message": message}

    @staticmethod
    def _pose_values(payload: dict) -> tuple[float, float, float]:
        values = (
            float(payload.get("x_m", 0.0)),
            float(payload.get("y_m", 0.0)),
            float(payload.get("yaw_deg", 0.0)),
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Pose values must be finite")
        return values

    def set_initial_pose(self, payload: dict) -> dict:
        x_m, y_m, yaw_deg = self._pose_values(payload)
        xy_std = max(0.05, min(2.0, float(payload.get("xy_std_m", 0.35))))
        yaw_std_deg = max(2.0, min(45.0, float(payload.get("yaw_std_deg", 12.0))))
        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.pose.position.x = x_m
        message.pose.pose.position.y = y_m
        z, w = _quaternion_from_yaw(math.radians(yaw_deg))
        message.pose.pose.orientation.z = z
        message.pose.pose.orientation.w = w
        message.pose.covariance[0] = xy_std * xy_std
        message.pose.covariance[7] = xy_std * xy_std
        message.pose.covariance[35] = math.radians(yaw_std_deg) ** 2
        self._initial_pose_publisher.publish(message)
        text = f"Initial pose published: ({x_m:.2f}, {y_m:.2f}, {yaw_deg:.1f} deg)"
        self._set_operation(text, True)
        return {"accepted": True, "message": text}

    def navigate_to_pose(self, payload: dict) -> dict:
        if not self._navigate_client.server_is_ready():
            message = "NavigateToPose action server is unavailable"
            self._set_operation(message, False)
            return {"accepted": False, "message": message}
        x_m, y_m, yaw_deg = self._pose_values(payload)
        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = x_m
        goal.pose.pose.position.y = y_m
        z, w = _quaternion_from_yaw(math.radians(yaw_deg))
        goal.pose.pose.orientation.z = z
        goal.pose.pose.orientation.w = w
        with self._lock:
            self._state["navigation"]["goal_state"] = "SENDING"
            self._state["navigation"]["goal"] = {
                "x_m": x_m,
                "y_m": y_m,
                "yaw_deg": yaw_deg,
            }
        future = self._navigate_client.send_goal_async(goal)

        def goal_response(completed):
            try:
                handle = completed.result()
                if not handle.accepted:
                    with self._lock:
                        self._state["navigation"]["goal_state"] = "REJECTED"
                    self._set_operation("Navigation goal was rejected", False)
                    return
                self._goal_handle = handle
                with self._lock:
                    self._state["navigation"]["goal_state"] = "ACTIVE"
                result_future = handle.get_result_async()

                def result_done(result_completed):
                    try:
                        status = int(result_completed.result().status)
                        labels = {
                            4: "SUCCEEDED",
                            5: "CANCELED",
                            6: "ABORTED",
                        }
                        label = labels.get(status, f"STATUS_{status}")
                        with self._lock:
                            self._state["navigation"]["goal_state"] = label
                        self._set_operation(
                            f"Navigation goal {label.lower()}", status == 4
                        )
                    except Exception as error:
                        self._set_operation(f"Navigation result failed: {error}", False)

                result_future.add_done_callback(result_done)
            except Exception as error:
                self._set_operation(f"Goal request failed: {error}", False)

        future.add_done_callback(goal_response)
        message = f"Goal requested: ({x_m:.2f}, {y_m:.2f}, {yaw_deg:.1f} deg)"
        self._set_operation(message, True)
        return {"accepted": True, "message": message}

    def cancel_goal(self) -> dict:
        handle = self._goal_handle
        if handle is None:
            return {"accepted": False, "message": "No active web goal to cancel"}
        handle.cancel_goal_async()
        with self._lock:
            self._state["navigation"]["goal_state"] = "CANCELING"
        self.stop_motion()
        self._set_operation("Navigation goal cancel requested", True)
        return {"accepted": True, "message": "Navigation goal cancel requested"}

    def runtime_action(self, action: str, payload: dict | None = None) -> dict:
        if action == "start":
            result = self._operations.start("field_navigation", payload or {})
        elif action == "stop":
            self.stop_motion()
            result = self._operations.stop("field_navigation")
        else:
            raise ValueError(f"Unknown runtime action: {action}")
        self._set_operation(result["message"], result["accepted"])
        return result

    def operation_action(
        self, operation_id: str, action: str, payload: dict | None = None
    ) -> dict:
        if action == "start":
            result = self._operations.start(operation_id, payload or {})
        elif action == "stop":
            if operation_id in (
                "field_navigation",
                "autoware_field",
                "autoware_planning_sim",
            ):
                self.stop_motion()
            result = self._operations.stop(operation_id)
        else:
            raise ValueError(f"Unknown operation action: {action}")
        self._set_operation(result["message"], result["accepted"])
        return result

    def operation_log(self, operation_id: str) -> dict:
        return self._operations.log_tail(operation_id)

    def stop_motion(self) -> None:
        with self._lock:
            self._command_deadline = 0.0
            self._state["command"].update(
                {
                    "active": False,
                    "mode": "2wis",
                    "speed_mps": 0.0,
                    "steer_deg": 0.0,
                    "pivot_rate_radps": 0.0,
                }
            )
        if rclpy.ok():
            for _ in range(3):
                self._publish_twist(0.0, 0.0)

    def program_stop(self) -> dict:
        self.stop_motion()
        arm_result = self.set_arm(False)
        self._set_operation("Program stop: zero command and Manual requested", True)
        return {"accepted": True, "arm_request": arm_result}

    def snapshot(self) -> dict:
        with self._lock:
            state = copy.deepcopy(self._state)
            times = {name: list(values) for name, values in self._topic_times.items()}
        now = time.monotonic()
        health = {}
        for name, samples in times.items():
            age = now - samples[-1] if samples else None
            rate = 0.0
            if len(samples) > 1 and samples[-1] > samples[0]:
                rate = (len(samples) - 1) / (samples[-1] - samples[0])
            health[name] = {
                "online": age is not None
                and age
                < (
                    1.0
                    if name == "localization"
                    else 0.6
                ),
                "age_sec": round(age, 2) if age is not None else None,
                "rate_hz": round(rate, 1),
            }
        health["lidar"] = health["lidar_filtered"]
        state["health"] = health
        state["services"] = {
            "arm": self._arm_client.service_is_ready(),
            "navigate_to_pose": self._navigate_client.server_is_ready(),
            **{
                f"waypoint_{name}": client.service_is_ready()
                for name, client in self._waypoint_clients.items()
            },
        }
        platform = state["platform"]
        bridge = state["diagnostics"].get("bridge_values", {})
        feedback_fresh = bool(
            platform["connected"] and health["platform"]["online"]
        )
        wheels_stopped = max(abs(value) for value in platform["wheel_speed_mps"]) < 0.02
        checks = [
            {
                "key": "serial",
                "label": "PCU 시리얼 피드백",
                "ok": feedback_fresh,
                "detail": "/dev/romo_b_pcu 실시간 수신",
            },
            {
                "key": "tx",
                "label": "명령 전송",
                "ok": bool(bridge)
                and bridge.get("receive_only", "false") == "false",
                "detail": "receive_only가 false여야 합니다",
            },
            {
                "key": "estop",
                "label": "물리 비상정지",
                "ok": feedback_fresh and not platform["estop"],
                "detail": "PCU 비상정지 피드백이 해제되어야 합니다",
            },
            {
                "key": "initial_mode",
                "label": "초기 조향 모드",
                "ok": feedback_fresh
                and (platform["steer_mode"] == 0 or platform["state"] == 2),
                "detail": "Arm 전환은 2WIS에서 시작합니다",
            },
            {
                "key": "stopped",
                "label": "바퀴 정지 상태",
                "ok": feedback_fresh
                and (wheels_stopped or platform["state"] == 2),
                "detail": "모든 바퀴 피드백이 0.02 m/s 미만이어야 합니다",
            },
            {
                "key": "calibration",
                "label": "LiDAR 장착 위치 승인",
                "ok": bridge.get("sensor_calibrated", "false") == "true",
                "detail": "자율주행 Arm 필수 조건입니다",
            },
            {
                "key": "manual_zero",
                "label": "수동 0 명령 확인",
                "ok": bridge.get("manual_zero_sent", "false") == "true"
                or platform["state"] == 2,
                "detail": "Auto 전환 전에 필요합니다",
            },
            {
                "key": "auto_confirmed",
                "label": "Auto 제어 가능",
                "ok": platform["auto_mode"]
                and bridge.get("auto_confirmed", "false") == "true",
                "detail": "PCU Auto 피드백과 브리지 확인이 모두 필요합니다",
            },
        ]
        state["readiness"] = {
            "bridge_armed": platform["state"] == 2,
            "pcu_auto_confirmed": feedback_fresh and platform["auto_mode"],
            "ready_to_arm": all(item["ok"] for item in checks[:-1]),
            "control_ready": feedback_fresh
            and platform["state"] == 2
            and platform["auto_mode"]
            and not platform["estop"],
            "checks": checks,
        }
        operations = self._operations.snapshot()
        state["operations"] = operations
        field = next(
            task
            for task in operations["tasks"]
            if task["id"] == "field_navigation"
        )
        state["runtime"] = {
            "field_running": field["running"],
            "field_pids": field["pids"],
            "owned_by_ui": field["owned_by_ui"],
            "log_path": field["log_path"],
        }
        return state


def create_app(node: OperatorNode, web_root: pathlib.Path) -> Flask:
    app = Flask(__name__, static_folder=str(web_root), static_url_path="")
    app.config["SEND_FILE_MAX_AGE_DEFAULT"] = 0

    @app.after_request
    def disable_cache(response):
        response.headers["Cache-Control"] = "no-store"
        return response

    @app.get("/")
    def index():
        return send_from_directory(web_root, "index.html")

    @app.get("/api/state")
    def state():
        return jsonify(node.snapshot())

    @app.get("/api/events")
    def events():
        @stream_with_context
        def generate():
            while rclpy.ok():
                yield f"data: {json.dumps(node.snapshot(), separators=(',', ':'))}\n\n"
                time.sleep(0.2)

        return Response(generate(), mimetype="text/event-stream")

    @app.post("/api/drive")
    def drive():
        try:
            return jsonify(node.set_drive_command(request.get_json(silent=True) or {}))
        except (TypeError, ValueError) as error:
            return jsonify({"accepted": False, "message": str(error)}), 400

    @app.post("/api/arm")
    def arm():
        payload = request.get_json(silent=True) or {}
        result = node.set_arm(bool(payload.get("armed", False)))
        return jsonify(result), 202 if result["accepted"] else 503

    @app.post("/api/waypoints/<action>")
    def waypoint_action(action: str):
        try:
            result = node.waypoint_action(action)
            return jsonify(result), 202 if result["accepted"] else 503
        except ValueError as error:
            return jsonify({"accepted": False, "message": str(error)}), 404

    @app.post("/api/navigation/initial-pose")
    def initial_pose():
        try:
            return jsonify(
                node.set_initial_pose(request.get_json(silent=True) or {})
            ), 202
        except (TypeError, ValueError) as error:
            return jsonify({"accepted": False, "message": str(error)}), 400

    @app.post("/api/navigation/goal")
    def navigation_goal():
        try:
            result = node.navigate_to_pose(request.get_json(silent=True) or {})
            return jsonify(result), 202 if result["accepted"] else 503
        except (TypeError, ValueError) as error:
            return jsonify({"accepted": False, "message": str(error)}), 400

    @app.post("/api/navigation/cancel")
    def navigation_cancel():
        result = node.cancel_goal()
        return jsonify(result), 202 if result["accepted"] else 409

    @app.post("/api/runtime/field/<action>")
    def runtime_field(action: str):
        try:
            result = node.runtime_action(
                action, request.get_json(silent=True) or {}
            )
            return jsonify(result), 202 if result["accepted"] else 409
        except ValueError as error:
            return jsonify({"accepted": False, "message": str(error)}), 400

    @app.post("/api/operations/<operation_id>/<action>")
    def operation_action(operation_id: str, action: str):
        try:
            result = node.operation_action(
                operation_id, action, request.get_json(silent=True) or {}
            )
            return jsonify(result), 202 if result["accepted"] else 409
        except ValueError as error:
            return jsonify({"accepted": False, "message": str(error)}), 400

    @app.get("/api/operations/<operation_id>/log")
    def operation_log(operation_id: str):
        try:
            return jsonify(node.operation_log(operation_id))
        except ValueError as error:
            return jsonify({"accepted": False, "message": str(error)}), 404

    @app.post("/api/program-stop")
    def program_stop():
        return jsonify(node.program_stop()), 202

    @app.get("/<path:filename>")
    def static_files(filename: str):
        candidate = web_root / filename
        if candidate.is_file():
            return send_from_directory(web_root, filename)
        return send_from_directory(web_root, "index.html")

    return app


def main(args=None):
    rclpy.init(args=args)
    repo_root = pathlib.Path(os.environ.get("ROMO_B_ROOT", pathlib.Path.cwd())).resolve()
    node = OperatorNode(repo_root)
    node.declare_parameter("host", "127.0.0.1")
    node.declare_parameter("port", 8765)
    node.declare_parameter("open_browser", True)
    host = str(node.get_parameter("host").value)
    port = int(node.get_parameter("port").value)
    open_browser = bool(node.get_parameter("open_browser").value)
    web_root = pathlib.Path(get_package_share_directory("romo_b_operator_ui")) / "web_dist"
    if not (web_root / "index.html").is_file():
        node.get_logger().fatal(f"Operator UI assets are missing from {web_root}")
        raise SystemExit(2)

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    def spin_ros():
        while rclpy.ok():
            try:
                executor.spin_once(timeout_sec=0.2)
            except Exception as error:
                node.get_logger().error(f"ROS callback failed but UI remains active: {error}")

    spin_thread = threading.Thread(target=spin_ros, daemon=True)
    spin_thread.start()
    app = create_app(node, web_root)
    logging.getLogger("werkzeug").setLevel(logging.ERROR)
    url = f"http://{host}:{port}/"
    node.get_logger().info(f"ROMO-B operator console: {url}")
    if open_browser:
        threading.Timer(0.8, lambda: webbrowser.open(url)).start()
    try:
        app.run(host=host, port=port, threaded=True, use_reloader=False)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motion()
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
