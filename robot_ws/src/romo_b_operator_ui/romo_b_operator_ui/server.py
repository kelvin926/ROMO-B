import copy
import json
import logging
import math
import pathlib
import threading
import time
import webbrowser
from collections import deque

import rclpy
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from flask import Flask, Response, jsonify, request, send_from_directory, stream_with_context
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from romo_b_msgs.msg import PlatformStatus
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import SetBool, Trigger

from .model import ackermann_twist, mode_name, pivot_twist, state_name, uint8_value


def _yaw_from_quaternion(quaternion) -> float:
    siny = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny, cosy)


class OperatorNode(Node):
    def __init__(self):
        super().__init__("romo_b_operator_ui")
        self._lock = threading.RLock()
        self._command_deadline = 0.0
        self._last_command_was_active = False
        self._topic_times = {
            "platform": deque(maxlen=40),
            "lidar": deque(maxlen=40),
            "localization": deque(maxlen=40),
            "odometry": deque(maxlen=40),
        }
        self._state = {
            "version": "0.1.0",
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
            "motion": {"wheel_odom_speed_mps": 0.0, "wheel_odom_yaw_rate_radps": 0.0},
            "localization": {
                "available": False,
                "x_m": 0.0,
                "y_m": 0.0,
                "yaw_deg": 0.0,
            },
            "navigation": {
                "plan_points": 0,
                "waypoint_count": 0,
                "last_action": "Waiting for operator",
                "last_action_success": True,
            },
            "diagnostics": {"level": 3, "summary": "No diagnostics", "items": []},
        }

        self._command_publisher = self.create_publisher(Twist, "/cmd_vel_teleop", 10)
        self.create_subscription(
            PlatformStatus, "/romo_b/platform_status", self._on_platform, 10
        )
        self.create_subscription(
            Odometry, "/wheel/odometry_raw", self._on_odometry, qos_profile_sensor_data
        )
        self.create_subscription(Twist, "/cmd_vel_safe", self._on_safe_command, 10)
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
            "/sensing/lidar/top/pointcloud_filtered",
            self._on_lidar,
            qos_profile_sensor_data,
        )
        self.create_subscription(NavPath, "/plan", self._on_plan, 10)
        self.create_subscription(
            NavPath, "/romo_b/waypoints/path", self._on_waypoints, 10
        )

        self._arm_client = self.create_client(SetBool, "/romo_b/arm")
        self._waypoint_clients = {
            action: self.create_client(Trigger, f"/romo_b/waypoints/{action}")
            for action in ("save", "reload", "clear", "execute", "cancel")
        }
        self.create_timer(0.05, self._publish_teleop)

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
        with self._lock:
            self._state["motion"].update(
                {
                    "wheel_odom_speed_mps": round(float(message.twist.twist.linear.x), 3),
                    "wheel_odom_yaw_rate_radps": round(float(message.twist.twist.angular.z), 3),
                }
            )
            self._mark_topic("odometry")

    def _on_safe_command(self, message: Twist) -> None:
        with self._lock:
            self._state["command"]["safe_linear_mps"] = round(float(message.linear.x), 3)
            self._state["command"]["safe_angular_radps"] = round(float(message.angular.z), 3)

    def _on_diagnostics(self, message: DiagnosticArray) -> None:
        items = sorted(
            (
                {
                    "name": status.name,
                    "level": uint8_value(status.level),
                    "message": status.message,
                }
                for status in message.status
            ),
            key=lambda item: item["level"],
            reverse=True,
        )[:10]
        if items:
            worst = items[0]
            summary = worst["message"] or worst["name"]
            level = worst["level"]
        else:
            summary = "Diagnostics are empty"
            level = 3
        with self._lock:
            self._state["diagnostics"] = {
                "level": level,
                "summary": summary,
                "items": items,
            }

    def _on_localization(self, message: PoseWithCovarianceStamped) -> None:
        pose = message.pose.pose
        with self._lock:
            self._state["localization"].update(
                {
                    "available": True,
                    "x_m": round(float(pose.position.x), 3),
                    "y_m": round(float(pose.position.y), 3),
                    "yaw_deg": round(math.degrees(_yaw_from_quaternion(pose.orientation)), 1),
                }
            )
            self._mark_topic("localization")

    def _on_lidar(self, _message: PointCloud2) -> None:
        with self._lock:
            self._mark_topic("lidar")

    def _on_plan(self, message: NavPath) -> None:
        with self._lock:
            self._state["navigation"]["plan_points"] = len(message.poses)

    def _on_waypoints(self, message: NavPath) -> None:
        with self._lock:
            self._state["navigation"]["waypoint_count"] = len(message.poses)

    def _publish_twist(self, linear: float, angular: float) -> None:
        message = Twist()
        message.linear.x = float(linear)
        message.angular.z = float(angular)
        self._command_publisher.publish(message)

    def _publish_teleop(self) -> None:
        with self._lock:
            active = self._command_deadline > time.monotonic()
            command = copy.deepcopy(self._state["command"])
            self._state["command"]["active"] = active
            should_send_zero = self._last_command_was_active and not active
            self._last_command_was_active = active
        if active:
            if command["mode"] == "pivot":
                linear, angular = pivot_twist(command["pivot_rate_radps"])
            else:
                linear, angular = ackermann_twist(
                    command["speed_mps"], command["steer_deg"]
                )
            self._publish_twist(linear, angular)
        elif should_send_zero:
            self._publish_twist(0.0, 0.0)

    def set_drive_command(self, payload: dict) -> dict:
        mode = str(payload.get("mode", "2wis")).lower()
        if mode not in ("2wis", "pivot"):
            raise ValueError("Only forward 2WIS and Pivot are supported")
        active = bool(payload.get("active", False))
        with self._lock:
            self._state["command"].update(
                {
                    "mode": mode,
                    "speed_mps": float(payload.get("speed_mps", 0.0)),
                    "steer_deg": float(payload.get("steer_deg", 0.0)),
                    "pivot_rate_radps": float(payload.get("pivot_rate_radps", 0.0)),
                    "active": active,
                }
            )
            self._command_deadline = time.monotonic() + 0.28 if active else 0.0
        if not active:
            self._publish_twist(0.0, 0.0)
        return {"accepted": True, "active": active, "mode": mode}

    def _set_operation(self, message: str, success: bool) -> None:
        with self._lock:
            self._state["navigation"]["last_action"] = message
            self._state["navigation"]["last_action_success"] = bool(success)

    def set_arm(self, armed: bool) -> dict:
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

    def stop_motion(self) -> None:
        with self._lock:
            self._command_deadline = 0.0
            self._state["command"].update(
                {
                    "active": False,
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
                "online": age is not None and age < (0.6 if name != "localization" else 1.0),
                "age_sec": round(age, 2) if age is not None else None,
                "rate_hz": round(rate, 1),
            }
        state["health"] = health
        state["services"] = {
            "arm": self._arm_client.service_is_ready(),
            **{
                f"waypoint_{name}": client.service_is_ready()
                for name, client in self._waypoint_clients.items()
            },
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
    node = OperatorNode()
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
