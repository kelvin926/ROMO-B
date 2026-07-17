#!/usr/bin/env python3
"""Submit the existing ROMO-B waypoint YAML through Autoware's routing API."""

import argparse
import json
import math
import pathlib
import time

import rclpy
from autoware_adapi_v1_msgs.msg import OperationModeState, RouteState
from autoware_adapi_v1_msgs.srv import SetRoutePoints
from autoware_internal_planning_msgs.msg import VelocityLimit
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from romo_b_msgs.msg import PlatformStatus
from romo_b_waypoints.model import infer_yaws, load_route


def pose_from_waypoint(waypoint) -> Pose:
    pose = Pose()
    pose.position.x = waypoint.x
    pose.position.y = waypoint.y
    pose.orientation.z = math.sin(waypoint.yaw * 0.5)
    pose.orientation.w = math.cos(waypoint.yaw * 0.5)
    return pose


class RouteSender(Node):
    def __init__(self) -> None:
        super().__init__("romo_b_autoware_waypoint_sender")
        transient = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.route_state = None
        self.operation_mode = None
        self.platform = None
        self.create_subscription(
            RouteState,
            "/api/routing/state",
            lambda message: setattr(self, "route_state", message.state),
            transient,
        )
        self.create_subscription(
            OperationModeState,
            "/api/operation_mode/state",
            lambda message: setattr(self, "operation_mode", message),
            transient,
        )
        self.create_subscription(
            PlatformStatus,
            "/romo_b/platform_status",
            lambda message: setattr(self, "platform", message),
            10,
        )
        self.set_client = self.create_client(
            SetRoutePoints, "/api/routing/set_route_points"
        )
        self.change_client = self.create_client(
            SetRoutePoints, "/api/routing/change_route_points"
        )
        self.velocity_publisher = self.create_publisher(
            VelocityLimit,
            "/planning/scenario_planning/max_velocity_candidates",
            transient,
        )

    def spin_until(self, predicate, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.10)
            if predicate():
                return True
        return False

    def submit(
        self, poses: list[Pose], speed_mps: float, replace: bool, timeout: float
    ):
        if not self.spin_until(
            lambda: self.route_state in (RouteState.UNSET, RouteState.SET), timeout
        ):
            raise RuntimeError("Autoware routing state is unavailable or busy")
        # Route changes can immediately affect an already engaged controller.
        # Require the operator to set/replace YAML routes before arming.
        self.spin_until(lambda: False, min(0.30, timeout))
        if (
            self.operation_mode
            and self.operation_mode.mode == OperationModeState.AUTONOMOUS
            and self.operation_mode.is_autoware_control_enabled
        ):
            # Match Autoware's own mission-planner live-reroute definition:
            # control enabled alone can also be reported while STOP is active.
            raise RuntimeError(
                "Leave AUTONOMOUS mode or disable Autoware Control before changing the route"
            )
        if self.platform and self.platform.state == PlatformStatus.STATE_ARMED_AUTO:
            raise RuntimeError("Disarm ROMO-B before changing the route")

        if self.route_state == RouteState.SET:
            if not replace:
                raise RuntimeError("A route already exists; use --replace while stopped")
            client = self.change_client
            endpoint = "/api/routing/change_route_points"
        else:
            client = self.set_client
            endpoint = "/api/routing/set_route_points"
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"Autoware route service unavailable: {endpoint}")
        if not self.spin_until(
            lambda: self.velocity_publisher.get_subscription_count() > 0,
            min(timeout, 5.0),
        ):
            raise RuntimeError(
                "Autoware maximum-velocity selector is unavailable; route was not changed"
            )

        request = SetRoutePoints.Request()
        request.header.stamp = self.get_clock().now().to_msg()
        request.header.frame_id = "map"
        request.option.allow_goal_modification = False
        request.goal = poses[-1]
        request.waypoints = poses[:-1]
        future = client.call_async(request)
        if not self.spin_until(future.done, timeout):
            raise RuntimeError(f"Autoware route request timed out: {endpoint}")
        response = future.result()
        if response is None or not response.status.success:
            detail = response.status.message if response else "empty response"
            raise RuntimeError(f"Autoware rejected route: {detail}")

        # This is the same transient-local candidate topic used by Autoware's
        # RViz state panel. The downstream selector chooses the most
        # restrictive active limit; the physical pipeline still caps at 0.2.
        limit = VelocityLimit()
        limit.stamp = self.get_clock().now().to_msg()
        limit.max_velocity = speed_mps
        limit.sender = "romo_b_waypoints"
        # Waited discovery plus a one-second burst makes a short-lived CLI
        # publisher reliable even when the selector and RViz are still
        # completing DDS endpoint matching.
        for _ in range(10):
            self.velocity_publisher.publish(limit)
            rclpy.spin_once(self, timeout_sec=0.10)
        return endpoint


def main() -> int:
    default_file = pathlib.Path(__file__).resolve().parents[1] / "config/local/waypoints.yaml"
    parser = argparse.ArgumentParser()
    parser.add_argument("waypoint_file", nargs="?", type=pathlib.Path, default=default_file)
    parser.add_argument(
        "--replace",
        action="store_true",
        help="replace an existing route; Autoware still rejects unsafe live reroutes",
    )
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    try:
        route = load_route(args.waypoint_file.expanduser())
        if route.frame_id != "map":
            raise ValueError("Autoware waypoint frame_id must be map")
        waypoints = infer_yaws(route.waypoints)
        if not waypoints:
            raise ValueError("waypoint file is empty")
        poses = [pose_from_waypoint(waypoint) for waypoint in waypoints]
    except Exception as error:
        print(json.dumps({"result": "FAIL", "error": str(error)}, indent=2))
        return 1

    rclpy.init()
    node = RouteSender()
    try:
        endpoint = node.submit(
            poses, route.default_speed_mps, args.replace, args.timeout
        )
        report = {
            "result": "PASS",
            "service": endpoint,
            "waypoint_file": str(args.waypoint_file.expanduser()),
            "intermediate_waypoints": max(0, len(poses) - 1),
            "goal_xy": [poses[-1].position.x, poses[-1].position.y],
            "maximum_speed_mps": route.default_speed_mps,
            "note": "Route set only; this command never arms or engages the robot",
        }
        print(json.dumps(report, indent=2))
        return 0
    except Exception as error:
        print(json.dumps({"result": "FAIL", "error": str(error)}, indent=2))
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
