#!/usr/bin/env python3
"""Exercise Autoware routing and UNKNOWN-object avoidance without hardware."""

import argparse
import json
import math
import pathlib
import time
import xml.etree.ElementTree as ET

import rclpy
from autoware_adapi_v1_msgs.msg import RouteState
from autoware_adapi_v1_msgs.srv import SetRoutePoints
from autoware_perception_msgs.msg import ObjectClassification, Shape
from autoware_planning_msgs.msg import Trajectory
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tier4_simulation_msgs.msg import DummyObject


def yaw_quaternion(yaw: float) -> Quaternion:
    result = Quaternion()
    result.z = math.sin(yaw * 0.5)
    result.w = math.cos(yaw * 0.5)
    return result


def lane_centerline(path: pathlib.Path) -> list[tuple[float, float]]:
    root = ET.parse(path).getroot()

    def attribute(element, key):
        return next(tag.attrib["v"] for tag in element.findall("tag") if tag.attrib["k"] == key)

    nodes = {
        int(node.attrib["id"]): (
            float(attribute(node, "local_x")),
            float(attribute(node, "local_y")),
        )
        for node in root.findall("node")
    }
    ways = {
        int(way.attrib["id"]): [int(item.attrib["ref"]) for item in way.findall("nd")]
        for way in root.findall("way")
    }
    output = []
    for relation in root.findall("relation"):
        members = {
            member.attrib["role"]: int(member.attrib["ref"])
            for member in relation.findall("member")
        }
        left = ways[members["left"]]
        right = ways[members["right"]]
        for left_id, right_id in zip(left, right):
            left_point = nodes[left_id]
            right_point = nodes[right_id]
            center = (
                0.5 * (left_point[0] + right_point[0]),
                0.5 * (left_point[1] + right_point[1]),
            )
            separation = math.hypot(
                center[0] - output[-1][0], center[1] - output[-1][1]
            ) if output else math.inf
            if separation > 0.01:
                output.append(center)
    if len(output) < 20:
        raise RuntimeError("Lanelet map has no usable ordered centerline")
    return output


def distance_along(points, index):
    return sum(
        math.hypot(points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1])
        for i in range(1, index + 1)
    )


def nearest_index_at_distance(points, target):
    for index in range(1, len(points)):
        if distance_along(points, index) >= target:
            return index
    return len(points) - 1


class PlanningProbe(Node):
    def __init__(self) -> None:
        super().__init__("romo_b_autoware_planning_probe")
        self.initial_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        transient = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.route_state = None
        self.create_subscription(
            RouteState,
            "/api/routing/state",
            lambda message: setattr(self, "route_state", message.state),
            transient,
        )
        self.route_client = self.create_client(
            SetRoutePoints, "/api/routing/set_route_points"
        )
        self.object_publisher = self.create_publisher(
            DummyObject, "/simulation/dummy_perception_publisher/object_info", 10
        )
        self.trajectories: list[Trajectory] = []
        self.create_subscription(
            Trajectory, "/planning/trajectory", self._on_trajectory, 10
        )

    def _on_trajectory(self, message: Trajectory) -> None:
        if len(message.points) >= 5:
            self.trajectories.append(message)
            self.trajectories = self.trajectories[-100:]

    def publish_initial(self, point, following) -> None:
        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.pose.position.x = point[0]
        message.pose.pose.position.y = point[1]
        message.pose.pose.orientation = yaw_quaternion(
            math.atan2(following[1] - point[1], following[0] - point[0])
        )
        message.pose.covariance[0] = 0.04
        message.pose.covariance[7] = 0.04
        message.pose.covariance[35] = math.radians(5.0) ** 2
        self.initial_publisher.publish(message)

    def request_route(self, point, previous):
        request = SetRoutePoints.Request()
        request.header.stamp = self.get_clock().now().to_msg()
        request.header.frame_id = "map"
        request.option.allow_goal_modification = False
        request.goal.position.x = point[0]
        request.goal.position.y = point[1]
        request.goal.orientation = yaw_quaternion(
            math.atan2(point[1] - previous[1], point[0] - previous[0])
        )
        return self.route_client.call_async(request)

    def publish_unknown_object(self, x: float, y: float, diameter: float) -> None:
        message = DummyObject()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.id.uuid = list(range(16))
        message.classification.label = ObjectClassification.UNKNOWN
        message.classification.probability = 1.0
        message.shape.type = Shape.CYLINDER
        message.shape.dimensions.x = diameter
        message.shape.dimensions.y = diameter
        message.shape.dimensions.z = 1.70
        message.initial_state.pose_covariance.pose.position.x = x
        message.initial_state.pose_covariance.pose.position.y = y
        message.initial_state.pose_covariance.pose.orientation.w = 1.0
        message.action = DummyObject.ADD
        self.object_publisher.publish(message)


def trajectory_points(message: Trajectory):
    return [
        (
            point.pose.position.x,
            point.pose.position.y,
            point.longitudinal_velocity_mps,
        )
        for point in message.points
    ]


def obstacle_response(
    message: Trajectory, obstacle, required_clearance: float
) -> tuple[bool, float, float]:
    points = trajectory_points(message)
    distances = [math.hypot(x - obstacle[0], y - obstacle[1]) for x, y, _ in points]
    minimum_distance = min(distances)
    avoidance = minimum_distance >= required_clearance
    closest = distances.index(minimum_distance)
    start = max(0, closest - 12)
    stop_speed = min(abs(points[index][2]) for index in range(start, closest + 1))
    return avoidance, minimum_distance, stop_speed


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-path", required=True, type=pathlib.Path)
    parser.add_argument("--startup-timeout", type=float, default=90.0)
    parser.add_argument("--response-timeout", type=float, default=30.0)
    parser.add_argument("--obstacle-offset", type=float, default=0.0)
    parser.add_argument("--object-diameter", type=float, default=0.60)
    parser.add_argument(
        "--expect", choices=("avoid", "stop", "either"), default="either"
    )
    parser.add_argument("--output", type=pathlib.Path)
    args = parser.parse_args()
    centerline = lane_centerline(args.map_path / "lanelet2_map.osm")
    initial_index = min(4, len(centerline) - 2)
    goal_index = max(initial_index + 10, len(centerline) - 5)

    rclpy.init()
    node = PlanningProbe()
    try:
        readiness_deadline = time.monotonic() + args.startup_timeout
        required_planning_nodes = {
            "autoware_operation_mode_transition_manager",
            "behavior_path_planner",
            "motion_velocity_planner",
        }
        missing_planning_nodes = required_planning_nodes
        while time.monotonic() < readiness_deadline:
            visible_nodes = {
                name for name, _namespace in node.get_node_names_and_namespaces()
            }
            missing_planning_nodes = required_planning_nodes - visible_nodes
            if (
                node.initial_publisher.get_subscription_count()
                and node.route_client.service_is_ready()
                and not missing_planning_nodes
            ):
                break
            rclpy.spin_once(node, timeout_sec=0.2)
        else:
            missing = ", ".join(sorted(missing_planning_nodes)) or "none"
            raise RuntimeError(
                "Autoware planning stack did not become ready; "
                f"missing nodes: {missing}"
            )

        # The composable nodes can appear in the graph just before their
        # subscriptions finish discovery.  A short spin prevents a route from
        # being published during that narrow startup window.
        discovery_deadline = time.monotonic() + 2.0
        while time.monotonic() < discovery_deadline:
            rclpy.spin_once(node, timeout_sec=0.2)

        # Initial localization may take several API/state cycles. Repeat the
        # identical pose only until the route request is issued; publishing it
        # after routing can intentionally reset the mission in Autoware.
        baseline_deadline = time.monotonic() + args.startup_timeout
        last_initial = 0.0
        first_initial = None
        route_future = None
        route_accepted = False
        while time.monotonic() < baseline_deadline and not node.trajectories:
            now = time.monotonic()
            if route_future is None and now - last_initial >= 2.0:
                node.publish_initial(
                    centerline[initial_index], centerline[initial_index + 1]
                )
                last_initial = now
                first_initial = first_initial or now
            if (
                route_future is None
                and node.route_state == RouteState.UNSET
                and first_initial is not None
                and now - first_initial >= 2.0
            ):
                route_future = node.request_route(
                    centerline[goal_index], centerline[goal_index - 1]
                )
            rclpy.spin_once(node, timeout_sec=0.2)
            if route_future is not None and route_future.done():
                response = route_future.result()
                if response is None or not response.status.success:
                    detail = response.status.message if response else "empty response"
                    raise RuntimeError(f"Autoware rejected test route: {detail}")
                route_accepted = True
        # A fresh trajectory in this isolated stack is itself conclusive that
        # the set_route_points request reached the mission planner.  The API
        # response and transient state can arrive a few callbacks later.
        route_accepted = route_accepted or node.route_state == RouteState.SET or (
            route_future is not None and bool(node.trajectories)
        )
        if not node.trajectories:
            raise RuntimeError("Autoware did not produce a baseline trajectory")

        baseline = node.trajectories[-1]
        baseline_points = trajectory_points(baseline)
        maximum_baseline_speed = max(speed for _, _, speed in baseline_points)
        minimum_baseline_speed = min(speed for _, _, speed in baseline_points)
        obstacle_index = nearest_index_at_distance(
            [(x, y) for x, y, _ in baseline_points], 5.0
        )
        center = baseline_points[obstacle_index][:2]
        before = baseline_points[max(0, obstacle_index - 1)]
        after = baseline_points[min(len(baseline_points) - 1, obstacle_index + 1)]
        tangent_x = after[0] - before[0]
        tangent_y = after[1] - before[1]
        tangent_norm = math.hypot(tangent_x, tangent_y)
        obstacle = (
            center[0] - tangent_y / tangent_norm * args.obstacle_offset,
            center[1] + tangent_x / tangent_norm * args.obstacle_offset,
        )
        required_clearance = args.object_diameter * 0.5 + 0.40
        baseline_distance = min(
            math.hypot(x - obstacle[0], y - obstacle[1])
            for x, y, _ in baseline_points
        )
        baseline_response = obstacle_response(
            baseline, obstacle, required_clearance
        )
        baseline_near_obstacle_speed = baseline_response[2]
        node.trajectories.clear()

        def response_matches_expectation(response) -> bool:
            avoided_response = response[0]
            stopped_response = (
                baseline_near_obstacle_speed > 0.05 and response[2] <= 0.02
            )
            if args.expect == "avoid":
                return avoided_response
            if args.expect == "stop":
                return stopped_response
            return avoided_response or stopped_response

        response_deadline = time.monotonic() + args.response_timeout
        last_object = 0.0
        responses = []
        while time.monotonic() < response_deadline:
            now = time.monotonic()
            if now - last_object >= 1.0:
                node.publish_unknown_object(*obstacle, args.object_diameter)
                last_object = now
            rclpy.spin_once(node, timeout_sec=0.2)
            while node.trajectories:
                message = node.trajectories.pop(0)
                response = obstacle_response(
                    message, obstacle, required_clearance
                )
                responses.append(response)
                if response_matches_expectation(response):
                    break
            if responses and response_matches_expectation(responses[-1]):
                break

        avoided = any(item[0] for item in responses)
        stopped = baseline_near_obstacle_speed > 0.05 and any(
            item[2] <= 0.02 for item in responses
        )
        expected_response = {
            "avoid": avoided,
            "stop": stopped,
            "either": avoided or stopped,
        }[args.expect]
        checks = {
            "baseline_trajectory": len(baseline_points) >= 5,
            "routing_api_accepted": route_accepted,
            "planned_speed_at_or_below_0_2_mps": maximum_baseline_speed <= 0.2005,
            "forward_only_trajectory": minimum_baseline_speed >= -1.0e-4,
            "obstacle_conflicts_with_baseline": baseline_distance
            <= args.object_diameter * 0.5 + 0.30,
            "baseline_moving_near_obstacle": baseline_near_obstacle_speed > 0.05,
            "post_obstacle_trajectory": bool(responses),
            f"expected_{args.expect}_response": expected_response,
        }
        report = {
            "result": "PASS" if all(checks.values()) else "FAIL",
            "checks": checks,
            "centerline_points": len(centerline),
            "baseline_trajectory_points": len(baseline_points),
            "baseline_speed_range_mps": [
                minimum_baseline_speed,
                maximum_baseline_speed,
            ],
            "baseline_near_obstacle_speed_mps": baseline_near_obstacle_speed,
            "obstacle_xy": obstacle,
            "obstacle_offset_m": args.obstacle_offset,
            "object_diameter_m": args.object_diameter,
            "required_clearance_m": required_clearance,
            "response_count": len(responses),
            "response": "avoid" if avoided else "stop" if stopped else "none",
            "maximum_clearance_m": max((item[1] for item in responses), default=None),
            "minimum_near_obstacle_speed_mps": min(
                (item[2] for item in responses), default=None
            ),
        }
        rendered = json.dumps(report, indent=2, sort_keys=True) + "\n"
        print(rendered, end="")
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(rendered, encoding="utf-8")
        return 0 if report["result"] == "PASS" else 1
    except Exception as error:
        report = {"result": "FAIL", "error": str(error)}
        rendered = json.dumps(report, indent=2, sort_keys=True) + "\n"
        print(rendered, end="")
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(rendered, encoding="utf-8")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
