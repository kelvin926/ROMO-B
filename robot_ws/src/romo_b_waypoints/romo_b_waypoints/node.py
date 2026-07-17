import math
import pathlib

import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from .model import Route, Waypoint, infer_yaws, load_route, save_route


class WaypointManager(Node):
    def __init__(self):
        super().__init__("waypoint_manager", namespace="/romo_b/waypoints")
        self.declare_parameter("waypoint_file", "~/.ros/romo_b_waypoints.yaml")
        self.declare_parameter("default_speed_mps", 0.2)
        self.file_path = pathlib.Path(self.get_parameter("waypoint_file").value).expanduser()
        self.route = Route(
            frame_id="map",
            default_speed_mps=float(self.get_parameter("default_speed_mps").value),
            waypoints=tuple(),
        )

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_publisher = self.create_publisher(Path, "/romo_b/waypoints/path", latched)
        self.marker_publisher = self.create_publisher(
            MarkerArray, "/romo_b/waypoints/markers", latched
        )
        self.speed_limit_publisher = self.create_publisher(
            SpeedLimit, "/speed_limit", latched
        )
        self.create_subscription(PointStamped, "/clicked_point", self.on_clicked_point, 10)
        self.create_service(Trigger, "/romo_b/waypoints/clear", self.on_clear)
        self.create_service(Trigger, "/romo_b/waypoints/reload", self.on_reload)
        self.create_service(Trigger, "/romo_b/waypoints/save", self.on_save)
        self.create_service(Trigger, "/romo_b/waypoints/execute", self.on_execute)
        self.navigation_client = ActionClient(self, NavigateThroughPoses, "/navigate_through_poses")

        if self.file_path.is_file():
            try:
                self.route = load_route(self.file_path)
            except Exception as error:  # keep the editor usable with a bad file
                self.get_logger().error(f"Cannot load {self.file_path}: {error}")
        self.publish_route()

    def on_clicked_point(self, message):
        frame = message.header.frame_id or "map"
        if frame != self.route.frame_id:
            self.get_logger().error(
                f"Clicked point frame '{frame}' is not '{self.route.frame_id}'"
            )
            return
        points = list(self.route.waypoints)
        points.append(Waypoint(message.point.x, message.point.y, None))
        self.route = Route(
            self.route.frame_id, self.route.default_speed_mps, tuple(points), self.route.mode
        )
        self.publish_route()

    def on_clear(self, _request, response):
        self.route = Route(
            self.route.frame_id, self.route.default_speed_mps, tuple(), self.route.mode
        )
        self.publish_route()
        response.success = True
        response.message = "Waypoints cleared (file unchanged until save)"
        return response

    def on_reload(self, _request, response):
        try:
            self.route = load_route(self.file_path)
            self.publish_route()
            response.success = True
            response.message = f"Loaded {len(self.route.waypoints)} waypoints"
        except Exception as error:
            response.message = str(error)
        return response

    def on_save(self, _request, response):
        try:
            save_route(self.file_path, self.route)
            response.success = True
            response.message = f"Saved {self.file_path}"
        except Exception as error:
            response.message = str(error)
        return response

    def on_execute(self, _request, response):
        if not self.route.waypoints:
            response.message = "No waypoints"
            return response
        if not self.navigation_client.wait_for_server(timeout_sec=0.2):
            response.message = "NavigateThroughPoses action server is unavailable"
            return response

        speed_limit = SpeedLimit()
        speed_limit.header.stamp = self.get_clock().now().to_msg()
        speed_limit.percentage = False
        speed_limit.speed_limit = self.route.default_speed_mps
        self.speed_limit_publisher.publish(speed_limit)

        goal = NavigateThroughPoses.Goal()
        goal.poses = self.to_pose_messages()
        future = self.navigation_client.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)
        response.success = True
        response.message = (
            f"Submitted {len(goal.poses)} continuous poses; final pose is the only stop"
        )
        return response

    def on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("NavigateThroughPoses goal rejected")
            return
        self.get_logger().info("NavigateThroughPoses goal accepted")
        result = handle.get_result_async()
        result.add_done_callback(
            lambda completed: self.get_logger().info(
                f"NavigateThroughPoses finished with status {completed.result().status}"
            )
        )

    def to_pose_messages(self):
        stamp = self.get_clock().now().to_msg()
        messages = []
        for waypoint in infer_yaws(self.route.waypoints):
            pose = PoseStamped()
            pose.header.frame_id = self.route.frame_id
            pose.header.stamp = stamp
            pose.pose.position.x = waypoint.x
            pose.pose.position.y = waypoint.y
            pose.pose.orientation.z = math.sin(waypoint.yaw * 0.5)
            pose.pose.orientation.w = math.cos(waypoint.yaw * 0.5)
            messages.append(pose)
        return messages

    def publish_route(self):
        poses = self.to_pose_messages()
        path = Path()
        path.header.frame_id = self.route.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = poses
        self.path_publisher.publish(path)

        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        line = Marker()
        line.header = path.header
        line.ns = "route"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.035
        line.color.r = 0.1
        line.color.g = 0.8
        line.color.b = 1.0
        line.color.a = 1.0
        line.points = [Point(x=pose.pose.position.x, y=pose.pose.position.y) for pose in poses]
        markers.markers.append(line)
        for index, pose in enumerate(poses, start=1):
            label = Marker()
            label.header = path.header
            label.ns = "labels"
            label.id = index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = pose.pose.position
            label.pose.position.z = 0.2
            label.scale.z = 0.16
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = str(index)
            markers.markers.append(label)
        self.marker_publisher.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
