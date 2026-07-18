import math

import rclpy
from autoware_perception_msgs.msg import PredictedObjects
from geometry_msgs.msg import Point
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class ObjectMarkers(Node):
    """Render clustered objects and their constant-velocity predictions in RViz."""

    def __init__(self) -> None:
        super().__init__("romo_b_object_markers")
        self.publisher = self.create_publisher(
            MarkerArray, "/romo_b/perception/object_markers", 10
        )
        self.create_subscription(
            PredictedObjects,
            "/perception/object_recognition/objects",
            self._on_objects,
            10,
        )

    @staticmethod
    def _identifier(raw_uuid) -> int:
        value = 0
        for byte in raw_uuid[:4]:
            value = (value << 8) | int(byte)
        return value & 0x1FFFFFFF

    @staticmethod
    def _base_marker(message, namespace: str, marker_id: int, marker_type: int):
        marker = Marker()
        marker.header = message.header
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.lifetime.sec = 1
        marker.pose.orientation.w = 1.0
        return marker

    def _on_objects(self, message: PredictedObjects) -> None:
        output = MarkerArray()
        clear = Marker()
        clear.header = message.header
        clear.action = Marker.DELETEALL
        output.markers.append(clear)

        for item in message.objects:
            marker_id = self._identifier(item.object_id.uuid)
            pose = item.kinematics.initial_pose_with_covariance.pose
            velocity = item.kinematics.initial_twist_with_covariance.twist.linear
            speed = math.hypot(velocity.x, velocity.y)

            body = self._base_marker(message, "objects", marker_id, Marker.CUBE)
            body.pose = pose
            body.pose.position.z += max(0.15, item.shape.dimensions.z * 0.5)
            body.scale.x = max(0.20, item.shape.dimensions.x)
            body.scale.y = max(0.20, item.shape.dimensions.y)
            body.scale.z = max(0.30, item.shape.dimensions.z)
            body.color.r = 1.0 if speed > 0.12 else 1.0
            body.color.g = 0.20 if speed > 0.12 else 0.65
            body.color.b = 0.05
            body.color.a = 0.38
            output.markers.append(body)

            outline = self._base_marker(
                message, "object_outline", marker_id, Marker.CUBE
            )
            outline.pose = body.pose
            outline.scale = body.scale
            outline.color.r = body.color.r
            outline.color.g = body.color.g
            outline.color.b = body.color.b
            outline.color.a = 0.85
            outline.type = Marker.CUBE
            output.markers.append(outline)

            if item.kinematics.predicted_paths:
                path = self._base_marker(
                    message, "predicted_paths", marker_id, Marker.LINE_STRIP
                )
                path.scale.x = 0.06
                path.color.r = 1.0
                path.color.g = 0.15
                path.color.b = 0.65
                path.color.a = 0.95
                for predicted_pose in item.kinematics.predicted_paths[0].path:
                    point = Point()
                    point.x = predicted_pose.position.x
                    point.y = predicted_pose.position.y
                    point.z = max(0.10, predicted_pose.position.z)
                    path.points.append(point)
                output.markers.append(path)

            label = self._base_marker(message, "object_labels", marker_id, Marker.TEXT_VIEW_FACING)
            label.pose.position.x = pose.position.x
            label.pose.position.y = pose.position.y
            label.pose.position.z = body.pose.position.z + body.scale.z * 0.65
            label.scale.z = 0.22
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = f"{'MOVING' if speed > 0.12 else 'STATIC'} {speed:.2f} m/s"
            output.markers.append(label)

        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectMarkers()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
