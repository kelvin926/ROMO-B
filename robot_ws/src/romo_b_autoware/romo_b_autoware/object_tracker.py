import copy
import math

import rclpy
from autoware_perception_msgs.msg import (
    DetectedObjects,
    PredictedObject,
    PredictedObjects,
    PredictedPath,
    TrackedObject,
    TrackedObjects,
)
from builtin_interfaces.msg import Duration as DurationMessage
from geometry_msgs.msg import Pose
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from unique_identifier_msgs.msg import UUID

from .control_math import quaternion_yaw
from .tracking import GreedyTracker


def _yaw_quaternion(yaw: float):
    from geometry_msgs.msg import Quaternion

    result = Quaternion()
    result.z = math.sin(yaw * 0.5)
    result.w = math.cos(yaw * 0.5)
    return result


class ObjectTracker(Node):
    """Turn clustered UNKNOWN objects into stable map-frame predictions.

    A Mid-360-only system cannot safely promise semantic person classification.
    Instead every cluster, including a person, remains an avoidance target as
    UNKNOWN. Motion is estimated so Autoware's dynamic avoidance can react.
    """

    def __init__(self) -> None:
        super().__init__("romo_b_autoware_object_tracker")
        self.output_frame = self.declare_parameter("output_frame", "map").value
        association_distance = self.declare_parameter(
            "association_distance", 0.80
        ).value
        expiry = self.declare_parameter("track_expiry", 0.75).value
        velocity_gain = self.declare_parameter("velocity_gain", 0.35).value
        self.stationary_speed = self.declare_parameter(
            "stationary_speed", 0.12
        ).value
        self.prediction_horizon = self.declare_parameter(
            "prediction_horizon", 3.0
        ).value
        self.prediction_step = self.declare_parameter("prediction_step", 0.5).value
        self.transform_timeout = self.declare_parameter(
            "transform_timeout", 0.08
        ).value
        self.tracker = GreedyTracker(
            association_distance=association_distance,
            expiry=expiry,
            velocity_gain=velocity_gain,
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tracked_publisher = self.create_publisher(
            TrackedObjects, "/perception/object_recognition/tracking/objects", 10
        )
        self.predicted_publisher = self.create_publisher(
            PredictedObjects, "/perception/object_recognition/objects", 10
        )
        self.create_subscription(
            DetectedObjects,
            "/perception/object_recognition/detection/objects",
            self._on_objects,
            10,
        )

    def _lookup_transform(self, message: DetectedObjects):
        return self.tf_buffer.lookup_transform(
            self.output_frame,
            message.header.frame_id,
            Time.from_msg(message.header.stamp),
            timeout=Duration(seconds=self.transform_timeout),
        )

    @staticmethod
    def _transform_pose(source: Pose, transform) -> Pose:
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        transform_yaw = quaternion_yaw(
            rotation.x, rotation.y, rotation.z, rotation.w
        )
        cosine = math.cos(transform_yaw)
        sine = math.sin(transform_yaw)
        output = Pose()
        output.position.x = (
            translation.x + cosine * source.position.x - sine * source.position.y
        )
        output.position.y = (
            translation.y + sine * source.position.x + cosine * source.position.y
        )
        output.position.z = translation.z + source.position.z
        source_yaw = quaternion_yaw(
            source.orientation.x,
            source.orientation.y,
            source.orientation.z,
            source.orientation.w,
        )
        if not math.isfinite(source_yaw):
            source_yaw = 0.0
        output.orientation = _yaw_quaternion(transform_yaw + source_yaw)
        return output

    def _prediction(self, pose: Pose, vx: float, vy: float) -> PredictedPath:
        path = PredictedPath()
        whole_seconds = int(self.prediction_step)
        path.time_step = DurationMessage(
            sec=whole_seconds,
            nanosec=int((self.prediction_step - whole_seconds) * 1.0e9),
        )
        path.confidence = 1.0
        steps = max(1, int(self.prediction_horizon / self.prediction_step))
        for index in range(steps + 1):
            future = copy.deepcopy(pose)
            elapsed = index * self.prediction_step
            future.position.x += vx * elapsed
            future.position.y += vy * elapsed
            if math.hypot(vx, vy) > self.stationary_speed:
                future.orientation = _yaw_quaternion(math.atan2(vy, vx))
            path.path.append(future)
        return path

    def _publish_empty(self, message: DetectedObjects) -> None:
        tracked = TrackedObjects()
        tracked.header = message.header
        tracked.header.frame_id = self.output_frame
        predicted = PredictedObjects()
        predicted.header = tracked.header
        self.tracked_publisher.publish(tracked)
        self.predicted_publisher.publish(predicted)

    def _on_objects(self, message: DetectedObjects) -> None:
        if not message.objects:
            self._publish_empty(message)
            return
        try:
            transform = self._lookup_transform(message)
        except TransformException as error:
            self.get_logger().warn(
                f"Object transform unavailable: {error}",
                throttle_duration_sec=2.0,
            )
            return

        poses = [
            self._transform_pose(
                detected.kinematics.pose_with_covariance.pose, transform
            )
            for detected in message.objects
        ]
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1.0e-9
        tracks = self.tracker.update(
            [(pose.position.x, pose.position.y) for pose in poses], stamp
        )

        tracked_message = TrackedObjects()
        tracked_message.header = message.header
        tracked_message.header.frame_id = self.output_frame
        predicted_message = PredictedObjects()
        predicted_message.header = tracked_message.header

        for detected, pose, track in zip(message.objects, poses, tracks):
            tracked = TrackedObject()
            tracked.object_id = UUID(uuid=list(track.identifier))
            tracked.existence_probability = detected.existence_probability
            tracked.classification = copy.deepcopy(detected.classification)
            tracked.shape = copy.deepcopy(detected.shape)
            tracked.kinematics.pose_with_covariance = copy.deepcopy(
                detected.kinematics.pose_with_covariance
            )
            tracked.kinematics.pose_with_covariance.pose = pose
            tracked.kinematics.twist_with_covariance.twist.linear.x = track.vx
            tracked.kinematics.twist_with_covariance.twist.linear.y = track.vy
            tracked.kinematics.orientation_availability = 0
            tracked.kinematics.is_stationary = (
                math.hypot(track.vx, track.vy) <= self.stationary_speed
            )
            tracked_message.objects.append(tracked)

            predicted = PredictedObject()
            predicted.object_id = tracked.object_id
            predicted.existence_probability = tracked.existence_probability
            predicted.classification = copy.deepcopy(tracked.classification)
            predicted.shape = copy.deepcopy(tracked.shape)
            predicted.kinematics.initial_pose_with_covariance = copy.deepcopy(
                tracked.kinematics.pose_with_covariance
            )
            predicted.kinematics.initial_twist_with_covariance = copy.deepcopy(
                tracked.kinematics.twist_with_covariance
            )
            predicted.kinematics.predicted_paths.append(
                self._prediction(pose, track.vx, track.vy)
            )
            predicted_message.objects.append(predicted)

        self.tracked_publisher.publish(tracked_message)
        self.predicted_publisher.publish(predicted_message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectTracker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
