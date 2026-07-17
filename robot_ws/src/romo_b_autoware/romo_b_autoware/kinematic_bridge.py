import math

import rclpy
from geometry_msgs.msg import AccelWithCovarianceStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class KinematicBridge(Node):
    """Combine NDT pose and wheel/EKF twist into Autoware kinematic state."""

    def __init__(self) -> None:
        super().__init__("romo_b_autoware_kinematic_bridge")
        self.output_frame = self.declare_parameter("output_frame", "map").value
        self.child_frame = self.declare_parameter("child_frame", "base_link").value
        self.odometry = None
        self.previous_velocity = None
        self.previous_stamp = None
        self.kinematic_publisher = self.create_publisher(
            Odometry, "/localization/kinematic_state", 10
        )
        self.acceleration_publisher = self.create_publisher(
            AccelWithCovarianceStamped, "/localization/acceleration", 10
        )
        self.create_subscription(
            Odometry, "/odometry/filtered", self._on_odometry, 20
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/localization/pose_with_covariance",
            self._on_pose,
            10,
        )

    def _on_odometry(self, message: Odometry) -> None:
        self.odometry = message

    def _on_pose(self, message: PoseWithCovarianceStamped) -> None:
        if self.odometry is None:
            return
        output = Odometry()
        output.header = message.header
        output.header.frame_id = self.output_frame
        output.child_frame_id = self.child_frame
        output.pose = message.pose
        output.twist = self.odometry.twist
        self.kinematic_publisher.publish(output)

        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1.0e-9
        velocity = float(output.twist.twist.linear.x)
        acceleration = AccelWithCovarianceStamped()
        acceleration.header = output.header
        if self.previous_stamp is not None and stamp > self.previous_stamp + 1.0e-3:
            value = (velocity - self.previous_velocity) / (stamp - self.previous_stamp)
            acceleration.accel.accel.linear.x = max(-3.0, min(3.0, value))
        self.acceleration_publisher.publish(acceleration)
        if math.isfinite(stamp) and math.isfinite(velocity):
            self.previous_stamp = stamp
            self.previous_velocity = velocity


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KinematicBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
