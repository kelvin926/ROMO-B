import math

import rclpy
from autoware_adapi_v1_msgs.msg import (
    LocalizationInitializationState,
    OperationModeState,
)
from autoware_planning_msgs.msg import Trajectory
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from romo_b_msgs.msg import PlatformStatus

from .control_math import (
    PathPoint,
    calculate_follow_command,
    curvature_to_twist,
    quaternion_yaw,
)


class TrajectoryFollower(Node):
    """Conservative pure-pursuit fallback for Autoware trajectories below 0.2 m/s."""

    def __init__(self) -> None:
        super().__init__("romo_b_autoware_trajectory_follower")
        self.wheel_base = self.declare_parameter("wheel_base", 0.323).value
        self.lookahead = self.declare_parameter("lookahead", 0.70).value
        self.stop_distance = self.declare_parameter("stop_distance", 0.30).value
        self.max_speed = self.declare_parameter("max_speed", 0.20).value
        self.max_steer = self.declare_parameter(
            "max_steer", math.radians(22.0)
        ).value
        self.trajectory_timeout = self.declare_parameter(
            "trajectory_timeout", 0.50
        ).value
        self.odometry_timeout = self.declare_parameter("odometry_timeout", 0.30).value
        self.require_operation_mode = self.declare_parameter(
            "require_operation_mode", True
        ).value

        self.trajectory = None
        self.trajectory_received = -math.inf
        self.odometry = None
        self.odometry_received = -math.inf
        self.platform = None
        self.operation_mode = None
        self.localization_state = None

        self.publisher = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self.target_publisher = self.create_publisher(
            PointStamped, "/romo_b/autoware/lookahead_point", 10
        )
        self.create_subscription(
            Trajectory, "/planning/trajectory", self._on_trajectory, 10
        )
        self.create_subscription(
            Odometry, "/localization/kinematic_state", self._on_odometry, 20
        )
        self.create_subscription(
            PlatformStatus, "/romo_b/platform_status", self._on_platform, 10
        )
        self.create_subscription(
            OperationModeState,
            "/api/operation_mode/state",
            self._on_operation_mode,
            10,
        )
        transient = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            LocalizationInitializationState,
            "/localization/initialization_state",
            self._on_localization_state,
            transient,
        )
        self.create_timer(0.05, self._on_timer)

    def _seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _on_trajectory(self, message: Trajectory) -> None:
        self.trajectory = message
        self.trajectory_received = self._seconds()

    def _on_odometry(self, message: Odometry) -> None:
        self.odometry = message
        self.odometry_received = self._seconds()

    def _on_platform(self, message: PlatformStatus) -> None:
        self.platform = message

    def _on_operation_mode(self, message: OperationModeState) -> None:
        self.operation_mode = message

    def _on_localization_state(
        self, message: LocalizationInitializationState
    ) -> None:
        self.localization_state = message

    def _ready(self) -> bool:
        platform_ready = bool(
            self.platform
            and self.platform.state == PlatformStatus.STATE_ARMED_AUTO
            and self.platform.connected
            and self.platform.auto_mode
            and not self.platform.estop
            and not self.platform.command_timed_out
            and not self.platform.feedback_timed_out
        )
        if not platform_ready:
            return False
        if not (
            self.localization_state
            and self.localization_state.state
            == LocalizationInitializationState.INITIALIZED
        ):
            return False
        if not self.require_operation_mode:
            return True
        return bool(
            self.operation_mode
            and self.operation_mode.mode == OperationModeState.AUTONOMOUS
            and self.operation_mode.is_autoware_control_enabled
        )

    def _on_timer(self) -> None:
        output = Twist()
        now = self._seconds()
        fresh = bool(
            self.trajectory
            and self.odometry
            and now - self.trajectory_received <= self.trajectory_timeout
            and now - self.odometry_received <= self.odometry_timeout
        )
        if not self._ready() or not fresh or not self.trajectory.points:
            self.publisher.publish(output)
            return

        pose = self.odometry.pose.pose
        orientation = pose.orientation
        path = [
            PathPoint(
                point.pose.position.x,
                point.pose.position.y,
                point.longitudinal_velocity_mps,
            )
            for point in self.trajectory.points
        ]
        command = calculate_follow_command(
            path,
            pose.position.x,
            pose.position.y,
            quaternion_yaw(
                orientation.x, orientation.y, orientation.z, orientation.w
            ),
            wheel_base=self.wheel_base,
            lookahead=self.lookahead,
            stop_distance=self.stop_distance,
            max_speed=self.max_speed,
            max_steer=self.max_steer,
        )
        output.linear.x = command.speed
        output.angular.z = curvature_to_twist(
            command.speed, command.steer, self.wheel_base
        )
        self.publisher.publish(output)

        target = PointStamped()
        target.header = self.odometry.header
        target.point.x = command.target_x
        target.point.y = command.target_y
        self.target_publisher.publish(target)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
