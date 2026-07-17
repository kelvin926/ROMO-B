import rclpy
from autoware_internal_planning_msgs.msg import VelocityLimit
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class SpeedLimitGuard(Node):
    """Keep an independent low-speed limit active in Autoware planning."""

    def __init__(self) -> None:
        super().__init__("romo_b_autoware_speed_limit_guard")
        self.max_velocity = float(
            self.declare_parameter("max_velocity_mps", 0.14).value
        )
        if not 0.0 < self.max_velocity <= 0.20:
            raise ValueError("max_velocity_mps must be in (0.0, 0.20]")
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            VelocityLimit,
            "/planning/scenario_planning/max_velocity_candidates",
            qos,
        )
        self.create_timer(0.50, self._publish_limit)
        self._publish_limit()

    def _publish_limit(self) -> None:
        message = VelocityLimit()
        message.stamp = self.get_clock().now().to_msg()
        message.max_velocity = self.max_velocity
        message.sender = "romo_b_speed_limit_guard"
        self.publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpeedLimitGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
