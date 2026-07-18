import copy

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class OccupancyGridRepublisher(Node):
    """Continuously bridge Nav2's latched map to Autoware's volatile readers."""

    def __init__(self) -> None:
        super().__init__("romo_b_autoware_occupancy_grid_republisher")
        input_topic = self.declare_parameter(
            "input_topic", "/romo_b/autoware/occupancy_map_static"
        ).value
        output_topic = self.declare_parameter(
            "output_topic", "/perception/occupancy_grid_map/map"
        ).value
        publish_rate = float(self.declare_parameter("publish_rate", 2.0).value)
        if publish_rate <= 0.0:
            raise ValueError("publish_rate must be positive")

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        volatile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.latest = None
        self.publisher = self.create_publisher(OccupancyGrid, output_topic, volatile)
        self.create_subscription(OccupancyGrid, input_topic, self._on_map, latched)
        self.create_timer(1.0 / publish_rate, self._publish)
        self.get_logger().info(
            f"Republishing static occupancy map at {publish_rate:.1f} Hz: "
            f"{input_topic} -> {output_topic}"
        )

    def _on_map(self, message: OccupancyGrid) -> None:
        self.latest = message
        self._publish()

    def _publish(self) -> None:
        if self.latest is None:
            return
        message = copy.deepcopy(self.latest)
        message.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OccupancyGridRepublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
