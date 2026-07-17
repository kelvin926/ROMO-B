"""Republish Autoware's latched vector map once after startup settles."""

import time

import rclpy
from autoware_map_msgs.msg import LaneletMapBin
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class VectorMapStartupGuard(Node):
    """Keep one identical map sample available for late planning components."""

    def __init__(self) -> None:
        super().__init__("romo_b_vector_map_startup_guard")
        self.declare_parameter("republish_delay_sec", 10.0)
        self.declare_parameter("map_wait_timeout_sec", 45.0)
        self._delay = float(self.get_parameter("republish_delay_sec").value)
        self._timeout = float(self.get_parameter("map_wait_timeout_sec").value)
        self._started = time.monotonic()
        self._map = None
        self._republished = False
        self._timeout_reported = False

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            LaneletMapBin, "/map/vector_map", qos
        )
        self._subscription = self.create_subscription(
            LaneletMapBin, "/map/vector_map", self._on_map, qos
        )
        self._timer = self.create_timer(0.25, self._on_timer)

    def _on_map(self, message: LaneletMapBin) -> None:
        if not self._republished:
            self._map = message

    def _on_timer(self) -> None:
        elapsed = time.monotonic() - self._started
        if self._map is not None and elapsed >= self._delay:
            # Set this before publish so the subscription ignores our own
            # sample. The transient publisher then retains the identical map
            # for any planning component that finishes discovery later.
            self._republished = True
            self._publisher.publish(self._map)
            self.get_logger().info(
                "Republished the unchanged vector map after startup settling"
            )
            self.destroy_timer(self._timer)
        elif elapsed >= self._timeout and not self._timeout_reported:
            self._timeout_reported = True
            self.get_logger().error(
                "No /map/vector_map sample received; planning must remain unavailable"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VectorMapStartupGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
