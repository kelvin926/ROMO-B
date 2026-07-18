"""Relay Autoware's vector map during the bounded planner startup window."""

import time

import rclpy
from autoware_map_msgs.msg import LaneletMapBin
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class VectorMapStartupGuard(Node):
    """Keep the unchanged map available while composable planners load."""

    def __init__(self) -> None:
        super().__init__("romo_b_vector_map_startup_guard")
        self.declare_parameter("republish_delay_sec", 10.0)
        self.declare_parameter("republish_period_sec", 2.0)
        self.declare_parameter("republish_duration_sec", 30.0)
        self.declare_parameter("map_wait_timeout_sec", 45.0)
        self._delay = float(self.get_parameter("republish_delay_sec").value)
        self._period = float(self.get_parameter("republish_period_sec").value)
        self._duration = float(self.get_parameter("republish_duration_sec").value)
        self._timeout = float(self.get_parameter("map_wait_timeout_sec").value)
        if self._delay < 0.0 or self._period <= 0.0 or self._duration < 0.0:
            raise ValueError("vector-map startup relay timing parameters are invalid")
        self._started = time.monotonic()
        self._map = None
        self._relay_started = False
        self._last_publish = -float("inf")
        self._publish_count = 0
        self._timeout_reported = False

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            LaneletMapBin, "/map/vector_map", qos
        )
        self._ready_publisher = self.create_publisher(
            Bool, "/romo_b/autoware/vector_map_ready", qos
        )
        self._subscription = self.create_subscription(
            LaneletMapBin, "/map/vector_map", self._on_map, qos
        )
        self._timer = self.create_timer(0.25, self._on_timer)

    def _on_map(self, message: LaneletMapBin) -> None:
        if not self._relay_started:
            self._map = message

    def _on_timer(self) -> None:
        elapsed = time.monotonic() - self._started
        if self._map is not None and elapsed >= self._delay:
            # Several Autoware composable planners use volatile subscriptions
            # created well after the transient map loader publishes.  Relay
            # the exact same message for a bounded startup window so each late
            # subscriber receives a live sample.  This node never edits map
            # content and stops automatically after the window.
            if elapsed - self._last_publish >= self._period:
                self._relay_started = True
                self._last_publish = elapsed
                self._publish_count += 1
                self._publisher.publish(self._map)
                if self._publish_count == 1:
                    self.get_logger().info(
                        "Started bounded unchanged vector-map startup relay"
                    )
            if elapsed >= self._delay + self._duration:
                ready = Bool()
                ready.data = True
                self._ready_publisher.publish(ready)
                self.get_logger().info(
                    "Completed vector-map startup relay after "
                    f"{self._publish_count} samples; routing is ready"
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
