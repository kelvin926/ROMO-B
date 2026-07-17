import rclpy
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState
from autoware_localization_msgs.srv import InitializeLocalization
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


def alignment_is_healthy(message: DiagnosticArray) -> bool:
    for status in message.status:
        if status.name != "lidar_localization_ros2/alignment":
            continue
        values = {item.key: item.value for item in status.values}
        return bool(
            status.level == DiagnosticStatus.OK
            and values.get("failure_category") == "healthy"
        )
    return False


class LocalizationInterface(Node):
    """Bridge ROMO-B NDT initialization into Autoware's component contract."""

    def __init__(self) -> None:
        super().__init__("romo_b_autoware_localization_interface")
        transient = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.direct_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            "/localization/initialpose_direct",
            10,
        )
        self.state_publisher = self.create_publisher(
            LocalizationInitializationState,
            "/localization/initialization_state",
            transient,
        )
        self.state = LocalizationInitializationState.UNINITIALIZED
        self.last_pose = None
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self._on_initial_pose,
            10,
        )
        self.create_subscription(
            DiagnosticArray,
            "/localization/alignment_status",
            self._on_alignment,
            transient,
        )
        self.create_subscription(
            Bool,
            "/localization/reinitialization_requested",
            self._on_reinitialization,
            transient,
        )
        self.create_service(
            InitializeLocalization,
            "/localization/initialize",
            self._on_initialize,
        )
        self.create_timer(0.5, self._publish_state)
        self._publish_state()

    def _set_state(self, state: int) -> None:
        if self.state != state:
            self.state = state
            self._publish_state()

    def _publish_state(self) -> None:
        message = LocalizationInitializationState()
        message.stamp = self.get_clock().now().to_msg()
        message.state = self.state
        self.state_publisher.publish(message)

    def _forward_pose(self, message: PoseWithCovarianceStamped) -> None:
        forwarded = PoseWithCovarianceStamped()
        forwarded.header = message.header
        if not forwarded.header.frame_id:
            forwarded.header.frame_id = "map"
        forwarded.pose = message.pose
        self.last_pose = forwarded
        self.direct_pose_publisher.publish(forwarded)
        self._set_state(LocalizationInitializationState.INITIALIZING)

    def _on_initial_pose(self, message: PoseWithCovarianceStamped) -> None:
        self._forward_pose(message)

    def _on_initialize(self, request, response):
        if request.pose_with_covariance:
            self._forward_pose(request.pose_with_covariance[0])
            response.status.success = True
            response.status.code = 0
            response.status.message = "Initial pose forwarded to ROMO-B NDT localization"
        else:
            response.status.success = False
            response.status.code = InitializeLocalization.Response.ERROR_GNSS_SUPPORT
            response.status.message = "ROMO-B requires a direct map-frame initial pose"
        return response

    def _on_alignment(self, message: DiagnosticArray) -> None:
        if alignment_is_healthy(message) and self.last_pose is not None:
            self._set_state(LocalizationInitializationState.INITIALIZED)
        elif self.last_pose is not None:
            self._set_state(LocalizationInitializationState.INITIALIZING)

    def _on_reinitialization(self, message: Bool) -> None:
        if message.data:
            self.last_pose = None
            self._set_state(LocalizationInitializationState.UNINITIALIZED)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationInterface()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
