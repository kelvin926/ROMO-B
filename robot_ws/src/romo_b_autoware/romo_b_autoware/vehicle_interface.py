import math

import rclpy
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import (
    ControlModeReport,
    GearCommand,
    GearReport,
    HazardLightsCommand,
    HazardLightsReport,
    SteeringReport,
    TurnIndicatorsCommand,
    TurnIndicatorsReport,
    VelocityReport,
)
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from romo_b_msgs.msg import PlatformStatus

from .control_math import clamp, curvature_to_twist


class VehicleInterface(Node):
    """Translate Autoware vehicle commands and publish the standard status contract."""

    def __init__(self) -> None:
        super().__init__("romo_b_autoware_vehicle_interface")
        self.wheel_base = self.declare_parameter("wheel_base", 0.323).value
        self.max_speed = self.declare_parameter("max_speed", 0.20).value
        self.max_steer = self.declare_parameter(
            "max_steer", math.radians(22.0)
        ).value
        self.command_timeout = self.declare_parameter("command_timeout", 0.15).value
        self.command_source = self.declare_parameter(
            "command_source", "control"
        ).value
        if self.command_source not in ("control", "trajectory"):
            raise ValueError("command_source must be control or trajectory")

        self.platform = None
        self.odometry = None
        self.control = None
        self.control_received = -math.inf
        self.turn_command = TurnIndicatorsCommand.DISABLE
        self.hazard_command = HazardLightsCommand.DISABLE

        self.command_publisher = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self.velocity_publisher = self.create_publisher(
            VelocityReport, "/vehicle/status/velocity_status", 10
        )
        self.steering_publisher = self.create_publisher(
            SteeringReport, "/vehicle/status/steering_status", 10
        )
        self.mode_publisher = self.create_publisher(
            ControlModeReport, "/vehicle/status/control_mode", 10
        )
        self.gear_publisher = self.create_publisher(
            GearReport, "/vehicle/status/gear_status", 10
        )
        self.turn_publisher = self.create_publisher(
            TurnIndicatorsReport, "/vehicle/status/turn_indicators_status", 10
        )
        self.hazard_publisher = self.create_publisher(
            HazardLightsReport, "/vehicle/status/hazard_lights_status", 10
        )

        self.create_subscription(
            PlatformStatus, "/romo_b/platform_status", self._on_platform, 10
        )
        self.create_subscription(
            Odometry, "/wheel/odometry_raw", self._on_odometry, 10
        )
        self.create_subscription(
            Control, "/control/command/control_cmd", self._on_control, 10
        )
        self.create_subscription(
            GearCommand, "/control/command/gear_cmd", self._on_gear, 10
        )
        self.create_subscription(
            TurnIndicatorsCommand,
            "/control/command/turn_indicators_cmd",
            self._on_turn,
            10,
        )
        self.create_subscription(
            HazardLightsCommand,
            "/control/command/hazard_lights_cmd",
            self._on_hazard,
            10,
        )
        self.gear_command = GearCommand.DRIVE
        self.create_timer(0.05, self._on_timer)

    def _seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _on_platform(self, message: PlatformStatus) -> None:
        self.platform = message

    def _on_odometry(self, message: Odometry) -> None:
        self.odometry = message

    def _on_control(self, message: Control) -> None:
        self.control = message
        self.control_received = self._seconds()

    def _on_gear(self, message: GearCommand) -> None:
        self.gear_command = message.command

    def _on_turn(self, message: TurnIndicatorsCommand) -> None:
        self.turn_command = message.command

    def _on_hazard(self, message: HazardLightsCommand) -> None:
        self.hazard_command = message.command

    def _platform_ready(self) -> bool:
        return bool(
            self.platform
            and self.platform.state == PlatformStatus.STATE_ARMED_AUTO
            and self.platform.connected
            and self.platform.auto_mode
            and not self.platform.estop
            and not self.platform.feedback_timed_out
        )

    def _publish_reports(self) -> None:
        now = self.get_clock().now().to_msg()
        ready = self._platform_ready()

        velocity = VelocityReport()
        velocity.header.stamp = now
        velocity.header.frame_id = "base_link"
        if self.odometry:
            velocity.longitudinal_velocity = float(
                self.odometry.twist.twist.linear.x
            )
            velocity.lateral_velocity = float(self.odometry.twist.twist.linear.y)
            velocity.heading_rate = float(self.odometry.twist.twist.angular.z)
        self.velocity_publisher.publish(velocity)

        steering = SteeringReport()
        steering.stamp = now
        if self.platform:
            steering.steering_tire_angle = float(
                0.5
                * (
                    self.platform.wheel_steer_rad[0]
                    + self.platform.wheel_steer_rad[1]
                )
            )
        self.steering_publisher.publish(steering)

        mode = ControlModeReport()
        mode.stamp = now
        if ready:
            mode.mode = ControlModeReport.AUTONOMOUS
        elif self.platform and self.platform.connected:
            mode.mode = ControlModeReport.MANUAL
        else:
            mode.mode = ControlModeReport.NOT_READY
        self.mode_publisher.publish(mode)

        gear = GearReport()
        gear.stamp = now
        gear.report = GearReport.DRIVE if self.platform and self.platform.connected else GearReport.NEUTRAL
        self.gear_publisher.publish(gear)

        turn = TurnIndicatorsReport()
        turn.stamp = now
        turn.report = (
            self.turn_command
            if self.turn_command
            in (
                TurnIndicatorsReport.DISABLE,
                TurnIndicatorsReport.ENABLE_LEFT,
                TurnIndicatorsReport.ENABLE_RIGHT,
            )
            else TurnIndicatorsReport.DISABLE
        )
        self.turn_publisher.publish(turn)

        hazard = HazardLightsReport()
        hazard.stamp = now
        hazard.report = (
            HazardLightsReport.ENABLE
            if self.hazard_command == HazardLightsCommand.ENABLE
            else HazardLightsReport.DISABLE
        )
        self.hazard_publisher.publish(hazard)

    def _publish_control_command(self) -> None:
        command = Twist()
        recent = self._seconds() - self.control_received <= self.command_timeout
        gear_forward = self.gear_command in (GearCommand.NONE, GearCommand.DRIVE)
        if self._platform_ready() and recent and self.control and gear_forward:
            speed = clamp(
                float(self.control.longitudinal.velocity), 0.0, self.max_speed
            )
            steer = clamp(
                float(self.control.lateral.steering_tire_angle),
                -self.max_steer,
                self.max_steer,
            )
            command.linear.x = speed
            command.angular.z = curvature_to_twist(speed, steer, self.wheel_base)
        self.command_publisher.publish(command)

    def _on_timer(self) -> None:
        self._publish_reports()
        if self.command_source == "control":
            self._publish_control_command()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VehicleInterface()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
