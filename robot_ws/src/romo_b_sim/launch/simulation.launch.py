from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device = LaunchConfiguration("device")
    simulator = Node(
        package="romo_b_sim",
        executable="pcu_simulator",
        output="screen",
        parameters=[{"symlink_path": device, "auto_switch": True}],
    )
    bridge = Node(
        package="romo_b_base",
        executable="romo_b_serial_bridge",
        name="romo_b_serial_bridge",
        output="screen",
        parameters=[
            {
                "device": device,
                "receive_only": False,
                "command_endian": "big",
                "safety_profile": "bench",
            }
        ],
    )
    configure = ExecuteProcess(
        cmd=["ros2", "lifecycle", "set", "/romo_b_serial_bridge", "configure"],
        output="screen",
    )
    activate = ExecuteProcess(
        cmd=["ros2", "lifecycle", "set", "/romo_b_serial_bridge", "activate"],
        output="screen",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("device", default_value="/tmp/romo_b_pcu"),
            simulator,
            TimerAction(period=0.5, actions=[bridge]),
            TimerAction(period=1.2, actions=[configure]),
            TimerAction(period=2.0, actions=[activate]),
        ]
    )
