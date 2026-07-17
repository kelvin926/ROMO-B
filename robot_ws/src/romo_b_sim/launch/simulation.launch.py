from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    simulator = Node(
        package="romo_b_sim",
        executable="pcu_simulator",
        output="screen",
        parameters=[{"symlink_path": "/tmp/romo_b_pcu", "auto_switch": True}],
    )
    bridge = Node(
        package="romo_b_base",
        executable="romo_b_serial_bridge",
        name="romo_b_serial_bridge",
        output="screen",
        parameters=[
            {
                "device": "/tmp/romo_b_pcu",
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
            simulator,
            TimerAction(period=0.5, actions=[bridge]),
            TimerAction(period=1.2, actions=[configure]),
            TimerAction(period=2.0, actions=[activate]),
        ]
    )
