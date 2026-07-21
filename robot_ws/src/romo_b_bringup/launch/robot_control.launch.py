from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("romo_b_bringup"), "launch", "hardware.launch.py"]
            )
        ),
        launch_arguments={
            "hardware_config": LaunchConfiguration("hardware_config"),
            "livox_config": LaunchConfiguration("livox_config"),
            "receive_only": "false",
            "safety_profile": "navigation",
            "max_navigation_speed_mps": LaunchConfiguration("max_speed_mps"),
            "allow_reverse": "true",
            "use_livox": "false",
            "use_ekf": "false",
            "use_sim_time": "false",
        }.items(),
    )
    direct_teleop = Node(
        package="romo_b_navigation",
        executable="safe_command_heartbeat",
        name="robot_control_command_relay",
        output="screen",
        parameters=[
            {
                "input_topic": "/cmd_vel_teleop",
                "output_topic": "/cmd_vel_safe",
                "input_timeout_sec": 0.12,
                "publish_frequency": 20.0,
                "max_forward_speed": ParameterValue(
                    LaunchConfiguration("max_speed_mps"), value_type=float
                ),
                "max_reverse_speed": ParameterValue(
                    LaunchConfiguration("max_speed_mps"), value_type=float
                ),
                "max_angular_speed": 0.80,
            }
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hardware_config", default_value="config/local/hardware.yaml"
            ),
            DeclareLaunchArgument(
                "livox_config", default_value="config/local/MID360_config.json"
            ),
            DeclareLaunchArgument("max_speed_mps", default_value="0.5"),
            hardware,
            direct_teleop,
        ]
    )
