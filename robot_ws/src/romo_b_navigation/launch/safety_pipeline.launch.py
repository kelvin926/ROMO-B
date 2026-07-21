from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("romo_b_navigation")
    use_sim_time = ParameterValue(
        LaunchConfiguration("use_sim_time"), value_type=bool
    )

    mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[PathJoinSubstitution([share, "config", "twist_mux.yaml"])],
        remappings=[("cmd_vel_out", "/cmd_vel_selected")],
    )
    smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[
            PathJoinSubstitution([share, "config", "velocity_smoother.yaml"]),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("cmd_vel", "/cmd_vel_selected"),
            ("cmd_vel_smoothed", "/cmd_vel_smoothed"),
            ("odom", "/odometry/filtered"),
        ],
    )
    monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[
            PathJoinSubstitution([share, "config", "collision_monitor.yaml"]),
            {"use_sim_time": use_sim_time},
        ],
    )
    heartbeat = Node(
        package="romo_b_navigation",
        executable="safe_command_heartbeat",
        name="safe_command_heartbeat",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_topic": "/cmd_vel_collision_checked",
                "output_topic": "/cmd_vel_safe",
                "input_timeout_sec": 0.12,
                "publish_frequency": 20.0,
                "max_forward_speed": 0.50,
                "max_reverse_speed": 0.50,
                "max_angular_speed": 0.80,
            }
        ],
    )
    manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_safety",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": ["velocity_smoother", "collision_monitor"],
            }
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            mux,
            smoother,
            monitor,
            heartbeat,
            manager,
        ]
    )
