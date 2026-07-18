from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def include(package, launch_file, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", launch_file])
        ),
        launch_arguments=arguments.items(),
    )


def generate_launch_description():
    hardware = include(
        "romo_b_bringup",
        "hardware.launch.py",
        {
            "hardware_config": LaunchConfiguration("hardware_config"),
            "livox_config": LaunchConfiguration("livox_config"),
            "receive_only": "false",
            "safety_profile": "navigation",
            "max_navigation_speed_mps": LaunchConfiguration("max_speed_mps"),
            "use_livox": "true",
            "use_ekf": "true",
            "use_sim_time": "false",
        },
    )
    localization = include(
        "romo_b_bringup",
        "localization.launch.py",
        {
            "pcd_map": LaunchConfiguration("pcd_map"),
            "use_sim_time": "false",
            "set_initial_pose": "false",
        },
    )
    navigation = include(
        "romo_b_navigation",
        "navigation.launch.py",
        {
            "map": LaunchConfiguration("map"),
            "waypoint_file": LaunchConfiguration("waypoint_file"),
            "use_sim_time": "false",
        },
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("romo_b_bringup"), "rviz", "navigation.rviz"]
            ),
        ],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware_config"),
            DeclareLaunchArgument("livox_config"),
            DeclareLaunchArgument("pcd_map"),
            DeclareLaunchArgument("map"),
            DeclareLaunchArgument("waypoint_file"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("max_speed_mps", default_value="1.0"),
            hardware,
            localization,
            navigation,
            rviz,
        ]
    )
