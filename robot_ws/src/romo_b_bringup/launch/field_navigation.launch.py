from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def include(package, launch_file, arguments, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", launch_file])
        ),
        launch_arguments=arguments.items(),
        condition=condition,
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
            "allow_reverse": "true",
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
    perception = include(
        "romo_b_autoware",
        "perception.launch.py",
        {},
        condition=IfCondition(LaunchConfiguration("use_object_tracking")),
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
        # PRIME offload keeps the large point cloud and costmap rendering off
        # the CPU/iGPU. Perception and planning remain deterministic on CPU.
        additional_env={
            "__NV_PRIME_RENDER_OFFLOAD": "1",
            "__GLX_VENDOR_LIBRARY_NAME": "nvidia",
        },
    )
    operator_ui = Node(
        package="romo_b_operator_ui",
        executable="operator_ui",
        name="romo_b_operator_ui",
        output="screen",
        parameters=[
            {
                "open_browser": LaunchConfiguration("open_operator_ui_browser"),
                "host": "127.0.0.1",
                "port": 8765,
            }
        ],
        condition=IfCondition(LaunchConfiguration("use_operator_ui")),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware_config"),
            DeclareLaunchArgument("livox_config"),
            DeclareLaunchArgument("pcd_map"),
            DeclareLaunchArgument("map"),
            DeclareLaunchArgument("waypoint_file"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_operator_ui", default_value="true"),
            DeclareLaunchArgument("open_operator_ui_browser", default_value="true"),
            DeclareLaunchArgument("use_object_tracking", default_value="true"),
            DeclareLaunchArgument("max_speed_mps", default_value="0.5"),
            hardware,
            localization,
            navigation,
            perception,
            operator_ui,
            rviz,
        ]
    )
