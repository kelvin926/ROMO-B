import pathlib

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _python_launch(package: str, filename: str, arguments: dict):
    path = pathlib.Path(get_package_share_directory(package), "launch", filename)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)),
        launch_arguments=arguments.items(),
    )


def _actions(context):
    map_path = pathlib.Path(LaunchConfiguration("map_path").perform(context)).resolve()
    pcd_map = pathlib.Path(LaunchConfiguration("pcd_map").perform(context)).resolve()
    hardware_config = pathlib.Path(
        LaunchConfiguration("hardware_config").perform(context)
    ).resolve()
    livox_config = pathlib.Path(
        LaunchConfiguration("livox_config").perform(context)
    ).resolve()
    required = [
        hardware_config,
        livox_config,
        pcd_map,
        map_path / "lanelet2_map.osm",
        map_path / "pointcloud_map.pcd",
        map_path / "map_projector_info.yaml",
    ]
    missing = [str(path) for path in required if not path.exists()]
    if missing:
        raise RuntimeError("Missing ROMO-B Autoware input(s): " + ", ".join(missing))

    hardware = _python_launch(
        "romo_b_bringup",
        "hardware.launch.py",
        {
            "hardware_config": str(hardware_config),
            "livox_config": str(livox_config),
            "receive_only": LaunchConfiguration("receive_only").perform(context),
            "safety_profile": "navigation",
            "use_livox": "true",
            "use_ekf": "true",
            "use_sim_time": "false",
        },
    )
    localization = _python_launch(
        "romo_b_bringup",
        "localization.launch.py",
        {
            "pcd_map": str(pcd_map),
            "use_sim_time": "false",
            "set_initial_pose": "false",
            "initial_pose_topic": "/localization/initialpose_direct",
        },
    )
    perception = _python_launch("romo_b_autoware", "perception.launch.py", {})
    safety = _python_launch(
        "romo_b_navigation", "safety_pipeline.launch.py", {"use_sim_time": "false"}
    )

    autoware_launch = pathlib.Path(
        get_package_share_directory("autoware_launch"), "launch", "autoware.launch.xml"
    )
    autoware = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(str(autoware_launch)),
        launch_arguments={
            "map_path": str(map_path),
            "vehicle_model": "romo_b",
            "sensor_model": "sample_sensor_kit",
            "planning_module_preset": "romo_b",
            "control_module_preset": "romo_b",
            "launch_vehicle": "false",
            "launch_sensing": "false",
            "launch_localization": "false",
            "launch_perception": "false",
            "launch_map": "true",
            "launch_planning": "true",
            "launch_control": "true",
            "launch_system": "true",
            "launch_api": "true",
            "launch_system_monitor": "false",
            "launch_dummy_diag_publisher": "true",
            "enable_all_modules_auto_mode": "true",
            "rviz": LaunchConfiguration("use_rviz").perform(context),
            "use_sim_time": "false",
            "is_simulation": "false",
        }.items(),
    )

    share = pathlib.Path(get_package_share_directory("romo_b_autoware"))
    adapters = [
        Node(
            package="autoware_map_loader",
            executable="autoware_lanelet2_map_loader",
            namespace="map",
            name="romo_b_lanelet2_map_preloader",
            output="screen",
            parameters=[
                {
                    "allow_unsupported_version": True,
                    "center_line_resolution": 5.0,
                    "use_waypoints": True,
                    "lanelet2_map_path": str(map_path / "lanelet2_map.osm"),
                }
            ],
            remappings=[("output/lanelet2_map", "/map/vector_map")],
        ),
        Node(
            package="romo_b_autoware",
            executable="vector_map_startup_guard",
            name="romo_b_vector_map_startup_guard",
            output="screen",
            # Autoware starts three seconds later below. Relay the unchanged
            # map only during the bounded composable-node startup window.
            parameters=[{"republish_delay_sec": 12.0}],
        ),
        Node(
            package="romo_b_autoware",
            executable="vehicle_interface",
            name="romo_b_autoware_vehicle_interface",
            output="screen",
            parameters=[str(share / "config" / "vehicle_interface.yaml")],
        ),
        Node(
            package="romo_b_autoware",
            executable="kinematic_bridge",
            name="romo_b_autoware_kinematic_bridge",
            output="screen",
        ),
        Node(
            package="romo_b_autoware",
            executable="localization_interface",
            name="romo_b_autoware_localization_interface",
            output="screen",
        ),
        Node(
            package="romo_b_autoware",
            executable="speed_limit_guard",
            name="romo_b_autoware_speed_limit_guard",
            output="screen",
            parameters=[str(share / "config" / "speed_limit_guard.yaml")],
        ),
        Node(
            package="romo_b_autoware",
            executable="trajectory_follower",
            name="romo_b_autoware_trajectory_follower",
            output="screen",
            parameters=[str(share / "config" / "trajectory_follower.yaml")],
        ),
    ]
    return [
        hardware,
        localization,
        perception,
        safety,
        *adapters,
        TimerAction(period=3.0, actions=[autoware]),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware_config"),
            DeclareLaunchArgument("livox_config"),
            DeclareLaunchArgument("pcd_map"),
            DeclareLaunchArgument("map_path"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            # The default field launch can inspect every live topic but cannot
            # write to the PCU. The operator must explicitly opt into TX and
            # still has to arm the lifecycle bridge separately.
            DeclareLaunchArgument("receive_only", default_value="true"),
            OpaqueFunction(function=_actions),
        ]
    )
