import pathlib

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _actions(context):
    map_path = pathlib.Path(LaunchConfiguration("map_path").perform(context)).resolve()
    for filename in (
        "lanelet2_map.osm",
        "pointcloud_map.pcd",
        "map_projector_info.yaml",
    ):
        if not (map_path / filename).exists():
            raise RuntimeError(f"Autoware simulation map is missing {map_path / filename}")
    launch_file = pathlib.Path(
        get_package_share_directory("autoware_launch"),
        "launch",
        "planning_simulator.launch.xml",
    )
    share = pathlib.Path(get_package_share_directory("romo_b_autoware"))
    return [
        # Autoware normally loads this component into a shared container.  On
        # a busy laptop the one-second component-service response can time out
        # before the Lanelet loader is queued, leaving the whole stack without
        # a vector map.  A standalone identical loader removes that startup
        # race; duplicate successful publishers contain the same map bytes.
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
            parameters=[{"republish_delay_sec": 10.0}],
        ),
        Node(
            package="romo_b_autoware",
            executable="speed_limit_guard",
            name="romo_b_autoware_speed_limit_guard",
            output="screen",
            parameters=[str(share / "config" / "speed_limit_guard.yaml")],
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(str(launch_file)),
            launch_arguments={
                "map_path": str(map_path),
                "vehicle_model": "romo_b",
                "sensor_model": "sample_sensor_kit",
                "planning_module_preset": "romo_b",
                "control_module_preset": "romo_b",
                "enable_all_modules_auto_mode": "true",
                "localization_sim_mode": "api",
                "vehicle_simulation": "true",
                # The validation object must not disappear through the dummy
                # simulator's optional stochastic detection-failure model.
                "perception/enable_detection_failure": "false",
                "initial_engage_state": LaunchConfiguration(
                    "initial_engage_state"
                ).perform(context),
                "rviz": LaunchConfiguration("use_rviz").perform(context),
            }.items(),
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("map_path"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("initial_engage_state", default_value="false"),
            OpaqueFunction(function=_actions),
        ]
    )
