import pathlib

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


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
    return [
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
                "rviz": LaunchConfiguration("use_rviz").perform(context),
            }.items(),
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("map_path"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            OpaqueFunction(function=_actions),
        ]
    )
