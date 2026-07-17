from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    upstream = PathJoinSubstitution(
        [FindPackageShare("lidarslam"), "launch", "rko_lio_slam.launch.py"]
    )
    mid360_parameters = PathJoinSubstitution(
        [FindPackageShare("lidarslam"), "param", "rko_lio_mid360.yaml"]
    )
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(upstream),
        launch_arguments={
            "bag_path": LaunchConfiguration("bag_path"),
            "save_dir": LaunchConfiguration("save_dir"),
            "lidar_topic": "/sensing/lidar/top/pointcloud_raw",
            "imu_topic": "/sensing/imu/imu_raw",
            "base_frame": "base_link",
            "odom_frame": "odom",
            "lidar_frame": "livox_frame",
            "imu_frame": "livox_frame",
            "rko_param_file": mid360_parameters,
            "voxel_size": "0.10",
            "min_range": "0.5",
            "max_range": "30.0",
            "initialization_phase": "true",
            "publish_static_tf": "false",
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("bag_path"),
            DeclareLaunchArgument("save_dir", default_value="data/local/maps/mapping_run"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            mapping,
        ]
    )
