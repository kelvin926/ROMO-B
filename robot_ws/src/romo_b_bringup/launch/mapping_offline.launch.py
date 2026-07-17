import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    upstream = PathJoinSubstitution(
        [FindPackageShare("lidarslam"), "launch", "rko_lio_slam.launch.py"]
    )
    mid360_parameters = LaunchConfiguration("rko_param_file")
    mapping_rviz = PathJoinSubstitution(
        [FindPackageShare("romo_b_bringup"), "rviz", "mapping.rviz"]
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
            # Offline RKO-LIO publishes odom -> base_link while the graph map is
            # expressed in map.  An identity map -> odom transform is valid for
            # this mapping-only launch and lets RViz render both streams.
            "publish_static_tf": "true",
            "static_tf_parent": "map",
            "static_tf_child": "odom",
            "static_tf_x": "0.0",
            "static_tf_y": "0.0",
            "static_tf_z": "0.0",
            "static_tf_qx": "0.0",
            "static_tf_qy": "0.0",
            "static_tf_qz": "0.0",
            "static_tf_qw": "1.0",
            "use_rviz": LaunchConfiguration("use_rviz"),
            "rviz_config": mapping_rviz,
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("bag_path"),
            DeclareLaunchArgument("save_dir", default_value="data/local/maps/mapping_run"),
            DeclareLaunchArgument(
                "rko_param_file",
                default_value=os.path.join(
                    os.environ.get("ROMO_B_ROOT", os.getcwd()),
                    "config",
                    "local",
                    "rko_lio_mid360.yaml",
                ),
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            mapping,
        ]
    )
