"""Run receive-only ROMO-B hardware bringup and live RKO-LIO mapping."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    repo_root = os.environ.get("ROMO_B_ROOT", os.getcwd())
    hardware_launch = PathJoinSubstitution(
        [FindPackageShare("romo_b_bringup"), "launch", "hardware.launch.py"]
    )
    mapping_rviz = PathJoinSubstitution(
        [FindPackageShare("romo_b_bringup"), "rviz", "mapping.rviz"]
    )
    graph_parameters = PathJoinSubstitution(
        [FindPackageShare("lidarslam"), "param", "lidarslam.yaml"]
    )

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hardware_launch),
        launch_arguments={
            "hardware_config": LaunchConfiguration("hardware_config"),
            "livox_config": LaunchConfiguration("livox_config"),
            "receive_only": "true",
            "safety_profile": "bench",
            "use_livox": "true",
            # RKO-LIO owns odom -> base_link during mapping. Starting the wheel
            # EKF here would create a duplicate TF owner.
            "use_ekf": "false",
        }.items(),
    )

    rko_lio = Node(
        package="rko_lio",
        executable="online_node",
        name="rko_lio_online_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "lidar_topic": "/sensing/lidar/top/pointcloud_raw",
                "imu_topic": "/sensing/imu/imu_raw",
                "lidar_frame": "livox_frame",
                "imu_frame": "livox_frame",
                "base_frame": "base_link",
                "odom_frame": "odom",
                "odom_topic": "/rko_lio/odometry",
                "deskew": True,
                "voxel_size": 0.10,
                "min_range": 0.5,
                "max_range": 30.0,
                "initialization_phase": True,
                "publish_deskewed_scan": True,
                "deskewed_scan_topic": "/rko_lio/frame",
                "publish_local_map": True,
                "map_topic": "/rko_lio/local_map",
                "publish_map_after": 1.0,
            },
            LaunchConfiguration("rko_param_file"),
        ],
    )

    graph_slam = Node(
        package="graph_based_slam",
        executable="graph_based_slam_node",
        name="graph_based_slam",
        output="screen",
        parameters=[
            graph_parameters,
            {
                "global_frame_id": "map",
                "use_odom_input": True,
                "map_save_dir": LaunchConfiguration("save_dir"),
                # Reject transient people from the final accumulated map while
                # retaining the complete raw bag for later reprocessing.
                "use_dynamic_object_filter": True,
            },
        ],
        remappings=[
            ("odom_input", "/rko_lio/odometry"),
            ("cloud_input", "/rko_lio/frame"),
        ],
    )

    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="mapping_map_to_odom",
        arguments=["0", "0", "0", "0", "0", "0", "1", "map", "odom"],
        output="screen",
    )

    recorder = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("record_bag")),
        cmd=[
            "ros2",
            "bag",
            "record",
            "--storage",
            "sqlite3",
            "--output",
            LaunchConfiguration("bag_path"),
            "/sensing/lidar/top/pointcloud_raw",
            "/sensing/imu/livox_raw",
            "/sensing/imu/imu_raw",
            "/wheel/odometry_raw",
            "/joint_states",
            "/rko_lio/odometry",
            "/rko_lio/frame",
            "/tf",
            "/tf_static",
            "/romo_b/platform_status",
            "/diagnostics",
        ],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="mapping_rviz",
        arguments=["-d", mapping_rviz],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hardware_config",
                default_value=os.path.join(repo_root, "config", "local", "hardware.yaml"),
            ),
            DeclareLaunchArgument(
                "livox_config",
                default_value=os.path.join(repo_root, "config", "local", "MID360_config.json"),
            ),
            DeclareLaunchArgument(
                "rko_param_file",
                default_value=os.path.join(
                    repo_root, "config", "local", "rko_lio_mid360.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "save_dir",
                default_value=os.path.join(repo_root, "data", "local", "maps", "mapping-live"),
            ),
            DeclareLaunchArgument(
                "bag_path",
                default_value=os.path.join(repo_root, "data", "local", "bags", "mapping-live"),
            ),
            DeclareLaunchArgument("record_bag", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            hardware,
            rko_lio,
            graph_slam,
            map_to_odom,
            recorder,
            rviz,
        ]
    )
