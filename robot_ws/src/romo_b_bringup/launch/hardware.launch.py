import pathlib

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _actions(context):
    hardware_path = pathlib.Path(
        LaunchConfiguration("hardware_config").perform(context)
    ).expanduser()
    if not hardware_path.is_file():
        raise RuntimeError(
            f"Missing {hardware_path}; run scripts/onboard_hardware.sh --generate first"
        )
    hardware = yaml.safe_load(hardware_path.read_text()) or {}
    serial = hardware.get("serial", {})
    lidar = hardware.get("lidar", {})
    transform = lidar.get("transform", {})
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    description_launch = pathlib.Path(
        get_package_share_directory("romo_b_description"), "launch", "description.launch.py"
    )
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(description_launch)),
        launch_arguments=(
            {
                f"lidar_{name}": str(transform.get(name, 0.0))
                for name in ("x", "y", "z", "roll", "pitch", "yaw")
            }
            | {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ).items(),
    )
    bridge = Node(
        package="romo_b_base",
        executable="romo_b_serial_bridge",
        name="romo_b_serial_bridge",
        output="screen",
        parameters=[
            {
                "device": serial.get("device", "/dev/romo_b_pcu"),
                "baud": int(serial.get("baud", 115200)),
                "data_bits": int(serial.get("data_bits", 8)),
                "parity": serial.get("parity", "none"),
                "stop_bits": int(serial.get("stop_bits", 1)),
                "command_endian": serial.get("command_endian", "unverified"),
                "receive_only": ParameterValue(
                    LaunchConfiguration("receive_only"), value_type=bool
                ),
                "safety_profile": LaunchConfiguration("safety_profile"),
                "sensor_calibrated": bool(lidar.get("calibrated", False)),
            }
        ],
    )
    configure = TimerAction(
        period=1.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/romo_b_serial_bridge", "configure"],
                output="screen",
                condition=IfCondition(LaunchConfiguration("autostart_bridge")),
            )
        ],
    )
    activate = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/romo_b_serial_bridge", "activate"],
                output="screen",
                condition=IfCondition(LaunchConfiguration("autostart_bridge")),
            )
        ],
    )

    livox_config = str(
        pathlib.Path(LaunchConfiguration("livox_config").perform(context)).expanduser().resolve()
    )
    livox = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_livox")),
        parameters=[
            {
                "xfer_format": 0,
                "multi_topic": 0,
                "data_src": 0,
                "publish_freq": 10.0,
                "output_data_type": 0,
                "frame_id": lidar.get("frame_id", "livox_frame"),
                "user_config_path": livox_config,
                "cmdline_input_bd_code": "livox0000000001",
            }
        ],
        remappings=[
            ("/livox/lidar", "/sensing/lidar/top/pointcloud_raw"),
            ("/livox/imu", "/sensing/imu/livox_raw"),
        ],
    )
    imu_normalizer = Node(
        package="romo_b_perception",
        executable="imu_normalizer",
        name="romo_b_imu_normalizer",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_livox")),
        parameters=[
            pathlib.Path(
                get_package_share_directory("romo_b_bringup"),
                "config",
                "imu_normalizer.yaml",
            ).as_posix(),
            {"use_sim_time": use_sim_time},
        ],
    )
    filter_node = Node(
        package="romo_b_perception",
        executable="pointcloud_filter",
        name="romo_b_pointcloud_filter",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_livox")),
        parameters=[
            pathlib.Path(
                get_package_share_directory("romo_b_bringup"),
                "config",
                "pointcloud_filter.yaml",
            ).as_posix(),
            {"use_sim_time": use_sim_time},
        ],
    )
    localization_filter_node = Node(
        package="romo_b_perception",
        executable="pointcloud_filter",
        name="romo_b_localization_cloud_filter",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_livox")),
        parameters=[
            pathlib.Path(
                get_package_share_directory("romo_b_bringup"),
                "config",
                "localization_cloud_filter.yaml",
            ).as_posix(),
            {"use_sim_time": use_sim_time},
        ],
    )
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_ekf")),
        parameters=[
            pathlib.Path(
                get_package_share_directory("romo_b_bringup"), "config", "ekf.yaml"
            ).as_posix(),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[("odometry/filtered", "/odometry/filtered")],
    )
    return [
        description,
        bridge,
        configure,
        activate,
        livox,
        imu_normalizer,
        filter_node,
        localization_filter_node,
        ekf,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware_config", default_value="config/local/hardware.yaml"),
            DeclareLaunchArgument("livox_config", default_value="config/local/MID360_config.json"),
            DeclareLaunchArgument("receive_only", default_value="true"),
            DeclareLaunchArgument("safety_profile", default_value="bench"),
            DeclareLaunchArgument("autostart_bridge", default_value="true"),
            DeclareLaunchArgument("use_livox", default_value="false"),
            DeclareLaunchArgument("use_ekf", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_actions),
        ]
    )
