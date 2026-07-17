import pathlib

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _first_graph_pose(pose_graph: pathlib.Path):
    with pose_graph.open(encoding="utf-8") as stream:
        for line in stream:
            fields = line.split()
            if len(fields) >= 9 and fields[0] == "VERTEX_SE3:QUAT":
                return tuple(float(value) for value in fields[2:9])
    raise RuntimeError(f"No VERTEX_SE3:QUAT pose found in {pose_graph}")


def _actions(context):
    bag_path = pathlib.Path(LaunchConfiguration("bag_path").perform(context))
    pcd_map = pathlib.Path(LaunchConfiguration("pcd_map").perform(context))
    pose_graph = pathlib.Path(LaunchConfiguration("pose_graph").perform(context))
    hardware_path = pathlib.Path(
        LaunchConfiguration("hardware_config").perform(context)
    )
    bag_path = bag_path.expanduser().resolve()
    pcd_map = pcd_map.expanduser().resolve()
    pose_graph = pose_graph.expanduser().resolve()
    hardware_path = hardware_path.expanduser().resolve()
    for required in (bag_path, pcd_map, pose_graph, hardware_path):
        if not required.exists():
            raise RuntimeError(f"Required replay input does not exist: {required}")

    hardware = yaml.safe_load(hardware_path.read_text(encoding="utf-8")) or {}
    transform = hardware.get("lidar", {}).get("transform", {})
    pose = _first_graph_pose(pose_graph)
    pose_names = ("x", "y", "z", "qx", "qy", "qz", "qw")
    initial_pose = dict(zip(pose_names, pose))

    description_launch = pathlib.Path(
        get_package_share_directory("romo_b_description"),
        "launch",
        "description.launch.py",
    )
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(description_launch)),
        launch_arguments=(
            {
                f"lidar_{name}": str(transform.get(name, 0.0))
                for name in ("x", "y", "z", "roll", "pitch", "yaw")
            }
            | {"use_sim_time": "true"}
        ).items(),
    )

    filter_node = Node(
        package="romo_b_perception",
        executable="pointcloud_filter",
        name="romo_b_localization_cloud_filter",
        output="screen",
        parameters=[
            pathlib.Path(
                get_package_share_directory("romo_b_bringup"),
                "config",
                "localization_cloud_filter.yaml",
            ).as_posix(),
            {"use_sim_time": True},
        ],
    )

    localization_launch = pathlib.Path(
        get_package_share_directory("romo_b_bringup"),
        "launch",
        "localization.launch.py",
    )
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(localization_launch)),
        launch_arguments={
            "pcd_map": str(pcd_map),
            "use_sim_time": "true",
            # Direct map -> base_link is intentionally used only for isolated
            # localization replay. Runtime Nav2 owns odom -> base_link through EKF.
            "enable_map_odom_tf": "false",
            "cloud_topic": "/sensing/lidar/top/pointcloud_localization",
            "use_odom": "false",
            "use_imu": "false",
            "use_imu_preintegration": "false",
            "set_initial_pose": "true",
            **{
                f"initial_pose_{name}": str(value)
                for name, value in initial_pose.items()
            },
        }.items(),
    )

    output_bag = pathlib.Path(
        LaunchConfiguration("output_bag").perform(context)
    ).expanduser().resolve()
    if LaunchConfiguration("record_output").perform(context).lower() in (
        "true",
        "1",
        "yes",
    ):
        if output_bag.exists():
            raise RuntimeError(f"Replay output already exists: {output_bag}")
        output_bag.parent.mkdir(parents=True, exist_ok=True)

    recorder = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "-o",
            str(output_bag),
            "/localization/pose_with_covariance",
            "/localization/alignment_status",
            "/localization/reinitialization_requested",
            "/clock",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("record_output")),
    )
    player = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            str(bag_path),
            "--clock",
            "50",
            "--rate",
            LaunchConfiguration("rate"),
            "--delay",
            "2",
            "--disable-keyboard-controls",
            "--topics",
            "/sensing/lidar/top/pointcloud_raw",
        ],
        output="screen",
    )
    shutdown_after_player = RegisterEventHandler(
        OnProcessExit(
            target_action=player,
            on_exit=[
                TimerAction(
                    period=2.0,
                    actions=[
                        EmitEvent(
                            event=Shutdown(reason="localization replay completed")
                        )
                    ],
                )
            ],
        )
    )
    return [
        # Publish the corrected static robot tree after rosbag /clock starts.
        # This avoids tf2 clearing a pre-clock static cache on the time jump.
        TimerAction(period=3.0, actions=[description]),
        filter_node,
        localization,
        TimerAction(period=1.5, actions=[recorder]),
        player,
        shutdown_after_player,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("bag_path"),
            DeclareLaunchArgument("pcd_map"),
            DeclareLaunchArgument("pose_graph"),
            DeclareLaunchArgument(
                "hardware_config", default_value="config/local/hardware.yaml"
            ),
            DeclareLaunchArgument("rate", default_value="1.0"),
            DeclareLaunchArgument("record_output", default_value="true"),
            DeclareLaunchArgument(
                "output_bag", default_value="/tmp/romo_b_localization_replay"
            ),
            OpaqueFunction(function=_actions),
        ]
    )
