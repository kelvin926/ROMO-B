from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, TimerAction
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    parameter_file = LaunchConfiguration("parameter_file")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    enable_map_odom_tf = ParameterValue(
        LaunchConfiguration("enable_map_odom_tf"), value_type=bool
    )
    set_initial_pose = ParameterValue(
        LaunchConfiguration("set_initial_pose"), value_type=bool
    )
    use_odom = ParameterValue(LaunchConfiguration("use_odom"), value_type=bool)
    use_imu = ParameterValue(LaunchConfiguration("use_imu"), value_type=bool)
    use_imu_preintegration = ParameterValue(
        LaunchConfiguration("use_imu_preintegration"), value_type=bool
    )
    localizer = LifecycleNode(
        package="lidar_localization_ros2",
        executable="lidar_localization_node",
        name="lidar_localization",
        namespace="",
        output="screen",
        parameters=[
            parameter_file,
            {
                "use_sim_time": use_sim_time,
                "map_path": LaunchConfiguration("pcd_map"),
                "enable_map_odom_tf": enable_map_odom_tf,
                "global_frame_id": "map",
                "odom_frame_id": "odom",
                "base_frame_id": "base_link",
                "set_initial_pose": set_initial_pose,
                "initial_pose_x": ParameterValue(
                    LaunchConfiguration("initial_pose_x"), value_type=float
                ),
                "initial_pose_y": ParameterValue(
                    LaunchConfiguration("initial_pose_y"), value_type=float
                ),
                "initial_pose_z": ParameterValue(
                    LaunchConfiguration("initial_pose_z"), value_type=float
                ),
                "initial_pose_qx": ParameterValue(
                    LaunchConfiguration("initial_pose_qx"), value_type=float
                ),
                "initial_pose_qy": ParameterValue(
                    LaunchConfiguration("initial_pose_qy"), value_type=float
                ),
                "initial_pose_qz": ParameterValue(
                    LaunchConfiguration("initial_pose_qz"), value_type=float
                ),
                "initial_pose_qw": ParameterValue(
                    LaunchConfiguration("initial_pose_qw"), value_type=float
                ),
                "use_odom": use_odom,
                "use_imu": use_imu,
                "use_imu_preintegration": use_imu_preintegration,
            },
        ],
        remappings=[
            ("cloud", LaunchConfiguration("cloud_topic")),
            ("imu", LaunchConfiguration("imu_topic")),
            ("odom", LaunchConfiguration("odom_topic")),
            ("initialpose", "/initialpose"),
            ("pcl_pose", "/localization/pose_with_covariance"),
            ("path", "/localization/path"),
            ("initial_map", "/localization/map"),
            ("alignment_status", "/localization/alignment_status"),
            ("reinitialization_requested", "/localization/reinitialization_requested"),
        ],
    )
    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=localizer,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(localizer),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )
    configure = TimerAction(
        period=1.0,
        actions=[
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(localizer),
                    transition_id=Transition.TRANSITION_CONFIGURE,
                )
            )
        ],
    )
    default_params = PathJoinSubstitution(
        [FindPackageShare("romo_b_bringup"), "config", "localization.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("pcd_map", default_value="data/local/maps/map.pcd"),
            DeclareLaunchArgument("parameter_file", default_value=default_params),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("enable_map_odom_tf", default_value="true"),
            DeclareLaunchArgument(
                "cloud_topic",
                default_value="/sensing/lidar/top/pointcloud_localization",
            ),
            DeclareLaunchArgument("imu_topic", default_value="/sensing/imu/imu_raw"),
            DeclareLaunchArgument("odom_topic", default_value="/odometry/filtered"),
            DeclareLaunchArgument("use_odom", default_value="true"),
            DeclareLaunchArgument("use_imu", default_value="false"),
            DeclareLaunchArgument("use_imu_preintegration", default_value="false"),
            DeclareLaunchArgument("set_initial_pose", default_value="false"),
            DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_z", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_qx", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_qy", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_qz", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_qw", default_value="1.0"),
            localizer,
            activate,
            configure,
        ]
    )
