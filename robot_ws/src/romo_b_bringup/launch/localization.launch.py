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
    localizer = LifecycleNode(
        package="lidar_localization_ros2",
        executable="lidar_localization_node",
        name="lidar_localization",
        output="screen",
        parameters=[
            parameter_file,
            {
                "use_sim_time": use_sim_time,
                "map_path": LaunchConfiguration("pcd_map"),
                "enable_map_odom_tf": True,
                "global_frame_id": "map",
                "odom_frame_id": "odom",
                "base_frame_id": "base_link",
            },
        ],
        remappings=[
            ("cloud", "/sensing/lidar/top/pointcloud_filtered"),
            ("imu", "/sensing/imu/imu_raw"),
            ("odom", "/odometry/filtered"),
            ("initialpose", "/initialpose"),
            ("pcl_pose", "/localization/pose_with_covariance"),
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
            localizer,
            activate,
            configure,
        ]
    )
