from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("romo_b_navigation")
    default_params = PathJoinSubstitution([share, "config", "nav2.yaml"])
    default_map = PathJoinSubstitution([share, "config", "maps", "empty.yaml"])
    twist_mux = PathJoinSubstitution([share, "config", "twist_mux.yaml"])
    velocity_smoother = PathJoinSubstitution([share, "config", "velocity_smoother.yaml"])
    collision_monitor = PathJoinSubstitution([share, "config", "collision_monitor.yaml"])
    behavior_tree = PathJoinSubstitution(
        [share, "behavior_trees", "navigate_through_poses_forward_only.xml"]
    )

    params = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    typed_sim_time = ParameterValue(use_sim_time, value_type=bool)

    nodes = [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[params, {"yaml_filename": map_yaml, "use_sim_time": typed_sim_time}],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[params, {"use_sim_time": typed_sim_time}],
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[params, {"use_sim_time": typed_sim_time}],
            remappings=[("cmd_vel", "/cmd_vel_nav"), ("odom", "/odometry/filtered")],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[
                params,
                {
                    "use_sim_time": typed_sim_time,
                    "default_nav_through_poses_bt_xml": behavior_tree,
                },
            ],
        ),
        Node(
            package="twist_mux",
            executable="twist_mux",
            name="twist_mux",
            output="screen",
            parameters=[twist_mux],
            remappings=[("cmd_vel_out", "/cmd_vel_selected")],
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=[velocity_smoother, {"use_sim_time": typed_sim_time}],
            remappings=[
                ("cmd_vel", "/cmd_vel_selected"),
                ("cmd_vel_smoothed", "/cmd_vel_smoothed"),
                ("odom", "/odometry/filtered"),
            ],
        ),
        Node(
            package="nav2_collision_monitor",
            executable="collision_monitor",
            name="collision_monitor",
            output="screen",
            parameters=[collision_monitor, {"use_sim_time": typed_sim_time}],
        ),
        Node(
            package="romo_b_waypoints",
            executable="waypoint_manager",
            output="screen",
            parameters=[{"waypoint_file": LaunchConfiguration("waypoint_file")}],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {
                    "use_sim_time": typed_sim_time,
                    "autostart": True,
                    "node_names": [
                        "map_server",
                        "planner_server",
                        "controller_server",
                        "bt_navigator",
                        "velocity_smoother",
                        "collision_monitor",
                    ],
                }
            ],
        ),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("waypoint_file", default_value="~/.ros/romo_b_waypoints.yaml"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            *nodes,
        ]
    )
