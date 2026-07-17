from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    names = ("lidar_x", "lidar_y", "lidar_z", "lidar_roll", "lidar_pitch", "lidar_yaw")
    defaults = ("0.25", "0.0", "0.25", "0.0", "0.0", "0.0")
    declarations = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in zip(names, defaults)
    ]
    declarations.append(DeclareLaunchArgument("use_sim_time", default_value="false"))
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("romo_b_description"), "urdf", "romo_b.urdf.xacro"]
    )
    command = ["xacro ", xacro_file]
    for name in names:
        command.extend([f" {name}:=", LaunchConfiguration(name)])
    robot_description = ParameterValue(Command(command), value_type=str)
    publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
            }
        ],
    )
    return LaunchDescription([*declarations, publisher])
