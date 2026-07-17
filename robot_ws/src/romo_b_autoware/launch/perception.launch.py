from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("romo_b_autoware")
    cluster = Node(
        package="autoware_euclidean_cluster_object_detector",
        executable="euclidean_cluster_node",
        name="euclidean_cluster_node",
        output="screen",
        parameters=[PathJoinSubstitution([share, "config", "euclidean_cluster.yaml"])],
        remappings=[
            ("input", "/sensing/lidar/top/pointcloud_filtered"),
            ("output", "/perception/object_recognition/detection/objects"),
            ("debug/clusters", "/perception/obstacle_segmentation/clusters"),
        ],
    )
    tracker = Node(
        package="romo_b_autoware",
        executable="object_tracker",
        name="romo_b_autoware_object_tracker",
        output="screen",
        parameters=[PathJoinSubstitution([share, "config", "object_tracker.yaml"])],
    )
    pointcloud_relay = Node(
        package="topic_tools",
        executable="relay",
        name="autoware_obstacle_pointcloud_relay",
        output="screen",
        arguments=[
            "/sensing/lidar/top/pointcloud_filtered",
            "/perception/obstacle_segmentation/pointcloud",
        ],
    )
    return LaunchDescription([cluster, tracker, pointcloud_relay])
