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
    # Autoware's AEB requires base_link and logs every non-base_link input as
    # an error.  Re-transform the already cropped cloud instead of relaying its
    # base_footprint header unchanged.  The primary Collision Monitor continues
    # to consume the original base_footprint cloud directly.
    pointcloud_relay = Node(
        package="romo_b_perception",
        executable="pointcloud_filter",
        name="autoware_obstacle_pointcloud_transformer",
        output="screen",
        parameters=[
            {
                "input_topic": "/sensing/lidar/top/pointcloud_filtered",
                "output_topic": "/perception/obstacle_segmentation/pointcloud",
                "target_frame": "base_link",
                "voxel_size": 0.05,
                "enable_height_filter": False,
                "min_z": -5.0,
                "max_z": 10.0,
                "self_half_x": 0.42,
                "self_half_y": 0.34,
                "self_min_z": -0.25,
                "self_max_z": 1.80,
                "transform_timeout_sec": 0.10,
            }
        ],
    )
    return LaunchDescription([cluster, tracker, pointcloud_relay])
