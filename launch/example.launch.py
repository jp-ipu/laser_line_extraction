"""Launch file for laser_line_extraction node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""
    # Declare launch arguments
    frequency_arg = DeclareLaunchArgument(
        "frequency", default_value="30.0", description="Processing frequency in Hz"
    )

    frame_id_arg = DeclareLaunchArgument(
        "frame_id", default_value="laser", description="Frame ID for the laser scan"
    )

    scan_topic_arg = DeclareLaunchArgument(
        "scan_topic", default_value="scan", description="Topic name for laser scan input"
    )

    publish_markers_arg = DeclareLaunchArgument(
        "publish_markers",
        default_value="true",
        description="Whether to publish visualization markers",
    )

    # Create the node
    line_extraction_node = Node(
        package="laser_line_extraction",
        executable="line_extraction_node",
        name="line_extractor",
        output="screen",
        parameters=[
            {
                "frequency": LaunchConfiguration("frequency"),
                "frame_id": LaunchConfiguration("frame_id"),
                "scan_topic": LaunchConfiguration("scan_topic"),
                "publish_markers": LaunchConfiguration("publish_markers"),
                "bearing_std_dev": 1e-5,
                "range_std_dev": 0.012,
                "least_sq_angle_thresh": 0.0001,
                "least_sq_radius_thresh": 0.0001,
                "max_line_gap": 0.5,
                "min_line_length": 0.7,
                "min_range": 0.5,
                "max_range": 250.0,
                "min_split_dist": 0.04,
                "outlier_dist": 0.06,
                "min_line_points": 10,
            }
        ],
    )

    return LaunchDescription(
        [
            frequency_arg,
            frame_id_arg,
            scan_topic_arg,
            publish_markers_arg,
            line_extraction_node,
        ]
    )
