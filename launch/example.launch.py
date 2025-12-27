"""Launch file for laser_line_extraction node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""
    # Declare all launch arguments
    return LaunchDescription(
        [
            # Node configuration parameters
            DeclareLaunchArgument(
                "frequency", default_value="30.0", description="Processing frequency in Hz"
            ),
            DeclareLaunchArgument(
                "frame_id", default_value="laser", description="Frame ID for the laser scan"
            ),
            DeclareLaunchArgument(
                "scan_topic", default_value="scan", description="Topic name for laser scan input"
            ),
            DeclareLaunchArgument(
                "publish_markers",
                default_value="true",
                description="Whether to publish visualization markers",
            ),
            # Line extraction algorithm parameters
            DeclareLaunchArgument(
                "bearing_std_dev",
                default_value="1e-5",
                description="Standard deviation of bearing uncertainty in laser scans (rad)",
            ),
            DeclareLaunchArgument(
                "range_std_dev",
                default_value="0.012",
                description="Standard deviation of range uncertainty in laser scans (m)",
            ),
            DeclareLaunchArgument(
                "least_sq_angle_thresh",
                default_value="0.0001",
                description="Angle threshold to stop iterating least squares (rad)",
            ),
            DeclareLaunchArgument(
                "least_sq_radius_thresh",
                default_value="0.0001",
                description="Radius threshold to stop iterating least squares (m)",
            ),
            DeclareLaunchArgument(
                "max_line_gap",
                default_value="0.5",
                description="Maximum distance between two points in the same line (m)",
            ),
            DeclareLaunchArgument(
                "min_line_length",
                default_value="0.7",
                description="Lines shorter than this are not published (m)",
            ),
            DeclareLaunchArgument(
                "min_range",
                default_value="0.5",
                description="Points closer than this are ignored (m)",
            ),
            DeclareLaunchArgument(
                "max_range",
                default_value="250.0",
                description="Points farther than this are ignored (m)",
            ),
            DeclareLaunchArgument(
                "min_split_dist",
                default_value="0.04",
                description="Minimum distance for split in split-and-merge (m)",
            ),
            DeclareLaunchArgument(
                "outlier_dist",
                default_value="0.06",
                description="Distance threshold for outlier detection (m)",
            ),
            DeclareLaunchArgument(
                "min_line_points",
                default_value="10",
                description="Lines with fewer points than this are not published",
            ),
            # Node
            Node(
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
                        "bearing_std_dev": LaunchConfiguration("bearing_std_dev"),
                        "range_std_dev": LaunchConfiguration("range_std_dev"),
                        "least_sq_angle_thresh": LaunchConfiguration("least_sq_angle_thresh"),
                        "least_sq_radius_thresh": LaunchConfiguration("least_sq_radius_thresh"),
                        "max_line_gap": LaunchConfiguration("max_line_gap"),
                        "min_line_length": LaunchConfiguration("min_line_length"),
                        "min_range": LaunchConfiguration("min_range"),
                        "max_range": LaunchConfiguration("max_range"),
                        "min_split_dist": LaunchConfiguration("min_split_dist"),
                        "outlier_dist": LaunchConfiguration("outlier_dist"),
                        "min_line_points": LaunchConfiguration("min_line_points"),
                    }
                ],
            ),
        ]
    )
