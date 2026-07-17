"""Launch the display-only occupancy-map artifact processor."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Expose the common live-map settings as launch arguments."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("input_topic", default_value="/map"),
            DeclareLaunchArgument(
                "output_dir", default_value="~/.ros/map_artifacts"
            ),
            DeclareLaunchArgument(
                "minimum_update_interval_s", default_value="0.75"
            ),
            Node(
                package="autopatrol_robot",
                executable="map_artifact_node",
                name="map_artifact_processor",
                output="screen",
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("input_topic"),
                        "output_dir": LaunchConfiguration("output_dir"),
                        "minimum_update_interval_s": LaunchConfiguration(
                            "minimum_update_interval_s"
                        ),
                    }
                ],
            ),
        ]
    )

