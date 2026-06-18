import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("scan_feature_matcher"),
        "config",
        "feature_matcher.yaml",
    )
    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Optional YAML parameter file for scan_feature_matcher.",
            ),
            Node(
                package="scan_feature_matcher",
                executable="scan_feature_matcher",
                name="scan_feature_matcher",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
