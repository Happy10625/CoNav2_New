"""Start the bounded empty-area search stack, disarmed by default."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("co_nav2_nav")
    nav_launch = os.path.join(share, "launch", "nav2_navigation.launch.py")
    semantic_launch = os.path.join(share, "launch", "semantic_exploration.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("enabled", default_value="false"),
        DeclareLaunchArgument("enable_odom_adapter", default_value="false"),
        DeclareLaunchArgument("publish_camera_tf", default_value="true"),
        DeclareLaunchArgument("configure_camera_sync", default_value="true"),
        DeclareLaunchArgument("allow_frontier_after_scan", default_value="true"),
        DeclareLaunchArgument("approach_enabled", default_value="true"),
        Node(
            package="co_nav2_nav",
            executable="camera_sync_configurator",
            name="camera_sync_configurator",
            output="screen",
            condition=IfCondition(LaunchConfiguration("configure_camera_sync")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_launch),
            launch_arguments={
                "overrides_file": os.path.join(
                    share, "config", "nav2_open_space.yaml"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(semantic_launch),
            launch_arguments={
                "enabled": LaunchConfiguration("enabled"),
                "enable_perception": "true",
                "open_space_mode": "true",
                "allow_frontier_after_scan": LaunchConfiguration("allow_frontier_after_scan"),
                "approach_enabled": LaunchConfiguration("approach_enabled"),
                "enable_odom_adapter": LaunchConfiguration("enable_odom_adapter"),
                "publish_camera_tf": LaunchConfiguration("publish_camera_tf"),
            }.items(),
        ),
    ])
