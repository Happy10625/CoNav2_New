from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    share = get_package_share_directory("co_nav2_nav")
    params = os.path.join(share, "config", "robot.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=params),
        DeclareLaunchArgument("enable_perception", default_value="false"),
        DeclareLaunchArgument("publish_map_odom", default_value="true"),
        DeclareLaunchArgument("enable_odom_adapter", default_value="false"),
        DeclareLaunchArgument("publish_lidar_tf", default_value="false"),
        Node(
            package="co_nav2_nav", executable="fastlio_odom_adapter",
            name="fastlio_odom_adapter", output="screen",
            parameters=[LaunchConfiguration("params_file")],
            condition=IfCondition(LaunchConfiguration("enable_odom_adapter")),
        ),
        # Remove this node when the mapping/localization stack owns map -> odom.
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="map_to_odom_identity",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            condition=IfCondition(LaunchConfiguration("publish_map_odom")),
        ),
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="base_to_livox",
            arguments=["0.31", "0", "0.365", "3.141592653589793", "0", "0",
                       "base_link", "livox_frame"],
            condition=IfCondition(LaunchConfiguration("publish_lidar_tf")),
        ),
        Node(
            package="co_nav2_nav", executable="semantic_explorer",
            name="semantic_explorer", output="screen",
            parameters=[LaunchConfiguration("params_file"), {
                "enable_perception": LaunchConfiguration("enable_perception")
            }],
        ),
    ])
