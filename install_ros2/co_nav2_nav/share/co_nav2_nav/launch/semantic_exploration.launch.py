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
    venv_site_packages = "/home/isee-cdh/ws/Co-NavGPT2/.venv/lib/python3.10/site-packages"
    python_path = venv_site_packages
    if os.environ.get("PYTHONPATH"):
        python_path += os.pathsep + os.environ["PYTHONPATH"]
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=params),
        DeclareLaunchArgument("enable_perception", default_value="false"),
        DeclareLaunchArgument("enabled", default_value="false"),
        DeclareLaunchArgument("open_space_mode", default_value="false"),
        DeclareLaunchArgument("publish_map_odom", default_value="false"),
        DeclareLaunchArgument("enable_odom_adapter", default_value="false"),
        DeclareLaunchArgument("publish_lidar_tf", default_value="false"),
        DeclareLaunchArgument("publish_camera_tf", default_value="true"),
        Node(
            package="co_nav2_nav", executable="fastlio_odom_adapter",
            name="fastlio_odom_adapter", output="screen",
            parameters=[LaunchConfiguration("params_file")],
            condition=IfCondition(LaunchConfiguration("enable_odom_adapter")),
        ),
        # The adapter defines odom to be coincident with FAST_LIO camera_init.
        # This connects /cloud_registered to the same TF tree as Nav2.
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="odom_to_fastlio_world",
            arguments=["0", "0", "0", "0", "0", "0", "odom", "camera_init"],
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
        # Measured fixed RGB-D camera pose.  base_link is the ground projection
        # of the vehicle centre; camera_link is 0.20 m behind it and 1.215 m
        # above the ground, level and facing vehicle +X.
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="base_to_camera",
            arguments=[
                "--x", "-0.20", "--y", "0.0", "--z", "1.215",
                "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                "--frame-id", "base_link",
                "--child-frame-id", "camera_link",
            ],
            condition=IfCondition(LaunchConfiguration("publish_camera_tf")),
        ),
        Node(
            package="co_nav2_nav", executable="semantic_explorer",
            name="semantic_explorer", output="screen",
            additional_env={"PYTHONPATH": python_path},
            parameters=[LaunchConfiguration("params_file"), {
                "enable_perception": LaunchConfiguration("enable_perception"),
                "enabled": LaunchConfiguration("enabled"),
                "open_space_mode": LaunchConfiguration("open_space_mode"),
            }],
        ),
    ])
