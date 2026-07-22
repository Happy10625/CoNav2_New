"""Start Nav2 with the upstream defaults plus this robot's safe overrides."""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def deep_merge(destination, source):
    for key, value in source.items():
        if isinstance(value, dict) and isinstance(destination.get(key), dict):
            deep_merge(destination[key], value)
        else:
            destination[key] = value


def configure_navigation(context):
    bringup = get_package_share_directory("nav2_bringup")
    package = get_package_share_directory("co_nav2_nav")
    default_params = os.path.join(bringup, "params", "nav2_params.yaml")
    overrides = LaunchConfiguration("overrides_file").perform(context)
    with open(default_params, encoding="utf-8") as stream:
        params = yaml.safe_load(stream)
    with open(overrides, encoding="utf-8") as stream:
        deep_merge(params, yaml.safe_load(stream))
    bt_params = params.get("bt_navigator", {}).get("ros__parameters", {})
    package_prefix = "package://co_nav2_nav/"
    for key in ("default_nav_to_pose_bt_xml", "default_nav_through_poses_bt_xml"):
        bt_xml = bt_params.get(key, "")
        if bt_xml.startswith(package_prefix):
            bt_params[key] = os.path.join(package, bt_xml[len(package_prefix):])
    # Replace TurtleBot defaults everywhere they occur.
    for server in ("controller_server", "planner_server", "behavior_server",
                   "bt_navigator", "waypoint_follower", "velocity_smoother"):
        if server in params:
            params[server].setdefault("ros__parameters", {})["use_sim_time"] = False
    handle = tempfile.NamedTemporaryFile(
        mode="w", prefix="co_nav2_", suffix=".yaml", delete=False)
    yaml.safe_dump(params, handle, sort_keys=False)
    handle.close()
    navigation = os.path.join(bringup, "launch", "navigation_launch.py")
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation),
        launch_arguments={
            "params_file": handle.name,
            "use_sim_time": "false",
            "autostart": "true",
            "use_composition": "False",
        }.items(),
    )]


def generate_launch_description():
    package = get_package_share_directory("co_nav2_nav")
    return LaunchDescription([
        DeclareLaunchArgument(
            "overrides_file",
            default_value=os.path.join(package, "config", "nav2_overrides.yaml")),
        OpaqueFunction(function=configure_navigation),
    ])
