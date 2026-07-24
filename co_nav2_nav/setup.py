from setuptools import find_packages, setup

package_name = "co_nav2_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/semantic_exploration.launch.py",
            "launch/fastlio_mapping_2d.launch.py",
            "launch/nav2_navigation.launch.py",
            "launch/open_space_search.launch.py",
        ]),
        ("share/" + package_name + "/config", [
            "config/robot.yaml", "config/nav2_overrides.yaml",
            "config/nav2_open_space.yaml", "config/slam_toolbox.yaml",
        ]),
        ("share/" + package_name + "/behavior_trees", [
            "behavior_trees/open_space_no_recovery.xml",
            "behavior_trees/open_space_through_poses_no_recovery.xml",
        ]),
        ("share/" + package_name + "/rviz", [
            "rviz/co_nav2_validation.rviz",
        ]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="robot",
    maintainer_email="robot@example.com",
    description="FAST_LIO to Nav2 semantic frontier exploration integration",
    license="MIT",
    entry_points={"console_scripts": [
        "fastlio_odom_adapter = co_nav2_nav.fastlio_odom_adapter:main",
        "semantic_explorer = co_nav2_nav.semantic_explorer:main",
        "camera_sync_configurator = co_nav2_nav.camera_sync_configurator:main",
    ]},
)
