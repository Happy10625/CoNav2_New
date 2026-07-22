# co_nav2_nav

ROS 2 integration layer between FAST_LIO, Nav2 and Co-NavGPT2. It does not
modify FAST_LIO's estimator and does not command the base directly.

## Build

Place `Co-NavGPT2/co_nav2_nav` under a ROS 2 workspace `src` directory, then:

```bash
colcon build --packages-select co_nav2_nav
source install/setup.bash
```

On systems combining the older ROS 2 Humble `colcon` Python plugin with recent
Setuptools (for example Setuptools 82), do not add `--symlink-install`. That
combination invokes the removed `setup.py develop --editable` option. A normal
install is supported; rebuild after changing Python source files.

This repository targets the native ROS 2 `FAST_LIO_ROS2` package. No ROS 1
bridge is used. FAST_LIO publishes `/Odometry`, `/cloud_registered`, and
`camera_init -> body` directly in ROS 2.

## Bring-up order

1. Start the base, Livox ROS 2 driver and FAST_LIO_ROS2:

   ```bash
   ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml rviz:=false
   ```
2. Start the 2-D mapping stack and Nav2. Merge `config/nav2_overrides.yaml` into
   that stack's complete parameter file; it is intentionally only an override.
3. Verify there is exactly one publisher for `odom -> base_link` and one for
   `map -> odom`.
4. Start exploration without the camera:

```bash
ros2 launch co_nav2_nav semantic_exploration.launch.py \
  enable_perception:=false publish_map_odom:=true
```

Set `publish_map_odom:=false` when the mapping stack already publishes that TF.
The odometry adapter defaults to disabled because `ranger_base_node` currently
publishes `odom -> base_link`. To use FAST_LIO as the Nav2 localization source,
first disable the ranger odometry TF publisher, then enable the adapter. Never
allow both nodes to publish the same transform:

```bash
ros2 launch co_nav2_nav semantic_exploration.launch.py \
  enable_odom_adapter:=true publish_lidar_tf:=false publish_map_odom:=false
```

The adapter performs the rigid transform from FAST_LIO `camera_init/body` to
Nav2 `odom/base_link`; it does not merely rename frames. Its default
`base_to_body_xyz/rpy` values are derived from the current Mid-360 mounting and
FAST_LIO extrinsic and must be rechecked after any sensor mounting change.
When enabled, the launch file also publishes the identity `odom -> camera_init`
required to transform FAST_LIO's registered cloud into the Nav2 TF tree.

If RViz proves that the Livox driver already aligns sensor +X with vehicle +X,
change the `base_to_livox` yaw in the launch file from pi to zero.

The measured RGB-D camera transform is included in
`semantic_exploration.launch.py`: `base_link -> camera_link` is translation
`[-0.20, 0.0, 1.215]` metres with zero roll/pitch/yaw. The camera is level and
faces vehicle +X. The launch publishes this transform by default; use
`publish_camera_tf:=false` only when another node already owns the same TF.
Verify `map -> camera_color_optical_frame` at image timestamps, then enable the
semantic pipeline:

```bash
ros2 launch co_nav2_nav semantic_exploration.launch.py enable_perception:=true
```

The model files `yolov8l-world.pt` and `mobile_sam.pt` must be available to
Ultralytics. State is published on `/semantic_explorer/state`; frontier and
target markers are on `/semantic_explorer/markers`.

## Safety gates

Before autonomous exploration, send ten manual Nav2 goals at the limited speeds
and verify emergency stop and goal cancellation. Do not enable perception until
RGB/depth alignment and the camera static transform have been measured.
