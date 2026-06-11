# Map Localization With AMCL

This milestone moves Nav2 from an `odom`-only demo to a standard map-based navigation stack:

```text
map_server -> /map
AMCL + /scan + /odom -> map -> odom transform
Nav2 goal in map frame -> planner/controller -> /cmd_vel
```

The first map is intentionally small. It contains the same two wall obstacles used in the previous custom-vs-Nav2 navigation comparison.

## Files

- `src/vision_guided_robot/maps/two_wall_map.yaml`: saved occupancy map metadata
- `src/vision_guided_robot/maps/two_wall_map.pgm`: saved occupancy map image
- `src/vision_guided_robot/config/nav2_map_params.yaml`: Nav2 + AMCL map-frame parameters
- `src/vision_guided_robot/launch/demo_nav2_amcl.launch.py`: Gazebo + map_server + AMCL + Nav2 launch
- `src/vision_guided_robot/rviz/nav2_map.rviz`: RViz layout with fixed frame `map`

## Copy Files Into WSL

```bash
cd ~/vision_guided_robot_ws

cp "/path/to/source_mirror/src/vision_guided_robot/setup.py" \
  src/vision_guided_robot/

cp "/path/to/source_mirror/src/vision_guided_robot/launch/demo_nav2_amcl.launch.py" \
  src/vision_guided_robot/launch/

cp "/path/to/source_mirror/src/vision_guided_robot/config/nav2_map_params.yaml" \
  src/vision_guided_robot/config/

mkdir -p src/vision_guided_robot/maps
cp -r "/path/to/source_mirror/src/vision_guided_robot/maps/." \
  src/vision_guided_robot/maps/

cp "/path/to/source_mirror/src/vision_guided_robot/rviz/nav2_map.rviz" \
  src/vision_guided_robot/rviz/

cp "/path/to/source_mirror/tools/analyze_bag.py" tools/
cp -r "/path/to/source_mirror/docs/." docs/
```

Build:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Launch

Terminal 1:

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch vision_guided_robot demo_nav2_amcl.launch.py rviz:=true
```

This spawns two Gazebo wall landmarks that match the saved map.

Gazebo publishes the laser scan with this sensor frame:

```text
vision_bot/lidar_link/front_lidar
```

The launch also publishes a static transform:

```text
lidar_link -> vision_bot/lidar_link/front_lidar
```

That connects the scan frame to the existing robot TF tree so AMCL can transform laser hits into `base_link`.

## Check Localization

Terminal 2:

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 topic echo --once /map --qos-durability transient_local --field info
ros2 topic echo --once /amcl_pose --field pose.pose.position
ros2 run tf2_ros tf2_echo map odom
```

Expected signs:

```text
/map_server: active
/amcl: active
/map publishes a 120 x 120 map
/amcl_pose publishes a pose near the robot
tf2_echo map odom prints a transform
```

If `/amcl_pose` is not moving or looks wrong, publish an initial pose:

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1]}}"
```

## Send A First Map-Frame Goal

First, validate localization with a short goal in front of the wall:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 0.8, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

This is different from the earlier Nav2 tests. The goal is now in `map`, not `odom`.

After that succeeds, use the tuned wall-passing experiment for the harder through-wall navigation goal:

```bash
ros2 launch vision_guided_robot demo_nav2_amcl_wall_pass.launch.py rviz:=true
```

Then send:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.8, z: 0.0}, orientation: {w: 1.0}}}}"
```

Full commands are in `docs/nav2_amcl_wall_passing.md`.

If the harder goal gets stuck near the first wall, that is a navigation/controller tuning issue, not an AMCL issue. Record the short goal first so the localization milestone has a clean pass.

## Record A Validation Bag

Terminal 3:

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 bag record --include-hidden-topics -o bags/final_nav2_amcl_goal \
  /cmd_vel \
  /odom \
  /scan \
  /map \
  /amcl_pose \
  /particle_cloud \
  /plan \
  /navigate_to_pose/_action/status
```

Start the bag before sending the goal. Stop it after the action succeeds or aborts.

Analyze:

```bash
python3 tools/analyze_bag.py bags/final_nav2_amcl_goal \
  --nav2-goal-x-m 0.8 \
  --nav2-goal-y-m 0.0 \
  --nav2-goal-tolerance-m 0.18
```

Useful evidence:

```text
map_samples: ...
amcl_pose_samples: ...
nav2_plan_samples: ...
nav2_action_state_counts:
  SUCCEEDED: ...
```

## Validated Result

The stronger AMCL validation run is:

```text
bags/final_nav2_amcl_clear_goal
```

Command:

```bash
python3 tools/analyze_bag.py bags/final_nav2_amcl_clear_goal \
  --nav2-goal-x-m 0.8 \
  --nav2-goal-y-m -0.6 \
  --nav2-goal-tolerance-m 0.18
```

Result:

```text
map_samples: 1
amcl_pose_samples: 37
map_size: 120x120 @ 0.050 m/px
initial_odom_xy_m: (-0.000, -0.000)
final_odom_xy_m: (0.714, -0.515)
odom_displacement_m: 0.881
initial_amcl_xy_m: (0.000, 0.000)
final_amcl_xy_m: (0.759, -0.529)
nav2_plan_samples: 47
nav2_action_state_counts:
  EXECUTING: 1
  SUCCEEDED: 1
nav2_goal_error_m: 0.120
nav2_goal_tolerance_m: 0.180
nav2_success: True
success: True
```

This validates the saved-map localization stack:

```text
map_server works
AMCL publishes pose estimates over time
map -> odom -> base_link is usable by Nav2
Nav2 accepts and completes map-frame goals
```

## Concepts

`odom` is locally smooth but drifts over time.

`map` is globally meaningful but needs localization to connect it to the robot.

AMCL estimates:

```text
map -> odom
```

Gazebo odometry still provides:

```text
odom -> base_link
```

Together, TF can answer:

```text
map -> base_link
```

That is the transform Nav2 needs for map-frame planning.
