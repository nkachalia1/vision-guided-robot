# SLAM Toolbox Mapping

This step teaches the robot to build a map from lidar instead of loading a hand-authored map.

Previous milestone:

```text
map_server -> /map
AMCL -> map -> odom
Nav2 -> map-frame goals
```

New milestone:

```text
Gazebo /scan + /odom
SLAM Toolbox -> /map and map -> odom
Nav2 -> map-frame goals while the map is being built
```

## Files

New files:

- `src/vision_guided_robot/launch/demo_slam.launch.py`
- `src/vision_guided_robot/config/slam_toolbox.yaml`
- `src/vision_guided_robot/config/nav2_slam_params.yaml`

Updated file:

- `src/vision_guided_robot/package.xml`

## Check Dependency

In Ubuntu:

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix slam_toolbox
```

If that says the package is not found:

```bash
sudo apt update
sudo apt install ros-humble-slam-toolbox
```

## Copy And Build

```bash
cd ~/vision_guided_robot_ws

cp "/path/to/source_mirror/src/vision_guided_robot/launch/demo_slam.launch.py" \
  src/vision_guided_robot/launch/

cp "/path/to/source_mirror/src/vision_guided_robot/config/slam_toolbox.yaml" \
  src/vision_guided_robot/config/

cp "/path/to/source_mirror/src/vision_guided_robot/config/nav2_slam_params.yaml" \
  src/vision_guided_robot/config/

cp "/path/to/source_mirror/src/vision_guided_robot/package.xml" \
  src/vision_guided_robot/

cp -r "/path/to/source_mirror/docs/." docs/

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Launch SLAM

Terminal 1:

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch vision_guided_robot demo_slam.launch.py rviz:=true
```

Wait for Gazebo, RViz, SLAM Toolbox, and Nav2 to start.

## Verify Mapping Topics

Terminal 2:

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 node list | grep -E "slam|planner|controller|bt_navigator"
ros2 topic list | grep -E "^/map$|/scan|/tf"
timeout 5 ros2 topic echo /map --field info
timeout 5 ros2 run tf2_ros tf2_echo map base_link
```

Healthy signs:

```text
/slam_toolbox exists
/map publishes
tf2_echo map base_link prints transforms
```

## Move The Robot To Build The Map

Start with slow manual motion. Hold each command for a few seconds, then stop it with `Ctrl+C`.

Forward:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.25}, angular: {z: 0.0}}"
```

Rotate left:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.6}}"
```

Rotate right:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: -0.6}}"
```

Stop:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

The RViz map should grow as the robot sees the walls from different angles.

## Optional Nav2 Goal During SLAM

After `/map` and `map -> base_link` are healthy, send a small clear goal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 0.8, y: -0.4, z: 0.0}, orientation: {w: 1.0}}}}"
```

This tests mapping and navigation together.

## Record A Validation Bag

```bash
ros2 bag record --include-hidden-topics -o bags/final_slam_mapping_first_run \
  /cmd_vel \
  /odom \
  /scan \
  /map \
  /tf \
  /plan \
  /navigate_to_pose/_action/status
```

Stop the bag with `Ctrl+C` after the map has grown and, ideally, after a short Nav2 goal succeeds.

Analyze:

```bash
python3 tools/analyze_bag.py bags/final_slam_mapping_first_run \
  --nav2-goal-x-m 0.8 \
  --nav2-goal-y-m -0.4 \
  --nav2-goal-tolerance-m 0.25
```

For the first SLAM milestone, the key evidence is:

```text
map_samples: > 0
map_size: nonzero
nav2_plan_samples: > 0 if you sent a Nav2 goal
nav2_success: True if the goal succeeded
final_map_base_xy_m: printed if /tf was recorded
nav2_goal_pose_source: map_tf
```

Important: once SLAM Toolbox publishes `map -> odom`, raw `/odom` is no longer a map-frame position. For SLAM bags, use the latest analyzer so goal error is computed from `/tf` as `map -> base_link`, not directly from `/odom`.

First validation bag:

```text
bags/final_slam_mapping_first_run
```

Initial evidence:

```text
map_samples: 233
map_size: 205x128 @ 0.050 m/px
nav2_plan_samples: 48
final_map_base_xy_m: (0.671, -0.284)
nav2_goal_error_m: 0.174
nav2_goal_pose_source: map_tf
nav2_action_state_counts:
  EXECUTING: 1
  SUCCEEDED: 1
nav2_odom_success: True
nav2_success: True
success: True
```

This validates live mapping plus Nav2 action completion.

## Save The Built Map

When the live map looks useful:

```bash
mkdir -p maps/slam

ros2 run nav2_map_server map_saver_cli \
  -f maps/slam/slam_two_wall_map
```

This creates:

```text
maps/slam/slam_two_wall_map.yaml
maps/slam/slam_two_wall_map.pgm
```

Saved-map evidence from the first run:

```text
maps/slam/slam_two_wall_map.pgm
maps/slam/slam_two_wall_map.yaml
```

The first saved map was small on disk, but that is normal in this compact world.

## Validate The SLAM Map With AMCL

The next step is to prove the generated map can replace the hand-authored map.

If Nav2 warns that the robot is out of map bounds, pad the saved map first:

```bash
cd ~/vision_guided_robot_ws

cp "/path/to/source_mirror/tools/pad_map.py" tools/

python3 tools/pad_map.py \
  maps/slam/slam_two_wall_map.yaml \
  maps/slam/slam_two_wall_map_padded.yaml \
  --pad-px 60
```

Use the padded map for AMCL validation:

```text
maps/slam/slam_two_wall_map_padded.yaml
```

Stop the SLAM launch, then launch AMCL/Nav2 using the generated map path:

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch vision_guided_robot demo_nav2_amcl.launch.py \
  map:=$(pwd)/maps/slam/slam_two_wall_map_padded.yaml \
  rviz:=true
```

For a much faster AMCL/Nav2 run on the generated map, use the tuned wall-pass Nav2 profile:

```bash
FAST_PARAMS=$(ros2 pkg prefix vision_guided_robot)/share/vision_guided_robot/config/nav2_map_wall_pass_params.yaml

ros2 launch vision_guided_robot demo_nav2_amcl.launch.py \
  map:=$(pwd)/maps/slam/slam_two_wall_map_padded.yaml \
  nav2_params_file:=$FAST_PARAMS \
  rviz:=true
```

This raises Nav2 from the conservative profile:

```text
0.60 m/s linear, 1.20 rad/s angular
```

to the fast profile:

```text
1.80 m/s linear, 3.00 rad/s angular
```

In a second terminal, seed AMCL near the origin:

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1]}}"

timeout 5 ros2 topic echo /amcl_pose --field pose.pose.position
timeout 5 ros2 run tf2_ros tf2_echo map base_link
```

Record a validation bag:

```bash
ros2 bag record --include-hidden-topics -o bags/final_slam_map_amcl_validation \
  /cmd_vel \
  /odom \
  /scan \
  /map \
  /amcl_pose \
  /tf \
  /plan \
  /navigate_to_pose/_action/status
```

Send a small clear goal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 0.8, y: -0.4, z: 0.0}, orientation: {w: 1.0}}}}"
```

Analyze:

```bash
python3 tools/analyze_bag.py bags/final_slam_map_amcl_validation \
  --nav2-goal-x-m 0.8 \
  --nav2-goal-y-m -0.4 \
  --nav2-goal-tolerance-m 0.30
```

This closes the loop:

```text
robot builds map -> map is saved -> AMCL localizes on that saved map -> Nav2 drives on it
```

## Fast AMCL-On-SLAM-Map Validation

Validated bag:

```text
bags/final_slam_map_amcl_fast_run2
```

Key evidence:

```text
map_size: 148x153 @ 0.050 m/px
amcl_pose_samples: 20
nav2_plan_samples: 29
odom_displacement_m: 0.953
final_map_base_xy_m: (0.992, -0.489)
max_linear_mps: 0.818
max_angular_radps: 1.000
nav2_goal_error_m: 0.236
nav2_goal_pose_source: map_tf
nav2_odom_success: True
nav2_success: True
success: True
```

This is the first complete robot-built-map loop:

```text
SLAM Toolbox map -> saved map -> padded map -> AMCL localization -> fast Nav2 goal
```

## Make The Generated Map A Project Artifact

Copy the saved padded map into the package maps folder:

```bash
cd ~/vision_guided_robot_ws

cp maps/slam/slam_two_wall_map_padded.yaml \
  src/vision_guided_robot/maps/

cp maps/slam/slam_two_wall_map_padded.pgm \
  src/vision_guided_robot/maps/
```

Copy the launch shortcut from the Windows source mirror:

```bash
cp "/path/to/source_mirror/src/vision_guided_robot/launch/demo_nav2_slam_map.launch.py" \
  src/vision_guided_robot/launch/

cp -r "/path/to/source_mirror/docs/." docs/
```

Build:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Then the robot-built map can be launched with one command:

```bash
ros2 launch vision_guided_robot demo_nav2_slam_map.launch.py rviz:=true
```

This launch is a shortcut for:

```text
demo_nav2_amcl.launch.py
map:=slam_two_wall_map_padded.yaml
nav2_params_file:=nav2_map_wall_pass_params.yaml
```

## Robotics Concepts

SLAM means simultaneous localization and mapping. The robot estimates where it is while also building the map it will use for navigation.

The important frame relationship is:

```text
map -> odom -> base_link
```

`odom -> base_link` comes from wheel/physics odometry. It is smooth but can drift. `map -> odom` comes from SLAM. It corrects accumulated drift by matching laser scans against the map.

Nav2 still needs a map-frame transform. The difference is that SLAM Toolbox provides it live, while AMCL provides it from a saved map.
