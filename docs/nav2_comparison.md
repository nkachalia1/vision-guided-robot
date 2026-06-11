# Nav2 Comparison Milestone

This milestone compares the custom planner stack against Nav2, the standard ROS 2 navigation framework.

The goal is not to throw away the custom planner. The goal is to understand what Nav2 gives you after you have already built the pieces yourself:

```text
custom stack:
/scan + /odom -> grid_planner -> /planned_path -> waypoint_driver -> /cmd_vel_raw -> safety_filter -> /cmd_vel

Nav2 stack:
/scan + /odom + /tf -> Nav2 costmaps -> Nav2 planner -> Nav2 controller -> /cmd_vel
```

## What This Adds

New files:

- `launch/demo_nav2.launch.py`: starts Gazebo, disables the custom controllers, then starts Nav2
- `launch/demo_two_obstacle_nav2.launch.py`: starts the matched two-obstacle Nav2 comparison scenario
- `config/nav2_params.yaml`: first odom-frame Nav2 configuration for this robot
- `docs/nav2_comparison.md`: this guide
- `docs/two_obstacle_recovery.md`: step-by-step custom-vs-Nav2 recovery comparison guide

Updated files:

- `launch/sim.launch.py`: adds `spawn_target`, `enable_perception`, and `enable_safety_filter` switches
- `package.xml`: adds Nav2 runtime dependencies
- `tools/analyze_bag.py`: reports Nav2 action status and optional behavior action status topics

## Why Odom-Frame Nav2 First?

Full Nav2 usually uses a map and localization:

```text
map -> AMCL or SLAM -> map->odom transform -> Nav2
```

For the first comparison, this project uses an odom-frame Nav2 demo:

```text
odom -> base_link
/scan -> local/global rolling costmaps
goal in odom frame -> Nav2 NavigateToPose
```

That avoids adding SLAM or AMCL before you have seen Nav2 plan and control the existing robot. It is a learning bridge, not the final production setup.

## Install Nav2

In Ubuntu:

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

## Copy Files Into WSL

```bash
cd ~/vision_guided_robot_ws

cp "/path/to/source_mirror/src/vision_guided_robot/launch/sim.launch.py" \
  src/vision_guided_robot/launch/

cp "/path/to/source_mirror/src/vision_guided_robot/launch/demo_nav2.launch.py" \
  src/vision_guided_robot/launch/

cp "/path/to/source_mirror/src/vision_guided_robot/config/nav2_params.yaml" \
  src/vision_guided_robot/config/

cp "/path/to/source_mirror/src/vision_guided_robot/package.xml" \
  src/vision_guided_robot/

cp -r "/path/to/source_mirror/docs/." docs/
```

Build:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install
source install/setup.bash
```

## Launch Nav2 Demo

```bash
ros2 launch vision_guided_robot demo_nav2.launch.py
```

Expected startup:

```text
Gazebo opens
RViz opens
/scan publishes
/odom publishes
/tf contains odom -> base_link
Nav2 lifecycle nodes become active
```

If the launch exits with this error:

```text
name 'false' is not defined
```

copy the latest `demo_nav2.launch.py`, rebuild, and relaunch. The included Nav2 launch path is picky here and expects `True`/`False` for its Python launch booleans.

The launch intentionally disables:

- `ball_tracker`
- `visual_servo`
- `waypoint_driver`
- `grid_planner`
- `safety_filter`

Nav2 owns `/cmd_vel` in this demo. This avoids two controllers fighting over the same robot velocity topic.

## Check Nav2 Is Alive

In a second Ubuntu terminal:

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 node list | grep -E "bt_navigator|planner_server|controller_server|behavior_server"
ros2 topic list | grep -E "scan|odom|cmd_vel|costmap|plan"
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
```

Expected lifecycle result:

```text
active [3]
```

## Send A First Nav2 Goal

Use a modest goal first:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: odom}, pose: {position: {x: 2.0, y: 0.8, z: 0.0}, orientation: {w: 1.0}}}}"
```

Watch:

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /plan --once
```

Expected behavior:

1. Nav2 accepts the goal.
2. `/plan` publishes a path.
3. `/cmd_vel` publishes velocity commands.
4. The robot moves toward the goal while reacting to the wall in the rolling costmap.

Validated smoke-test result:

```text
/navigate_to_pose action server appeared
Goal accepted
controller_server: Reached the goal!
bt_navigator: Goal succeeded
action result: SUCCEEDED
```

## Record A Nav2 Bag

```bash
ros2 bag record -o bags/final_nav2_first_goal \
  /cmd_vel \
  /odom \
  /scan \
  /plan \
  /local_costmap/costmap \
  /global_costmap/costmap \
  /navigate_to_pose/_action/status
```

Analyze:

```bash
python3 tools/analyze_bag.py bags/final_nav2_first_goal
```

Look for:

```text
nav2_action_state_counts:
  SUCCEEDED: ...
nav2_success: True
success: True
```

If the action status topic was not captured, the analyzer can still validate the default Nav2 goal from odometry:

```text
nav2_goal_error_m: <= 0.180
nav2_odom_success: True
nav2_success: True
success: True
```

Validated first bag:

```text
bags/final_nav2_first_goal_run2
nav2_plan_samples: 54
final_odom_xy_m: (1.864, 0.688)
distance to default goal (2.0, 0.8): ~0.176 m
action command result: SUCCEEDED
```

This bag is the first comparison point against:

- `bags/final_pure_pursuit_planner`
- `bags/final_persistent_costmap_planner`
- `bags/final_recovery_behavior_run2`

## Debug Checklist

If Nav2 does not move the robot:

```bash
ros2 topic info /cmd_vel -v
ros2 topic echo /cmd_vel
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
```

Expected:

- `/cmd_vel` should have one publisher from Nav2 in this demo
- `/scan` should publish lidar data
- `/odom` should publish robot pose
- `tf2_echo odom base_link` should show a transform

If Nav2 is not active:

```bash
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
```

If the package is missing:

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## Concepts To Learn

- lifecycle nodes: Nav2 nodes must configure and activate before doing work
- action servers: Nav2 goals are sent through `/navigate_to_pose`, not just a velocity topic
- global costmap: obstacle grid used by the planner
- local costmap: obstacle grid used by the controller
- planner server: computes a path
- controller server: follows a path
- behavior server: handles recovery behaviors like backup and spin
- behavior tree navigator: coordinates planning, control, and recovery

## Next Nav2 Steps

1. Add a second obstacle and compare custom recovery against Nav2 behavior-tree recovery.
2. Create the final navigation comparison table across custom, baseline Nav2, fast Nav2, and tight Nav2.
3. Continue from the validated saved-map AMCL demo into either harder map-frame wall-passing or online mapping with SLAM Toolbox.

## Two-Obstacle Recovery Comparison

The next comparison scenario uses the same two wall obstacles for both stacks:

```text
wall 1: x=1.2, y=0.4
wall 2: x=1.6, y=-0.45
goal:   x=2.0, y=0.8
```

Launch Nav2:

```bash
ros2 launch vision_guided_robot demo_two_obstacle_nav2.launch.py rviz:=true
```

Record with hidden action topics so Nav2 recovery actions can be detected:

```bash
ros2 bag record --include-hidden-topics -o bags/final_two_obstacle_nav2 \
  /cmd_vel \
  /odom \
  /scan \
  /plan \
  /local_costmap/costmap \
  /global_costmap/costmap \
  /navigate_to_pose/_action/status \
  /backup/_action/status \
  /spin/_action/status \
  /wait/_action/status \
  /drive_on_heading/_action/status
```

Send the goal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: odom}, pose: {position: {x: 2.0, y: 0.8, z: 0.0}, orientation: {w: 1.0}}}}"
```

Analyze:

```bash
python3 tools/analyze_bag.py bags/final_two_obstacle_nav2 \
  --nav2-goal-tolerance-m 0.12
```

The matching custom-planner procedure is in `docs/two_obstacle_recovery.md`.

## Forced Nav2 Recovery Stress Test

If the two-obstacle scenario does not trigger recovery, run the blocked-goal stress test:

```bash
ros2 launch vision_guided_robot demo_forced_recovery_nav2.launch.py rviz:=true
```

This launch uses `config/nav2_recovery_stress_params.yaml`, which sets exact planner tolerance and a tighter `0.03 m` goal tolerance so Nav2 cannot mark a nearby pose as success.

Record with hidden behavior action topics:

```bash
ros2 bag record --include-hidden-topics -o bags/final_forced_recovery_nav2 \
  /cmd_vel \
  /odom \
  /scan \
  /plan \
  /local_costmap/costmap \
  /global_costmap/costmap \
  /navigate_to_pose/_action/status \
  /backup/_action/status \
  /spin/_action/status \
  /wait/_action/status \
  /drive_on_heading/_action/status
```

Send the intentionally blocked goal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: odom}, pose: {position: {x: 1.2, y: 0.4, z: 0.0}, orientation: {w: 1.0}}}}"
```

Analyze:

```bash
python3 tools/analyze_bag.py bags/final_forced_recovery_nav2 \
  --nav2-goal-x-m 1.2 \
  --nav2-goal-y-m 0.4 \
  --nav2-goal-tolerance-m 0.03
```

Look for `nav2_behavior_recovery_detected: True` or an `ABORTED` navigation action after recovery attempts.

## Faster Nav2 Tuning

After the first successful Nav2 comparison, the default Nav2 config was tuned slightly faster:

```text
FollowPath.max_vel_x: 0.45 -> 0.60
FollowPath.max_speed_xy: 0.45 -> 0.60
FollowPath.acc_lim_x: 0.8 -> 1.0
velocity_smoother.max_velocity: [0.45, 0.0, 1.20] -> [0.60, 0.0, 1.20]
```

Record the faster comparison bag as:

```text
bags/final_nav2_fast_goal
```

First faster-run result:

```text
bags/final_nav2_fast_goal
duration_s: 84.17
nav2_plan_samples: 45
final_odom_xy_m: (1.863, 0.692)
nav2_goal_error_m: 0.175
max_linear_mps: 0.600
max_angular_radps: 0.442
success: True
```

The faster config did increase the commanded velocity. Total bag duration was longer because recording started earlier before motion, but actual command-motion time improved:

```text
baseline motion_command_span_s: 55.25
fast motion_command_span_s:     45.50
baseline avg_abs_linear_mps:    0.282
fast avg_abs_linear_mps:        0.309
```

## Tighter Nav2 Goal Tolerance

After validating the faster config, the next tuning step tightens the final position tolerance:

```text
general_goal_checker.xy_goal_tolerance: 0.18 -> 0.12
FollowPath.xy_goal_tolerance: 0.18 -> 0.12
```

Keep the faster speed settings. Record this bag as:

```text
bags/final_nav2_tight_goal
```

Analyze it with the tighter tolerance:

```bash
python3 tools/analyze_bag.py bags/final_nav2_tight_goal \
  --nav2-goal-tolerance-m 0.12
```

Look for:

```text
nav2_goal_error_m: <= 0.120
nav2_goal_tolerance_m: 0.120
nav2_odom_success: True
success: True
```

Validated tight-goal result:

```text
bags/final_nav2_tight_goal
duration_s: 76.71
nav2_plan_samples: 42
nav2_action_state_counts:
  EXECUTING: 1
  SUCCEEDED: 1
final_odom_xy_m: (1.913, 0.720)
nav2_goal_error_m: 0.118
nav2_goal_tolerance_m: 0.120
motion_command_span_s: 42.60
max_linear_mps: 0.600
max_angular_radps: 0.442
success: True
```

This is the best Nav2 result so far. It kept the faster `0.60 m/s` speed, improved the final goal error from `0.175 m` to `0.118 m`, and captured the action status transition from `EXECUTING` to `SUCCEEDED`.
