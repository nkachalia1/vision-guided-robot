# AMCL Nav2 Wall-Passing Experiment

This experiment keeps the validated saved-map AMCL stack, then tunes Nav2 for the harder map-frame goal:

```text
goal: x=2.0, y=0.8, frame=map
```

That goal forces the robot to pass around the mapped wall instead of only proving localization with a clear short goal.

## What Changed

New files:

- `src/vision_guided_robot/config/nav2_map_wall_pass_params.yaml`
- `src/vision_guided_robot/launch/demo_nav2_amcl_wall_pass.launch.py`

The original AMCL validation launch remains unchanged:

```text
demo_nav2_amcl.launch.py
```

Use that original launch when you want the known-good localization baseline. Use the wall-pass launch for this harder navigation tuning.

## Tuning Intent

The wall-pass config changes the navigation behavior in five ways:

1. Uses the saved static map for the global plan and keeps lidar obstacle marking in the local costmap.
2. Enlarges the local costmap from `3 m x 3 m` to `4 m x 4 m` so DWB sees more of the nearby obstacle geometry.
3. Raises the Nav2 command limits to `1.80 m/s` linear and `3.00 rad/s` angular.
4. Uses stronger acceleration/deceleration limits for faster response.
5. Makes the progress checker more patient so the controller does not fall into slow recovery loops.
6. Reduces DWB sample count and controller frequency so the WSL simulation has less control-loop CPU load.
7. Reduces inflation and path-alignment dominance so the planner/controller is less likely to reject the tight wall-passing route.

The first wall-pass run aborted with:

```text
GridBased: failed to create plan with tolerance 0.18.
Planning algorithm GridBased failed to generate a valid path to (2.00, 0.80)
```

That points to global planning/costmap feasibility, not AMCL. The current wall-pass config therefore disables the global scan obstacle layer and raises planner tolerance to `0.50 m`, while preserving local lidar collision checking.

After the first retry still aborted, the current fast wall-pass profile became more aggressive:

```text
controller_frequency: 10.0
progress_checker.required_movement_radius: 0.03
progress_checker.movement_time_allowance: 30.0
general_goal_checker.yaw_goal_tolerance: 3.14
FollowPath.max_vel_x: 1.80
FollowPath.max_vel_theta: 3.00
FollowPath.acc_lim_x: 5.0
FollowPath.acc_lim_theta: 8.0
velocity_smoother.max_velocity: [1.80, 0.0, 3.00]
planner tolerance: 0.50
goal xy tolerance: 0.22
inflation radius: 0.16
```

The next retry reached controller execution but repeatedly printed:

```text
controller_server: Failed to make progress
```

That points to controller progress/recovery behavior rather than global planning. The current config therefore makes progress checking more tolerant, reduces controller computation load, and removes rotate-to-goal pressure so intermediate waypoints are treated as positions rather than places where the robot must face a specific heading.

The next retries reached the second-wall area but oscillated left/right and then clipped the wall. That indicates the route and local costmap were allowing too little clearance. The current config restores the real footprint in costmaps, increases local inflation, increases `BaseObstacle` cost, and moves the staged route farther below and right of the second wall.

## Copy Files Into WSL

```bash
cd ~/vision_guided_robot_ws

cp "/mnt/c/Users/Neel/Documents/New project/src/vision_guided_robot/config/nav2_map_wall_pass_params.yaml" \
  src/vision_guided_robot/config/

cp "/mnt/c/Users/Neel/Documents/New project/src/vision_guided_robot/launch/demo_nav2_amcl_wall_pass.launch.py" \
  src/vision_guided_robot/launch/

cp -r "/mnt/c/Users/Neel/Documents/New project/docs/." docs/
```

Build:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Launch The Experiment

Terminal 1:

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch vision_guided_robot demo_nav2_amcl_wall_pass.launch.py rviz:=true
```

Wait until Gazebo, RViz, AMCL, and Nav2 are up.

## Seed AMCL If Needed

Terminal 2:

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1]}}"

timeout 5 ros2 topic echo /amcl_pose --field pose.pose.position
```

You should see `/amcl_pose` print a position near the origin.

## Record The Bag

Terminal 3:

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 bag record --include-hidden-topics -o bags/final_nav2_amcl_wall_pass \
  /cmd_vel \
  /odom \
  /scan \
  /map \
  /amcl_pose \
  /particle_cloud \
  /plan \
  /local_costmap/costmap \
  /global_costmap/costmap \
  /navigate_to_pose/_action/status \
  /backup/_action/status \
  /spin/_action/status \
  /wait/_action/status \
  /drive_on_heading/_action/status
```

Start recording before sending the goal.

## Send The Hard Goal

Terminal 2:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.8, z: 0.0}, orientation: {w: 1.0}}}}"
```

Let it finish, abort, or clearly get stuck. Then stop the bag with `Ctrl+C`.

## Send A Staged Wall-Passing Route

If the single hard goal still aborts with `failed to create plan`, use a staged route. For a clean start near the origin, use smaller first steps so Nav2 does not need to immediately commit to a deep detour:

```bash
ros2 action send_goal /navigate_through_poses nav2_msgs/action/NavigateThroughPoses \
  "{poses: [
    {header: {frame_id: map}, pose: {position: {x: 0.45, y: -0.45, z: 0.0}, orientation: {w: 1.0}}},
    {header: {frame_id: map}, pose: {position: {x: 0.90, y: -1.05, z: 0.0}, orientation: {w: 1.0}}},
    {header: {frame_id: map}, pose: {position: {x: 1.25, y: -1.45, z: 0.0}, orientation: {w: 1.0}}},
    {header: {frame_id: map}, pose: {position: {x: 2.65, y: -1.45, z: 0.0}, orientation: {w: 1.0}}},
    {header: {frame_id: map}, pose: {position: {x: 2.65, y: 1.10, z: 0.0}, orientation: {w: 1.0}}},
    {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.8, z: 0.0}, orientation: {w: 1.0}}}
  ]}"
```

If the robot is already near the second wall from a previous attempt, do not send the clean-start route. Its first waypoint is behind the robot, which can force impossible backtracking. Instead, send a continuation route:

```bash
ros2 action send_goal /navigate_through_poses nav2_msgs/action/NavigateThroughPoses \
  "{poses: [
    {header: {frame_id: map}, pose: {position: {x: 2.35, y: 0.25, z: 0.0}, orientation: {w: 1.0}}},
    {header: {frame_id: map}, pose: {position: {x: 2.35, y: 0.95, z: 0.0}, orientation: {w: 1.0}}},
    {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.8, z: 0.0}, orientation: {w: 1.0}}}
  ]}"
```

This is not cheating. It is a navigation lesson: a global goal can be valid but still hard for a planner/controller pair to reach through a tight layout. Intermediate poses are a standard way to constrain the route, and the first intermediate pose must make sense from the robot's current pose.

## Analyze

```bash
python3 tools/analyze_bag.py bags/final_nav2_amcl_wall_pass \
  --nav2-goal-x-m 2.0 \
  --nav2-goal-y-m 0.8 \
  --nav2-goal-tolerance-m 0.22
```

Strong success looks like:

```text
amcl_pose_samples: > 0
nav2_plan_samples: > 0
nav2_action_state_counts:
  EXECUTING: ...
  SUCCEEDED: ...
nav2_goal_error_m: <= 0.180
nav2_success: True
success: True
```

## Current Successful Route

The widened route below is the first user-observed successful hard wall-pass route. The action finished with:

```text
Goal finished with status: SUCCEEDED
```

Validated bag:

```text
bags/final_nav2_amcl_wall_pass_success
```

Then analyze it with:

```bash
python3 tools/analyze_bag.py bags/final_nav2_amcl_wall_pass_success \
  --nav2-goal-x-m 2.0 \
  --nav2-goal-y-m 0.8 \
  --nav2-goal-tolerance-m 0.22
```

Key result:

```text
/navigate_through_poses/_action/status:
  EXECUTING: 1
  SUCCEEDED: 1
amcl_pose_samples: 70
nav2_plan_samples: 12
max_linear_mps: 1.800
max_angular_radps: 2.200
motion_command_span_s: 33.65
final_odom_xy_m: (2.372, 0.787)
nav2_goal_error_m: 0.372
```

This validates the wall-passing behavior. It does not validate tight final-position accuracy, because the final odom pose is `0.372 m` from the nominal `(2.0, 0.8)` target. That is expected after raising planner tolerance to make the tight wall route feasible. If precise final pose matters, run a second correction goal after the wall-pass route or lower planner tolerance once the robot is past the wall.

Analyzer note: the current `tools/analyze_bag.py` counts both `/navigate_to_pose/_action/status` and `/navigate_through_poses/_action/status` as main Nav2 action evidence. Copy the latest analyzer before re-running old bags if the result does not show `nav2_success: True`.

Useful failure evidence looks like:

```text
nav2_action_state_counts:
  ABORTED: ...
```

or behavior action evidence:

```text
nav2_behavior_recovery_detected: True
```

## Robotics Lesson

AMCL answering "where am I?" does not automatically mean Nav2 can pass a tight obstacle layout. This experiment separates localization from navigation:

```text
Localization success: /map, /amcl_pose, map -> odom
Navigation success: /plan, /cmd_vel, action SUCCEEDED, final pose near goal
```

If localization is healthy but the robot still stalls near the wall, the next tuning target is the controller/costmap/planner interaction, not AMCL.
