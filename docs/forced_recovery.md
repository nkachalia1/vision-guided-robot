# Forced Recovery Stress Test

This experiment intentionally gives the robot an unreachable navigation goal inside a large obstacle.

The goal is not to reach the goal. The goal is to prove that recovery behavior is triggered and observable in rosbag data.

```text
large blocker: x=1.2, y=0.4, size=1.20 m x 1.60 m
blocked goal:  x=1.2, y=0.4
```

## Copy Files Into WSL

```bash
cd ~/vision_guided_robot_ws

cp "/mnt/c/Users/Neel/Documents/New project/src/vision_guided_robot/launch/demo_forced_recovery_planned.launch.py" \
  src/vision_guided_robot/launch/

cp "/mnt/c/Users/Neel/Documents/New project/src/vision_guided_robot/launch/demo_forced_recovery_nav2.launch.py" \
  src/vision_guided_robot/launch/

cp "/mnt/c/Users/Neel/Documents/New project/src/vision_guided_robot/launch/demo_nav2.launch.py" \
  src/vision_guided_robot/launch/

cp "/mnt/c/Users/Neel/Documents/New project/src/vision_guided_robot/config/nav2_recovery_stress_params.yaml" \
  src/vision_guided_robot/config/

mkdir -p src/vision_guided_robot/models/recovery_blocker
cp -r "/mnt/c/Users/Neel/Documents/New project/src/vision_guided_robot/models/recovery_blocker/." \
  src/vision_guided_robot/models/recovery_blocker/

cp "/mnt/c/Users/Neel/Documents/New project/tools/analyze_bag.py" tools/
cp -r "/mnt/c/Users/Neel/Documents/New project/docs/." docs/
```

Build:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Custom Recovery Stress

Terminal 1:

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch vision_guided_robot demo_forced_recovery_planned.launch.py rviz:=true
```

Terminal 2:

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 bag record -o bags/final_forced_recovery_custom \
  /planner/state \
  /planner/clear_costmap \
  /planned_path \
  /planning/occupancy_grid \
  /waypoint/state \
  /waypoint/progress \
  /mission/state \
  /safety/state \
  /cmd_vel \
  /odom \
  /scan
```

Let it run long enough to see at least one recovery cycle, then stop the bag with `Ctrl+C`.

Analyze:

```bash
python3 tools/analyze_bag.py bags/final_forced_recovery_custom \
  --nav2-goal-x-m 1.2 \
  --nav2-goal-y-m 0.4
```

Look for:

```text
custom_recovery_detected: True
waypoint_state_counts:
  RECOVERING: ...
mission_state_counts:
  RECOVERING: ...
planner_state_counts:
  COSTMAP_CLEARED: ...
```

`success: False` is acceptable here because the goal is intentionally blocked.

## Nav2 Recovery Stress

This launch uses `config/nav2_recovery_stress_params.yaml`, which differs from the normal validated Nav2 config:

```text
NavfnPlanner.tolerance: 0.5 -> 0.0
xy_goal_tolerance: 0.12 -> 0.03
movement_time_allowance: 10.0 -> 4.0
```

Those values make this a recovery/abort test instead of a normal navigation test.

Terminal 1:

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch vision_guided_robot demo_forced_recovery_nav2.launch.py rviz:=true
```

Terminal 2:

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

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

Terminal 3:

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: odom}, pose: {position: {x: 1.2, y: 0.4, z: 0.0}, orientation: {w: 1.0}}}}"
```

Let Nav2 finish or visibly cycle through recovery, then stop the bag.

Analyze:

```bash
python3 tools/analyze_bag.py bags/final_forced_recovery_nav2 \
  --nav2-goal-x-m 1.2 \
  --nav2-goal-y-m 0.4 \
  --nav2-goal-tolerance-m 0.03
```

Look for either Nav2 behavior action topics:

```text
nav2_behavior_recovery_detected: True
action_status_topics:
  /backup/_action/status:
    ...
  /spin/_action/status:
    ...
```

or a failed navigation action:

```text
nav2_action_state_counts:
  ABORTED: ...
```

For this stress test, an abort is a valid result if recovery was attempted. The robot should not be expected to reach a goal that is inside a large obstacle.

## What This Teaches

Normal navigation validation asks:

```text
Can the robot reach the goal?
```

Recovery validation asks a different question:

```text
When the goal or path is invalid, does the system detect the problem and run its recovery machinery?
```

The custom stack exposes recovery directly through `RECOVERING` and `COSTMAP_CLEARED`. Nav2 exposes recovery through behavior-tree action topics such as `/backup/_action/status`, `/spin/_action/status`, `/wait/_action/status`, and `/drive_on_heading/_action/status`.
