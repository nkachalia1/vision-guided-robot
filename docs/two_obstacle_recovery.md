# Two-Obstacle Recovery Comparison

This experiment compares the custom planned-navigation recovery behavior against Nav2 in the same harder Gazebo world.

The world contains two static wall obstacles:

```text
wall 1: x=1.2, y=0.4
wall 2: x=1.6, y=-0.45
goal:   x=2.0, y=0.8
```

Both stacks use the same robot, `/scan`, `/odom`, `/tf`, and Gazebo obstacle geometry.

## Copy Files Into WSL

```bash
cd ~/vision_guided_robot_ws

cp "/path/to/source_mirror/src/vision_guided_robot/launch/sim.launch.py" \
  src/vision_guided_robot/launch/

cp "/path/to/source_mirror/src/vision_guided_robot/launch/demo_live_planned.launch.py" \
  src/vision_guided_robot/launch/

cp "/path/to/source_mirror/src/vision_guided_robot/launch/demo_nav2.launch.py" \
  src/vision_guided_robot/launch/

cp "/path/to/source_mirror/src/vision_guided_robot/launch/demo_two_obstacle_planned.launch.py" \
  src/vision_guided_robot/launch/

cp "/path/to/source_mirror/src/vision_guided_robot/launch/demo_two_obstacle_nav2.launch.py" \
  src/vision_guided_robot/launch/

cp "/path/to/source_mirror/tools/analyze_bag.py" tools/
cp -r "/path/to/source_mirror/docs/." docs/
```

Build:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Custom Planner Run

Terminal 1:

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch vision_guided_robot demo_two_obstacle_planned.launch.py rviz:=true
```

Terminal 2:

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 bag record -o bags/final_two_obstacle_custom \
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

If the robot reaches the goal without needing recovery, trigger recovery once during motion:

```bash
ros2 topic pub --once /recovery/trigger std_msgs/msg/Empty "{}"
```

Analyze:

```bash
python3 tools/analyze_bag.py bags/final_two_obstacle_custom
```

Look for:

```text
planner_success: True
success: True
waypoint_state_counts:
  FOLLOW_PATH: ...
  RECOVERING: ...
  DONE: ...
planner_state_counts:
  COSTMAP_CLEARED: ...
```

`RECOVERING` and `COSTMAP_CLEARED` prove the custom recovery sequence actually ran.

## Nav2 Run

Terminal 1:

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch vision_guided_robot demo_two_obstacle_nav2.launch.py rviz:=true
```

Terminal 2:

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

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

Terminal 3:

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: odom}, pose: {position: {x: 2.0, y: 0.8, z: 0.0}, orientation: {w: 1.0}}}}"
```

Analyze:

```bash
python3 tools/analyze_bag.py bags/final_two_obstacle_nav2 \
  --nav2-goal-tolerance-m 0.12
```

Look for:

```text
nav2_action_state_counts:
  EXECUTING: ...
  SUCCEEDED: ...
nav2_goal_error_m: <= 0.120
nav2_success: True
success: True
```

If Nav2 recovery behaviors run, the analyzer will also print action status topics such as:

```text
action_status_topics:
  /backup/_action/status:
    EXECUTING: ...
    SUCCEEDED: ...
  /spin/_action/status:
    EXECUTING: ...
    SUCCEEDED: ...
```

## Compare

Use these questions:

1. Did both systems reach the same goal?
2. Which one produced a shorter `motion_command_span_s`?
3. Which one got closer to `(2.0, 0.8)`?
4. Did custom recovery run?
5. Did Nav2 behavior-tree recovery run?
6. Did either planner get stuck or oscillate?

The learning goal is not only which system wins. The useful lesson is seeing the same concepts twice: once in the custom stack where every piece is visible, and once in Nav2 where the same ideas are packaged into a production ROS navigation architecture.

## Validated Result

Both systems reached the `(2.0, 0.8)` goal in the two-obstacle world.

| Metric | Custom Planner | Nav2 |
| --- | ---: | ---: |
| Bag | `final_two_obstacle_custom` | `final_two_obstacle_nav2` |
| Duration | 51.38 s | 70.45 s |
| Final odom | `(1.925, 0.848)` | `(1.912, 0.718)` |
| Goal error | ~0.089 m | 0.120 m |
| Odom displacement | 1.928 m | 2.043 m |
| Max linear speed | 0.900 m/s | 0.600 m/s |
| Max angular speed | 1.542 rad/s | 0.442 rad/s |
| Average linear command | 0.332 m/s | 0.313 m/s |
| Average angular command | 0.313 rad/s | 0.117 rad/s |
| Motion command span | 31.11 s | 42.10 s |
| Plan samples | 17 custom `/planned_path` | 41 Nav2 `/plan` |
| Main state | `FOLLOW_PATH -> DONE` | `EXECUTING -> SUCCEEDED` |
| Recovery evidence | None | None |
| Success | True | True |

This validates the harder two-obstacle navigation scenario for both stacks. It does not prove recovery behavior in this layout because neither system needed recovery. The next harder experiment should intentionally force recovery with a temporarily blocked path, a tighter obstacle gap, or a dead-end/trap layout.
