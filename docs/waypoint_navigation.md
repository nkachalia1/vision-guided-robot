# Waypoint Navigation

This step adds a simple odometry-based waypoint driver.

Unlike the vision controller, the waypoint driver does not look for the ball. It uses `/odom` to drive the robot toward a goal in the odom frame:

```text
goal = (x, y, yaw)
```

The command path stays safety-aware:

```text
waypoint_driver -> /cmd_vel_raw -> safety_filter -> /cmd_vel -> Gazebo
```

The node can receive goals two ways:

- launch parameters: `goal_x_m`, `goal_y_m`, `goal_yaw_rad`
- RViz: `geometry_msgs/msg/PoseStamped` on `/goal_pose`

## Run A Waypoint Test

```bash
ros2 launch vision_guided_robot sim.launch.py \
  control_mode:=waypoint \
  goal_x_m:=2.0 \
  goal_y_m:=0.0 \
  rviz:=true
```

Try an off-axis goal:

```bash
ros2 launch vision_guided_robot sim.launch.py \
  control_mode:=waypoint \
  goal_x_m:=2.0 \
  goal_y_m:=1.0 \
  rviz:=true
```

Try a final yaw:

```bash
ros2 launch vision_guided_robot sim.launch.py \
  control_mode:=waypoint \
  goal_x_m:=1.5 \
  goal_y_m:=0.5 \
  goal_yaw_rad:=1.57 \
  use_final_yaw:=true \
  rviz:=true
```

## Run With RViz Click-To-Goal

Launch waypoint mode without an initial parameter goal:

```bash
ros2 launch vision_guided_robot sim.launch.py \
  control_mode:=waypoint \
  start_with_parameter_goal:=false \
  rviz:=true
```

In RViz, add or select the `2D Goal Pose` tool. Set its topic to `/goal_pose`, click a point on the ground, then drag to choose the final yaw. The waypoint driver will receive `/goal_pose` and drive there.

## Run A Multi-Waypoint Mission

Use `waypoints_text` for a semicolon-separated list of waypoints:

```text
x,y;x,y,yaw
```

Each waypoint can be either:

```text
x,y
x,y,yaw
```

If yaw is included, the robot rotates to that final heading before advancing to the next waypoint.

Example:

```bash
ros2 launch vision_guided_robot sim.launch.py \
  control_mode:=waypoint \
  waypoints_text:="1.0,0.0;1.5,0.8,1.57;0.5,1.2;0.0,0.0,3.14" \
  rviz:=true
```

Watch mission progress:

```bash
ros2 topic echo /waypoint/progress
```

## Safety-Aware Mission State

The waypoint controller still publishes local controller progress on `/waypoint/state`, but it also publishes a higher-level mission state on `/mission/state`.

Watch it with:

```bash
ros2 topic echo /mission/state
```

Mission states:

- `WAITING_FOR_GOAL`: no active goal exists
- `NAVIGATING`: the waypoint controller is actively trying to reach a goal
- `REROUTING`: the waypoint controller is following a temporary detour waypoint
- `PAUSED_FOR_SAFETY`: the safety layer is avoiding or blocking an obstacle
- `BLOCKED`: the robot has been safety-paused too long, or safety keeps interrupting and the robot stops making progress
- `DONE`: the current goal has been reached and safety is clear

The waypoint driver listens to `/safety/state`. It pauses waypoint advancement during `AVOID`, `BLOCKED`, and `STALE_SCAN`, then resumes once the safety filter returns to `CLEAR` or `SLOW`.

It also runs a progress watchdog. If safety has interrupted the mission and distance-to-goal has not improved by at least `stuck_min_progress_m` within `stuck_timeout_s`, the mission becomes `BLOCKED`.

It also watches for safety oscillation. If the robot repeatedly flips between clear motion and safety pause within `safety_oscillation_window_s`, it treats that as a blocked route even if the odometry distance is still slowly changing.

Run a simple obstacle-aware waypoint mission:

```bash
ros2 launch vision_guided_robot sim.launch.py \
  control_mode:=waypoint \
  ball_x:=4.0 \
  ball_y:=-2.0 \
  waypoints_text:="1.0,0.0;2.0,0.8;0.0,0.0" \
  spawn_occluder:=true \
  occluder_x:=1.2 \
  occluder_y:=0.4 \
  stuck_timeout_s:=6.0 \
  safety_oscillation_max_interruptions:=2 \
  safety_oscillation_window_s:=8.0 \
  rviz:=true
```

Useful live monitors:

```bash
ros2 topic echo /mission/state
ros2 topic echo /waypoint/progress
ros2 topic echo /safety/state
```

Expected result for a blocked wall scenario:

```text
NAVIGATING
PAUSED_FOR_SAFETY
NAVIGATING
BLOCKED
```

That is not global navigation yet. It means the mission supervisor correctly detected that reactive safety could not make progress.

## Run A Simple Rerouting Mission

Enable rerouting to make the waypoint driver insert a temporary detour when it detects a blocked route:

```bash
ros2 launch vision_guided_robot sim.launch.py \
  control_mode:=waypoint \
  ball_x:=4.0 \
  ball_y:=-2.0 \
  waypoints_text:="2.0,0.8;0.0,0.0" \
  spawn_occluder:=true \
  occluder_x:=1.2 \
  occluder_y:=0.4 \
  stuck_timeout_s:=6.0 \
  safety_oscillation_max_interruptions:=2 \
  safety_oscillation_window_s:=8.0 \
  enable_rerouting:=true \
  detour_forward_offset_m:=0.6 \
  detour_lateral_offset_m:=1.0 \
  waypoint_linear_kp:=1.1 \
  waypoint_angular_kp:=2.2 \
  waypoint_max_linear_speed_mps:=0.9 \
  waypoint_max_angular_speed_radps:=1.8 \
  max_detour_attempts_per_goal:=2 \
  rviz:=false
```

Watch:

```bash
ros2 topic echo /mission/state
ros2 topic echo /waypoint/progress
```

Expected healthy rerouting pattern:

```text
NAVIGATING
PAUSED_FOR_SAFETY
REROUTING
NAVIGATING
DONE
```

If the detour also fails, the mission eventually returns `BLOCKED`. Increase `detour_lateral_offset_m` if the temporary waypoint is not far enough around the wall.

Optional loop:

```bash
ros2 launch vision_guided_robot sim.launch.py \
  control_mode:=waypoint \
  waypoints_text:="1.0,0.0;1.5,0.8;0.0,0.0" \
  loop_waypoints:=true \
  rviz:=true
```

## Watch State

```bash
ros2 topic echo /waypoint/state
ros2 topic echo /waypoint/progress
```

Expected states:

- `WAITING_FOR_GOAL`: no waypoint has been received yet
- `WAITING_FOR_ODOM`: no odometry received yet
- `ROTATE_TO_GOAL`: turn in place until pointed at the goal
- `DRIVE_TO_GOAL`: drive forward while correcting heading
- `ROTATE_TO_FINAL`: rotate to final yaw after reaching the position
- `DONE`: stop at the waypoint

During a multi-waypoint mission, `DONE` appears briefly at each reached waypoint before the node advances to the next goal.

## Debug Commands

```bash
ros2 topic echo --once /odom
ros2 topic echo --once /goal_pose
ros2 topic echo /waypoint/progress
ros2 topic echo /mission/state
ros2 topic echo /cmd_vel_raw
ros2 topic echo /cmd_vel
ros2 run tf2_ros tf2_echo odom base_link
```

## Robotics Concept

This is a pose-control problem. The controller computes:

```text
dx = goal_x - robot_x
dy = goal_y - robot_y
distance = sqrt(dx^2 + dy^2)
desired_heading = atan2(dy, dx)
heading_error = desired_heading - robot_yaw
```

The robot first rotates to reduce heading error, then drives forward while continuing to correct heading. This is not global planning yet; it does not know about maps or obstacles beyond the reactive safety filter.
