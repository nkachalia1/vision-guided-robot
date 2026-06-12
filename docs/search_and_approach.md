# Search And Approach Demo

This demo places two wall obstacles between the robot and a red ball. The robot must:

1. scan from the current pose
2. if the ball is not visible, drive to the next search pose
3. scan again from the new pose
4. repeat until the camera detects the ball
5. send a Nav2 approach goal near the detected ball
6. stop near the target

## Run

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch vision_guided_robot demo_search_ball_two_walls.launch.py rviz:=true
```

The default scene uses:

```text
wall 1: x=1.15, y=0.35
wall 2: x=1.65, y=-0.45
ball:   x=2.45, y=0.85
```

## Watch State

In another terminal:

```bash
ros2 topic echo /explorer/state
```

In another:

```bash
ros2 topic echo /target_search/state
```

Expected high-level sequence:

```text
/target_search/state: SCANNING_FOR_TARGET
/target_search/state: SCAN_HEADING_COMPLETE
/target_search/state: MOVING_TO_SEARCH_POSE
/target_search/state: SEARCH_POSE_REACHED
/target_search/state: SCANNING_FOR_TARGET
/target_search/state: CONFIRMING_TARGET
/target_search/state: SENDING_TARGET_GOAL
/target_search/state: APPROACHING_TARGET
/target_search/state: TARGET_REACHED
```

## Open The Camera View

```bash
ros2 run rqt_image_view rqt_image_view /ball/annotated_image
```

If the annotated topic is not listed yet:

```bash
ros2 topic list | grep image
ros2 run rqt_image_view rqt_image_view /camera/image
```

## Record A Validation Bag

```bash
ros2 bag record -o bags/final_search_ball_two_walls \
  /cmd_vel \
  /odom \
  /scan \
  /map \
  /tf \
  /ball/relative_position \
  /explorer/state \
  /explorer/goal \
  /target_search/state \
  /target_search/goal \
  /navigate_to_pose/_action/status
```

Analyze:

```bash
python3 tools/analyze_bag.py bags/final_search_ball_two_walls
```

Success indicators:

```text
target_samples: > 0
target_search_goal_samples: > 0
target_search_state_counts:
  MOVING_TO_SEARCH_POSE
  TARGET_REACHED
target_search_success: True
success: True
```

## Tuning

The default search route is:

```text
0.8,-0.6;1.4,-1.0;1.9,-0.4;2.2,0.4;2.4,1.0
```

Each point is in the `map` frame. At each point, the robot scans these yaw headings:

```text
0, +90 deg, +180 deg, -90 deg
```

If the robot needs to look from a different side of the walls:

```bash
ros2 launch vision_guided_robot demo_search_ball_two_walls.launch.py \
  rviz:=true \
  search_waypoints_text:="0.8,-0.8;1.4,-1.2;2.0,-0.6;2.4,0.3;2.6,1.0"
```

If the robot stops too far from the ball:

```bash
ros2 launch vision_guided_robot demo_search_ball_two_walls.launch.py \
  rviz:=true \
  stand_off_distance_m:=0.40
```

If the ball is detected too late, move it slightly higher or closer to the opening:

```bash
ros2 launch vision_guided_robot demo_search_ball_two_walls.launch.py \
  rviz:=true \
  ball_x:=2.2 \
  ball_y:=0.7
```
