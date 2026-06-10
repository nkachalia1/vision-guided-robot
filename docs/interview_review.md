# Interview Review

Use this after every feature. Answer out loud. If an answer feels vague, that is your next study topic.

## Design Questions

1. Why are perception and control separate nodes instead of one large node?
2. What is the topic contract between the detector and the controller?
3. Which parts of this system can be tested without ROS?
4. What assumptions does the controller make about the detector output?
5. What would you change before putting this on real hardware?

## Robotics Questions

1. What does a differential-drive robot control directly?
2. Why does a target on the right side of the image create a negative yaw command?
3. What happens if the robot drives forward while the target is far off-center?
4. Why do we stop at a nonzero distance instead of driving until contact?
5. What failure modes come from latency in the camera pipeline?

## ROS Questions

1. What is the difference between a topic, a node, and a launch file?
2. Why does this project bridge `/cmd_vel` from ROS to Gazebo?
3. Why does this project bridge `/camera/image` from Gazebo to ROS?
4. What does `use_sim_time` change?
5. What would you inspect first if the robot does not move?

## Computer Vision Questions

1. Why use HSV instead of RGB for color segmentation?
2. Why does red require two hue ranges?
3. What is circularity and why is it useful here?
4. How does object radius in pixels relate to distance?
5. What makes this detector fail under real-world lighting?

## ML Questions For Later

1. What problem would a neural detector solve better than HSV thresholding?
2. What new problems would a neural detector introduce?
3. How would you measure accuracy for this task?
4. How would you measure runtime performance?
5. Why should the controller interface stay the same after replacing perception?

## Weakness Checklist

- If you cannot explain `/cmd_vel`, study mobile robot velocity commands.
- If you cannot derive `distance = real_diameter * focal_length / pixel_diameter`, study the pinhole camera model.
- If you cannot debug missing camera images, study ROS/Gazebo topic bridging.
- If you cannot explain the sign of angular velocity, draw the camera optical frame and base frame.
- If you cannot tune the robot smoothly, study proportional control and saturation.

## Model Answer Sheet

Use this section as your first pass answer key. Do not memorize it word-for-word. Practice explaining each idea in your own words.

### Core Architecture

The system follows this pattern:

```text
Sensors -> Perception / Mapping -> Decision -> Control -> Safety -> Robot
```

In vision mode:

```text
Gazebo camera -> ball_tracker -> visual_servo -> /cmd_vel_raw -> safety_filter -> /cmd_vel -> Gazebo DiffDrive
```

In planned navigation mode:

```text
Gazebo lidar + odom -> grid_planner -> /planned_path -> waypoint_driver -> /cmd_vel_raw -> safety_filter -> /cmd_vel -> Gazebo DiffDrive
```

The planner decides where to go, the controller decides how to move, and the safety filter decides what motion is allowed.

### Design Answers

1. Why are perception and control separate nodes?

Perception answers "what do I see?" Control answers "how should I move?" Keeping them separate makes each part easier to test, replace, and debug. This is why the HSV detector could later be replaced with ONNX YOLO without rewriting the visual-servo controller.

2. Why use `/cmd_vel_raw` before `/cmd_vel`?

`/cmd_vel_raw` is the controller's requested motion. `/cmd_vel` is the final safe motion. The safety filter sits between them so it can slow, stop, or avoid obstacles before a command reaches Gazebo.

3. What are `visual_servo`, `grid_planner`, and `waypoint_driver`?

`visual_servo` follows the detected ball. `grid_planner` computes a collision-aware path through an occupancy grid. `waypoint_driver` turns a goal or planned path into velocity commands using waypoint or pure-pursuit-style path following.

4. How do you isolate a robot behavior bug?

Check the topics in order. If `/camera/image` is wrong, the sensor or bridge is wrong. If `/ball/relative_position` is wrong, perception is wrong. If `/planned_path` is wrong, planning is wrong. If `/cmd_vel_raw` is wrong, control is wrong. If `/cmd_vel_raw` is right but `/cmd_vel` differs, safety is intervening. If `/cmd_vel` is right but the robot moves wrong, inspect the Gazebo model or DiffDrive setup.

### Robotics Answers

1. What is differential drive?

A differential-drive robot has left and right driven wheels. Equal wheel speeds move straight. Different wheel speeds rotate or curve. In ROS this is commanded with `linear.x` for forward speed and `angular.z` for yaw rate.

2. What is odometry?

Odometry is the robot's estimate of its pose and velocity. It tells the robot where it thinks it is in the `odom` frame. It can drift because real motion has wheel slip, noise, imperfect actuation, and sensor error.

3. Why inflate obstacles?

The robot is not a point. Inflation adds a safety margin around blocked cells so A* does not plan a route that mathematically clears a wall but physically clips the robot body.

4. What is pure pursuit?

Pure pursuit selects a target point ahead on the path and steers toward it. Small lookahead follows corners tightly but can be twitchy. Large lookahead is smoother but can cut corners.

5. Why is recovery needed?

Even with a valid plan, the robot can get stuck because the map is stale, the controller cannot execute the path cleanly, the robot is too close to an obstacle, or the costmap contains bad obstacle memory. Recovery gives the robot a second chance by backing up, rotating, clearing local costmap memory, and replanning.

### ROS 2 Answers

1. What is a node?

A node is a running robotics process with one job, such as `/ball_tracker`, `/grid_planner`, `/waypoint_driver`, or `/safety_filter`.

2. What is a topic?

A topic is a named stream of messages. Examples are `/camera/image`, `/scan`, `/odom`, `/planned_path`, `/cmd_vel_raw`, and `/cmd_vel`.

3. What is a message?

A message defines the data type on a topic. For example, `/cmd_vel` uses `geometry_msgs/msg/Twist`, `/odom` uses `nav_msgs/msg/Odometry`, and `/planned_path` uses `nav_msgs/msg/Path`.

4. What does a launch file do?

A launch file starts many nodes with parameters. Instead of manually starting Gazebo, the bridge, planner, controller, safety filter, and RViz, `ros2 launch vision_guided_robot demo_live_planned.launch.py` starts the full system.

5. What does `source install/setup.bash` do?

It teaches the current terminal where the built workspace packages are. Without it, ROS may only see `/opt/ros/humble`, which causes `Package 'vision_guided_robot' not found`.

6. Why are duplicate nodes bad?

Duplicate nodes can publish conflicting messages on the same topics. Two `/grid_planner` nodes can both publish `/planned_path`, `/planner/state`, and `/planning/occupancy_grid`, making behavior confusing.

7. Why use rosbag?

Gazebo shows a visual result, but rosbag records topic evidence. Bags let you prove what happened with data such as `FOLLOW_PATH`, `RECOVERING`, `COSTMAP_CLEARED`, `DONE`, `planner_success: True`, and `success: True`.

### Perception And ML Answers

1. Why did generic YOLO fail?

Generic YOLO was trained for broad object categories, not this exact small red ball in Gazebo and robot-control conditions. It did not have enough task-specific knowledge.

2. Why did custom data help?

The custom dataset included real red-ball images, negatives, Gazebo images, far/small examples, and visible sim examples. That reduced domain shift between training and deployment.

3. Why is HSV faster than ONNX YOLO?

HSV detection is a simple color rule: threshold red pixels, find contours, estimate circle size. YOLO is a neural network with many layers of matrix math and post-processing, so it is slower on CPU.

4. Why can HSV estimate distance better for this target?

The red ball is circular, so HSV can estimate a clean pixel radius. Distance uses pinhole geometry:

```text
distance ~= real_diameter * focal_length / pixel_diameter
```

YOLO returns bounding boxes, and a box may be too loose, too tight, shifted, or clipped.

5. Why are negative images important?

Positive images teach what a red ball looks like. Negative images teach what not to detect. Without negatives, the model can hallucinate red-ball detections.

### Validation Evidence

Use these examples in an interview:

- Vision approach: `bags/final_vision_approach`, `success: True`, `APPROACH -> STOP`.
- Obstacle avoidance: `bags/final_obstacle_avoidance`, `AVOID` present, `success: True`.
- ONNX detector behavior: `bags/final_onnx_far_improved`, `success: True`.
- Static A*: `bags/final_grid_planner_full_run`, `planner_success: True`, `success: True`.
- Live lidar planner: `bags/final_live_grid_planner_stable_state`, `PLANNED: 43`, `success: True`.
- Pure pursuit: `bags/final_pure_pursuit_planner`, `FOLLOW_PATH: 86`, `success: True`.
- Persistent costmap: `bags/final_persistent_costmap_planner`, `FOLLOW_PATH: 119`, `success: True`.
- Recovery: `bags/final_recovery_behavior_run2`, `RECOVERING: 127`, `COSTMAP_CLEARED: 2`, `success: True`.

### One-Minute Project Pitch

I built a ROS 2 and Gazebo vision-guided differential-drive robot in Python. The robot can detect and approach a colored target using OpenCV or a custom YOLO11n ONNX model. It also supports odometry-based waypoint navigation, A* grid planning, live lidar obstacle mapping, a short-lived persistent local costmap, pure-pursuit-style path following, and a simple recovery behavior that backs up, rotates, clears the local costmap, and replans. I validated each behavior with rosbag analysis instead of only visual inspection.

## Baseline System Review

Use these after the final validation bags.

### Design

1. Why does the system use `/cmd_vel_raw` before `/cmd_vel`?
2. What are the tradeoffs of keeping `visual_servo` and `waypoint_driver` as separate control modes?
3. What topic contracts let the HSV detector be replaced later without rewriting the controller?
4. Which parts of the system are pure Python and which parts require ROS?
5. What evidence from the validation bags would you show in a robotics interview?

### Robotics

1. Why can odometry reach a local waypoint but still drift over time?
2. Why is reactive obstacle avoidance different from path planning?
3. Why did the robot need `SEARCH` to rotate continuously after obstacle avoidance?
4. What does `odom -> base_link` represent in TF?
5. Why does final waypoint position tolerance matter more than exact final coordinates?

### ROS

1. Which nodes publish and subscribe to `/cmd_vel_raw` and `/cmd_vel`?
2. Why does RViz click-to-goal publish a `PoseStamped` instead of a `Twist`?
3. What would you inspect if `/waypoint/state` is not published?
4. What would you inspect if RViz shows `/scan` in the wrong place?
5. How does `ros2 bag` help validate behavior beyond watching the screen?

### ML

1. What failure modes of HSV detection would a neural detector improve?
2. What latency risks would a neural detector introduce?
3. What should stay identical when swapping HSV for ML?
4. What metrics should compare HSV and ML detectors?
5. Why is this baseline valuable before adding ML?
