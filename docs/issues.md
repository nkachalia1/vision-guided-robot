# GitHub-Style Issues

## Issue 1: Build And Launch The Empty System

Goal: Build the ROS 2 package and launch Gazebo.

Files affected: `package.xml`, `setup.py`, `launch/sim.launch.py`, `worlds/red_ball_world.sdf`

Estimated difficulty: Beginner

Concepts learned: ROS 2 packages, `colcon`, Gazebo launch, simulation time

Acceptance criteria:

- `colcon build --symlink-install` succeeds.
- `ros2 launch vision_guided_robot sim.launch.py` starts Gazebo.
- `/clock` is visible in `ros2 topic list`.

## Issue 2: Confirm Robot Motion From `/cmd_vel`

Goal: Prove the robot can move before adding autonomy.

Files affected: `models/vision_bot/model.sdf`, `config/bridge.yaml`

Estimated difficulty: Beginner

Concepts learned: differential drive, Gazebo plugins, ROS/Gazebo bridging

Acceptance criteria:

- Publishing a positive `linear.x` command moves the robot forward.
- Publishing a nonzero `angular.z` command rotates the robot.
- Stopping the publisher stops the robot.

## Issue 3: Confirm Camera Images In ROS

Goal: Bridge the simulated camera into ROS 2.

Files affected: `models/vision_bot/model.sdf`, `config/bridge.yaml`

Estimated difficulty: Beginner

Concepts learned: camera sensors, image topics, bridge direction, QoS

Acceptance criteria:

- `/camera/image` appears in `ros2 topic list`.
- `ros2 topic hz /camera/image` reports frames.
- `rqt_image_view /camera/image` shows the red ball when visible.

## Issue 4: Implement Red Ball Detection

Goal: Detect a red spherical object using OpenCV.

Files affected: `vision_guided_robot/red_ball_detector.py`, `config/detector.yaml`, `test/test_red_ball_detector.py`

Estimated difficulty: Intermediate

Concepts learned: HSV color segmentation, masks, contours, circularity

Acceptance criteria:

- Synthetic red-ball test image produces one detection.
- Non-red test image produces no detection.
- Detection reports center, radius, bounding box, and confidence.

## Issue 5: Estimate Distance From Object Size

Goal: Estimate target distance from apparent diameter.

Files affected: `vision_guided_robot/red_ball_detector.py`, `test/test_red_ball_detector.py`

Estimated difficulty: Intermediate

Concepts learned: pinhole camera model, focal length, field of view

Acceptance criteria:

- Larger detected radius produces smaller estimated distance.
- Estimated distance is positive and finite.
- Test covers a known synthetic radius.

## Issue 6: Publish Target Position

Goal: Convert detector output into a ROS topic for control.

Files affected: `vision_guided_robot/ball_tracker_node.py`, `launch/perception.launch.py`

Estimated difficulty: Intermediate

Concepts learned: ROS image subscription, `cv_bridge`, `PointStamped`, debug image publishing

Acceptance criteria:

- `/ball/relative_position` publishes while the ball is visible.
- `point.z` represents forward distance.
- `point.x` changes sign when the ball moves left versus right in the image.
- `/ball/annotated_image` shows detector overlays.

## Issue 7: Implement Visual Servo Control

Goal: Command the robot toward the detected ball.

Files affected: `vision_guided_robot/control_law.py`, `vision_guided_robot/visual_servo_node.py`, `config/controller.yaml`, `test/test_control_law.py`

Estimated difficulty: Intermediate

Concepts learned: proportional control, angular error, velocity saturation

Acceptance criteria:

- Ball on the right produces negative `angular.z`.
- Ball on the left produces positive `angular.z`.
- Far target produces positive `linear.x`.
- Close target produces zero velocity.

## Issue 8: Close The Loop In Simulation

Goal: Run perception and control together in Gazebo.

Files affected: `launch/sim.launch.py`, `config/detector.yaml`, `config/controller.yaml`

Estimated difficulty: Intermediate

Concepts learned: closed-loop behavior, gain tuning, debugging with topics

Acceptance criteria:

- Robot turns toward the ball.
- Robot drives forward after aligning.
- Robot stops near the configured stop distance.
- The system recovers if the ball starts off-center.

## Issue 9: Add Robustness Experiments

Goal: Make the simple detector and controller less brittle.

Files affected: `worlds/red_ball_world.sdf`, `config/detector.yaml`, `config/controller.yaml`, tests

Estimated difficulty: Advanced beginner

Concepts learned: lighting, noise, thresholds, timeouts, parameter tuning

Acceptance criteria:

- Detector tolerates modest lighting changes.
- Controller stops or searches when the target disappears.
- Failure cases are documented.

## Issue 10: Replace Detector With A Neural Model

Goal: Swap HSV detection for an ML detector while preserving the controller interface.

Files affected: new detector module, launch/config files, docs

Estimated difficulty: Advanced

Concepts learned: model inference, latency, accuracy metrics, interface design

Acceptance criteria:

- ML detector publishes the same `/ball/relative_position` contract.
- HSV and ML detectors are compared on accuracy, speed, and complexity.
- Controller code does not need to change.
