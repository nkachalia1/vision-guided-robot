# File Guide

## Root Files

`README.md`: Project overview, architecture sketch, run commands, and learning path.

`.gitignore`: Ignores ROS build outputs, Python caches, virtual environments, and runtime logs.

`requirements-dev.txt`: Minimal Python dependencies for local detector and controller tests.

`pyproject.toml`: Test discovery and optional lint configuration.

## Docs

`docs/architecture.md`: System diagram, topic contract, coordinate convention, and control math.

`docs/status.md`: Current milestone status, known limitations, final validation bags, and resume commands.

`docs/final_demo_report.md`: One-command HSV and ONNX demo workflow with the final speed, accuracy, and behavior comparison.

`docs/navigation_final_report.md`: Final custom-vs-Nav2 navigation report table covering baseline, tuning, two-obstacle, forced recovery, and manual Nav2 behavior validation.

`docs/roadmap.md`: Milestones, weekly plan, and skills to learn while building the project.

`docs/issues.md`: GitHub-style tickets ordered from beginner to advanced.

`docs/experiments.md`: Repeatable ball-position experiments using launch arguments.

`docs/controller_tuning.md`: A repeatable protocol for comparing controller gains and speed limits.

`docs/perception_robustness.md`: A repeatable protocol for testing object size, color, lighting, and occlusion.

`docs/webcam_perception.md`: Standalone webcam and saved-image workflow for debugging the OpenCV detector outside ROS/Gazebo.

`docs/detector_evaluation.md`: Repeatable saved-image detector benchmark for comparing HSV and ML backends.

`docs/distance_calibration.md`: Workflow for calibrating detector pixel size into metric target distance using known-distance images.

`docs/dataset_prep.md`: Commands and inspection workflow for creating a YOLO-format custom red-ball dataset.

`docs/ml_detector_plan.md`: Staged plan for replacing the HSV detector with an ML backend and comparing accuracy, speed, complexity, and robot behavior.

`docs/ml_onnx_backend.md`: ONNX detector backend usage notes, command shape, current limitations, and ROS/Gazebo integration path.

`docs/ml_detector_comparison.md`: HSV vs custom YOLO comparison, offline detector results, working ONNX launch command, and final ONNX robot validation bag.

`docs/ml_dataset_plan.md`: Custom red-ball dataset plan after generic YOLO failed to beat HSV.

`docs/obstacle_avoidance.md`: Lidar safety-layer architecture, commands, and debugging workflow.

`docs/odometry_rviz.md`: Odometry, TF, and RViz visualization workflow.

`docs/map_localization.md`: Saved-map + AMCL localization workflow that moves Nav2 from `odom` goals to `map` goals.

`docs/nav2_amcl_wall_passing.md`: Harder saved-map AMCL/Nav2 wall-passing experiment for the map-frame `(2.0, 0.8)` goal.

`docs/waypoint_navigation.md`: Odometry-based waypoint controller workflow and math.

`docs/path_planning.md`: A* grid-planning workflow, live lidar obstacle cells, planned-path topics, RViz debugging, and validation commands.

`docs/interview_review.md`: Design, robotics, ROS, and ML questions to test your understanding.

`docs/file_guide.md`: This file. It explains the purpose of each project file.

## ROS 2 Package Metadata

`src/vision_guided_robot/package.xml`: ROS package manifest with runtime and test dependencies.

`src/vision_guided_robot/setup.py`: Python package installation metadata and console entry points.

`src/vision_guided_robot/setup.cfg`: Installs ROS console scripts into the expected package-local path.

`src/vision_guided_robot/resource/vision_guided_robot`: Ament resource marker used by ROS 2 package discovery.

## Python Package

`src/vision_guided_robot/vision_guided_robot/__init__.py`: Marks the Python package.

`src/vision_guided_robot/vision_guided_robot/red_ball_detector.py`: Pure OpenCV detector. It finds red circular objects, estimates distance from radius, and can draw debug overlays.

`src/vision_guided_robot/vision_guided_robot/detector_backend.py`: Detector backend interface and factory. It selects the HSV detector or the ONNX YOLO detector.

`src/vision_guided_robot/vision_guided_robot/yolo_onnx_detector.py`: OpenCV DNN ONNX detector backend scaffold. It parses YOLO-style model output into the same `Detection` contract as the HSV detector.

`src/vision_guided_robot/vision_guided_robot/control_law.py`: Pure Python visual-servo control law. It converts target distance and lateral offset into linear and angular velocity.

`src/vision_guided_robot/vision_guided_robot/behavior_state_machine.py`: Pure Python finite-state behavior layer. It switches between search, tracking, approach, and stop modes.

`src/vision_guided_robot/vision_guided_robot/safety_filter.py`: Pure Python lidar safety logic. It slows, blocks, or commands a latched avoidance maneuver when an obstacle is close in front.

`src/vision_guided_robot/vision_guided_robot/waypoint_driver.py`: Pure Python waypoint controller. It converts odometry pose and an optional goal pose into velocity commands, and parses multi-waypoint mission text.

`src/vision_guided_robot/vision_guided_robot/path_follower.py`: Pure Python lookahead path follower. It selects a target point ahead on a planned path and turns that target into smoother velocity commands.

`src/vision_guided_robot/vision_guided_robot/detour_planner.py`: Pure Python detour planner. It generates a temporary geometric waypoint to the left or right of a blocked route.

`src/vision_guided_robot/vision_guided_robot/grid_planner.py`: Pure Python occupancy-grid A* planner. It converts static rectangular obstacles into blocked cells and returns a planned path.

`src/vision_guided_robot/vision_guided_robot/persistent_costmap.py`: Pure Python short-lived costmap memory. It remembers scan-observed grid cells for a tunable timeout.

`src/vision_guided_robot/vision_guided_robot/recovery_behavior.py`: Pure Python planned-navigation recovery sequence. It backs up, rotates, requests a costmap clear, then waits for replanning.

`src/vision_guided_robot/vision_guided_robot/mission_state.py`: Pure Python mission-state policy. It combines waypoint progress with safety state to report whether a mission is navigating, rerouting, paused, blocked, done, or waiting, and includes watchdogs for stuck progress and repeated safety oscillation.

`src/vision_guided_robot/vision_guided_robot/robot_visualization.py`: Pure Python helper for robot footprint geometry.

`src/vision_guided_robot/vision_guided_robot/ball_tracker_node.py`: ROS 2 perception node. It subscribes to camera images, selects a detector backend, and publishes target position plus annotated images.

`src/vision_guided_robot/vision_guided_robot/visual_servo_node.py`: ROS 2 control node. It subscribes to target position and publishes desired velocity on `/cmd_vel_raw`.

`src/vision_guided_robot/vision_guided_robot/safety_filter_node.py`: ROS 2 safety node. It subscribes to `/cmd_vel_raw` and `/scan`, then publishes final `/cmd_vel`.

`src/vision_guided_robot/vision_guided_robot/waypoint_driver_node.py`: ROS 2 waypoint node. It subscribes to `/odom`, `/goal_pose`, `/planned_path`, `/scan`, `/planner/state`, and `/safety/state`, manages waypoint missions, follows planned paths, runs recovery when planned navigation fails, publishes desired velocity on `/cmd_vel_raw`, and reports `/mission/state`.

`src/vision_guided_robot/vision_guided_robot/grid_planner_node.py`: ROS 2 planning node. It subscribes to odometry, goals, optional lidar scans, and `/planner/clear_costmap`, runs A* over static and scan-derived occupancy cells, publishes `/planned_path`, and republishes `/planning/occupancy_grid` for RViz.

`src/vision_guided_robot/vision_guided_robot/robot_visualization_node.py`: ROS 2 visualization helper. It publishes `/odom_path` and `/robot_footprint` for RViz.

`src/vision_guided_robot/vision_guided_robot/webcam_detector.py`: Standalone webcam and saved-image visualization tool. It shows annotated detections, optional masks, distance/lateral estimates, and debug snapshots outside ROS/Gazebo.

`src/vision_guided_robot/vision_guided_robot/distance_calibrator.py`: Known-distance calibration tool. It estimates detector distance error and recommends an effective target diameter for the pinhole-distance model.

`src/vision_guided_robot/vision_guided_robot/detector_evaluator.py`: Offline detector benchmark tool. It runs a backend on saved images, prints a comparison table, and can save annotated images, masks, CSV, and JSON results.

`src/vision_guided_robot/vision_guided_robot/detector_compare.py`: One-command HSV vs ONNX comparison runner. It evaluates both detectors on the same images and writes combined CSV/JSON results plus separate annotated output folders.

`src/vision_guided_robot/vision_guided_robot/dataset_prep.py`: YOLO dataset preparation tool. It copies positive and negative images, creates labels, HSV pseudo-labels positives, saves previews, and writes `data.yaml` plus `manifest.csv`.

`src/vision_guided_robot/vision_guided_robot/manual_label.py`: Manual YOLO label helper. It writes one bounding box label from typed coordinates or an OpenCV click workflow, and saves a corrected preview.

`src/vision_guided_robot/vision_guided_robot/manual_label_batch.py`: Batch manual YOLO label helper. It steps through many dataset images in sequence so failed pseudo-label batches can be corrected quickly.

`src/vision_guided_robot/vision_guided_robot/dataset_audit.py`: YOLO dataset audit tool. It checks image/label pairing, label syntax, box ranges, split counts, and readiness for training.

`src/vision_guided_robot/vision_guided_robot/ros_image_capture.py`: ROS image capture helper. It saves images from a ROS image topic and can require a recent target detection before saving positive training frames.

## Launch And Config

`src/vision_guided_robot/launch/sim.launch.py`: Starts Gazebo, the ROS/Gazebo bridge, the detector node, and the controller node.

`src/vision_guided_robot/launch/demo_hsv.launch.py`: One-command final demo preset for the HSV detector baseline.

`src/vision_guided_robot/launch/demo_onnx.launch.py`: One-command final demo preset for the calibrated custom YOLO ONNX detector.

`src/vision_guided_robot/launch/demo_planned.launch.py`: One-command planned-navigation demo that spawns a wall, runs the A* planner, and follows `/planned_path`.

`src/vision_guided_robot/launch/demo_live_planned.launch.py`: One-command live-planning demo that builds occupied grid cells from `/scan` instead of configured obstacle rectangles.

`src/vision_guided_robot/launch/demo_two_obstacle_planned.launch.py`: Custom planned-navigation comparison demo with two Gazebo wall obstacles, live scan planning, persistent costmap memory, and recovery enabled.

`src/vision_guided_robot/launch/demo_two_obstacle_nav2.launch.py`: Nav2 comparison demo that spawns the same two wall obstacles before launching Nav2.

`src/vision_guided_robot/launch/demo_forced_recovery_planned.launch.py`: Custom planned-navigation stress test that places the goal inside a large blocker so recovery behavior is expected.

`src/vision_guided_robot/launch/demo_forced_recovery_nav2.launch.py`: Nav2 stress test with the same blocked-goal recovery scenario.

`src/vision_guided_robot/launch/demo_nav2_amcl.launch.py`: Map-based Nav2 demo that starts Gazebo, map_server, AMCL, and Nav2 with goals in the `map` frame.

`src/vision_guided_robot/launch/demo_nav2_amcl_wall_pass.launch.py`: Experimental AMCL/Nav2 launch using a wall-passing tuned parameter file for the harder map-frame `(2.0, 0.8)` goal.

`src/vision_guided_robot/launch/perception.launch.py`: Starts only the perception node for debugging with any ROS image source.

`src/vision_guided_robot/config/bridge.yaml`: Defines bridged Gazebo and ROS topics.

`src/vision_guided_robot/config/detector.yaml`: Detector backend and parameters such as HSV thresholds, ball diameter, and camera field of view.

`src/vision_guided_robot/config/controller.yaml`: Controller gains, speed limits, stop distance, and target timeout.

`src/vision_guided_robot/config/safety.yaml`: Lidar safety filter distances, front sector, scan timeout, avoidance speed, and the narrower center cone used for forward creep during avoidance.

`src/vision_guided_robot/config/waypoint.yaml`: Waypoint goal source, mission-state topics, safety pause timeout, stuck-detection thresholds, tolerances, gains, and speed limits for odometry-based navigation.

`src/vision_guided_robot/config/planner.yaml`: Static occupancy-grid planner map bounds, resolution, obstacle rectangles, inflation radius, and planner topics.

`src/vision_guided_robot/config/nav2_params.yaml`: Odom-frame Nav2 configuration for comparing the standard Nav2 stack against the custom planner.

`src/vision_guided_robot/config/nav2_recovery_stress_params.yaml`: Tighter Nav2 configuration for blocked-goal recovery stress tests, with exact planner tolerance, tighter goal tolerance, and faster progress-failure detection.

`src/vision_guided_robot/config/nav2_map_params.yaml`: Nav2 + AMCL configuration for map-frame navigation using `map_server`, AMCL, global planning in `map`, and local control in `odom`.

`src/vision_guided_robot/config/nav2_map_wall_pass_params.yaml`: Experimental map-frame Nav2 + AMCL tuning for passing around the mapped walls using reduced inflation, a larger local costmap, and more flexible DWB trajectory sampling.

`src/vision_guided_robot/config/visualization.yaml`: Path and footprint visualization parameters.

`src/vision_guided_robot/rviz/vision_guided_robot.rviz`: RViz layout for TF, lidar scan, odometry, and camera debug image.

`src/vision_guided_robot/rviz/nav2_map.rviz`: RViz layout for map localization, with fixed frame `map`, `/map`, `/plan`, laser scan, odometry, and TF.

`src/vision_guided_robot/maps/two_wall_map.yaml`: Occupancy map metadata for the two-wall AMCL/Nav2 demo.

`src/vision_guided_robot/maps/two_wall_map.pgm`: Occupancy image for the two-wall AMCL/Nav2 demo.

## Gazebo Assets

`src/vision_guided_robot/worlds/red_ball_world.sdf`: The Gazebo world containing ground, light, the robot, and the red ball.

`src/vision_guided_robot/models/vision_bot/model.config`: Gazebo model metadata for the robot.

`src/vision_guided_robot/models/vision_bot/model.sdf`: Differential-drive robot model with wheels, camera sensor, lidar sensor, odometry, TF, and DiffDrive plugin.

`src/vision_guided_robot/models/red_ball/model.config`: Gazebo model metadata for the target object.

`src/vision_guided_robot/models/red_ball/model.sdf`: Red ball model used as the visual target.

`src/vision_guided_robot/models/recovery_blocker/model.sdf`: Large static box used for forced recovery stress tests.

## Tests

`src/vision_guided_robot/test/test_red_ball_detector.py`: Tests red object detection and distance behavior using synthetic images.

`src/vision_guided_robot/test/test_detector_backend.py`: Tests detector backend selection, HSV behavior through the backend contract, and clear errors for unsupported backend names.

`src/vision_guided_robot/test/test_detector_evaluator.py`: Tests saved-image detector evaluation, table formatting, and debug output saving.

`src/vision_guided_robot/test/test_distance_calibrator.py`: Tests known-distance sample parsing and the effective-diameter calibration math.

`src/vision_guided_robot/test/test_dataset_prep.py`: Tests YOLO label creation, dataset folder layout, HSV pseudo-labeling, empty negative labels, and preview generation.

`src/vision_guided_robot/test/test_manual_label.py`: Tests manual label path inference, bounding-box conversion, YOLO normalization, and preview writing.

`src/vision_guided_robot/test/test_dataset_audit.py`: Tests dataset label validation, split summaries, readiness checks, and issue formatting.

`src/vision_guided_robot/test/test_yolo_onnx_detector.py`: Tests ONNX/YOLO output parsing, target-class lookup, and detection conversion without requiring a real model file.

`src/vision_guided_robot/test/test_webcam_detector.py`: Tests standalone webcam-frame processing using synthetic images.

`src/vision_guided_robot/test/test_control_law.py`: Tests the control law without ROS.

`src/vision_guided_robot/test/test_safety_filter.py`: Tests lidar safety behavior without ROS.

`src/vision_guided_robot/test/test_waypoint_driver.py`: Tests waypoint-control behavior without ROS.

`src/vision_guided_robot/test/test_path_follower.py`: Tests lookahead target selection and smooth path-following commands without ROS.

`src/vision_guided_robot/test/test_persistent_costmap.py`: Tests short-lived obstacle-cell memory and timeout behavior without ROS.

`src/vision_guided_robot/test/test_recovery_behavior.py`: Tests backup, rotate, clear-costmap, and wait phases of planned-navigation recovery without ROS.

`src/vision_guided_robot/test/test_detour_planner.py`: Tests geometric detour waypoint generation without ROS.

`src/vision_guided_robot/test/test_mission_state.py`: Tests safety-aware mission-state behavior without ROS.

`src/vision_guided_robot/test/test_robot_visualization.py`: Tests robot footprint geometry without ROS.
