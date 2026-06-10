# Project Status

Last updated: 2026-06-10

This document tracks what has been built, what has been validated in simulation, and what remains.

## Current Active Step

Final baseline validation is complete. The custom ML detector replacement milestone is complete for the simulated vision behavior. A trained YOLO11n ONNX model detects the Gazebo red ball, publishes `/ball/relative_position`, and successfully drives the visual-servo approach loop. Distance calibration is complete for the current best ONNX model, using `detector_real_diameter_m:=0.272`.

The real path planning/navigation milestone is now validated. Static occupancy-grid A* planning, live lidar obstacle mapping, controlled live replanning, pure-pursuit-style path following, persistent local costmap memory, and lightweight recovery behavior all have successful Gazebo validation bags.

The persistent local costmap memory milestone is validated. Live planned mode can accumulate scan-derived obstacle cells for a short time instead of treating each scan as isolated, and the robot reached the planned goal using the remembered scan map.

The map-based localization milestone is now validated. The project can launch `map_server`, AMCL, Nav2, Gazebo, and RViz together, publish a saved two-wall map, localize with `/amcl_pose`, and complete a `map`-frame Nav2 goal. `bags/final_nav2_amcl_clear_goal` is the validation run: `map_samples: 1`, `amcl_pose_samples: 37`, `nav2_plan_samples: 47`, action status `EXECUTING -> SUCCEEDED`, `nav2_goal_error_m: 0.120`, and `success: True`.

The harder map-frame wall-passing goal is now validated with an endpoint caveat. `bags/final_nav2_amcl_wall_pass_success` proves Nav2 can localize with AMCL, plan a staged `map`-frame route around the wall layout, execute it at the faster wall-pass speed profile, and finish the `/navigate_through_poses` action with `SUCCEEDED`. The final odom pose is intentionally treated as loose because the planner tolerance was raised to make the tight route feasible.

The final portfolio wrap-up is now in progress. The SLAM-generated map has been validated with AMCL and fast Nav2: SLAM Toolbox produced a live map from `/scan`, the map was saved and padded, AMCL localized on that robot-built map, and Nav2 drove a farther map-frame goal using the fast profile.

Recorded and analyzed validation bags:

1. `bags/final_vision_approach`
2. `bags/final_obstacle_avoidance`
3. `bags/final_waypoint_param`
4. `bags/final_click_to_goal`
5. `bags/final_simple_rerouting`
6. `bags/final_onnx_vision_approach`
7. `bags/final_onnx_far_approach_run2`
8. `bags/final_onnx_far_improved`
9. `bags/final_onnx_calibrated_alignment`
10. `bags/final_grid_planner_full_run`
11. `bags/final_live_grid_planner_demo`
12. `bags/final_live_grid_planner_stable_state`
13. `bags/final_pure_pursuit_planner`
14. `bags/final_persistent_costmap_planner`
15. `bags/final_recovery_behavior_run2`
16. `bags/final_nav2_first_goal_run2`
17. `bags/final_nav2_fast_goal`
18. `bags/final_nav2_tight_goal`
19. `bags/final_two_obstacle_custom`
20. `bags/final_two_obstacle_nav2`
21. `bags/final_forced_recovery_custom`
22. `bags/final_forced_recovery_nav2_run2`
23. `bags/final_nav2_amcl_clear_goal`
24. `bags/final_nav2_amcl_wall_pass_success`
25. `bags/final_slam_mapping_first_run`
26. `bags/final_slam_map_amcl_fast_run2`

## Current System

The project is a ROS 2 + Gazebo simulation for a differential-drive robot with:

- camera-based colored-object tracking
- OpenCV red-ball detection
- visual servo approach behavior
- lidar safety filtering
- reactive obstacle avoidance
- odometry and TF
- RViz visualization
- odometry path and robot footprint visualization
- odometry-based waypoint driving
- RViz click-to-goal waypoint input
- multi-waypoint mission input
- safety-aware waypoint mission state
- waypoint stuck/block detection
- simple waypoint rerouting
- static occupancy-grid A* path planning
- planned path following through the waypoint driver
- live lidar obstacle cells for the planning grid
- pure-pursuit-style planned path following
- short-lived persistent local costmap memory
- lightweight planned-navigation recovery behavior
- standalone webcam perception workflow
- detector backend selection for HSV and ONNX YOLO
- offline saved-image detector evaluation harness
- ONNX/YOLO-style detector backend scaffold for offline ML experiments
- YOLO-format custom dataset preparation tool with HSV pseudo-label previews and bulk folder import
- manual YOLO label correction helper
- dataset audit tool for label validity and training readiness
- ROS image capture helper for collecting live Gazebo camera frames
- trained custom YOLO11n ONNX detector for the red-ball task
- live ONNX visual-servo approach in Gazebo
- distance calibration workflow for detector pixel size to metric distance
- one-command HSV and ONNX demo launch files
- first Nav2 comparison launch/config scaffold
- custom planner vs Nav2 comparison write-up
- tuned Nav2 speed and tighter goal-tolerance validation
- two-obstacle custom-vs-Nav2 navigation comparison
- forced custom-vs-Nav2 recovery stress-test scaffold
- final navigation comparison report
- saved-map + AMCL localization
- validated AMCL/Nav2 wall-passing route for the harder `(2.0, 0.8)` map-frame goal, with a known loose-endpoint caveat
- SLAM Toolbox mapping scaffold for live `/scan` to `/map` mapping
- saved SLAM-generated map artifact: `maps/slam/slam_two_wall_map.yaml` and `maps/slam/slam_two_wall_map.pgm`
- launch shortcut for robot-built-map navigation: `demo_nav2_slam_map.launch.py`
- portfolio summary: `docs/project_portfolio.md`

The main command path is:

```text
controller -> /cmd_vel_raw -> safety_filter -> /cmd_vel -> Gazebo DiffDrive
```

The controller can be either:

- `visual_servo`: follows the detected colored ball
- `waypoint_driver`: drives to a goal in the `odom` frame

## Milestone Status

| Milestone | Status | Evidence |
| --- | --- | --- |
| Roadmap, milestones, weekly plan, skills | Done | `docs/roadmap.md` |
| Repository structure and file guide | Done | `docs/file_guide.md` |
| GitHub-style curriculum issues | Done | `docs/issues.md` |
| Gazebo world and differential-drive robot | Done | Gazebo launches; robot moves from `/cmd_vel` |
| Camera bridge | Done | `/camera/image` publishes; image visible in `rqt_image_view` |
| OpenCV red-ball detector | Done | `/ball/annotated_image`; synthetic detector tests |
| Distance estimate from object size | Done | `/ball/relative_position.point.z` behaves correctly |
| Visual servo controller | Done | `SEARCH -> TRACK -> APPROACH -> STOP` validated |
| Controller tuning | Done | Aggressive defaults saved in config |
| Rosbag experiment analyzer | Done | `tools/analyze_bag.py` used on multiple trials |
| Perception robustness experiments | Partially done | Saved-image webcam table complete; Gazebo robustness table still open |
| Lidar bridge and safety filter | Done | `/scan`, `/cmd_vel_raw`, `/cmd_vel`, `/safety/state` validated |
| Reactive obstacle avoidance | Done | Robot avoids wall, reacquires ball, approaches target |
| Odometry and TF | Done | `/odom`, `/tf`, static sensor frames added |
| RViz visualization | Done | RViz opens with project layout after conservative config fix |
| RViz path and footprint helper | Done | `/odom_path`, `/robot_footprint`, and `/robot_footprint_array` visible in RViz |
| Waypoint navigation from launch args | Done | `ROTATE_TO_GOAL -> DRIVE_TO_GOAL -> DONE` validated |
| RViz click-to-goal | Done | `/goal_pose` input added; manual RViz goal tool works |
| Multi-waypoint navigation | Done | Mission queue implemented and user-validated in Gazebo/RViz |
| Safety-aware waypoint mission state | Done | User observed `NAVIGATING`, `PAUSED_FOR_SAFETY`, `NAVIGATING`, and `DONE` |
| Waypoint stuck/block detection | Done | `bags/waypoint_blocked_wall`: `blocked_detected: True`, mission `BLOCKED` for 27.37 s |
| Simple waypoint rerouting | Done | `bags/final_simple_rerouting`: `rerouting_detected: True`, `success: True` |
| Static A* path planning | Done | `bags/final_grid_planner_full_run`: `planner_success: True`, `planned_path_last_poses: 4`, `odom_displacement_m: 2.087`, `success: True` |
| Live lidar planning grid | Done | `bags/final_live_grid_planner_demo`: `planner_success: True`, `success: True`, `odom_displacement_m: 2.087` |
| Controlled live replanning | Done | `bags/final_live_grid_planner_stable_state`: `planner_success: True`, `planner_state_counts: PLANNED: 43`, `success: True` |
| Smooth local path following | Done | `bags/final_pure_pursuit_planner`: `FOLLOW_PATH: 86`, `planner_success: True`, `success: True`, final odom `(1.925, 0.853)` |
| Persistent local costmap | Done | `bags/final_persistent_costmap_planner`: `FOLLOW_PATH: 119`, `planner_success: True`, `success: True`, final odom `(1.992, 0.712)` |
| Planned-navigation recovery | Done | `bags/final_recovery_behavior_run2`: `RECOVERING: 127`, `COSTMAP_CLEARED: 2`, `planner_success: True`, `success: True` |
| Standalone webcam perception workflow | Done | Saved-image mode validated on centered, left-edge, far, and negative cases |
| Interview review habit | Started | `docs/interview_review.md` now includes architecture, robotics, ROS 2, perception, ML, and validation model answers; continue using it after future features |
| ML detector replacement | Done | `docs/ml_detector_comparison.md`; `bags/final_onnx_far_improved`: `success: True`, `initial_z_m: 1.788`, `final_z_m: 0.488` |
| Detector distance calibration | Done | `docs/distance_calibration.md`; `bags/final_onnx_calibrated_alignment`: `success: True`, `final_x_m: 0.000`, `final_z_m: 0.529` |
| Final demo/report workflow | Done | `demo_hsv.launch.py`, `demo_onnx.launch.py`, `docs/final_demo_report.md` |
| Nav2 comparison scaffold | Validated | `bags/final_nav2_first_goal_run2`: `/plan` published 54 samples, final odom `(1.864, 0.688)`, goal command returned `SUCCEEDED` |
| Custom planner vs Nav2 comparison | Done | `docs/navigation_comparison.md` compares `bags/final_recovery_behavior_run2` and `bags/final_nav2_first_goal_run2` |
| Faster Nav2 tuning | Improved | `bags/final_nav2_fast_goal`: `success: True`, `motion_command_span_s: 45.50` vs baseline `55.25`, `nav2_goal_error_m: 0.175` |
| Tight Nav2 goal tolerance | Improved | `bags/final_nav2_tight_goal`: `success: True`, `nav2_goal_error_m: 0.118`, `motion_command_span_s: 42.60`, action status `EXECUTING -> SUCCEEDED` |
| Two-obstacle navigation comparison | Validated | Custom: `bags/final_two_obstacle_custom`, `success: True`, `motion_command_span_s: 31.11`; Nav2: `bags/final_two_obstacle_nav2`, `success: True`, `motion_command_span_s: 42.10`; recovery did not trigger in either stack |
| Forced recovery stress test | Validated | Custom: `bags/final_forced_recovery_custom`, `custom_recovery_detected: True`; Nav2: `bags/final_forced_recovery_nav2_run2`, action `ABORTED` at `0.046 m` error under `0.030 m` tolerance; manual `/backup` and `/spin` succeeded in open space |
| Final navigation report | Done | `docs/navigation_final_report.md` summarizes custom planner, Nav2 tuning, two-obstacle runs, forced recovery, and manual Nav2 behavior validation |
| Saved map + AMCL localization | Validated | `bags/final_nav2_amcl_clear_goal`: `map_samples: 1`, `amcl_pose_samples: 37`, `nav2_plan_samples: 47`, action `SUCCEEDED`, `nav2_goal_error_m: 0.120`, `success: True` |
| AMCL/Nav2 wall-passing tune | Validated with endpoint caveat | `bags/final_nav2_amcl_wall_pass_success`: `/navigate_through_poses` action `EXECUTING -> SUCCEEDED`, `amcl_pose_samples: 70`, `nav2_plan_samples: 12`, `max_linear_mps: 1.800`, `motion_command_span_s: 33.65`. Final odom error to `(2.0, 0.8)` was `0.372 m`, so this validates wall-passing behavior but not tight final-position accuracy. |
| SLAM Toolbox mapping | Validated | `bags/final_slam_mapping_first_run`: `map_samples: 233`, `map_size: 205x128 @ 0.050 m/px`, `nav2_plan_samples: 48`, `final_map_base_xy_m: (0.671, -0.284)`, `nav2_goal_error_m: 0.174`, `nav2_goal_pose_source: map_tf`, Nav2 action `EXECUTING -> SUCCEEDED`, `success: True`. |
| Save SLAM-generated map | Done | `maps/slam/slam_two_wall_map.pgm` and `maps/slam/slam_two_wall_map.yaml` created in the WSL workspace. |
| AMCL on SLAM-generated map | Validated | `bags/final_slam_map_amcl_fast_run2`: `map_size: 148x153 @ 0.050 m/px`, `amcl_pose_samples: 20`, `nav2_plan_samples: 29`, `odom_displacement_m: 0.953`, `max_linear_mps: 0.818`, `final_map_base_xy_m: (0.992, -0.489)`, `nav2_goal_error_m: 0.236`, `success: True`. |
| Robot-built-map launch shortcut | Added | `demo_nav2_slam_map.launch.py` wraps the validated AMCL launch with `slam_two_wall_map_padded.yaml` and `nav2_map_wall_pass_params.yaml` defaults. Copy the generated map pair into `src/vision_guided_robot/maps/` before building. |
| Portfolio wrap-up | Done | `docs/project_portfolio.md` summarizes the final architecture, demo commands, validation evidence, engineering choices, limitations, and next extensions. |

## Tuned Defaults

These values are saved as the current obstacle-avoidance baseline:

```text
safety_filter.avoid_hold_time_s: 1.2
safety_filter.avoid_forward_speed_mps: 0.50
safety_filter.avoid_turn_speed_radps: 0.55
visual_servo.post_avoid_recover_time_s: 0.1
visual_servo.recover_timeout_s: 0.1
visual_servo.recover_angular_speed_radps: 0.35
visual_servo.search_angular_speed_radps: 0.85
visual_servo.stop_lateral_tolerance_m: 0.06
waypoint_driver.linear_kp: 1.1
waypoint_driver.angular_kp: 2.2
waypoint_driver.max_linear_speed_mps: 0.9
waypoint_driver.max_angular_speed_radps: 1.8
```

## Known Limitations

- The visual-servo obstacle behavior is reactive; planned mode now performs global A* planning around known or lidar-observed obstacles.
- Pure-pursuit mode smooths path tracking, but it is still a simple local follower and not a full trajectory optimizer.
- Simple rerouting inserts geometric detour waypoints, but the new grid planner is the preferred next navigation path for known static obstacles.
- Live lidar planning keeps a short-lived local costmap and can replan when the current path becomes blocked, but it is still a small custom planner rather than a full costmap/local-planner stack.
- Planned-navigation recovery is deliberately simple: it backs up, rotates, clears scan-derived costmap memory, and retries; it is not a full behavior tree.
- Rerouting can trigger from stuck distance-to-goal progress or repeated safety pause oscillation.
- Safety-aware waypoint state can detect pause/block/stuck conditions, but obstacle avoidance is still reactive.
- Odometry is useful for local motion but will drift over time.
- The HSV detector depends on color, lighting, camera exposure, and physical calibration.
- Generic COCO YOLO11n is not a good detector for the current red-target task without task-specific fine-tuning.
- The custom YOLO detector works in Gazebo and detects the current saved far/small test image after the `far_improved` dataset pass.
- The ONNX detector is slower than HSV on CPU and depends on dataset balance, confidence threshold, and deployment-domain examples.
- The ONNX distance estimate for far/small targets is less reliable than HSV because bounding-box size is not the same as a clean circle radius.
- RViz click-to-goal requires adding or selecting the `2D Goal Pose` tool and setting its topic to `/goal_pose`.
- The project has baseline validation bags for the main behaviors.
- The click-to-goal validation goal was very close to the robot, so it proves interactive input but is a weak motion-distance test.

## Final Validation Results

These bags are the current baseline evidence.

Analyze any bag with:

```bash
python3 tools/analyze_bag.py <bag_dir>
```

For vision bags, success means the final target estimate is centered, close to stop distance, and the robot stopped. For waypoint bags, success means `/waypoint/state` reached `DONE`.

| Bag | Result | Key Evidence | Notes |
| --- | --- | --- | --- |
| `bags/final_vision_approach` | Pass | `success: True`; `APPROACH -> STOP`; `final_z_m: 0.484`; `first_stop_time_s: 23.22` | Safety entered `SLOW`, but robot stopped correctly at the ball. |
| `bags/final_obstacle_avoidance` | Pass | `success: True`; `AVOID` present; `final_z_m: 0.489`; `odom_displacement_m: 2.654` | Robot avoided wall, reacquired target, and stopped. |
| `bags/final_waypoint_param` | Pass | `success: True`; `ROTATE_TO_GOAL -> DRIVE_TO_GOAL -> ROTATE_TO_FINAL -> DONE`; final odom `(1.418, 0.754)` | Goal was `(1.5, 0.8)`, within tolerance. |
| `bags/final_click_to_goal` | Pass | `success: True`; `WAITING_FOR_GOAL`; `goal_samples: 2`; `DONE` | Goal was close: `(0.098, 0.072)`. Optional rerun should use a farther click. |
| `bags/final_simple_rerouting` | Pass | `success: True`; `rerouting_detected: True`; `REROUTING` for 53.59 s; final odom `(0.091, -0.019)` | Mission completed both waypoints, but took 299.35 s before waypoint speed tuning. |
| `bags/final_onnx_vision_approach` | Pass | `success: True`; `STOP`; `final_z_m: 0.480`; `cmd_vel: 0.0` | ONNX stop behavior validated, but recording started near stop distance. |
| `bags/final_onnx_far_approach_run2` | Pass | `success: True`; `initial_z_m: 1.879`; `final_z_m: 0.486`; `APPROACH -> STOP` | Strong ONNX proof run: custom YOLO drove the robot from distance to stop. |
| `bags/final_onnx_far_improved` | Pass | `success: True`; `initial_z_m: 1.788`; `final_z_m: 0.488`; `APPROACH -> STOP` | Current best ONNX model proof run using `models/ml/red_ball_yolo11n_best.onnx`. |
| `bags/final_onnx_calibrated_alignment` | Pass | `success: True`; `initial_z_m: 2.472`; `final_z_m: 0.529`; `final_x_m: 0.000` | Current best calibrated ONNX proof run using `detector_real_diameter_m:=0.272`. |
| `bags/final_grid_planner_full_run` | Pass | `success: True`; `planner_success: True`; `planned_path_last_poses: 4`; `odom_displacement_m: 2.087` | Static A* planner produced a 4-pose route, and the waypoint driver followed all 3 planned waypoints to `DONE`. |
| `bags/final_live_grid_planner_demo` | Pass | `success: True`; `planner_success: True`; `planned_path_last_poses: 4`; `odom_displacement_m: 2.087` | Live `/scan` points were projected into the planning grid, then A* produced a 4-pose route that the waypoint driver completed. |
| `bags/final_live_grid_planner_stable_state` | Pass | `success: True`; `planner_success: True`; `planner_state_counts: PLANNED: 43`; final odom `(1.975, 0.714)` | Controlled live replanning validated. No `WAITING_FOR_SCAN` churn after startup, and the robot completed the planned route. |
| `bags/final_pure_pursuit_planner` | Pass | `success: True`; `planner_success: True`; `FOLLOW_PATH: 86`; final odom `(1.925, 0.853)` | Smooth lookahead path following validated. The robot followed the planned path without the old rotate-drive waypoint sequence. |
| `bags/final_persistent_costmap_planner` | Pass | `success: True`; `planner_success: True`; `FOLLOW_PATH: 119`; final odom `(1.992, 0.712)` | Persistent local costmap validated. `WAITING_FOR_SCAN` only appeared briefly at startup, then the planner remained planned and the robot reached the goal. |
| `bags/final_recovery_behavior_run2` | Pass | `success: True`; `planner_success: True`; `RECOVERING: 127`; `COSTMAP_CLEARED: 2`; final odom `(1.961, 0.717)` | Planned-navigation recovery validated. The robot backed up, rotated, cleared the planner costmap, waited for replan, resumed `FOLLOW_PATH`, and reached `DONE`. |
| `bags/final_nav2_first_goal_run2` | Pass | action result `SUCCEEDED`; `nav2_plan_samples: 54`; final odom `(1.864, 0.688)` | First Nav2 comparison bag. The final pose is about 0.176 m from the `(2.0, 0.8)` goal, inside the configured 0.18 m tolerance. |
| `bags/final_nav2_fast_goal` | Pass | `success: True`; `max_linear_mps: 0.600`; `motion_command_span_s: 45.50`; `nav2_goal_error_m: 0.175`; final odom `(1.863, 0.692)` | Faster Nav2 config improved motion time by about 9.75 s versus baseline while preserving final accuracy. |
| `bags/final_nav2_tight_goal` | Pass | `success: True`; `nav2_action_state_counts: EXECUTING: 1, SUCCEEDED: 1`; `motion_command_span_s: 42.60`; `nav2_goal_error_m: 0.118`; final odom `(1.913, 0.720)` | Current best Nav2 bag. The robot reached the `(2.0, 0.8)` goal inside the tighter 0.12 m tolerance while keeping the faster 0.60 m/s speed. |
| `bags/final_two_obstacle_custom` | Pass | `success: True`; `planner_success: True`; `FOLLOW_PATH: 93`; `DONE: 58`; `motion_command_span_s: 31.11`; final odom `(1.925, 0.848)` | Custom stack handled the two-obstacle world cleanly without needing recovery. |
| `bags/final_two_obstacle_nav2` | Pass | `success: True`; `nav2_action_state_counts: EXECUTING: 1, SUCCEEDED: 1`; `motion_command_span_s: 42.10`; `nav2_goal_error_m: 0.120`; final odom `(1.912, 0.718)` | Nav2 handled the same two-obstacle world within the 0.12 m tolerance. No behavior recovery action was recorded. |
| `bags/final_forced_recovery_custom` | Expected fail | `custom_recovery_detected: True`; `RECOVERING: 202`; `NO_PATH: 97`; `COSTMAP_CLEARED: 3`; final odom `(0.293, 1.121)` | Goal was intentionally inside the blocker. The useful result is recovery evidence, not goal completion. |
| `bags/final_forced_recovery_nav2_run2` | Expected abort | `nav2_action_state_counts: EXECUTING: 1, ABORTED: 1`; `nav2_goal_error_m: 0.046`; final odom `(1.165, 0.371)` | Nav2 approached the blocked/tight goal and aborted because it could not meet the strict 0.03 m tolerance. Manual `/backup` and `/spin` succeeded in open space. |
| `bags/final_nav2_amcl_clear_goal` | Pass | `success: True`; `map_samples: 1`; `amcl_pose_samples: 37`; `nav2_plan_samples: 47`; action `EXECUTING -> SUCCEEDED`; `nav2_goal_error_m: 0.120` | Saved-map localization validated. AMCL produced map-frame pose estimates and Nav2 completed a `map` goal at `(0.8, -0.6)`. |
| `bags/final_nav2_amcl_wall_pass_success` | Pass with caveat | `/navigate_through_poses` action `EXECUTING -> SUCCEEDED`; `amcl_pose_samples: 70`; `nav2_plan_samples: 12`; `max_linear_mps: 1.800`; `motion_command_span_s: 33.65`; final odom `(2.372, 0.787)` | Hard AMCL/Nav2 wall-passing route validated. Final pose was outside the strict `0.22 m` goal-error check because the tuned planner allowed a tolerant endpoint. |
| `bags/final_slam_mapping_first_run` | Pass | `success: True`; `map_samples: 233`; `map_size: 205x128 @ 0.050 m/px`; `nav2_plan_samples: 48`; `final_map_base_xy_m: (0.671, -0.284)`; `nav2_goal_error_m: 0.174`; Nav2 action `EXECUTING -> SUCCEEDED` | SLAM Toolbox built a live map from `/scan`, published `map -> odom`, and Nav2 completed a map-frame goal. |
| `bags/final_slam_map_amcl_fast_run2` | Pass | `success: True`; `amcl_pose_samples: 20`; `nav2_plan_samples: 29`; `odom_displacement_m: 0.953`; `max_linear_mps: 0.818`; `nav2_goal_error_m: 0.236`; `nav2_goal_pose_source: map_tf` | Robot-built map was padded, loaded with AMCL, and used by fast Nav2 to drive a farther map-frame goal. |

## Next Milestones

1. Copy `maps/slam/slam_two_wall_map_padded.yaml/.pgm` into `src/vision_guided_robot/maps/` and rebuild.
2. Run `demo_nav2_slam_map.launch.py` and record one shortcut-validation bag.
3. Review `docs/project_portfolio.md` and use it as the top-level project presentation artifact.

## Resume Commands

Default vision mode:

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vision_guided_robot sim.launch.py rviz:=true
```

HSV demo:

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vision_guided_robot demo_hsv.launch.py
```

Waypoint click-to-goal mode:

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vision_guided_robot sim.launch.py \
  control_mode:=waypoint \
  start_with_parameter_goal:=false \
  rviz:=true
```

Planned navigation mode:

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vision_guided_robot demo_planned.launch.py
```

Live lidar planned navigation mode:

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vision_guided_robot demo_live_planned.launch.py
```

Nav2 comparison mode:

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vision_guided_robot demo_nav2.launch.py
```

Robot-built SLAM map + AMCL + fast Nav2 mode:

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vision_guided_robot demo_nav2_slam_map.launch.py rviz:=true
```

ONNX vision mode:

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vision_guided_robot demo_onnx.launch.py
```
