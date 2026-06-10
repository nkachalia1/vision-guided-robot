# Roadmap

## Milestones

| Milestone | Outcome | Done When |
| --- | --- | --- |
| 1. Simulation boots | Gazebo loads a robot, camera, and red ball | `ros2 launch vision_guided_robot sim.launch.py` opens the world |
| 2. Perception works | OpenCV detects the red ball | `/ball/relative_position` updates when the ball is visible |
| 3. Control works | Robot turns and approaches | `/cmd_vel` changes with target position |
| 4. Full loop works | Robot approaches and stops | Robot stops near the ball without overshooting badly |
| 5. Debug tooling works | You can inspect perception | `/ball/annotated_image` shows bounding circle and distance |
| 6. Tests protect behavior | Core math is tested outside ROS | `python -m pytest` passes |
| 7. Harder scenarios | Robot handles shifted targets and noise | Multiple ball poses still converge |
| 8. ML upgrade | Neural detector replaces HSV detector | Accuracy, speed, and complexity are measured |

## Weekly Plan

### Week 1: ROS 2 And Simulation Basics

- Build and launch the package.
- Learn nodes, topics, launch files, package manifests, and `colcon`.
- Inspect `gz topic -l`, `ros2 topic list`, and `rqt_graph`.
- Move the robot manually by publishing `/cmd_vel`.

### Week 2: Camera And Perception

- Subscribe to `/camera/image`.
- Convert ROS images to OpenCV arrays with `cv_bridge`.
- Tune HSV thresholds for red.
- Draw debug overlays and publish annotated images.

### Week 3: Distance And Geometry

- Learn the pinhole camera model.
- Estimate distance from ball diameter in pixels.
- Convert pixel error into a lateral offset.
- Test the detector using synthetic images.

### Week 4: Visual Servo Control

- Implement proportional angular control.
- Add proportional forward speed.
- Stop at a target distance.
- Tune gains in simulation.

### Week 5: Robustness

- Add target timeout behavior.
- Add search behavior when the ball disappears.
- Test different ball positions, lighting, and camera noise.
- Record failure cases.

### Week 6: Robotics Engineering Polish

- Add launch arguments.
- Improve docs and architecture diagrams.
- Create a demo checklist.
- Prepare interview-style explanations of every design decision.

### Week 7 And Later: ML Upgrade

- Replace HSV segmentation with a neural detector.
- Measure frames per second, detection quality, and integration complexity.
- Keep the same `/ball/relative_position` interface so the controller does not care which detector is used.

## Skills To Learn

### Robotics

- differential-drive kinematics
- feedback control
- simulation versus real-world assumptions
- coordinate frames
- sensor noise and latency

### ROS 2

- packages and workspaces
- nodes, topics, messages, and parameters
- launch files
- simulation time
- bridging Gazebo and ROS

### Computer Vision

- color spaces
- HSV thresholding
- morphological filtering
- contours and circularity
- camera field of view
- pinhole camera geometry

### Software Engineering

- testable core logic
- separating perception and control
- configuration files
- reproducible demos
- documenting system contracts

### AI And ML Later

- object detection datasets
- inference speed and latency
- precision and recall
- model deployment tradeoffs
- comparing learned and handcrafted perception
