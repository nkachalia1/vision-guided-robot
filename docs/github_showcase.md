# GitHub Showcase Guide

Use this checklist to make the repository easy for a hiring manager to review.

The goal is simple:

```text
In 60 seconds, a reviewer should understand what the robot does, see it running, and know where the technical proof lives.
```

## 1. Record One Short Demo

Best demo to record:

```text
Autonomous SLAM frontier exploration:
Gazebo on one side, RViz on the other, robot moving while the map grows.
```

Run:

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch vision_guided_robot demo_slam_explore.launch.py \
  rviz:=true \
  max_goals:=6 \
  goal_cooldown_s:=0.2 \
  max_goal_distance_m:=4.5 \
  distance_weight:=0.5
```

In a second terminal:

```bash
ros2 topic echo /explorer/state
```

Record 20-40 seconds showing:

- Gazebo robot moving
- RViz map growing
- `/explorer/state` showing `NAVIGATING` and `GOAL_SUCCEEDED`

## 2. Save Demo Media

Recommended file:

```text
media/frontier_exploration.gif
```

Good options:

- Use ScreenToGif on Windows and export a GIF.
- Use OBS or Xbox Game Bar to record MP4, then convert to GIF.
- If the GIF is too large, upload MP4 to a GitHub Release and link it from the README.

Keep the clip short. A 10-30 MB GIF is usually enough for a portfolio repo.

## 3. Add The Demo To README

After saving `media/frontier_exploration.gif`, add this under `## Demo Preview` in `README.md`:

```markdown
![Autonomous frontier exploration demo](media/frontier_exploration.gif)
```

Then commit:

```bash
git add README.md media/frontier_exploration.gif
git commit -m "Add project demo media"
git push
```

## 4. Set GitHub Repo Metadata

On the GitHub repo page, set:

```text
Description:
ROS 2 + Gazebo mobile robot with vision, Nav2, SLAM, and autonomous frontier exploration.

Topics:
ros2
gazebo
robotics
nav2
slam
opencv
python
computer-vision
autonomous-robots
```

Pin this repository on your GitHub profile.

## 5. What A Hiring Manager Should Click

The README should guide them here:

1. Demo preview or command
2. `docs/project_portfolio.md`
3. `docs/status.md`
4. `docs/autonomous_exploration.md`
5. `docs/navigation_final_report.md`
6. `src/vision_guided_robot/vision_guided_robot/frontier_explorer_node.py`

## 6. Suggested Resume Bullet

```text
Built a ROS 2/Gazebo mobile robot that performs camera-based object tracking, custom YOLO ONNX perception, lidar safety filtering, A* path planning, Nav2 navigation, AMCL localization, SLAM Toolbox mapping, and autonomous frontier exploration; validated milestones with rosbag analysis.
```

Shorter version:

```text
Built and validated a ROS 2/Gazebo robot with computer vision, Nav2 navigation, SLAM, and autonomous frontier exploration.
```

## 7. What Not To Upload

Keep these out of Git:

- ROS bags
- raw datasets
- trained `.pt` or `.onnx` model files
- `build/`, `install/`, `log/`
- large debug videos that are not part of the final demo

The repository `.gitignore` is configured to block those by default.
