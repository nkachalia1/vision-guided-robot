# Odometry And RViz

This step makes the robot's internal motion estimate visible.

Gazebo's differential-drive system publishes:

```text
/odom
/tf
```

The bridge converts those Gazebo messages into ROS 2 messages:

```text
/odom: nav_msgs/msg/Odometry
/tf: tf2_msgs/msg/TFMessage
```

The launch file also publishes static transforms for the sensors:

```text
base_link -> lidar_link
base_link -> camera_link
camera_link -> camera_optical_frame
```

## Run With RViz

```bash
ros2 launch vision_guided_robot sim.launch.py rviz:=true
```

RViz should show:

- `odom` as the fixed frame
- TF frames for `odom`, `base_link`, `lidar_link`, and camera frames
- `/scan` as red lidar points
- `/odom` as a blue odometry arrow/path trail
- `/odom_path` as the robot's accumulated path
- `/robot_footprint` as a top-down marker around the robot base
- `/ball/annotated_image` as the camera debug image

## Path And Footprint Displays

The launch file starts `robot_visualization`, which publishes:

```text
/odom_path: nav_msgs/msg/Path
/robot_footprint: visualization_msgs/msg/Marker
```

The saved RViz layout loads these automatically when launched with:

```bash
ros2 launch vision_guided_robot sim.launch.py rviz:=true
```

If you open RViz manually, add them yourself:

1. Add a `Path` display and set its topic to `/odom_path`.
2. Add a `Marker` display and set its topic to `/robot_footprint`.
3. Keep the fixed frame set to `odom`.

## Debug Commands

Check odometry:

```bash
ros2 topic echo --once /odom
ros2 topic hz /odom
```

Check TF:

```bash
ros2 topic echo --once /tf
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link lidar_link
```

Check RViz input topics:

```bash
ros2 topic hz /scan
ros2 topic hz /ball/annotated_image
ros2 topic hz /odom_path
ros2 topic echo --once /robot_footprint
```

## Robotics Concept

Odometry estimates how the robot moved by integrating wheel motion over time. It is useful, but it drifts because small wheel errors accumulate. TF is the coordinate-frame system that lets ROS answer questions like:

```text
Where is the lidar relative to the robot base?
Where is the robot base relative to odom?
Where should RViz draw this laser scan?
```

This is the foundation for mapping, localization, and navigation.

The path display helps you see how odometry accumulates over time. The footprint marker helps you reason about the robot's physical size when it approaches walls or goals.
