# Final Demo Report

This report is the repeatable end-of-project demo workflow for the current vision-guided robot.

It compares:

- HSV color segmentation baseline
- custom YOLO11n ONNX detector

Both detectors publish the same ROS contract:

```text
/camera/image -> ball_tracker -> /ball/relative_position -> visual_servo -> /cmd_vel_raw -> safety_filter -> /cmd_vel
```

## One-Command Demos

Build and source first:

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

HSV baseline:

```bash
ros2 launch vision_guided_robot demo_hsv.launch.py
```

Custom ONNX detector:

```bash
ros2 launch vision_guided_robot demo_onnx.launch.py
```

Optional RViz:

```bash
ros2 launch vision_guided_robot demo_hsv.launch.py rviz:=true
ros2 launch vision_guided_robot demo_onnx.launch.py rviz:=true
```

Optional ball position:

```bash
ros2 launch vision_guided_robot demo_onnx.launch.py ball_x:=2.5 ball_y:=0.5
```

## Demo Profiles

| Profile | Backend | Model | Target class | Confidence | Distance diameter |
| --- | --- | --- | --- | ---: | ---: |
| HSV | `hsv` | n/a | red color mask | n/a | `0.20 m` |
| ONNX best | `onnx` | `models/ml/red_ball_yolo11n_best.onnx` | `red_ball` | `0.10` | `0.272 m` |

The ONNX diameter is an effective calibration value from known-distance images. It is not the physical ball size.

## Saved-Image Detector Comparison

Command:

```bash
ros2 run vision_guided_robot detector_compare \
  --model-path models/ml/red_ball_yolo11n_best.onnx \
  --class-names models/ml/red_ball_classes.txt \
  --target-class red_ball \
  --confidence-threshold 0.10 \
  --image center=/mnt/c/Users/Neel/Pictures/red_center.jpg \
  --image left=/mnt/c/Users/Neel/Pictures/red_left.jpeg \
  --image far=/mnt/c/Users/Neel/Pictures/red_far.jpeg \
  --image negative=/mnt/c/Users/Neel/Pictures/negative.jpg \
  --diameter-m 0.20 \
  --fov-deg 60
```

Result:

| Image | HSV result | HSV runtime | ONNX result | ONNX runtime |
| --- | --- | ---: | --- | ---: |
| `center` | detected, `0.90` | `3.5 ms` | detected, `1.00` | `125.8 ms` |
| `left` | detected, `0.74` | `11.8 ms` | detected, `0.97` | `105.2 ms` |
| `far` | detected, `0.42` | `9.7 ms` | detected, `0.41` | `66.0 ms` |
| `negative` | rejected | `1.4 ms` | rejected | `84.0 ms` |

## Live Robot Validation

HSV baseline evidence:

```text
bag: bags/final_vision_approach
success: True
final_z_m: 0.484
state: APPROACH -> STOP
```

ONNX calibrated evidence:

```text
bag: bags/final_onnx_calibrated_alignment
success: True
initial_x_m: 0.009
final_x_m: 0.000
initial_z_m: 2.472
final_z_m: 0.529
max_linear_mps: 1.200
max_angular_radps: 0.016
```

The calibrated ONNX detector drove from about `2.47 m` estimated range to the stop region and ended centered on the target.

## Engineering Interpretation

| Dimension | HSV baseline | Custom ONNX |
| --- | --- | --- |
| Speed | Fast, usually under `15 ms` per saved image | Slower, about `66-126 ms` per saved image on CPU |
| Sim behavior | Stable and simple | Stable after custom training, sim-domain examples, and distance calibration |
| Accuracy on current saved set | Detects positives and rejects negative | Detects positives and rejects negative |
| Distance estimate | Clean for circular red blobs | Needs effective-diameter calibration because YOLO boxes are not metric measurements |
| Complexity | Low | High: data collection, labels, training, export, confidence tuning, calibration |
| Best role | Production/debugging baseline | ML learning branch and learned detector demonstration |

## Lesson

The ML detector can now complete the same robot task as the handcrafted detector, but it took more engineering:

1. train on task-specific data
2. add hard examples from the deployment domain
3. validate inside the live ROS node
4. calibrate distance separately from class detection
5. tune the controller so "stop" means close and centered

That is the real robotics lesson: perception accuracy, metric geometry, and control behavior are separate layers that must all work together.
