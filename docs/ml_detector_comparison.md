# ML Detector Comparison

This document compares the project detectors after training a custom YOLO model for the red-ball task.

## Summary

The ML replacement milestone is complete for the simulated vision behavior.

Best live model:

```text
models/ml/red_ball_yolo11n_best.onnx
class file: models/ml/red_ball_classes.txt
target class: red_ball
confidence threshold: 0.10
calibrated detector_real_diameter_m: 0.272
```

Final calibrated Gazebo validation:

```text
bag: bags/final_onnx_calibrated_alignment
initial_x_m: 0.009
final_x_m: 0.000
initial_z_m: 2.472
final_z_m: 0.529
success: True
```

The robot used ONNX detections from `/ball_tracker`, drove toward the Gazebo ball, and stopped near the configured stop distance.

## Working ONNX Launch

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch vision_guided_robot sim.launch.py \
  control_mode:=vision \
  ball_x:=2.0 \
  ball_y:=0.0 \
  detector_backend:=onnx \
  detector_model_path:=$(pwd)/models/ml/red_ball_yolo11n_best.onnx \
  detector_class_names_path:=$(pwd)/models/ml/red_ball_classes.txt \
  detector_target_class_names:=red_ball \
  detector_confidence_threshold:=0.10 \
  detector_real_diameter_m:=0.272
```

Shortcut:

```bash
ros2 launch vision_guided_robot demo_onnx.launch.py
```

Verification:

```bash
ros2 node list | grep ball_tracker
ros2 param get /ball_tracker detector_target_class_names
timeout 10 ros2 topic echo /ball/relative_position
ros2 run rqt_image_view rqt_image_view /ball/annotated_image
```

## Offline Detector Results

### Generic YOLO11n COCO

The off-the-shelf COCO model was a poor fit.

| Image | Detected | Result |
| --- | --- | --- |
| `center` | no | missed positive |
| `left` | no | missed positive |
| `far` | no | missed positive |
| `negative` | yes | false positive |

Conclusion: generic object detection was solving a different task than this robot needed.

### Custom YOLO, Real Images Only

After training on the custom red-ball dataset:

| Image | Detected | Confidence | Runtime |
| --- | --- | ---: | ---: |
| `center` | yes | 0.93 | 117.6 ms |
| `left` | yes | 0.99 | 70.6 ms |
| `far` | no | n/a | 70.5 ms |
| `negative` | no | n/a | 73.8 ms |

With `--confidence-threshold 0.10`, the far image was detected:

| Image | Detected | Confidence | Runtime |
| --- | --- | ---: | ---: |
| `far` | yes | 0.25 | 93.8 ms |

Conclusion: the model learned the class, but low-confidence far detections needed tuning.

### Custom YOLO, Real + Sim Mixed

After adding Gazebo camera frames to the dataset:

| Image | Detected | Confidence | Distance | Runtime |
| --- | --- | ---: | ---: | ---: |
| `center` | yes | 1.00 | 0.49 m | 122.3 ms |
| `left` | yes | 0.92 | 1.07 m | 70.2 ms |
| `far` | yes | 0.63 | 7.99 m | 85.1 ms |
| `negative` | no | n/a | n/a | 71.3 ms |

Conclusion: adding sim-domain examples improved far detection and preserved negative rejection.

### Custom YOLO, Visible Sim Hard Examples

The visible-sim model:

| Image | Detected | Confidence | Distance | Runtime |
| --- | --- | ---: | ---: | ---: |
| `center` | yes | 1.00 | 0.49 m | 107.5 ms |
| `left` | yes | 0.89 | 1.19 m | 80.8 ms |
| `far` | no | n/a | n/a | 74.7 ms |
| `negative` | no | n/a | n/a | 72.0 ms |

This model regressed on the real far image, but it detected the live Gazebo frame and completed the robot approach run.

### Custom YOLO, Far Improved

After adding and manually correcting 30 far/small target images, the stable best model detected all saved positives and rejected the negative:

| Image | Detected | Confidence | Distance | Runtime |
| --- | --- | ---: | ---: | ---: |
| `center` | yes | 1.00 | 0.48 m | 125.8 ms |
| `left` | yes | 0.97 | 1.15 m | 105.2 ms |
| `far` | yes | 0.41 | 7.17 m | 66.0 ms |
| `negative` | no | n/a | n/a | 84.0 ms |

Conclusion: the far/small-target training gap was fixed for the saved-image test set. The ONNX distance estimate for `far` is still less reliable than HSV because a neural bounding box is not the same as a clean circular contour radius.

## Robot Trial Result

Analyze:

```bash
python3 tools/analyze_bag.py bags/final_onnx_calibrated_alignment
```

Result:

```text
duration_s: 78.29
target_samples: 251
cmd_samples: 262
initial_x_m: 0.009
final_x_m: 0.000
initial_z_m: 2.472
final_z_m: 0.529
min_z_m: 0.529
max_linear_mps: 1.200
max_angular_radps: 0.016
visual_servo_state_counts:
  APPROACH: 175
success: True
```

Interpretation: the robot started with the ball about 2.47 m away in calibrated ONNX distance, approached using ONNX detections, and stopped centered at about 0.53 m.

## HSV vs Custom YOLO

| Dimension | HSV Detector | Custom YOLO ONNX |
| --- | --- | --- |
| Accuracy on simple red ball | Strong | Strong after training |
| Negative rejection | Good for non-red objects | Good after custom training |
| Far/small target | Sensitive to color/size thresholds | Works after adding far/small examples; still sensitive to dataset balance |
| Sim-to-real behavior | Needs color/lighting tuning | Needs examples from each domain |
| Runtime | Lightweight | About 70-120 ms per saved image on CPU |
| Complexity | Low | High: dataset, labels, training, export, thresholding |
| Debuggability | Easy with HSV masks | Requires dataset inspection and hard-example mining |
| Best use right now | Stable baseline and debugging | Learned detector branch for ML practice and sim trials |

## Lessons Learned

- A neural detector is not automatically better than a handcrafted detector.
- The dataset must match the deployment domain.
- Offline detector success is necessary but not sufficient; the detector must also work inside the live ROS node.
- Type-safe ROS parameters matter. The ONNX node initially crashed because `detector_target_class_names` was declared as a string array but passed as a string.
- Hard-example mining worked: failed live frames were inspected, added to the dataset when valid, and used to improve the detector.
- The backend interface paid off: HSV and ONNX both publish the same `/ball/relative_position` contract, so the controller did not need to change.

## Recommended Next Step

Keep HSV as the default production baseline. Keep ONNX as a launch-selectable ML backend.

Use the final demo report for repeatable runs:

- [final_demo_report.md](final_demo_report.md)

Next useful improvements:

1. Add more varied Gazebo camera frames: left, right, far, near, partial occlusion.
2. Record an ONNX bag with `/odom` and `/ball/annotated_image` for richer analysis.
3. Add map-based obstacle-aware waypoint behavior beyond reactive safety.
