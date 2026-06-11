# ONNX Detector Backend

This is the ML detector backend used by the trained red-ball YOLO model.

It uses OpenCV DNN to load a YOLO-style ONNX object detector and converts detections into the same project `Detection` contract used by the HSV backend.

## Current Status

Implemented:

- `onnx`, `ml`, `yolo`, and `yolo_onnx` backend names
- ONNX model loading through OpenCV DNN
- COCO class-name defaults
- target-class filtering, defaulting to `sports ball`
- YOLO-style output parsing
- non-maximum suppression
- distance estimate from bounding-box size
- saved-image evaluation through `detector_evaluator`

Validated:

- custom red-ball YOLO11n model exported to ONNX
- saved-image evaluation with `detector_evaluator`
- live ROS/Gazebo detection through `ball_tracker`
- visual-servo approach using ONNX detections

Best validated model:

```text
models/ml/red_ball_yolo11n_best.onnx
models/ml/red_ball_classes.txt
target class: red_ball
confidence threshold: 0.10
```

## First YOLO11n Result

Model tested:

```text
yolo11n.pt -> yolo11n.onnx
backend: onnx
target class: sports ball
```

Saved-image result:

| Image | Detected | Confidence | Distance | Lateral | Runtime |
| --- | --- | ---: | ---: | ---: | ---: |
| `center` | no | n/a | n/a | n/a | 101.5 ms |
| `left` | no | n/a | n/a | n/a | 69.6 ms |
| `far` | no | n/a | n/a | n/a | 138.6 ms |
| `negative` | yes | 0.79 | 0.40 m | +0.01 m | 105.3 ms |

Conclusion: off-the-shelf COCO YOLO11n is not a good replacement for the HSV detector on this task. It misses the actual red target images and produces a false positive on the negative cyan ball image.

That does not mean ML is useless. It means generic pretrained COCO detection is solving a different problem than this robot needs.

## Offline Evaluation Command

After you have an ONNX object detector model, run:

```bash
ros2 run vision_guided_robot detector_evaluator \
  --backend onnx \
  --model-path /path/to/model.onnx \
  --target-class "sports ball" \
  --image center=/path/to/sample_images/red_center.jpg \
  --image left=/path/to/sample_images/red_left.jpeg \
  --image far=/path/to/sample_images/red_far.jpeg \
  --image negative=/path/to/sample_images/negative.jpg \
  --diameter-m 0.20 \
  --fov-deg 60 \
  --save-dir detector_eval/onnx \
  --csv detector_eval/onnx_results.csv \
  --json detector_eval/onnx_results.json
```

If your model uses a custom class list, add:

```bash
--class-names /path/to/classes.txt
```

If the target class is not named `sports ball`, change:

```bash
--target-class ball
```

## Custom Red-Ball Evaluation

```bash
ros2 run vision_guided_robot detector_evaluator \
  --backend onnx \
  --model-path models/ml/red_ball_yolo11n_best.onnx \
  --class-names models/ml/red_ball_classes.txt \
  --target-class red_ball \
  --confidence-threshold 0.10 \
  --image center=/path/to/sample_images/red_center.jpg \
  --image left=/path/to/sample_images/red_left.jpeg \
  --image far=/path/to/sample_images/red_far.jpeg \
  --image negative=/path/to/sample_images/negative.jpg \
  --diameter-m 0.20 \
  --fov-deg 60 \
  --save-dir detector_eval/real_sim_visible
```

Latest saved-image result:

| Image | Detected | Confidence | Distance | Runtime |
| --- | --- | ---: | ---: | ---: |
| `center` | yes | 1.00 | 0.48 m | 125.8 ms |
| `left` | yes | 0.97 | 1.15 m | 105.2 ms |
| `far` | yes | 0.41 | 7.17 m | 66.0 ms |
| `negative` | no | n/a | n/a | 84.0 ms |

The ONNX far-distance estimate is less reliable than HSV because bounding-box size is not the same as a clean circle radius.

## ROS/Gazebo Command Shape

HSV remains the default baseline. ONNX is launch-selectable:

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
  detector_confidence_threshold:=0.10
```

The published topics should stay unchanged:

- `/ball/relative_position`
- `/ball/annotated_image`

Successful robot trial:

```text
bag: bags/final_onnx_far_improved
initial_z_m: 1.788
final_z_m: 0.488
success: True
```

## Debugging Note

If `/ball_tracker` is missing after launching ONNX mode, check for parameter type errors. The node now declares `detector_target_class_names` as a string so launch arguments like `detector_target_class_names:=red_ball` work correctly.

## Robotics Concept

The neural detector only replaces this part:

```text
image -> detector backend -> Detection | None
```

It does not replace:

- camera calibration
- distance geometry
- control
- safety filtering
- odometry
- mission logic

That separation is the reason the backend interface was built first.
