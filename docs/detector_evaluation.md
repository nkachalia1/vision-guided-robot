# Detector Evaluation

This tool creates a repeatable scoreboard for detector backends.

Use it before adding ML so the HSV detector has a fair baseline.

## Run The HSV Baseline

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run vision_guided_robot detector_evaluator \
  --backend hsv \
  --image center=/path/to/sample_images/red_center.jpg \
  --image left=/path/to/sample_images/red_left.jpeg \
  --image far=/path/to/sample_images/red_far.jpeg \
  --image negative=/path/to/sample_images/negative.jpg \
  --diameter-m 0.20 \
  --fov-deg 60 \
  --min-area 50 \
  --min-circularity 0.30 \
  --save-dir detector_eval/hsv \
  --save-mask \
  --csv detector_eval/hsv_results.csv \
  --json detector_eval/hsv_results.json
```

Expected output shape:

```text
image     backend  detected  confidence  distance_m  lateral_m  runtime_ms  error
--------  -------  --------  ----------  ----------  ---------  ----------  -----
center    hsv      yes       0.90        0.48        +0.00      ...
left      hsv      yes       0.74        1.10        -0.61      ...
far       hsv      yes       0.42        3.83        +1.49      ...
negative  hsv      no        n/a         n/a         n/a        ...
```

## Outputs

The command writes:

- annotated images in `detector_eval/hsv`
- optional masks in `detector_eval/hsv`
- CSV results for spreadsheet-style comparison
- JSON results for scripts

## What To Compare Later

When an ONNX model is available, rerun the same images with:

```bash
ros2 run vision_guided_robot detector_evaluator \
  --backend onnx \
  --model-path /path/to/model.onnx \
  --target-class "sports ball" \
  --image center=/path/to/sample_images/red_center.jpg \
  --image left=/path/to/sample_images/red_left.jpeg \
  --image far=/path/to/sample_images/red_far.jpeg \
  --image negative=/path/to/sample_images/negative.jpg
```

For now, `--backend onnx` requires you to provide a model path. See `docs/ml_onnx_backend.md`.

## First HSV vs ONNX Comparison

The first ONNX run used YOLO11n exported to ONNX:

```text
model: models/ml/yolo11n.onnx
backend: onnx
target class: sports ball
```

Result:

| Image | HSV | YOLO11n ONNX |
| --- | --- | --- |
| `center` | detected | missed |
| `left` | detected | missed |
| `far` | detected | missed |
| `negative` | rejected | false positive |

Interpretation: generic object detection did not beat the handcrafted detector. For this robot, color-specific HSV is currently more reliable than off-the-shelf COCO `sports ball` detection.

## Custom YOLO Comparison

After creating a YOLO-format `red_ball` dataset, manually cleaning labels, training YOLO11n, exporting ONNX, and adding Gazebo camera frames, the custom detector became usable in the live robot loop.

Best live model:

```text
models/ml/red_ball_yolo11n_best.onnx
class names: models/ml/red_ball_classes.txt
target class: red_ball
confidence threshold: 0.10
```

Saved-image evaluation for the latest live model:

| Image | Custom YOLO ONNX |
| --- | --- |
| `center` | detected, confidence 1.00 |
| `left` | detected, confidence 0.97 |
| `far` | detected, confidence 0.41 |
| `negative` | rejected |

Robot validation:

```text
bag: bags/final_onnx_far_improved
initial_z_m: 1.788
final_z_m: 0.488
success: True
```

Interpretation: the custom ONNX backend is successful in Gazebo vision mode and now detects the current saved-image positive set. HSV remains faster and simpler; ONNX is the learned detector branch.

See `docs/ml_detector_comparison.md` for the full comparison.

## One-Command HSV vs ONNX Comparison

Use `detector_compare` when you want one combined table for both detectors:

```bash
ros2 run vision_guided_robot detector_compare \
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
  --save-dir detector_eval/hsv_vs_onnx \
  --csv detector_eval/hsv_vs_onnx_results.csv \
  --json detector_eval/hsv_vs_onnx_results.json
```

The command writes separate annotated output folders:

```text
detector_eval/hsv_vs_onnx/hsv
detector_eval/hsv_vs_onnx/onnx
```

## Robotics Concept

This separates perception evaluation from robot behavior.

The robot can fail because perception is wrong, control is wrong, or the scenario is hard. Offline detector evaluation isolates only the perception part:

```text
saved image -> detector backend -> detection table
```

That is how robotics engineers avoid guessing which subsystem caused the failure.
