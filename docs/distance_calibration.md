# Distance Calibration

The detector estimates distance with a pinhole camera model:

```text
distance_m = real_diameter_m * focal_length_px / measured_diameter_px
```

This works only if the measured pixel diameter matches the real visible diameter. HSV contour radius and YOLO bounding-box size can disagree, especially for far or partially cropped targets.

Use `distance_calibrator` to estimate an effective diameter from images where you know the true target distance.

## When To Use This

Use this after the detector can find the ball but reports bad distances.

Example symptom:

```text
HSV far distance: 3.83 m
ONNX far distance: 7.17 m
```

That means the ONNX bounding box is probably smaller than the contour-based diameter, so the pinhole equation thinks the object is farther away.

## Capture Known-Distance Images

For Gazebo, launch the ball at known positions and save one camera frame per pose.

Example:

```bash
ros2 launch vision_guided_robot sim.launch.py \
  control_mode:=vision \
  ball_x:=1.0 \
  ball_y:=0.0 \
  detector_backend:=hsv
```

In another terminal:

```bash
ros2 run vision_guided_robot ros_image_capture \
  --topic /camera/image \
  --output-dir calibration_images \
  --prefix ball_1p0 \
  --count 1 \
  --every-n 1 \
  --max-seconds 10
```

Repeat at several distances such as `1.0`, `1.5`, `2.0`, and `2.5` meters.

## Run Calibration

Use the current best ONNX model:

```bash
ros2 run vision_guided_robot distance_calibrator \
  --backend onnx \
  --model-path models/ml/red_ball_yolo11n_best.onnx \
  --class-names models/ml/red_ball_classes.txt \
  --target-class red_ball \
  --confidence-threshold 0.10 \
  --sample ball_1p0=calibration_images/ball_1p0_0001.jpg:1.0 \
  --sample ball_1p5=calibration_images/ball_1p5_0001.jpg:1.5 \
  --sample ball_2p0=calibration_images/ball_2p0_0001.jpg:2.0 \
  --sample ball_2p5=calibration_images/ball_2p5_0001.jpg:2.5 \
  --diameter-m 0.20 \
  --fov-deg 60 \
  --csv detector_eval/distance_calibration.csv \
  --json detector_eval/distance_calibration.json
```

The output includes:

- known distance
- estimated distance
- error
- measured detector diameter in pixels
- effective diameter per sample
- recommended median effective diameter

Current ONNX calibration result:

```text
configured_diameter_m: 0.200
recommended_diameter_m: 0.272
median_abs_error_m: 0.418
median_abs_error_pct: 26.2
```

## How To Interpret It

If `recommended_diameter_m` differs from `0.20`, use that as a temporary calibration parameter for ONNX trials:

```bash
ros2 launch vision_guided_robot sim.launch.py \
  control_mode:=vision \
  detector_backend:=onnx \
  detector_model_path:=$(pwd)/models/ml/red_ball_yolo11n_best.onnx \
  detector_class_names_path:=$(pwd)/models/ml/red_ball_classes.txt \
  detector_target_class_names:=red_ball \
  detector_confidence_threshold:=0.10 \
  detector_real_diameter_m:=0.272
```

This does not change the neural network. It changes the metric geometry layer that converts pixel size into meters.

Validated calibrated run:

```text
bag: bags/final_onnx_calibrated_alignment
success: True
initial_z_m: 2.472
final_z_m: 0.529
final_x_m: 0.000
```

## Robotics Concept

Object detection gives an image-space box. Control needs a metric-space estimate.

The conversion depends on:

- camera field of view
- image width
- physical target size
- how the detector measures the object

This is why perception and geometry are separate engineering problems.
