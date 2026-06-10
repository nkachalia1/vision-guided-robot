# ML Detector Replacement Plan

The current detector is intentionally handcrafted:

```text
HSV threshold -> mask cleanup -> contours -> circularity filter -> distance estimate
```

Do not replace it just because ML sounds more advanced. Replace it only when the neural detector clearly improves real behavior.

## Goal

Replace the HSV red-object detector with a neural-network detector while keeping the rest of the robot architecture stable:

```text
camera image -> detector -> /ball/relative_position -> controller -> safety_filter -> robot
```

The first ML milestone is not autonomous general object recognition. It is a fair comparison against the existing red-object baseline.

## Baseline To Beat

Current saved-image results:

| Case | HSV Result | YOLO11n ONNX Result |
| --- | --- | --- |
| centered red object | detected, confidence 0.90 | missed |
| red object at left edge | detected, confidence 0.74 | missed |
| far/small red object | detected, confidence 0.42 | missed |
| blue/cyan negative object | correctly rejected | false positive, confidence 0.79 |

Current conclusion: HSV is still the stronger detector for this specific red-target task. Generic COCO YOLO11n is not task-specific enough.

Current simulation behavior:

- robot detects the red ball
- tracks it
- approaches it
- stops near it
- recovers from some occlusion and obstacle cases

## Comparison Metrics

Use the same images and simulation runs for both detectors.

| Metric | HSV Detector | ML Detector | Notes |
| --- | ---: | ---: | --- |
| centered object detected | yes/no | yes/no | Must not regress. |
| edge object detected | yes/no | yes/no | ML should improve partial-object robustness. |
| far object detected | yes/no | yes/no | ML should improve low-pixel cases. |
| negative object rejected | yes/no | yes/no | Must avoid false positives. |
| average FPS |  |  | Measure on the same machine. |
| install complexity | low |  | Count new dependencies and setup steps. |
| ROS integration complexity | low |  | Keep the published topic contract stable. |
| simulation approach success | yes/no | yes/no | Robot must still reach and stop. |

## Architecture Requirement

Keep detector implementations behind the same conceptual interface:

```text
detect(image) -> Detection | None
```

The controller should not care whether the detection came from HSV contours or a neural network.

The ROS node should keep publishing:

- `/ball/relative_position`
- `/ball/annotated_image`

## Phased Plan

### Phase 1: Detector Interface Cleanup

Goal: make the current detector easier to swap.

Acceptance criteria:

- HSV detector still passes all current tests.
- Webcam and Gazebo workflows still work.
- A detector backend can be selected by parameter or CLI option.

Implementation status: done and user-validated in Ubuntu.

Current backend selection:

```bash
ros2 launch vision_guided_robot sim.launch.py detector_backend:=hsv
```

```bash
ros2 run vision_guided_robot webcam_detector \
  --backend hsv \
  --image /mnt/c/Users/Neel/Pictures/red_center.jpg \
  --no-display
```

The backend factory lives in `detector_backend.py`. It supports `hsv` and ONNX YOLO through names such as `onnx`, `ml`, `yolo`, and `yolo_onnx`. ONNX-style names require a model path.

### Phase 1.5: Offline Detector Evaluation Harness

Goal: create the scoreboard that HSV and ML backends must both run through.

Acceptance criteria:

- A single command evaluates the same saved images with a selected backend.
- Output includes detected yes/no, confidence, distance, lateral offset, and runtime.
- Annotated images can be saved for visual inspection.
- CSV or JSON can be saved for later comparison.

Current command:

```bash
ros2 run vision_guided_robot detector_evaluator \
  --backend hsv \
  --image center=/mnt/c/Users/Neel/Pictures/red_center.jpg \
  --image left=/mnt/c/Users/Neel/Pictures/red_left.jpeg \
  --image far=/mnt/c/Users/Neel/Pictures/red_far.jpeg \
  --image negative=/mnt/c/Users/Neel/Pictures/negative.jpg \
  --diameter-m 0.20 \
  --fov-deg 60 \
  --min-area 50 \
  --min-circularity 0.30 \
  --save-dir detector_eval/hsv \
  --csv detector_eval/hsv_results.csv \
  --json detector_eval/hsv_results.json
```

### Phase 2: Offline ML Prototype

Goal: run an ML detector on saved images, outside ROS first.

Acceptance criteria:

- The same four saved-image cases can be evaluated.
- Output includes bounding box, confidence, center pixel, approximate radius or box size, and runtime.
- Results are saved as annotated images.

Implementation status: done.

Current backend shape:

```bash
ros2 run vision_guided_robot detector_evaluator \
  --backend onnx \
  --model-path /path/to/model.onnx \
  --target-class "sports ball" \
  --image center=/mnt/c/Users/Neel/Pictures/red_center.jpg
```

This uses OpenCV DNN and a YOLO-style ONNX output parser. Generic COCO YOLO11n was rejected, then a custom `red_ball` YOLO11n model was trained and exported to ONNX.

### Phase 3: ROS Integration

Goal: allow `ball_tracker` to use the ML backend.

Acceptance criteria:

- `/ball/relative_position` message format does not change.
- `/ball/annotated_image` still shows useful overlays.
- Launch/config can choose `detector_backend:=hsv` or `detector_backend:=ml`.

Implementation status: done. The final ONNX launch uses:

```bash
ros2 launch vision_guided_robot sim.launch.py \
  control_mode:=vision \
  detector_backend:=onnx \
  detector_model_path:=$(pwd)/models/ml/red_ball_yolo11n_best.onnx \
  detector_class_names_path:=$(pwd)/models/ml/red_ball_classes.txt \
  detector_target_class_names:=red_ball \
  detector_confidence_threshold:=0.10
```

### Phase 4: Behavior Comparison

Goal: prove whether ML actually helps the robot.

Acceptance criteria:

- Run the same Gazebo approach test with HSV and ML.
- Run the same occlusion or small-target test with HSV and ML.
- Compare success, time to stop, target samples, and command stability.

Implementation status: core approach comparison done. The current best ONNX approach bag `bags/final_onnx_far_improved` succeeded from `initial_z_m: 1.788` to `final_z_m: 0.488`.

## Tradeoffs To Watch

ML advantages:

- More robust to lighting and partial occlusion if trained well.
- Can detect object category instead of only color.
- Can reject red non-ball clutter better if the training data covers it.

ML costs:

- More dependencies.
- More compute.
- Harder debugging.
- Model confidence is not the same thing as geometric accuracy.
- Distance still needs camera geometry or depth, even if detection improves.

## Recommendation

Keep HSV as the default production baseline, and keep ONNX as a selectable learned detector backend.

The custom dataset workflow is complete for the first ML milestone:

1. Positive and negative images were collected.
2. Labels were prepared and manually cleaned.
3. A small YOLO model was trained for one class: `red_ball`.
4. The model was exported to ONNX.
5. Saved-image evaluation and Gazebo robot trials were run.

Next ML improvements:

1. Add more far/small real images.
2. Add more varied Gazebo frames.
3. Compare HSV and ONNX on occlusion runs, not only direct approach.
