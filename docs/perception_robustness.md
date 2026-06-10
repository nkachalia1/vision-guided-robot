# Perception Robustness

The control loop works when the target is easy. This milestone asks a more realistic question:

```text
When does the vision system stop being trustworthy?
```

For real-camera or saved-image perception debugging, use `docs/webcam_perception.md`. The same detector core is used in both workflows.

Record every run with:

```bash
ros2 bag record -o bags/<bag_name> \
  /ball/relative_position \
  /cmd_vel \
  /visual_servo/state
```

Analyze every run with:

```bash
python3 tools/analyze_bag.py bags/<bag_name>
```

## Baseline

```bash
ros2 launch vision_guided_robot sim.launch.py \
  ball_x:=3.0 ball_y:=1.5 \
  target_model:=red_ball \
  detector_real_diameter_m:=0.20
```

Expected: success.

## Small Ball, Correct Calibration

```bash
ros2 launch vision_guided_robot sim.launch.py \
  ball_x:=3.0 ball_y:=1.5 \
  target_model:=small_red_ball \
  detector_real_diameter_m:=0.12 \
  detector_min_area_px:=50.0
```

Expected: should still work, but detection may be noisier because the target occupies fewer pixels.

## Small Ball, Wrong Calibration

```bash
ros2 launch vision_guided_robot sim.launch.py \
  ball_x:=3.0 ball_y:=1.5 \
  target_model:=small_red_ball \
  detector_real_diameter_m:=0.20 \
  detector_min_area_px:=50.0
```

Expected: color detection may work, but distance estimation is wrong. This teaches that perception is not just detection; geometry calibration matters.

## Dark Red Ball

```bash
ros2 launch vision_guided_robot sim.launch.py \
  ball_x:=3.0 ball_y:=1.5 \
  target_model:=dark_red_ball \
  detector_real_diameter_m:=0.20
```

Expected: may work or flicker depending on brightness. If it fails, inspect `/ball/annotated_image` and tune HSV value/saturation thresholds.

## Dim Lighting

```bash
DIM_WORLD="$(ros2 pkg prefix vision_guided_robot)/share/vision_guided_robot/worlds/red_ball_world_dim.sdf"

ros2 launch vision_guided_robot sim.launch.py \
  world:="$DIM_WORLD" \
  ball_x:=3.0 ball_y:=1.5 \
  target_model:=red_ball \
  detector_real_diameter_m:=0.20
```

Expected: may reduce detector confidence or cause intermittent target loss.

## Blue Ball Negative Test

```bash
ros2 launch vision_guided_robot sim.launch.py \
  ball_x:=2.0 ball_y:=0.0 \
  target_model:=blue_ball
```

Expected: failure is correct. The current detector is a red-object detector, not a generic ball detector.

## Partial Occlusion

```bash
ros2 launch vision_guided_robot sim.launch.py \
  ball_x:=3.0 ball_y:=1.5 \
  target_model:=red_ball \
  spawn_occluder:=true \
  occluder_x:=1.6 occluder_y:=0.8 occluder_z:=0.3
```

Expected: detector may flicker or lose the target. Watch whether the state machine recovers through `SEARCH` or `TRACK`.

If color-only detection works but occlusion fails, test whether the circularity filter is too strict:

```bash
ros2 launch vision_guided_robot sim.launch.py \
  ball_x:=3.0 ball_y:=1.5 \
  target_model:=dark_red_ball \
  detector_real_diameter_m:=0.20 \
  detector_min_area_px:=40.0 \
  detector_min_circularity:=0.20 \
  spawn_occluder:=true \
  occluder_x:=1.6 occluder_y:=0.8 occluder_z:=0.3
```

If this works, the lesson is precise: the target color was visible, but the contour was no longer circular enough after occlusion. That is a shape-filter failure, not a color-threshold failure.

## Results Table

| Test | Bag | Target Samples | First Stop Time | State Timing | Success | Notes |
| --- | --- | ---: | ---: | --- | --- | --- |
| baseline | `bags/robust_baseline` |  |  |  |  |  |
| small calibrated | `bags/robust_small_calibrated` |  |  |  |  |  |
| small wrong calibration | `bags/robust_small_wrong_calib` |  |  |  |  |  |
| dark red | `bags/robust_dark_red` |  |  |  |  |  |
| dim lighting | `bags/robust_dim_lighting` |  |  |  |  |  |
| blue negative | `bags/robust_blue_negative` |  |  |  |  |  |
| occlusion | `bags/robust_occlusion` |  |  |  |  |  |

## Saved-Image Webcam Results

These results were produced with:

```bash
ros2 run vision_guided_robot webcam_detector \
  --image <image_path> \
  --diameter-m 0.20 \
  --fov-deg 60 \
  --min-area 50 \
  --min-circularity 0.30 \
  --save-dir webcam_debug/<case> \
  --no-display
```

| Test | Image | Detected | Confidence | Distance | Lateral | Notes |
| --- | --- | --- | ---: | ---: | ---: | --- |
| centered red object | `red_center.jpg` | yes | 0.90 | 0.48 m | +0.00 m | Clean detection; good baseline. |
| red object at left edge | `red_left.jpeg` | yes | 0.74 | 1.10 m | -0.61 m | Partial/edge object still detected, with lower confidence. |
| far/small red object | `red_far.jpeg` | yes | 0.42 | 3.83 m | +1.49 m | Detection works, but confidence drops significantly. |
| blue/cyan negative object | `negative.jpg` | no | n/a | n/a | n/a | Correct rejection by red-object detector. |

The distance values assume the red target is physically `0.20 m` wide. If the object is not 20 cm, the absolute distance is scaled incorrectly, but the trend is still useful: a smaller pixel radius produces a larger estimated distance.

## HSV Baseline Before ML

The handcrafted detector is good enough to define a baseline:

- It handles a clean centered red object.
- It handles a partially visible edge object, but confidence drops.
- It handles a small/far target, but confidence drops sharply.
- It correctly rejects a blue/cyan object.

Any ML detector replacement should beat this baseline without making the ROS integration much harder.

## Robotics Concepts

- Detection asks: is the object visible?
- Estimation asks: where is it and how far away is it?
- Calibration asks: are the assumptions behind the estimate correct?
- Robustness asks: how does behavior degrade when the assumptions are stressed?

The OpenCV detector is intentionally simple. The purpose of these tests is to learn its failure modes before replacing it with a neural detector.
