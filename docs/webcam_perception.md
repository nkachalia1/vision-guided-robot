# Webcam Perception

This step moves perception out of Gazebo and onto a real camera or saved image.

The same `RedBallDetector` core is used by:

- `ball_tracker` in ROS/Gazebo
- `webcam_detector` for standalone OpenCV practice

That lets you tune and understand the vision system without running the full robot simulation.

## Run A Live Webcam

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run vision_guided_robot webcam_detector \
  --backend hsv \
  --camera 0 \
  --width 640 \
  --height 480 \
  --diameter-m 0.20 \
  --fov-deg 60 \
  --show-mask
```

Controls:

- `q` or `Esc`: quit
- `s`: save a snapshot, if `--save-dir` is set

Example with snapshots:

```bash
ros2 run vision_guided_robot webcam_detector \
  --backend hsv \
  --camera 0 \
  --show-mask \
  --save-dir webcam_debug
```

## Run On A Saved Image

```bash
ros2 run vision_guided_robot webcam_detector \
  --backend hsv \
  --image path/to/test_image.jpg \
  --save-dir webcam_debug \
  --no-display
```

This writes:

```text
webcam_debug/test_image_annotated.png
webcam_debug/test_image_mask.png
```

## Tune Detection

If the detector misses the object:

```bash
ros2 run vision_guided_robot webcam_detector \
  --backend hsv \
  --camera 0 \
  --show-mask \
  --min-area 50 \
  --min-circularity 0.35
```

If the distance estimate is wrong:

```bash
ros2 run vision_guided_robot webcam_detector \
  --backend hsv \
  --camera 0 \
  --diameter-m 0.07 \
  --fov-deg 60
```

`--diameter-m` must match the real object's physical diameter. `--fov-deg` should match the webcam horizontal field of view.

## What The Overlay Means

The overlay reports:

- `center`: object center in image pixels
- `radius`: apparent object radius in pixels
- `confidence`: circularity score, clamped to `0.0` through `1.0`
- `distance`: estimated forward distance using the pinhole camera model
- `lateral`: estimated left/right offset from the camera centerline

Positive `lateral` means the object appears to the right side of the image. Negative means it appears to the left.

## Robotics Concept

This is still not object recognition in the ML sense. It is color segmentation plus geometry:

```text
HSV threshold -> binary mask -> contours -> circularity filter -> distance estimate
```

The distance model is:

```text
focal_px = image_width / (2 * tan(horizontal_fov / 2))
distance_m = real_diameter_m * focal_px / measured_diameter_px
```

This is why calibration matters. If the object's real diameter or camera FOV is wrong, detection can be perfect while distance is wrong.

## Detector Backend

`--backend hsv` selects the current handcrafted detector. The argument exists so saved-image and webcam tests use the same backend-selection pattern as the ROS node.

`--backend onnx` selects the OpenCV DNN ONNX backend scaffold. It requires `--model-path /path/to/model.onnx` and is meant for offline ML experiments before making ML part of the robot behavior.
