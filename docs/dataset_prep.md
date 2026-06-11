# Dataset Prep Tool

`dataset_prep` creates a YOLO-format dataset for a custom `red_ball` detector.

It is the next step after generic YOLO11n failed to beat HSV.

## What It Does

The tool:

- copies positive images into `images/train` or `images/val`
- copies negative images into `images/train` or `images/val`
- creates YOLO `.txt` label files
- uses HSV to create first-pass labels for positive images
- creates empty label files for negative images
- saves preview images for inspection
- writes `data.yaml`
- writes `manifest.csv`

## Run On Current Saved Images

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run vision_guided_robot dataset_prep \
  --dataset-root datasets/red_ball_yolo \
  --positive train=/path/to/sample_images/red_center.jpg \
  --positive train=/path/to/sample_images/red_left.jpeg \
  --positive val=/path/to/sample_images/red_far.jpeg \
  --negative val=/path/to/sample_images/negative.jpg \
  --min-area 50 \
  --min-circularity 0.30
```

Expected output shape:

```text
split  type      status        confidence  image
-----  --------  ------------  ----------  ----------------
train  positive  auto_labeled  0.90        red_center.jpg
train  positive  auto_labeled  0.74        red_left.jpeg
val    positive  auto_labeled  0.42        red_far.jpeg
val    negative  negative      n/a         negative.jpg
```

## Inspect The Previews

Open the generated previews folder from your file explorer. From WSL, this command opens the right location on Windows:

```bash
explorer.exe "$(wslpath -w ~/vision_guided_robot_ws/datasets/red_ball_yolo/previews)"
```

The previews are not training data. They are visual checks so you can confirm whether the HSV pseudo-label is reasonable.

## Dataset Layout

The command creates:

```text
datasets/red_ball_yolo/
  images/
    train/
    val/
  labels/
    train/
    val/
  previews/
    train/
    val/
  data.yaml
  manifest.csv
```

Positive label files contain one YOLO box:

```text
0 center_x center_y width height
```

Negative label files are intentionally empty.

## Add More Images

Use the same command shape as you collect more examples:

```bash
ros2 run vision_guided_robot dataset_prep \
  --dataset-root datasets/red_ball_yolo \
  --positive train=/path/to/new_red_image.jpg \
  --negative train=/path/to/no_target_image.jpg
```

Use `val=` for validation images:

```bash
--positive val=/path/to/red_validation_image.jpg
```

## Bulk Import Folders

For larger collection, make folders in Windows:

```text
C:\path\to\red_ball_dataset\positive
C:\path\to\red_ball_dataset\negative
```

Put red-target images in `positive` and no-target or wrong-object images in `negative`.

Then import the folders:

```bash
ros2 run vision_guided_robot dataset_prep \
  --dataset-root datasets/red_ball_yolo \
  --positive-dir /path/to/red_ball_dataset/positive \
  --negative-dir /path/to/red_ball_dataset/negative \
  --val-ratio 0.20 \
  --min-area 50 \
  --min-circularity 0.30
```

`--val-ratio 0.20` sends about 20% of the folder images to validation and the rest to training.

After you have manually fixed labels, append new images without touching existing labels:

```bash
ros2 run vision_guided_robot dataset_prep \
  --dataset-root datasets/red_ball_yolo \
  --positive-dir /path/to/red_ball_dataset/positive \
  --negative-dir /path/to/red_ball_dataset/negative \
  --val-ratio 0.20 \
  --min-area 50 \
  --min-circularity 0.30 \
  --skip-existing
```

`--skip-existing` is the safe mode for adding more photos after manual cleanup. It leaves existing copied images, labels, and previews alone.

You can also use glob patterns:

```bash
ros2 run vision_guided_robot dataset_prep \
  --dataset-root datasets/red_ball_yolo \
  --positive-glob "/path/to/red_ball_dataset/positive/*.jpg" \
  --negative-glob "/path/to/red_ball_dataset/negative/*.jpg"
```

Use unique filenames when adding batches. If you intentionally want to replace existing copied images and regenerate their labels, add:

```bash
--overwrite
```

## Important

HSV pseudo-labels are a starting point, not truth. Inspect the previews before training.

If a positive image says `needs_manual_label`, HSV did not find the red target. That image is still useful, but it needs a manually drawn label before training.

## Fix A Bad Auto-Label

If a preview box is on the wrong object, replace that label manually.

For example, if `red_far` is wrong, use the interactive helper:

```bash
ros2 run vision_guided_robot manual_label \
  --image datasets/red_ball_yolo/images/val/red_far.jpeg \
  --interactive
```

Controls:

- left click: first box corner
- left click again: second box corner
- `s` or `Enter`: save
- `r`: reset
- `q` or `Esc`: cancel

The tool writes:

```text
datasets/red_ball_yolo/labels/val/red_far.txt
datasets/red_ball_yolo/previews/val/red_far_manual_preview.png
```

If the image window is too large, scale it down:

```bash
ros2 run vision_guided_robot manual_label \
  --image datasets/red_ball_yolo/images/val/red_far.jpeg \
  --interactive \
  --display-scale 0.5
```

You can also type coordinates directly:

```bash
ros2 run vision_guided_robot manual_label \
  --image datasets/red_ball_yolo/images/val/red_far.jpeg \
  --corners x1,y1,x2,y2
```

or:

```bash
ros2 run vision_guided_robot manual_label \
  --image datasets/red_ball_yolo/images/val/red_far.jpeg \
  --bbox x,y,width,height
```

## Fix Many Bad Auto-Labels

Far/small targets often break HSV pseudo-labeling. If a whole batch is mislabeled, use the batch manual label helper:

```bash
ros2 run vision_guided_robot manual_label_batch \
  --glob "datasets/red_ball_yolo/images/train/YOUR_PREFIX*.jpg" \
  --display-scale 0.5
```

Use the matching split. For validation images:

```bash
ros2 run vision_guided_robot manual_label_batch \
  --glob "datasets/red_ball_yolo/images/val/YOUR_PREFIX*.jpg" \
  --display-scale 0.5
```

For each image:

- left-click two opposite corners around the target
- press `s` or `Enter` to save and move on
- press `r` to reset the current box
- press `q` or `Esc` to cancel the image, then answer `y` if it should be negative

The tool writes the normal YOLO label file and a `_manual_preview.png` next to the existing previews.

After saving, inspect:

```bash
explorer.exe "$(wslpath -w ~/vision_guided_robot_ws/datasets/red_ball_yolo/previews/val)"
```

## Audit The Dataset

After creating or fixing labels, run:

```bash
ros2 run vision_guided_robot dataset_audit \
  --dataset-root datasets/red_ball_yolo \
  --show-issues \
  --report-csv datasets/red_ball_yolo/audit.csv
```

The audit checks:

- every image has a matching label file
- positive label lines have 5 YOLO fields
- normalized box values are between `0.0` and `1.0`
- box width and height are positive
- negative images have empty label files
- train/val counts are large enough for training

With the first few images, `ready_for_training` should still be `False`. That is correct. The audit is telling you the dataset format is valid, but the dataset is too small for training.

For a tiny smoke test, lower the count thresholds:

```bash
ros2 run vision_guided_robot dataset_audit \
  --dataset-root datasets/red_ball_yolo \
  --min-train-positives 2 \
  --min-val-positives 1 \
  --min-negatives 1 \
  --show-issues
```
