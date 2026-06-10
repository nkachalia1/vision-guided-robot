# Custom ML Dataset Plan

The first generic ML model did not beat HSV:

- YOLO11n missed the red target images.
- YOLO11n produced a false positive on the negative cyan ball image.

That result is useful. It means the robot's task is not generic `sports ball` detection. The task is custom red-target detection under your camera and lighting conditions.

## Goal

Create a small custom dataset for one class:

```text
red_ball
```

Then fine-tune a small detector and compare it against HSV using `detector_evaluator`.

## Dataset Targets

Start small but varied:

| Split | Positive Images | Negative Images |
| --- | ---: | ---: |
| train | 40-80 | 20-40 |
| val | 10-20 | 5-10 |

Positive images should include:

- centered red target
- left/right/top/bottom edge positions
- near and far distances
- different lighting
- partial occlusion
- cluttered backgrounds
- screenshots or monitor views, if those are part of the real use case

Negative images should include:

- blue/cyan balls
- red clutter that is not the target
- empty backgrounds
- laptop/desk scenes without the target
- shiny objects and screen glare

## Label Format

Use YOLO label format:

```text
class_id center_x center_y width height
```

Values are normalized from `0.0` to `1.0`.

For one class, `class_id` is always `0`.

Example:

```text
0 0.500 0.520 0.180 0.180
```

## Suggested Folder Layout

```text
datasets/red_ball_yolo/
  images/
    train/
    val/
  labels/
    train/
    val/
  data.yaml
```

`data.yaml` should look like:

```yaml
path: datasets/red_ball_yolo
train: images/train
val: images/val
names:
  0: red_ball
```

## Recommended Next Step

Before training, use `dataset_prep` to:

1. copy selected images into the YOLO folder layout
2. create empty label files for negative images
3. optionally use the HSV detector to generate first-pass labels for obvious positives
4. save an annotated preview so labels can be inspected
5. bulk-import positive and negative folders with an automatic train/val split

HSV pseudo-labels are not perfect, but they are a useful starting point because HSV already works well on the current red-target images.

See `docs/dataset_prep.md` for commands.

## Acceptance Criteria

This dataset step is complete when:

- at least 50 labeled positive images exist
- at least 20 negative images exist
- every image has a matching `.txt` label file
- a human has inspected the label previews
- `data.yaml` exists

Only then should training begin.
