import pytest

cv2 = pytest.importorskip("cv2")
np = pytest.importorskip("numpy")

from vision_guided_robot.manual_label import (  # noqa: E402
    bbox_from_corners,
    bbox_from_xywh,
    infer_label_path,
    infer_preview_path,
    parse_box,
    write_manual_label,
    yolo_line_from_bbox,
)


def test_parse_box_accepts_commas_or_spaces():
    assert parse_box("1,2,3,4") == (1.0, 2.0, 3.0, 4.0)
    assert parse_box("1 2 3 4") == (1.0, 2.0, 3.0, 4.0)


def test_bbox_from_corners_clips_and_sorts():
    bbox = bbox_from_corners(120, 80, -5, 20, image_width_px=100, image_height_px=60)

    assert bbox == (0, 20, 100, 40)


def test_bbox_from_xywh_converts_to_clipped_integer_bbox():
    bbox = bbox_from_xywh(10, 20, 30, 40, image_width_px=100, image_height_px=100)

    assert bbox == (10, 20, 30, 40)


def test_yolo_line_from_bbox_normalizes_values():
    line = yolo_line_from_bbox((80, 60, 40, 20), 200, 100)

    assert line == "0 0.500000 0.700000 0.200000 0.200000\n"


def test_infers_dataset_label_and_preview_paths():
    image_path = (
        "datasets/red_ball_yolo/images/val/red_far.jpeg"
    )

    from pathlib import Path

    path = Path(image_path)
    assert infer_label_path(path) == Path("datasets/red_ball_yolo/labels/val/red_far.txt")
    assert infer_preview_path(path) == Path(
        "datasets/red_ball_yolo/previews/val/red_far_manual_preview.png"
    )


def test_write_manual_label_creates_label_and_preview(tmp_path):
    image_path = tmp_path / "datasets" / "red_ball_yolo" / "images" / "val" / "red_far.jpeg"
    label_path = tmp_path / "datasets" / "red_ball_yolo" / "labels" / "val" / "red_far.txt"
    preview_path = (
        tmp_path
        / "datasets"
        / "red_ball_yolo"
        / "previews"
        / "val"
        / "red_far_manual_preview.png"
    )
    image_path.parent.mkdir(parents=True)
    image = np.zeros((100, 200, 3), dtype=np.uint8)
    cv2.imwrite(str(image_path), image)

    result = write_manual_label(
        image_path,
        label_path,
        preview_path,
        (80, 60, 40, 20),
    )

    assert label_path.read_text(encoding="utf-8") == (
        "0 0.500000 0.700000 0.200000 0.200000\n"
    )
    assert preview_path.exists()
    assert result.bbox_xywh == (80, 60, 40, 20)
