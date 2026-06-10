import pytest

cv2 = pytest.importorskip("cv2")
np = pytest.importorskip("numpy")

from vision_guided_robot.dataset_prep import (  # noqa: E402
    DatasetItem,
    parse_dataset_items,
    prepare_dataset,
    safe_stem,
    yolo_label_line,
)
from vision_guided_robot.red_ball_detector import DetectorConfig, RedBallDetector  # noqa: E402


def test_yolo_label_line_normalizes_bbox():
    detection = type(
        "DetectionLike",
        (),
        {"bbox": (80, 60, 40, 20)},
    )()

    line = yolo_label_line(detection, image_width_px=200, image_height_px=100)

    assert line == "0 0.500000 0.700000 0.200000 0.200000\n"


def test_parse_dataset_items_accepts_split_prefixes():
    items = parse_dataset_items(
        positives=["train=/tmp/red.jpg", "val=/tmp/red_val.jpg"],
        negatives=["/tmp/negative.jpg"],
        default_split="train",
    )

    assert items[0].split == "train"
    assert items[0].is_positive
    assert items[1].split == "val"
    assert items[2].split == "train"
    assert not items[2].is_positive


def test_parse_dataset_items_accepts_directories_and_auto_split(tmp_path):
    positive_dir = tmp_path / "positive"
    negative_dir = tmp_path / "negative"
    positive_dir.mkdir()
    negative_dir.mkdir()
    image = np.zeros((10, 10, 3), dtype=np.uint8)

    for index in range(5):
        cv2.imwrite(str(positive_dir / f"red_{index}.jpg"), image)
    for index in range(2):
        cv2.imwrite(str(negative_dir / f"negative_{index}.jpg"), image)

    items = parse_dataset_items(
        positives=None,
        negatives=None,
        positive_dirs=[str(positive_dir)],
        negative_dirs=[str(negative_dir)],
        val_ratio=0.20,
    )

    positive_items = [item for item in items if item.is_positive]
    negative_items = [item for item in items if not item.is_positive]

    assert len(positive_items) == 5
    assert sum(item.split == "train" for item in positive_items) == 4
    assert sum(item.split == "val" for item in positive_items) == 1
    assert len(negative_items) == 2
    assert sum(item.split == "train" for item in negative_items) == 1
    assert sum(item.split == "val" for item in negative_items) == 1


def test_prepare_dataset_auto_labels_positive_and_empty_negative(tmp_path):
    positive_path = tmp_path / "red target.png"
    negative_path = tmp_path / "negative.png"
    dataset_root = tmp_path / "dataset"

    positive = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.circle(positive, (160, 120), 30, (0, 0, 255), -1)
    negative = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.imwrite(str(positive_path), positive)
    cv2.imwrite(str(negative_path), negative)

    detector = RedBallDetector(DetectorConfig(min_area_px=50.0, min_circularity=0.30))
    prepared = prepare_dataset(
        [
            DatasetItem(positive_path, "train", True),
            DatasetItem(negative_path, "val", False),
        ],
        dataset_root,
        detector,
    )

    positive_label = dataset_root / "labels" / "train" / "red_target.txt"
    negative_label = dataset_root / "labels" / "val" / "negative.txt"

    assert prepared[0].status == "auto_labeled"
    assert positive_label.read_text(encoding="utf-8").startswith("0 ")
    assert prepared[1].status == "negative"
    assert negative_label.read_text(encoding="utf-8") == ""
    assert (dataset_root / "data.yaml").exists()
    assert (dataset_root / "manifest.csv").exists()
    assert (dataset_root / "previews" / "train" / "red_target_preview.png").exists()
    assert (dataset_root / "previews" / "val" / "negative_preview.png").exists()


def test_prepare_dataset_marks_positive_without_detection_for_manual_label(tmp_path):
    image_path = tmp_path / "empty.png"
    dataset_root = tmp_path / "dataset"
    cv2.imwrite(str(image_path), np.zeros((240, 320, 3), dtype=np.uint8))

    detector = RedBallDetector(DetectorConfig(min_area_px=50.0, min_circularity=0.30))
    prepared = prepare_dataset(
        [DatasetItem(image_path, "train", True)],
        dataset_root,
        detector,
    )

    assert prepared[0].status == "needs_manual_label"
    assert (dataset_root / "labels" / "train" / "empty.txt").read_text(
        encoding="utf-8"
    ) == ""


def test_prepare_dataset_skip_existing_preserves_manual_label(tmp_path):
    image_path = tmp_path / "red target.png"
    dataset_root = tmp_path / "dataset"

    positive = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.circle(positive, (160, 120), 30, (0, 0, 255), -1)
    cv2.imwrite(str(image_path), positive)

    detector = RedBallDetector(DetectorConfig(min_area_px=50.0, min_circularity=0.30))
    prepare_dataset([DatasetItem(image_path, "train", True)], dataset_root, detector)

    label_path = dataset_root / "labels" / "train" / "red_target.txt"
    label_path.write_text("0 0.111111 0.222222 0.333333 0.444444\n", encoding="utf-8")

    prepared = prepare_dataset(
        [DatasetItem(image_path, "train", True)],
        dataset_root,
        detector,
        skip_existing=True,
    )

    assert prepared[0].status == "skipped_existing"
    assert label_path.read_text(encoding="utf-8") == (
        "0 0.111111 0.222222 0.333333 0.444444\n"
    )


def test_safe_stem_removes_unfriendly_characters():
    assert safe_stem("red target (left)") == "red_target__left"
