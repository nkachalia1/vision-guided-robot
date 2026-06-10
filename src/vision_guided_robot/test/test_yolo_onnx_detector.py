import pytest

cv2 = pytest.importorskip("cv2")
np = pytest.importorskip("numpy")

from vision_guided_robot.red_ball_detector import DetectorConfig  # noqa: E402
from vision_guided_robot.yolo_onnx_detector import (  # noqa: E402
    COCO_CLASS_NAMES,
    detection_from_candidates,
    load_class_names,
    parse_yolo_candidates,
    target_class_ids,
)


def test_target_class_ids_finds_coco_sports_ball():
    ids = target_class_ids(COCO_CLASS_NAMES, ["sports ball"])

    assert ids == {32}


def test_parse_yolo_v8_style_output_returns_candidate():
    output = np.zeros((1, 84, 1), dtype=np.float32)
    output[0, 0, 0] = 320.0
    output[0, 1, 0] = 240.0
    output[0, 2, 0] = 80.0
    output[0, 3, 0] = 80.0
    output[0, 4 + 32, 0] = 0.90

    candidates = parse_yolo_candidates(
        output,
        image_shape=(480, 640, 3),
        input_size_px=640,
        class_names=COCO_CLASS_NAMES,
        target_class_names=("sports ball",),
        confidence_threshold=0.25,
    )

    assert len(candidates) == 1
    assert candidates[0].confidence == pytest.approx(0.90)
    assert candidates[0].bbox == (280, 150, 80, 60)


def test_detection_from_candidates_estimates_distance():
    output = np.zeros((1, 84, 1), dtype=np.float32)
    output[0, 0, 0] = 320.0
    output[0, 1, 0] = 320.0
    output[0, 2, 0] = 80.0
    output[0, 3, 0] = 80.0
    output[0, 4 + 32, 0] = 0.90
    candidates = parse_yolo_candidates(
        output,
        image_shape=(640, 640, 3),
        input_size_px=640,
        class_names=COCO_CLASS_NAMES,
        target_class_names=("sports ball",),
        confidence_threshold=0.25,
    )

    detection = detection_from_candidates(
        candidates,
        image_width_px=640,
        detector_config=DetectorConfig(real_diameter_m=0.20),
        nms_threshold=0.45,
    )

    assert detection is not None
    assert detection.center_x == 320
    assert detection.center_y == 320
    assert detection.radius_px == pytest.approx(40.0)
    assert detection.distance_m is not None


def test_load_class_names_uses_file_when_provided(tmp_path):
    class_file = tmp_path / "classes.txt"
    class_file.write_text("ball\ncube\n", encoding="utf-8")

    assert load_class_names(str(class_file)) == ("ball", "cube")
