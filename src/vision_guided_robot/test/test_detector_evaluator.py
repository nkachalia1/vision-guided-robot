import pytest

cv2 = pytest.importorskip("cv2")
np = pytest.importorskip("numpy")

from vision_guided_robot.detector_backend import create_detector_backend  # noqa: E402
from vision_guided_robot.detector_evaluator import (  # noqa: E402
    EvaluationCase,
    evaluate_cases,
    format_table,
    parse_cases,
    safe_filename,
)
from vision_guided_robot.red_ball_detector import DetectorConfig  # noqa: E402


def test_parse_cases_accepts_optional_labels():
    cases = parse_cases(["center=/tmp/red_center.jpg", "/tmp/red_left.jpeg"])

    assert cases[0].label == "center"
    assert str(cases[0].path) == "/tmp/red_center.jpg"
    assert cases[1].label == "red_left"


def test_evaluate_cases_reports_detection_and_missing_target(tmp_path):
    red_path = tmp_path / "red.png"
    negative_path = tmp_path / "negative.png"

    red = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.circle(red, (190, 120), 30, (0, 0, 255), -1)
    cv2.imwrite(str(red_path), red)
    cv2.imwrite(str(negative_path), np.zeros((240, 320, 3), dtype=np.uint8))

    detector = create_detector_backend("hsv", DetectorConfig(min_area_px=50.0))
    results = evaluate_cases(
        [
            EvaluationCase("red", red_path),
            EvaluationCase("negative", negative_path),
        ],
        detector,
        "hsv",
    )

    assert results[0].detected
    assert results[0].confidence is not None
    assert results[0].distance_m is not None
    assert results[0].lateral_m is not None
    assert results[0].runtime_ms is not None
    assert not results[1].detected


def test_evaluate_cases_saves_annotated_outputs(tmp_path):
    image_path = tmp_path / "red.png"
    save_dir = tmp_path / "outputs"

    image = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.circle(image, (160, 120), 30, (0, 0, 255), -1)
    cv2.imwrite(str(image_path), image)

    detector = create_detector_backend("hsv", DetectorConfig(min_area_px=50.0))
    evaluate_cases(
        [EvaluationCase("red ball", image_path)],
        detector,
        "hsv",
        save_dir=save_dir,
        save_mask=True,
    )

    assert (save_dir / "red_ball_annotated.png").exists()
    assert (save_dir / "red_ball_mask.png").exists()


def test_format_table_includes_expected_columns(tmp_path):
    image_path = tmp_path / "missing.png"
    detector = create_detector_backend("hsv", DetectorConfig(min_area_px=50.0))
    results = evaluate_cases([EvaluationCase("missing", image_path)], detector, "hsv")

    table = format_table(results)

    assert "image" in table
    assert "runtime_ms" in table
    assert "could not read image" in table


def test_safe_filename_removes_path_unfriendly_characters():
    assert safe_filename("red ball/left") == "red_ball_left"
