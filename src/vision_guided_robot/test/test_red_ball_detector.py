import math

import pytest

cv2 = pytest.importorskip("cv2")
np = pytest.importorskip("numpy")

from vision_guided_robot.red_ball_detector import (  # noqa: E402
    DetectorConfig,
    RedBallDetector,
    annotate_detection,
    detection_summary_lines,
    estimate_distance_m,
    estimate_lateral_offset_m,
)


def test_detects_synthetic_red_ball():
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.circle(image, (170, 110), 28, (0, 0, 255), -1)

    detector = RedBallDetector(DetectorConfig(min_area_px=50.0))
    detection = detector.detect(image)

    assert detection is not None
    assert abs(detection.center_x - 170) <= 2
    assert abs(detection.center_y - 110) <= 2
    assert detection.radius_px == pytest.approx(28, abs=3)
    assert detection.distance_m is not None
    assert detection.distance_m > 0.0


def test_ignores_non_red_object():
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.circle(image, (160, 120), 30, (255, 0, 0), -1)

    detector = RedBallDetector(DetectorConfig(min_area_px=50.0))

    assert detector.detect(image) is None


def test_larger_pixel_diameter_means_closer_distance():
    near = estimate_distance_m(
        real_diameter_m=0.20,
        measured_diameter_px=80,
        image_width_px=640,
        horizontal_fov_rad=1.047,
    )
    far = estimate_distance_m(
        real_diameter_m=0.20,
        measured_diameter_px=40,
        image_width_px=640,
        horizontal_fov_rad=1.047,
    )

    assert near is not None
    assert far is not None
    assert near < far
    assert far == pytest.approx(near * 2.0)


def test_lateral_offset_sign_matches_image_side():
    width = 640
    fov = 1.047
    distance = 2.0

    left = estimate_lateral_offset_m(220, distance, width, fov)
    right = estimate_lateral_offset_m(420, distance, width, fov)

    assert left < 0.0
    assert right > 0.0
    assert math.isclose(abs(left), abs(right), rel_tol=0.05)


def test_detection_summary_includes_distance_and_lateral_offset():
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.circle(image, (190, 120), 30, (0, 0, 255), -1)
    detector = RedBallDetector(DetectorConfig(min_area_px=50.0))
    detection = detector.detect(image)

    lines = detection_summary_lines(
        detection,
        image_width_px=image.shape[1],
        horizontal_fov_rad=detector.config.horizontal_fov_rad,
    )

    assert any(line.startswith("distance=") for line in lines)
    assert any(line.startswith("lateral=") for line in lines)


def test_annotate_detection_preserves_image_shape():
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.circle(image, (160, 120), 30, (0, 0, 255), -1)
    detector = RedBallDetector(DetectorConfig(min_area_px=50.0))

    annotated = annotate_detection(
        image,
        detector.detect(image),
        horizontal_fov_rad=detector.config.horizontal_fov_rad,
        fps=30.0,
    )

    assert annotated.shape == image.shape
    assert np.any(annotated != image)
