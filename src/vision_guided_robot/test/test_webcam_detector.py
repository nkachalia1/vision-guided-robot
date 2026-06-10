import pytest

cv2 = pytest.importorskip("cv2")
np = pytest.importorskip("numpy")

from vision_guided_robot.red_ball_detector import DetectorConfig, RedBallDetector  # noqa: E402
from vision_guided_robot.webcam_detector import (  # noqa: E402
    process_frame,
    should_print,
    should_save,
)


def test_process_frame_returns_detection_mask_and_lateral_offset():
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.circle(image, (190, 120), 30, (0, 0, 255), -1)
    detector = RedBallDetector(DetectorConfig(min_area_px=50.0))

    result = process_frame(image, detector)

    assert result.detection is not None
    assert result.annotated.shape == image.shape
    assert result.mask.shape == image.shape[:2]
    assert result.lateral_offset_m is not None
    assert result.lateral_offset_m > 0.0


def test_process_frame_mirror_flips_lateral_offset_sign():
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.circle(image, (210, 120), 30, (0, 0, 255), -1)
    detector = RedBallDetector(DetectorConfig(min_area_px=50.0))

    unmirrored = process_frame(image, detector, mirror=False)
    mirrored = process_frame(image, detector, mirror=True)

    assert unmirrored.lateral_offset_m is not None
    assert mirrored.lateral_offset_m is not None
    assert unmirrored.lateral_offset_m > 0.0
    assert mirrored.lateral_offset_m < 0.0


def test_process_frame_handles_missing_target():
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    detector = RedBallDetector(DetectorConfig(min_area_px=50.0))

    result = process_frame(image, detector)

    assert result.detection is None
    assert result.lateral_offset_m is None
    assert result.annotated.shape == image.shape
    assert result.mask.shape == image.shape[:2]


def test_print_and_save_cadence_helpers():
    assert should_print(frame_count=30, print_every_n=30)
    assert not should_print(frame_count=29, print_every_n=30)
    assert not should_print(frame_count=30, print_every_n=0)

    assert should_save(frame_count=10, save_every_n=10)
    assert not should_save(frame_count=9, save_every_n=10)
    assert not should_save(frame_count=10, save_every_n=0)
