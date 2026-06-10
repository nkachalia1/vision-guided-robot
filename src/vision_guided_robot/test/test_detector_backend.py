import pytest

cv2 = pytest.importorskip("cv2")
np = pytest.importorskip("numpy")

from vision_guided_robot.detector_backend import create_detector_backend  # noqa: E402
from vision_guided_robot.red_ball_detector import DetectorConfig  # noqa: E402


def test_create_hsv_backend_detects_synthetic_red_ball():
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.circle(image, (160, 120), 30, (0, 0, 255), -1)

    detector = create_detector_backend("hsv", DetectorConfig(min_area_px=50.0))
    detection = detector.detect(image)

    assert detection is not None
    assert detection.center_x == pytest.approx(160, abs=2)
    assert detection.center_y == pytest.approx(120, abs=2)


def test_hsv_alias_uses_same_backend_contract():
    detector = create_detector_backend("red_hsv", DetectorConfig(min_area_px=50.0))

    assert detector.config.min_area_px == 50.0


def test_onnx_backend_requires_model_path():
    with pytest.raises(ValueError, match="requires an ONNX model path"):
        create_detector_backend("ml")


def test_unknown_backend_fails_with_supported_list():
    with pytest.raises(ValueError, match="Supported backends: hsv, onnx"):
        create_detector_backend("depth_magic")
