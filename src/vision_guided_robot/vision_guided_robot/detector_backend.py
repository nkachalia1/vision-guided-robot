from __future__ import annotations

from typing import Protocol, Sequence

import numpy as np

from vision_guided_robot.red_ball_detector import (
    Detection,
    DetectorConfig,
    RedBallDetector,
)
from vision_guided_robot.yolo_onnx_detector import (
    YoloOnnxConfig,
    YoloOnnxDetector,
    load_class_names,
)


SUPPORTED_DETECTOR_BACKENDS = ("hsv", "onnx")
_HSV_ALIASES = {"hsv", "red", "red_hsv"}
_ONNX_ALIASES = {"ml", "onnx", "yolo", "yolo_onnx"}


class DetectorBackend(Protocol):
    config: DetectorConfig

    def detect(self, bgr_image: np.ndarray) -> Detection | None:
        ...

    def mask(self, bgr_image: np.ndarray) -> np.ndarray:
        ...


def create_detector_backend(
    backend_name: str,
    config: DetectorConfig | None = None,
    *,
    model_path: str | None = None,
    class_names_path: str | None = None,
    target_class_names: Sequence[str] | None = None,
    confidence_threshold: float = 0.25,
    nms_threshold: float = 0.45,
    input_size_px: int = 640,
) -> DetectorBackend:
    normalized = backend_name.strip().lower()
    if normalized in _HSV_ALIASES:
        return RedBallDetector(config)

    if normalized in _ONNX_ALIASES:
        if not model_path:
            raise ValueError(
                f"Detector backend '{backend_name}' requires an ONNX model path."
            )
        class_names = load_class_names(class_names_path)
        yolo_config = YoloOnnxConfig(
            model_path=model_path,
            input_size_px=input_size_px,
            confidence_threshold=confidence_threshold,
            nms_threshold=nms_threshold,
            class_names=class_names,
            target_class_names=tuple(target_class_names or ("sports ball",)),
        )
        return YoloOnnxDetector(config=config, yolo_config=yolo_config)

    supported = ", ".join(SUPPORTED_DETECTOR_BACKENDS)
    raise ValueError(
        f"Unsupported detector backend '{backend_name}'. Supported backends: {supported}."
    )
