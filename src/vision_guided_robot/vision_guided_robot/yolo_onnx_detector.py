from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import cv2
import numpy as np

from vision_guided_robot.red_ball_detector import (
    Detection,
    DetectorConfig,
    estimate_distance_m,
)


COCO_CLASS_NAMES = (
    "person",
    "bicycle",
    "car",
    "motorcycle",
    "airplane",
    "bus",
    "train",
    "truck",
    "boat",
    "traffic light",
    "fire hydrant",
    "stop sign",
    "parking meter",
    "bench",
    "bird",
    "cat",
    "dog",
    "horse",
    "sheep",
    "cow",
    "elephant",
    "bear",
    "zebra",
    "giraffe",
    "backpack",
    "umbrella",
    "handbag",
    "tie",
    "suitcase",
    "frisbee",
    "skis",
    "snowboard",
    "sports ball",
    "kite",
    "baseball bat",
    "baseball glove",
    "skateboard",
    "surfboard",
    "tennis racket",
    "bottle",
    "wine glass",
    "cup",
    "fork",
    "knife",
    "spoon",
    "bowl",
    "banana",
    "apple",
    "sandwich",
    "orange",
    "broccoli",
    "carrot",
    "hot dog",
    "pizza",
    "donut",
    "cake",
    "chair",
    "couch",
    "potted plant",
    "bed",
    "dining table",
    "toilet",
    "tv",
    "laptop",
    "mouse",
    "remote",
    "keyboard",
    "cell phone",
    "microwave",
    "oven",
    "toaster",
    "sink",
    "refrigerator",
    "book",
    "clock",
    "vase",
    "scissors",
    "teddy bear",
    "hair drier",
    "toothbrush",
)


@dataclass(frozen=True)
class YoloOnnxConfig:
    model_path: str
    input_size_px: int = 640
    confidence_threshold: float = 0.25
    nms_threshold: float = 0.45
    class_names: tuple[str, ...] = COCO_CLASS_NAMES
    target_class_names: tuple[str, ...] = ("sports ball",)


@dataclass(frozen=True)
class YoloCandidate:
    bbox: tuple[int, int, int, int]
    confidence: float
    class_id: int


class YoloOnnxDetector:
    def __init__(
        self,
        config: DetectorConfig | None = None,
        yolo_config: YoloOnnxConfig | None = None,
    ):
        if yolo_config is None:
            raise ValueError("YoloOnnxDetector requires a YoloOnnxConfig")

        model_path = Path(yolo_config.model_path)
        if not model_path.exists():
            raise FileNotFoundError(f"ONNX model not found: {model_path}")

        self.config = config or DetectorConfig()
        self.yolo_config = yolo_config
        self.net = cv2.dnn.readNetFromONNX(str(model_path))
        self._last_detection: Detection | None = None

    def detect(self, bgr_image: np.ndarray) -> Detection | None:
        if bgr_image is None or bgr_image.size == 0:
            return None

        blob = cv2.dnn.blobFromImage(
            bgr_image,
            scalefactor=1.0 / 255.0,
            size=(self.yolo_config.input_size_px, self.yolo_config.input_size_px),
            mean=(0, 0, 0),
            swapRB=True,
            crop=False,
        )
        self.net.setInput(blob)
        outputs = self.net.forward()

        candidates = parse_yolo_candidates(
            outputs,
            image_shape=bgr_image.shape,
            input_size_px=self.yolo_config.input_size_px,
            class_names=self.yolo_config.class_names,
            target_class_names=self.yolo_config.target_class_names,
            confidence_threshold=self.yolo_config.confidence_threshold,
        )
        detection = detection_from_candidates(
            candidates,
            image_width_px=bgr_image.shape[1],
            detector_config=self.config,
            nms_threshold=self.yolo_config.nms_threshold,
        )
        self._last_detection = detection
        return detection

    def mask(self, bgr_image: np.ndarray) -> np.ndarray:
        mask = np.zeros(bgr_image.shape[:2], dtype=np.uint8)
        detection = self._last_detection
        if detection is None:
            detection = self.detect(bgr_image)
        if detection is None:
            return mask

        x, y, width, height = detection.bbox
        cv2.rectangle(mask, (x, y), (x + width, y + height), 255, -1)
        return mask


def parse_yolo_candidates(
    outputs,
    *,
    image_shape: tuple[int, ...],
    input_size_px: int,
    class_names: Sequence[str],
    target_class_names: Sequence[str],
    confidence_threshold: float,
) -> list[YoloCandidate]:
    predictions = normalize_yolo_output(outputs)
    target_ids = target_class_ids(class_names, target_class_names)
    image_height, image_width = image_shape[:2]
    scale_x = image_width / float(input_size_px)
    scale_y = image_height / float(input_size_px)

    candidates = []
    for row in predictions:
        parsed = parse_prediction_row(
            row,
            class_count=len(class_names),
            target_class_ids=target_ids,
            confidence_threshold=confidence_threshold,
        )
        if parsed is None:
            continue

        center_x, center_y, width, height, confidence, class_id = parsed
        x = int(round((center_x - width / 2.0) * scale_x))
        y = int(round((center_y - height / 2.0) * scale_y))
        box_width = int(round(width * scale_x))
        box_height = int(round(height * scale_y))

        x = max(0, min(image_width - 1, x))
        y = max(0, min(image_height - 1, y))
        box_width = max(1, min(image_width - x, box_width))
        box_height = max(1, min(image_height - y, box_height))

        candidates.append(
            YoloCandidate(
                bbox=(x, y, box_width, box_height),
                confidence=confidence,
                class_id=class_id,
            )
        )
    return candidates


def normalize_yolo_output(outputs) -> np.ndarray:
    array = outputs[0] if isinstance(outputs, (list, tuple)) else outputs
    array = np.asarray(array)

    if array.ndim == 3:
        array = array[0]

    if array.ndim != 2:
        raise ValueError(f"Expected 2D YOLO output, got shape {array.shape}")

    # Common YOLO ONNX exports return either anchors x attributes or attributes x anchors.
    if array.shape[0] < array.shape[1] and array.shape[0] <= 300:
        array = array.T
    return array


def parse_prediction_row(
    row: np.ndarray,
    *,
    class_count: int,
    target_class_ids: set[int],
    confidence_threshold: float,
) -> tuple[float, float, float, float, float, int] | None:
    if len(row) < class_count + 4:
        return None

    box = row[:4]
    if len(row) >= class_count + 5:
        objectness = float(row[4])
        class_scores = row[5 : 5 + class_count]
        class_confidence = float(np.max(class_scores))
        class_id = int(np.argmax(class_scores))
        confidence = objectness * class_confidence
    else:
        class_scores = row[4 : 4 + class_count]
        class_id = int(np.argmax(class_scores))
        confidence = float(class_scores[class_id])

    if class_id not in target_class_ids or confidence < confidence_threshold:
        return None

    center_x, center_y, width, height = (float(value) for value in box)
    if width <= 0.0 or height <= 0.0:
        return None

    return center_x, center_y, width, height, confidence, class_id


def detection_from_candidates(
    candidates: list[YoloCandidate],
    *,
    image_width_px: int,
    detector_config: DetectorConfig,
    nms_threshold: float,
) -> Detection | None:
    if not candidates:
        return None

    boxes = [candidate.bbox for candidate in candidates]
    confidences = [candidate.confidence for candidate in candidates]
    kept_indexes = cv2.dnn.NMSBoxes(
        boxes,
        confidences,
        score_threshold=0.0,
        nms_threshold=nms_threshold,
    )
    if len(kept_indexes) == 0:
        return None

    kept = np.asarray(kept_indexes).reshape(-1)
    best_index = max(kept, key=lambda index: confidences[int(index)])
    best = candidates[int(best_index)]
    x, y, width, height = best.bbox
    center_x = x + width / 2.0
    center_y = y + height / 2.0
    diameter_px = max(width, height)
    radius_px = diameter_px / 2.0
    distance_m = estimate_distance_m(
        detector_config.real_diameter_m,
        diameter_px,
        image_width_px,
        detector_config.horizontal_fov_rad,
    )

    return Detection(
        center_x=int(round(center_x)),
        center_y=int(round(center_y)),
        radius_px=float(radius_px),
        bbox=best.bbox,
        area_px=float(width * height),
        distance_m=distance_m,
        confidence=float(best.confidence),
    )


def target_class_ids(
    class_names: Sequence[str],
    target_class_names: Sequence[str],
) -> set[int]:
    normalized_targets = {name.strip().lower() for name in target_class_names}
    ids = {
        index
        for index, class_name in enumerate(class_names)
        if class_name.strip().lower() in normalized_targets
    }
    if not ids:
        raise ValueError(
            "No target class names matched the class list: "
            f"{', '.join(target_class_names)}"
        )
    return ids


def load_class_names(path: str | None) -> tuple[str, ...]:
    if not path:
        return COCO_CLASS_NAMES
    class_path = Path(path)
    names = [
        line.strip()
        for line in class_path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    if not names:
        raise ValueError(f"No class names found in {class_path}")
    return tuple(names)
