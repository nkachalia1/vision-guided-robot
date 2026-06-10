from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional

import cv2
import numpy as np


@dataclass(frozen=True)
class Detection:
    center_x: int
    center_y: int
    radius_px: float
    bbox: tuple[int, int, int, int]
    area_px: float
    distance_m: Optional[float]
    confidence: float


@dataclass(frozen=True)
class DetectorConfig:
    min_area_px: float = 150.0
    min_circularity: float = 0.55
    real_diameter_m: float = 0.20
    horizontal_fov_rad: float = 1.047
    blur_kernel_size: int = 5
    morph_kernel_size: int = 5
    lower_red_1: tuple[int, int, int] = (0, 90, 80)
    upper_red_1: tuple[int, int, int] = (10, 255, 255)
    lower_red_2: tuple[int, int, int] = (170, 90, 80)
    upper_red_2: tuple[int, int, int] = (180, 255, 255)


def focal_length_px(image_width_px: int, horizontal_fov_rad: float) -> float:
    """Return pinhole-camera focal length in pixels from horizontal field of view."""
    if image_width_px <= 0:
        raise ValueError("image_width_px must be positive")
    if not 0.0 < horizontal_fov_rad < math.pi:
        raise ValueError("horizontal_fov_rad must be between 0 and pi")
    return image_width_px / (2.0 * math.tan(horizontal_fov_rad / 2.0))


def estimate_distance_m(
    real_diameter_m: float,
    measured_diameter_px: float,
    image_width_px: int,
    horizontal_fov_rad: float,
) -> Optional[float]:
    """Estimate distance using z = real_diameter * focal_length / pixel_diameter."""
    if real_diameter_m <= 0.0 or measured_diameter_px <= 0.0:
        return None
    focal_px = focal_length_px(image_width_px, horizontal_fov_rad)
    return real_diameter_m * focal_px / measured_diameter_px


def estimate_lateral_offset_m(
    center_x_px: float,
    distance_m: float,
    image_width_px: int,
    horizontal_fov_rad: float,
) -> float:
    """Estimate camera optical-frame lateral offset from horizontal pixel error."""
    focal_px = focal_length_px(image_width_px, horizontal_fov_rad)
    pixel_error = center_x_px - (image_width_px / 2.0)
    return pixel_error * distance_m / focal_px


class RedBallDetector:
    def __init__(self, config: DetectorConfig | None = None):
        self.config = config or DetectorConfig()

    def detect(self, bgr_image: np.ndarray) -> Detection | None:
        if bgr_image is None or bgr_image.size == 0:
            return None

        image = self._blur(bgr_image)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = self._red_mask(hsv)
        mask = self._clean_mask(mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_detection = None
        best_score = -1.0

        for contour in contours:
            detection = self._detection_from_contour(contour, bgr_image.shape[1])
            if detection is None:
                continue

            score = detection.area_px * detection.confidence
            if score > best_score:
                best_score = score
                best_detection = detection

        return best_detection

    def mask(self, bgr_image: np.ndarray) -> np.ndarray:
        image = self._blur(bgr_image)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        return self._clean_mask(self._red_mask(hsv))

    def _blur(self, bgr_image: np.ndarray) -> np.ndarray:
        kernel_size = _odd_kernel_size(self.config.blur_kernel_size)
        if kernel_size <= 1:
            return bgr_image
        return cv2.GaussianBlur(bgr_image, (kernel_size, kernel_size), 0)

    def _red_mask(self, hsv_image: np.ndarray) -> np.ndarray:
        lower_1 = np.array(self.config.lower_red_1, dtype=np.uint8)
        upper_1 = np.array(self.config.upper_red_1, dtype=np.uint8)
        lower_2 = np.array(self.config.lower_red_2, dtype=np.uint8)
        upper_2 = np.array(self.config.upper_red_2, dtype=np.uint8)
        mask_1 = cv2.inRange(hsv_image, lower_1, upper_1)
        mask_2 = cv2.inRange(hsv_image, lower_2, upper_2)
        return cv2.bitwise_or(mask_1, mask_2)

    def _clean_mask(self, mask: np.ndarray) -> np.ndarray:
        kernel_size = _odd_kernel_size(self.config.morph_kernel_size)
        if kernel_size <= 1:
            return mask

        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)

    def _detection_from_contour(
        self,
        contour: np.ndarray,
        image_width_px: int,
    ) -> Detection | None:
        area = float(cv2.contourArea(contour))
        if area < self.config.min_area_px:
            return None

        perimeter = float(cv2.arcLength(contour, True))
        if perimeter <= 0.0:
            return None

        circularity = 4.0 * math.pi * area / (perimeter * perimeter)
        if circularity < self.config.min_circularity:
            return None

        (center_x, center_y), radius = cv2.minEnclosingCircle(contour)
        if radius <= 0.0:
            return None

        bbox = tuple(int(value) for value in cv2.boundingRect(contour))
        distance = estimate_distance_m(
            self.config.real_diameter_m,
            2.0 * radius,
            image_width_px,
            self.config.horizontal_fov_rad,
        )
        confidence = max(0.0, min(1.0, circularity))

        return Detection(
            center_x=int(round(center_x)),
            center_y=int(round(center_y)),
            radius_px=float(radius),
            bbox=bbox,
            area_px=area,
            distance_m=distance,
            confidence=confidence,
        )


def detection_summary_lines(
    detection: Detection | None,
    image_width_px: int | None = None,
    horizontal_fov_rad: float | None = None,
) -> list[str]:
    if detection is None:
        return ["no target"]

    lines = [
        "red ball",
        f"center=({detection.center_x}, {detection.center_y})",
        f"radius={detection.radius_px:.1f}px",
        f"confidence={detection.confidence:.2f}",
    ]
    if detection.distance_m is not None:
        lines.append(f"distance={detection.distance_m:.2f}m")

        if image_width_px is not None and horizontal_fov_rad is not None:
            lateral_m = estimate_lateral_offset_m(
                detection.center_x,
                detection.distance_m,
                image_width_px,
                horizontal_fov_rad,
            )
            lines.append(f"lateral={lateral_m:+.2f}m")

    return lines


def annotate_detection(
    bgr_image: np.ndarray,
    detection: Detection | None,
    *,
    horizontal_fov_rad: float | None = None,
    fps: float | None = None,
    show_crosshair: bool = True,
) -> np.ndarray:
    annotated = bgr_image.copy()
    height, width = annotated.shape[:2]

    if show_crosshair:
        cv2.line(
            annotated,
            (width // 2, 0),
            (width // 2, height),
            (120, 120, 120),
            1,
            cv2.LINE_AA,
        )

    if fps is not None:
        cv2.putText(
            annotated,
            f"{fps:.1f} FPS",
            (16, height - 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

    if detection is None:
        cv2.putText(
            annotated,
            "no target",
            (16, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return annotated

    center = (detection.center_x, detection.center_y)
    radius = int(round(detection.radius_px))
    x, y, width, height = detection.bbox
    cv2.circle(annotated, center, radius, (0, 255, 0), 2)
    cv2.rectangle(annotated, (x, y), (x + width, y + height), (255, 0, 0), 2)

    lines = detection_summary_lines(
        detection,
        image_width_px=width,
        horizontal_fov_rad=horizontal_fov_rad,
    )
    for index, line in enumerate(lines):
        cv2.putText(
            annotated,
            line,
            (max(0, x), max(24, y - 8 + index * 24)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
    return annotated


def _odd_kernel_size(value: int) -> int:
    value = max(1, int(value))
    return value if value % 2 == 1 else value + 1
