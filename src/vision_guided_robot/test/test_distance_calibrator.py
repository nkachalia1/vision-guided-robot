from pathlib import Path

import pytest

from vision_guided_robot.distance_calibrator import (
    effective_diameter_from_measurement,
    parse_sample,
    summarize_results,
    CalibrationResult,
)
from vision_guided_robot.red_ball_detector import focal_length_px


def test_parse_sample_accepts_label_path_and_distance():
    sample = parse_sample("near=/tmp/image.jpg:1.25")

    assert sample.label == "near"
    assert sample.path == Path("/tmp/image.jpg")
    assert sample.known_distance_m == 1.25


def test_parse_sample_uses_stem_when_label_is_omitted():
    sample = parse_sample("/tmp/far_image.jpg:3.0")

    assert sample.label == "far_image"
    assert sample.path == Path("/tmp/far_image.jpg")
    assert sample.known_distance_m == 3.0


def test_parse_sample_rejects_bad_distance():
    with pytest.raises(ValueError):
        parse_sample("near=/tmp/image.jpg:not-a-distance")


def test_effective_diameter_from_measurement_inverts_pinhole_distance():
    image_width_px = 640
    horizontal_fov_rad = 1.047
    focal_px = focal_length_px(image_width_px, horizontal_fov_rad)
    known_distance_m = 2.0
    real_diameter_m = 0.20
    measured_diameter_px = real_diameter_m * focal_px / known_distance_m

    effective = effective_diameter_from_measurement(
        known_distance_m=known_distance_m,
        measured_diameter_px=measured_diameter_px,
        image_width_px=image_width_px,
        horizontal_fov_rad=horizontal_fov_rad,
    )

    assert effective == pytest.approx(real_diameter_m)


def test_summarize_results_recommends_median_effective_diameter():
    results = [
        CalibrationResult(
            label="near",
            path="/tmp/near.jpg",
            detected=True,
            known_distance_m=1.0,
            estimated_distance_m=1.2,
            distance_error_m=0.2,
            distance_error_pct=20.0,
            measured_diameter_px=100.0,
            effective_diameter_m=0.18,
            confidence=0.9,
            runtime_ms=10.0,
        ),
        CalibrationResult(
            label="far",
            path="/tmp/far.jpg",
            detected=True,
            known_distance_m=2.0,
            estimated_distance_m=2.2,
            distance_error_m=0.2,
            distance_error_pct=10.0,
            measured_diameter_px=50.0,
            effective_diameter_m=0.22,
            confidence=0.8,
            runtime_ms=11.0,
        ),
    ]

    summary = summarize_results(results, configured_diameter_m=0.20)

    assert summary.detected_count == 2
    assert summary.recommended_diameter_m == pytest.approx(0.20)
    assert summary.median_abs_error_pct == pytest.approx(15.0)
