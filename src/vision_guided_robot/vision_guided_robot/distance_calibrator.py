from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
import json
from pathlib import Path
from statistics import median
import time

import cv2
import numpy as np

from vision_guided_robot.detector_backend import create_detector_backend
from vision_guided_robot.red_ball_detector import DetectorConfig, focal_length_px


@dataclass(frozen=True)
class CalibrationSample:
    label: str
    path: Path
    known_distance_m: float


@dataclass(frozen=True)
class CalibrationResult:
    label: str
    path: str
    detected: bool
    known_distance_m: float
    estimated_distance_m: float | None
    distance_error_m: float | None
    distance_error_pct: float | None
    measured_diameter_px: float | None
    effective_diameter_m: float | None
    confidence: float | None
    runtime_ms: float | None
    error: str | None = None


@dataclass(frozen=True)
class CalibrationSummary:
    sample_count: int
    detected_count: int
    configured_diameter_m: float
    recommended_diameter_m: float | None
    median_distance_error_m: float | None
    median_abs_error_m: float | None
    median_abs_error_pct: float | None


def parse_sample(raw: str) -> CalibrationSample:
    if ":" not in raw:
        raise ValueError(
            "Expected --sample as label=/path/image.jpg:known_distance_m"
        )
    image_text, distance_text = raw.rsplit(":", 1)
    try:
        known_distance_m = float(distance_text)
    except ValueError as exc:
        raise ValueError(f"Invalid known distance in sample: {raw}") from exc
    if known_distance_m <= 0.0:
        raise ValueError("Known distance must be positive")

    if "=" in image_text:
        label, path_text = image_text.split("=", 1)
        label = label.strip()
    else:
        path_text = image_text
        label = Path(path_text).stem

    if not label:
        raise ValueError(f"Sample label cannot be empty: {raw}")
    return CalibrationSample(
        label=label,
        path=Path(path_text),
        known_distance_m=known_distance_m,
    )


def evaluate_sample(sample: CalibrationSample, detector, config: DetectorConfig) -> CalibrationResult:
    frame = cv2.imread(str(sample.path))
    if frame is None:
        return CalibrationResult(
            label=sample.label,
            path=str(sample.path),
            detected=False,
            known_distance_m=sample.known_distance_m,
            estimated_distance_m=None,
            distance_error_m=None,
            distance_error_pct=None,
            measured_diameter_px=None,
            effective_diameter_m=None,
            confidence=None,
            runtime_ms=None,
            error="could not read image",
        )

    start_s = time.perf_counter()
    detection = detector.detect(frame)
    runtime_ms = (time.perf_counter() - start_s) * 1000.0
    if detection is None or detection.distance_m is None:
        return CalibrationResult(
            label=sample.label,
            path=str(sample.path),
            detected=False,
            known_distance_m=sample.known_distance_m,
            estimated_distance_m=None,
            distance_error_m=None,
            distance_error_pct=None,
            measured_diameter_px=None,
            effective_diameter_m=None,
            confidence=None,
            runtime_ms=runtime_ms,
            error="no detection",
        )

    measured_diameter_px = 2.0 * detection.radius_px
    effective_diameter_m = effective_diameter_from_measurement(
        known_distance_m=sample.known_distance_m,
        measured_diameter_px=measured_diameter_px,
        image_width_px=frame.shape[1],
        horizontal_fov_rad=config.horizontal_fov_rad,
    )
    distance_error_m = detection.distance_m - sample.known_distance_m
    distance_error_pct = 100.0 * distance_error_m / sample.known_distance_m

    return CalibrationResult(
        label=sample.label,
        path=str(sample.path),
        detected=True,
        known_distance_m=sample.known_distance_m,
        estimated_distance_m=detection.distance_m,
        distance_error_m=distance_error_m,
        distance_error_pct=distance_error_pct,
        measured_diameter_px=measured_diameter_px,
        effective_diameter_m=effective_diameter_m,
        confidence=detection.confidence,
        runtime_ms=runtime_ms,
    )


def effective_diameter_from_measurement(
    *,
    known_distance_m: float,
    measured_diameter_px: float,
    image_width_px: int,
    horizontal_fov_rad: float,
) -> float | None:
    if known_distance_m <= 0.0 or measured_diameter_px <= 0.0:
        return None
    focal_px = focal_length_px(image_width_px, horizontal_fov_rad)
    return known_distance_m * measured_diameter_px / focal_px


def summarize_results(
    results: list[CalibrationResult],
    *,
    configured_diameter_m: float,
) -> CalibrationSummary:
    detected = [result for result in results if result.detected]
    effective_diameters = [
        result.effective_diameter_m
        for result in detected
        if result.effective_diameter_m is not None
    ]
    errors = [
        result.distance_error_m
        for result in detected
        if result.distance_error_m is not None
    ]
    abs_errors = [abs(error) for error in errors]
    abs_error_pct = [
        abs(result.distance_error_pct)
        for result in detected
        if result.distance_error_pct is not None
    ]
    return CalibrationSummary(
        sample_count=len(results),
        detected_count=len(detected),
        configured_diameter_m=configured_diameter_m,
        recommended_diameter_m=median(effective_diameters) if effective_diameters else None,
        median_distance_error_m=median(errors) if errors else None,
        median_abs_error_m=median(abs_errors) if abs_errors else None,
        median_abs_error_pct=median(abs_error_pct) if abs_error_pct else None,
    )


def format_results_table(results: list[CalibrationResult]) -> str:
    columns = [
        ("sample", lambda result: result.label),
        ("detected", lambda result: "yes" if result.detected else "no"),
        ("known_m", lambda result: format_optional(result.known_distance_m, 2)),
        ("estimated_m", lambda result: format_optional(result.estimated_distance_m, 2)),
        ("error_m", lambda result: format_optional(result.distance_error_m, 2, signed=True)),
        ("error_pct", lambda result: format_optional(result.distance_error_pct, 1, signed=True)),
        ("diameter_px", lambda result: format_optional(result.measured_diameter_px, 1)),
        ("effective_diam_m", lambda result: format_optional(result.effective_diameter_m, 3)),
        ("confidence", lambda result: format_optional(result.confidence, 2)),
        ("runtime_ms", lambda result: format_optional(result.runtime_ms, 1)),
        ("error", lambda result: result.error or ""),
    ]
    rows = [[formatter(result) for _, formatter in columns] for result in results]
    widths = [
        max(len(header), *(len(row[index]) for row in rows)) if rows else len(header)
        for index, (header, _) in enumerate(columns)
    ]
    header = "  ".join(
        header.ljust(widths[index]) for index, (header, _) in enumerate(columns)
    )
    divider = "  ".join("-" * width for width in widths)
    body = [
        "  ".join(value.ljust(widths[index]) for index, value in enumerate(row))
        for row in rows
    ]
    return "\n".join([header, divider, *body])


def format_summary(summary: CalibrationSummary) -> str:
    lines = [
        "",
        "summary:",
        f"  samples: {summary.sample_count}",
        f"  detected: {summary.detected_count}",
        f"  configured_diameter_m: {summary.configured_diameter_m:.3f}",
        "  recommended_diameter_m: "
        + format_optional(summary.recommended_diameter_m, 3),
        "  median_distance_error_m: "
        + format_optional(summary.median_distance_error_m, 3, signed=True),
        "  median_abs_error_m: "
        + format_optional(summary.median_abs_error_m, 3),
        "  median_abs_error_pct: "
        + format_optional(summary.median_abs_error_pct, 1),
    ]
    return "\n".join(lines)


def format_optional(
    value: float | None,
    digits: int,
    *,
    signed: bool = False,
) -> str:
    if value is None:
        return "n/a"
    sign = "+" if signed else ""
    return f"{value:{sign}.{digits}f}"


def write_csv(path: Path, results: list[CalibrationResult]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = [asdict(result) for result in results]
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_json(
    path: Path,
    results: list[CalibrationResult],
    summary: CalibrationSummary,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "summary": asdict(summary),
        "results": [asdict(result) for result in results],
    }
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Estimate detector distance calibration from known-distance images."
    )
    parser.add_argument(
        "--backend",
        default="onnx",
        help="Detector backend to calibrate: hsv or onnx.",
    )
    parser.add_argument("--model-path", help="ONNX model path for --backend onnx/ml/yolo.")
    parser.add_argument("--class-names", help="Class-name file for the ONNX model.")
    parser.add_argument(
        "--target-class",
        action="append",
        help="Target class name for ONNX detection. Repeatable. Default: red_ball.",
    )
    parser.add_argument(
        "--confidence-threshold",
        type=float,
        default=0.10,
        help="Minimum ONNX detection confidence.",
    )
    parser.add_argument("--nms-threshold", type=float, default=0.45)
    parser.add_argument("--input-size", type=int, default=640)
    parser.add_argument(
        "--sample",
        action="append",
        required=True,
        help="Known-distance sample as label=/path/image.jpg:distance_m.",
    )
    parser.add_argument("--diameter-m", type=float, default=0.20)
    parser.add_argument("--fov-deg", type=float, default=60.0)
    parser.add_argument("--min-area", type=float, default=50.0)
    parser.add_argument("--min-circularity", type=float, default=0.30)
    parser.add_argument("--csv", help="Optional CSV output path.")
    parser.add_argument("--json", help="Optional JSON output path.")
    return parser


def config_from_args(args: argparse.Namespace) -> DetectorConfig:
    return DetectorConfig(
        min_area_px=args.min_area,
        min_circularity=args.min_circularity,
        real_diameter_m=args.diameter_m,
        horizontal_fov_rad=np.deg2rad(args.fov_deg),
    )


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()

    samples = [parse_sample(raw) for raw in args.sample]
    config = config_from_args(args)
    detector = create_detector_backend(
        args.backend,
        config,
        model_path=args.model_path,
        class_names_path=args.class_names,
        target_class_names=args.target_class or ["red_ball"],
        confidence_threshold=args.confidence_threshold,
        nms_threshold=args.nms_threshold,
        input_size_px=args.input_size,
    )

    results = [
        evaluate_sample(sample, detector, config)
        for sample in samples
    ]
    summary = summarize_results(results, configured_diameter_m=args.diameter_m)

    print(format_results_table(results))
    print(format_summary(summary))

    if args.csv:
        write_csv(Path(args.csv), results)
    if args.json:
        write_json(Path(args.json), results, summary)


if __name__ == "__main__":
    main()
