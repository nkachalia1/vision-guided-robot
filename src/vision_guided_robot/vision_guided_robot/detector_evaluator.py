from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
import json
from pathlib import Path
import time

import cv2
import numpy as np

from vision_guided_robot.detector_backend import DetectorBackend, create_detector_backend
from vision_guided_robot.red_ball_detector import (
    DetectorConfig,
    annotate_detection,
    estimate_lateral_offset_m,
)


@dataclass(frozen=True)
class EvaluationCase:
    label: str
    path: Path


@dataclass(frozen=True)
class EvaluationResult:
    image: str
    path: str
    backend: str
    detected: bool
    center_x: int | None
    center_y: int | None
    radius_px: float | None
    confidence: float | None
    distance_m: float | None
    lateral_m: float | None
    runtime_ms: float | None
    error: str | None = None


def evaluate_case(
    case: EvaluationCase,
    detector: DetectorBackend,
    backend_name: str,
    *,
    save_dir: Path | None = None,
    save_mask: bool = False,
) -> EvaluationResult:
    frame = cv2.imread(str(case.path))
    if frame is None:
        return EvaluationResult(
            image=case.label,
            path=str(case.path),
            backend=backend_name,
            detected=False,
            center_x=None,
            center_y=None,
            radius_px=None,
            confidence=None,
            distance_m=None,
            lateral_m=None,
            runtime_ms=None,
            error="could not read image",
        )

    start_s = time.perf_counter()
    detection = detector.detect(frame)
    runtime_ms = (time.perf_counter() - start_s) * 1000.0

    lateral_m = None
    if detection is not None and detection.distance_m is not None:
        lateral_m = estimate_lateral_offset_m(
            detection.center_x,
            detection.distance_m,
            frame.shape[1],
            detector.config.horizontal_fov_rad,
        )

    if save_dir is not None:
        save_outputs(save_dir, case.label, frame, detector, detection, save_mask=save_mask)

    return EvaluationResult(
        image=case.label,
        path=str(case.path),
        backend=backend_name,
        detected=detection is not None,
        center_x=detection.center_x if detection is not None else None,
        center_y=detection.center_y if detection is not None else None,
        radius_px=detection.radius_px if detection is not None else None,
        confidence=detection.confidence if detection is not None else None,
        distance_m=detection.distance_m if detection is not None else None,
        lateral_m=lateral_m,
        runtime_ms=runtime_ms,
    )


def evaluate_cases(
    cases: list[EvaluationCase],
    detector: DetectorBackend,
    backend_name: str,
    *,
    save_dir: Path | None = None,
    save_mask: bool = False,
) -> list[EvaluationResult]:
    return [
        evaluate_case(
            case,
            detector,
            backend_name,
            save_dir=save_dir,
            save_mask=save_mask,
        )
        for case in cases
    ]


def save_outputs(
    save_dir: Path,
    label: str,
    frame: np.ndarray,
    detector: DetectorBackend,
    detection,
    *,
    save_mask: bool = False,
) -> None:
    save_dir.mkdir(parents=True, exist_ok=True)
    safe_label = safe_filename(label)
    annotated = annotate_detection(
        frame,
        detection,
        horizontal_fov_rad=detector.config.horizontal_fov_rad,
    )
    cv2.imwrite(str(save_dir / f"{safe_label}_annotated.png"), annotated)
    if save_mask:
        cv2.imwrite(str(save_dir / f"{safe_label}_mask.png"), detector.mask(frame))


def safe_filename(label: str) -> str:
    safe = "".join(char if char.isalnum() or char in ("-", "_") else "_" for char in label)
    return safe.strip("_") or "image"


def parse_cases(raw_images: list[str]) -> list[EvaluationCase]:
    cases = []
    for raw in raw_images:
        if "=" in raw:
            label, path_text = raw.split("=", 1)
            label = label.strip()
        else:
            path_text = raw
            label = Path(path_text).stem

        if not label:
            raise ValueError(f"Image label cannot be empty: {raw}")
        cases.append(EvaluationCase(label=label, path=Path(path_text)))
    return cases


def format_table(results: list[EvaluationResult]) -> str:
    columns = [
        ("image", lambda result: result.image),
        ("backend", lambda result: result.backend),
        ("detected", lambda result: "yes" if result.detected else "no"),
        ("confidence", lambda result: format_optional(result.confidence, 2)),
        ("distance_m", lambda result: format_optional(result.distance_m, 2)),
        ("lateral_m", lambda result: format_optional(result.lateral_m, 2, signed=True)),
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


def write_csv(path: Path, results: list[EvaluationResult]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = [asdict(result) for result in results]
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, results: list[EvaluationResult]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = [asdict(result) for result in results]
    path.write_text(json.dumps(rows, indent=2), encoding="utf-8")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Evaluate a detector backend on saved images."
    )
    parser.add_argument(
        "--backend",
        default="hsv",
        help="Detector backend to evaluate: hsv or onnx.",
    )
    parser.add_argument("--model-path", help="ONNX model path for --backend onnx/ml/yolo.")
    parser.add_argument(
        "--class-names",
        help="Optional class-name file. Defaults to COCO class names.",
    )
    parser.add_argument(
        "--target-class",
        action="append",
        help="Target class name for ONNX detection. Repeatable. Default: sports ball.",
    )
    parser.add_argument(
        "--confidence-threshold",
        type=float,
        default=0.25,
        help="Minimum ONNX detection confidence.",
    )
    parser.add_argument(
        "--nms-threshold",
        type=float,
        default=0.45,
        help="ONNX non-maximum suppression threshold.",
    )
    parser.add_argument(
        "--input-size",
        type=int,
        default=640,
        help="Square ONNX model input size in pixels.",
    )
    parser.add_argument(
        "--image",
        action="append",
        required=True,
        help="Image path, optionally labeled as label=/path/to/image.jpg. Repeatable.",
    )
    parser.add_argument(
        "--diameter-m",
        type=float,
        default=0.20,
        help="Real target diameter used for distance estimation.",
    )
    parser.add_argument("--fov-deg", type=float, default=60.0, help="Horizontal camera FOV.")
    parser.add_argument("--min-area", type=float, default=150.0, help="Minimum contour area.")
    parser.add_argument(
        "--min-circularity",
        type=float,
        default=0.55,
        help="Minimum contour circularity.",
    )
    parser.add_argument("--save-dir", help="Directory for annotated evaluation images.")
    parser.add_argument("--save-mask", action="store_true", help="Also save binary masks.")
    parser.add_argument("--csv", help="Optional CSV results path.")
    parser.add_argument("--json", help="Optional JSON results path.")
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

    cases = parse_cases(args.image)
    detector = create_detector_backend(
        args.backend,
        config_from_args(args),
        model_path=args.model_path,
        class_names_path=args.class_names,
        target_class_names=args.target_class,
        confidence_threshold=args.confidence_threshold,
        nms_threshold=args.nms_threshold,
        input_size_px=args.input_size,
    )
    results = evaluate_cases(
        cases,
        detector,
        args.backend,
        save_dir=Path(args.save_dir) if args.save_dir else None,
        save_mask=args.save_mask,
    )

    print(format_table(results))

    if args.csv:
        write_csv(Path(args.csv), results)
    if args.json:
        write_json(Path(args.json), results)


if __name__ == "__main__":
    main()
