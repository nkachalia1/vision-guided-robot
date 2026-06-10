from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from vision_guided_robot.detector_backend import create_detector_backend
from vision_guided_robot.detector_evaluator import (
    evaluate_cases,
    format_table,
    parse_cases,
    write_csv,
    write_json,
)
from vision_guided_robot.red_ball_detector import DetectorConfig


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Compare HSV and custom ONNX detectors on the same saved images."
    )
    parser.add_argument(
        "--image",
        action="append",
        required=True,
        help="Image path, optionally labeled as label=/path/to/image.jpg. Repeatable.",
    )
    parser.add_argument(
        "--model-path",
        required=True,
        help="ONNX model path for the learned detector.",
    )
    parser.add_argument(
        "--class-names",
        required=True,
        help="Class-name file for the custom ONNX model.",
    )
    parser.add_argument(
        "--target-class",
        action="append",
        default=None,
        help="ONNX target class. Repeatable. Default: red_ball.",
    )
    parser.add_argument(
        "--confidence-threshold",
        type=float,
        default=0.10,
        help="Minimum ONNX confidence threshold.",
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
        "--diameter-m",
        type=float,
        default=0.20,
        help="Real target diameter used for distance estimation.",
    )
    parser.add_argument("--fov-deg", type=float, default=60.0, help="Horizontal camera FOV.")
    parser.add_argument("--min-area", type=float, default=50.0, help="HSV minimum contour area.")
    parser.add_argument(
        "--min-circularity",
        type=float,
        default=0.30,
        help="HSV minimum contour circularity.",
    )
    parser.add_argument(
        "--save-dir",
        default="detector_eval/hsv_vs_onnx",
        help="Directory for annotated comparison images.",
    )
    parser.add_argument(
        "--csv",
        default="detector_eval/hsv_vs_onnx_results.csv",
        help="CSV output path.",
    )
    parser.add_argument(
        "--json",
        default="detector_eval/hsv_vs_onnx_results.json",
        help="JSON output path.",
    )
    parser.add_argument("--save-mask", action="store_true", help="Also save HSV/ONNX masks.")
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
    config = config_from_args(args)
    save_root = Path(args.save_dir)

    hsv_detector = create_detector_backend("hsv", config)
    onnx_detector = create_detector_backend(
        "onnx",
        config,
        model_path=args.model_path,
        class_names_path=args.class_names,
        target_class_names=args.target_class or ["red_ball"],
        confidence_threshold=args.confidence_threshold,
        nms_threshold=args.nms_threshold,
        input_size_px=args.input_size,
    )

    results = [
        *evaluate_cases(
            cases,
            hsv_detector,
            "hsv",
            save_dir=save_root / "hsv",
            save_mask=args.save_mask,
        ),
        *evaluate_cases(
            cases,
            onnx_detector,
            "onnx",
            save_dir=save_root / "onnx",
            save_mask=args.save_mask,
        ),
    ]

    print(format_table(results))
    write_csv(Path(args.csv), results)
    write_json(Path(args.json), results)
    print(f"\nAnnotated outputs: {save_root}")
    print(f"CSV: {Path(args.csv)}")
    print(f"JSON: {Path(args.json)}")


if __name__ == "__main__":
    main()
