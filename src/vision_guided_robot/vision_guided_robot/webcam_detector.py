from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import time

import cv2
import numpy as np

from vision_guided_robot.detector_backend import DetectorBackend, create_detector_backend
from vision_guided_robot.red_ball_detector import (
    Detection,
    DetectorConfig,
    annotate_detection,
    detection_summary_lines,
    estimate_lateral_offset_m,
)


@dataclass(frozen=True)
class WebcamFrameResult:
    detection: Detection | None
    annotated: np.ndarray
    mask: np.ndarray
    lateral_offset_m: float | None


def process_frame(
    frame: np.ndarray,
    detector: DetectorBackend,
    *,
    mirror: bool = False,
    fps: float | None = None,
) -> WebcamFrameResult:
    if mirror:
        frame = cv2.flip(frame, 1)

    detection = detector.detect(frame)
    mask = detector.mask(frame)
    lateral_offset_m = None
    if detection is not None and detection.distance_m is not None:
        lateral_offset_m = estimate_lateral_offset_m(
            detection.center_x,
            detection.distance_m,
            frame.shape[1],
            detector.config.horizontal_fov_rad,
        )

    annotated = annotate_detection(
        frame,
        detection,
        horizontal_fov_rad=detector.config.horizontal_fov_rad,
        fps=fps,
    )
    return WebcamFrameResult(
        detection=detection,
        annotated=annotated,
        mask=mask,
        lateral_offset_m=lateral_offset_m,
    )


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()

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
    if args.image:
        run_on_image(Path(args.image), detector, args)
    else:
        run_on_camera(detector, args)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Visualize red-ball detection from a webcam or saved image."
    )
    parser.add_argument("--camera", type=int, default=0, help="OpenCV camera index.")
    parser.add_argument(
        "--backend",
        default="hsv",
        help="Detector backend to use: hsv or onnx.",
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
    parser.add_argument("--image", help="Run once on an image file instead of a camera.")
    parser.add_argument("--window", default="red ball detector", help="Display window name.")
    parser.add_argument("--mask-window", default="red mask", help="Mask display window name.")
    parser.add_argument("--width", type=int, default=640, help="Requested camera width.")
    parser.add_argument("--height", type=int, default=480, help="Requested camera height.")
    parser.add_argument("--fov-deg", type=float, default=60.0, help="Horizontal camera FOV.")
    parser.add_argument(
        "--diameter-m",
        type=float,
        default=0.20,
        help="Real target diameter used for distance estimation.",
    )
    parser.add_argument("--min-area", type=float, default=150.0, help="Minimum contour area.")
    parser.add_argument(
        "--min-circularity",
        type=float,
        default=0.55,
        help="Minimum contour circularity.",
    )
    parser.add_argument("--show-mask", action="store_true", help="Display the binary red mask.")
    parser.add_argument("--mirror", action="store_true", help="Mirror frames like a selfie camera.")
    parser.add_argument("--no-display", action="store_true", help="Do not open OpenCV windows.")
    parser.add_argument(
        "--save-dir",
        help="Directory for snapshots or offline output images.",
    )
    parser.add_argument(
        "--save-every-n",
        type=int,
        default=0,
        help="Save every Nth live frame; 0 disables periodic saving.",
    )
    parser.add_argument(
        "--print-every-n",
        type=int,
        default=30,
        help="Print detection summary every N frames; 0 disables printing.",
    )
    return parser


def config_from_args(args: argparse.Namespace) -> DetectorConfig:
    return DetectorConfig(
        min_area_px=args.min_area,
        min_circularity=args.min_circularity,
        real_diameter_m=args.diameter_m,
        horizontal_fov_rad=np.deg2rad(args.fov_deg),
    )


def run_on_image(path: Path, detector: DetectorBackend, args: argparse.Namespace) -> None:
    frame = cv2.imread(str(path))
    if frame is None:
        raise RuntimeError(f"Could not read image: {path}")

    result = process_frame(frame, detector, mirror=args.mirror)
    print_summary(result)

    if args.save_dir:
        save_outputs(Path(args.save_dir), path.stem, result)

    if not args.no_display:
        cv2.imshow(args.window, result.annotated)
        if args.show_mask:
            cv2.imshow(args.mask_window, result.mask)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


def run_on_camera(detector: DetectorBackend, args: argparse.Namespace) -> None:
    capture = cv2.VideoCapture(args.camera)
    if not capture.isOpened():
        raise RuntimeError(f"Could not open camera index {args.camera}")

    capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    frame_count = 0
    previous_time_s = time.monotonic()
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                break

            now_s = time.monotonic()
            dt_s = max(1e-6, now_s - previous_time_s)
            previous_time_s = now_s
            fps = 1.0 / dt_s

            result = process_frame(frame, detector, mirror=args.mirror, fps=fps)
            frame_count += 1

            if should_print(frame_count, args.print_every_n):
                print_summary(result)

            if args.save_dir and should_save(frame_count, args.save_every_n):
                save_outputs(Path(args.save_dir), f"frame_{frame_count:06d}", result)

            if not args.no_display:
                cv2.imshow(args.window, result.annotated)
                if args.show_mask:
                    cv2.imshow(args.mask_window, result.mask)

                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key == ord("s") and args.save_dir:
                    save_outputs(Path(args.save_dir), f"snapshot_{frame_count:06d}", result)
    finally:
        capture.release()
        cv2.destroyAllWindows()


def should_print(frame_count: int, print_every_n: int) -> bool:
    return print_every_n > 0 and frame_count % print_every_n == 0


def should_save(frame_count: int, save_every_n: int) -> bool:
    return save_every_n > 0 and frame_count % save_every_n == 0


def save_outputs(output_dir: Path, name: str, result: WebcamFrameResult) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_dir / f"{name}_annotated.png"), result.annotated)
    cv2.imwrite(str(output_dir / f"{name}_mask.png"), result.mask)


def print_summary(result: WebcamFrameResult) -> None:
    lines = detection_summary_lines(
        result.detection,
        image_width_px=result.annotated.shape[1],
        horizontal_fov_rad=None,
    )
    if result.lateral_offset_m is not None:
        lines.append(f"lateral={result.lateral_offset_m:+.2f}m")
    print(" | ".join(lines))


if __name__ == "__main__":
    main()
