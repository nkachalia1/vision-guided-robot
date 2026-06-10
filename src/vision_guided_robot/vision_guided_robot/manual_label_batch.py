from __future__ import annotations

import argparse
from glob import glob
from pathlib import Path

import cv2

from vision_guided_robot.manual_label import (
    InteractiveBoxSelector,
    infer_label_path,
    infer_preview_path,
    write_manual_label,
)


def collect_images(patterns: list[str]) -> list[Path]:
    paths: list[Path] = []
    for pattern in patterns:
        matches = [Path(path) for path in glob(pattern, recursive=True)]
        paths.extend(path for path in matches if path.is_file())
    return sorted(dict.fromkeys(paths))


def mark_negative(image_path: Path) -> Path:
    label_path = infer_label_path(image_path)
    label_path.parent.mkdir(parents=True, exist_ok=True)
    label_path.write_text("", encoding="utf-8")
    return label_path


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Manually label many YOLO dataset images in sequence."
    )
    parser.add_argument(
        "--glob",
        action="append",
        required=True,
        help="Image glob to label, for example 'datasets/red_ball_yolo/images/train/far_*.jpg'.",
    )
    parser.add_argument(
        "--display-scale",
        type=float,
        default=0.5,
        help="Scale factor for the interactive display window.",
    )
    parser.add_argument("--class-id", type=int, default=0, help="YOLO class id.")
    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()

    image_paths = collect_images(args.glob)
    if not image_paths:
        raise SystemExit("No images matched --glob.")

    print("Controls per image:")
    print("- draw a box: left-click two corners, then press s/Enter")
    print("- reset current box: r")
    print("- skip/quit current image: q or Esc")
    print("- mark negative: close/cancel the image, then answer n at the prompt")
    print()

    for index, image_path in enumerate(image_paths, start=1):
        frame = cv2.imread(str(image_path))
        if frame is None:
            print(f"[{index}/{len(image_paths)}] could not read: {image_path}")
            continue

        print(f"[{index}/{len(image_paths)}] {image_path}")
        selector = InteractiveBoxSelector(frame, display_scale=args.display_scale)
        bbox = selector.run(window_name=f"manual label {index}/{len(image_paths)}")

        if bbox is None:
            answer = input("No box saved. Mark as negative? [y/N/q] ").strip().lower()
            if answer == "q":
                break
            if answer == "y":
                label_path = mark_negative(image_path)
                print(f"negative label: {label_path}")
            else:
                print("skipped")
            continue

        result = write_manual_label(
            image_path,
            infer_label_path(image_path),
            infer_preview_path(image_path),
            bbox,
            class_id=args.class_id,
        )
        print(f"label: {result.label_path}")
        print(f"preview: {result.preview_path}")

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
