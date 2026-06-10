from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
from glob import glob
from pathlib import Path
import shutil

import cv2
import numpy as np

from vision_guided_robot.red_ball_detector import (
    Detection,
    DetectorConfig,
    RedBallDetector,
    annotate_detection,
)


VALID_SPLITS = {"train", "val", "test"}
IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


@dataclass(frozen=True)
class DatasetItem:
    source_path: Path
    split: str
    is_positive: bool


@dataclass(frozen=True)
class PreparedItem:
    source_path: str
    image_path: str
    label_path: str
    preview_path: str
    split: str
    is_positive: bool
    auto_labeled: bool
    status: str
    bbox_xywh: tuple[int, int, int, int] | None
    confidence: float | None


def prepare_dataset(
    items: list[DatasetItem],
    dataset_root: Path,
    detector: RedBallDetector,
    *,
    class_name: str = "red_ball",
    preview_dir: Path | None = None,
    overwrite: bool = False,
    skip_existing: bool = False,
) -> list[PreparedItem]:
    create_dataset_dirs(dataset_root)
    write_data_yaml(dataset_root, class_name)

    preview_dir = preview_dir or dataset_root / "previews"
    preview_dir.mkdir(parents=True, exist_ok=True)

    prepared = []
    used_names: set[str] = set()
    for item in items:
        prepared.append(
            prepare_item(
                item,
                dataset_root,
                detector,
                preview_dir=preview_dir,
                used_names=used_names,
                overwrite=overwrite,
                skip_existing=skip_existing,
            )
        )

    write_manifest(dataset_root / "manifest.csv", prepared)
    return prepared


def prepare_item(
    item: DatasetItem,
    dataset_root: Path,
    detector: RedBallDetector,
    *,
    preview_dir: Path,
    used_names: set[str],
    overwrite: bool = False,
    skip_existing: bool = False,
) -> PreparedItem:
    frame = cv2.imread(str(item.source_path))
    if frame is None:
        raise RuntimeError(f"Could not read image: {item.source_path}")

    image_name = unique_image_name(item.source_path, used_names)
    image_path = dataset_root / "images" / item.split / image_name
    label_path = dataset_root / "labels" / item.split / f"{Path(image_name).stem}.txt"
    preview_path = preview_dir / item.split / f"{Path(image_name).stem}_preview.png"
    preview_path.parent.mkdir(parents=True, exist_ok=True)

    existing_image_path = find_existing_image_path(dataset_root, image_name)
    if existing_image_path is not None and skip_existing and not overwrite:
        return skipped_existing_item(item, existing_image_path, preview_dir)

    if image_path.exists() and not overwrite:
        raise FileExistsError(f"Dataset image already exists: {image_path}")

    image_path.parent.mkdir(parents=True, exist_ok=True)
    label_path.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(item.source_path, image_path)

    detection = detector.detect(frame) if item.is_positive else None
    auto_labeled = item.is_positive and detection is not None

    if auto_labeled:
        label_path.write_text(
            yolo_label_line(detection, frame.shape[1], frame.shape[0]),
            encoding="utf-8",
        )
        status = "auto_labeled"
    else:
        label_path.write_text("", encoding="utf-8")
        status = "negative" if not item.is_positive else "needs_manual_label"

    preview = make_preview(frame, detection, item.is_positive, status)
    cv2.imwrite(str(preview_path), preview)

    return PreparedItem(
        source_path=str(item.source_path),
        image_path=str(image_path),
        label_path=str(label_path),
        preview_path=str(preview_path),
        split=item.split,
        is_positive=item.is_positive,
        auto_labeled=auto_labeled,
        status=status,
        bbox_xywh=detection.bbox if detection is not None else None,
        confidence=detection.confidence if detection is not None else None,
    )


def find_existing_image_path(dataset_root: Path, image_name: str) -> Path | None:
    for split in sorted(VALID_SPLITS):
        candidate = dataset_root / "images" / split / image_name
        if candidate.exists():
            return candidate
    return None


def skipped_existing_item(
    item: DatasetItem,
    image_path: Path,
    preview_dir: Path,
) -> PreparedItem:
    split = image_path.parent.name
    label_path = dataset_root_label_path(image_path)
    preview_path = best_existing_preview_path(preview_dir, split, image_path.stem)
    return PreparedItem(
        source_path=str(item.source_path),
        image_path=str(image_path),
        label_path=str(label_path),
        preview_path=str(preview_path),
        split=split,
        is_positive=item.is_positive,
        auto_labeled=False,
        status="skipped_existing",
        bbox_xywh=None,
        confidence=None,
    )


def dataset_root_label_path(image_path: Path) -> Path:
    parts = list(image_path.parts)
    if "images" not in parts:
        return image_path.with_suffix(".txt")
    index = parts.index("images")
    if index + 1 >= len(parts):
        return image_path.with_suffix(".txt")
    root = Path(*parts[:index])
    split = parts[index + 1]
    return root / "labels" / split / f"{image_path.stem}.txt"


def best_existing_preview_path(preview_dir: Path, split: str, stem: str) -> Path:
    manual_preview = preview_dir / split / f"{stem}_manual_preview.png"
    if manual_preview.exists():
        return manual_preview
    return preview_dir / split / f"{stem}_preview.png"


def make_preview(
    frame: np.ndarray,
    detection: Detection | None,
    is_positive: bool,
    status: str,
) -> np.ndarray:
    preview = annotate_detection(frame, detection)
    label = "positive" if is_positive else "negative"
    cv2.putText(
        preview,
        f"{label}: {status}",
        (16, max(56, preview.shape[0] - 16)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return preview


def yolo_label_line(
    detection: Detection,
    image_width_px: int,
    image_height_px: int,
) -> str:
    x, y, width, height = detection.bbox
    center_x = (x + width / 2.0) / image_width_px
    center_y = (y + height / 2.0) / image_height_px
    norm_width = width / image_width_px
    norm_height = height / image_height_px
    values = [
        clamp01(center_x),
        clamp01(center_y),
        clamp01(norm_width),
        clamp01(norm_height),
    ]
    return "0 " + " ".join(f"{value:.6f}" for value in values) + "\n"


def clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


def create_dataset_dirs(dataset_root: Path) -> None:
    for split in ("train", "val"):
        (dataset_root / "images" / split).mkdir(parents=True, exist_ok=True)
        (dataset_root / "labels" / split).mkdir(parents=True, exist_ok=True)


def write_data_yaml(dataset_root: Path, class_name: str) -> None:
    content = (
        f"path: {dataset_root.as_posix()}\n"
        "train: images/train\n"
        "val: images/val\n"
        "names:\n"
        f"  0: {class_name}\n"
    )
    (dataset_root / "data.yaml").write_text(content, encoding="utf-8")


def write_manifest(path: Path, items: list[PreparedItem]) -> None:
    if not items:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = [asdict(item) for item in items]
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def unique_image_name(source_path: Path, used_names: set[str]) -> str:
    suffix = source_path.suffix.lower()
    if suffix not in IMAGE_EXTENSIONS:
        suffix = ".jpg"

    base = safe_stem(source_path.stem)
    candidate = f"{base}{suffix}"
    index = 2
    while candidate in used_names:
        candidate = f"{base}_{index}{suffix}"
        index += 1
    used_names.add(candidate)
    return candidate


def safe_stem(stem: str) -> str:
    safe = "".join(char if char.isalnum() or char in ("-", "_") else "_" for char in stem)
    return safe.strip("_") or "image"


def parse_dataset_items(
    positives: list[str] | None,
    negatives: list[str] | None,
    *,
    positive_dirs: list[str] | None = None,
    negative_dirs: list[str] | None = None,
    positive_globs: list[str] | None = None,
    negative_globs: list[str] | None = None,
    default_split: str = "train",
    val_ratio: float = 0.20,
) -> list[DatasetItem]:
    items = []
    for raw in positives or []:
        split, path = parse_split_path(raw, default_split)
        items.append(DatasetItem(source_path=path, split=split, is_positive=True))
    for raw in negatives or []:
        split, path = parse_split_path(raw, default_split)
        items.append(DatasetItem(source_path=path, split=split, is_positive=False))

    items.extend(
        bulk_dataset_items(
            collect_bulk_paths(positive_dirs, positive_globs),
            is_positive=True,
            default_split=default_split,
            val_ratio=val_ratio,
        )
    )
    items.extend(
        bulk_dataset_items(
            collect_bulk_paths(negative_dirs, negative_globs),
            is_positive=False,
            default_split=default_split,
            val_ratio=val_ratio,
        )
    )
    return items


def parse_split_path(raw: str, default_split: str) -> tuple[str, Path]:
    split = default_split
    path_text = raw
    if "=" in raw:
        possible_split, possible_path = raw.split("=", 1)
        if possible_split in VALID_SPLITS:
            split = possible_split
            path_text = possible_path

    if split not in VALID_SPLITS:
        raise ValueError(f"Unsupported split '{split}'. Use train, val, or test.")
    return split, Path(path_text)


def collect_bulk_paths(
    directories: list[str] | None,
    patterns: list[str] | None,
) -> list[Path]:
    paths: list[Path] = []
    for directory_text in directories or []:
        directory = Path(directory_text)
        if not directory.exists():
            raise FileNotFoundError(f"Dataset source directory not found: {directory}")
        if not directory.is_dir():
            raise NotADirectoryError(f"Dataset source is not a directory: {directory}")
        paths.extend(
            path
            for path in directory.rglob("*")
            if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
        )

    for pattern in patterns or []:
        paths.extend(
            path
            for path in (Path(match) for match in glob(pattern, recursive=True))
            if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
        )

    return sorted(dict.fromkeys(paths))


def bulk_dataset_items(
    paths: list[Path],
    *,
    is_positive: bool,
    default_split: str,
    val_ratio: float,
) -> list[DatasetItem]:
    return [
        DatasetItem(
            source_path=path,
            split=split_for_bulk_item(index, len(paths), default_split, val_ratio),
            is_positive=is_positive,
        )
        for index, path in enumerate(paths)
    ]


def split_for_bulk_item(
    index: int,
    total: int,
    default_split: str,
    val_ratio: float,
) -> str:
    if default_split != "train" or total <= 1 or val_ratio <= 0.0:
        return default_split

    val_count = max(1, int(round(total * val_ratio)))
    val_count = min(total - 1, val_count)
    train_count = total - val_count
    return "val" if index >= train_count else "train"


def format_summary(items: list[PreparedItem]) -> str:
    columns = [
        ("split", lambda item: item.split),
        ("type", lambda item: "positive" if item.is_positive else "negative"),
        ("status", lambda item: item.status),
        ("confidence", lambda item: "n/a" if item.confidence is None else f"{item.confidence:.2f}"),
        ("image", lambda item: Path(item.image_path).name),
    ]
    rows = [[formatter(item) for _, formatter in columns] for item in items]
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


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Prepare a YOLO-format custom red-ball dataset."
    )
    parser.add_argument(
        "--dataset-root",
        default="datasets/red_ball_yolo",
        help="YOLO dataset output directory.",
    )
    parser.add_argument(
        "--positive",
        action="append",
        help="Positive image path, optionally split-qualified as train=/path.jpg or val=/path.jpg.",
    )
    parser.add_argument(
        "--negative",
        action="append",
        help="Negative image path, optionally split-qualified as train=/path.jpg or val=/path.jpg.",
    )
    parser.add_argument(
        "--positive-dir",
        action="append",
        help="Directory of positive images. Images are split using --val-ratio.",
    )
    parser.add_argument(
        "--negative-dir",
        action="append",
        help="Directory of negative images. Images are split using --val-ratio.",
    )
    parser.add_argument(
        "--positive-glob",
        action="append",
        help="Glob pattern for positive images. Images are split using --val-ratio.",
    )
    parser.add_argument(
        "--negative-glob",
        action="append",
        help="Glob pattern for negative images. Images are split using --val-ratio.",
    )
    parser.add_argument(
        "--default-split",
        default="train",
        choices=sorted(VALID_SPLITS),
        help="Split for images without an explicit split prefix.",
    )
    parser.add_argument(
        "--val-ratio",
        type=float,
        default=0.20,
        help="Validation fraction for directory/glob inputs when default split is train.",
    )
    parser.add_argument("--class-name", default="red_ball", help="YOLO class name.")
    parser.add_argument("--preview-dir", help="Optional preview image directory.")
    parser.add_argument("--overwrite", action="store_true", help="Overwrite existing copied images.")
    parser.add_argument(
        "--skip-existing",
        action="store_true",
        help="Skip images already present in the dataset without touching labels or previews.",
    )
    parser.add_argument(
        "--min-area",
        type=float,
        default=50.0,
        help="Minimum HSV contour area for positive pseudo-labels.",
    )
    parser.add_argument(
        "--min-circularity",
        type=float,
        default=0.30,
        help="Minimum HSV contour circularity for positive pseudo-labels.",
    )
    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()
    if not 0.0 <= args.val_ratio < 1.0:
        raise SystemExit("--val-ratio must be between 0.0 and 1.0")
    if args.overwrite and args.skip_existing:
        raise SystemExit("Use either --overwrite or --skip-existing, not both.")

    items = parse_dataset_items(
        args.positive,
        args.negative,
        positive_dirs=args.positive_dir,
        negative_dirs=args.negative_dir,
        positive_globs=args.positive_glob,
        negative_globs=args.negative_glob,
        default_split=args.default_split,
        val_ratio=args.val_ratio,
    )
    if not items:
        raise SystemExit("Provide at least one --positive or --negative image.")

    detector = RedBallDetector(
        DetectorConfig(
            min_area_px=args.min_area,
            min_circularity=args.min_circularity,
        )
    )
    prepared = prepare_dataset(
        items,
        Path(args.dataset_root),
        detector,
        class_name=args.class_name,
        preview_dir=Path(args.preview_dir) if args.preview_dir else None,
        overwrite=args.overwrite,
        skip_existing=args.skip_existing,
    )
    print(format_summary(prepared))
    print(f"\nDataset written to: {Path(args.dataset_root)}")
    print(f"Manifest: {Path(args.dataset_root) / 'manifest.csv'}")


if __name__ == "__main__":
    main()
