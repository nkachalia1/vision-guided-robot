from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
from pathlib import Path


IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
DEFAULT_SPLITS = ("train", "val")


@dataclass(frozen=True)
class ImageAudit:
    split: str
    image_path: str
    label_path: str
    status: str
    box_count: int
    issues: tuple[str, ...]


@dataclass(frozen=True)
class SplitSummary:
    split: str
    images: int
    positives: int
    negatives: int
    missing_labels: int
    invalid_labels: int
    boxes: int


@dataclass(frozen=True)
class DatasetAudit:
    dataset_root: str
    split_summaries: tuple[SplitSummary, ...]
    image_audits: tuple[ImageAudit, ...]
    ready_for_training: bool
    readiness_notes: tuple[str, ...]


def audit_dataset(
    dataset_root: Path,
    *,
    splits: tuple[str, ...] = DEFAULT_SPLITS,
    min_train_positives: int = 40,
    min_val_positives: int = 10,
    min_negatives: int = 20,
) -> DatasetAudit:
    image_audits = []
    for split in splits:
        image_audits.extend(audit_split(dataset_root, split))

    split_summaries = tuple(summarize_split(split, image_audits) for split in splits)
    readiness_notes = readiness_checks(
        split_summaries,
        min_train_positives=min_train_positives,
        min_val_positives=min_val_positives,
        min_negatives=min_negatives,
    )
    return DatasetAudit(
        dataset_root=str(dataset_root),
        split_summaries=split_summaries,
        image_audits=tuple(image_audits),
        ready_for_training=not readiness_notes,
        readiness_notes=tuple(readiness_notes),
    )


def audit_split(dataset_root: Path, split: str) -> list[ImageAudit]:
    image_dir = dataset_root / "images" / split
    label_dir = dataset_root / "labels" / split
    if not image_dir.exists():
        return []

    audits = []
    for image_path in sorted(iter_image_paths(image_dir)):
        label_path = label_dir / f"{image_path.stem}.txt"
        audits.append(audit_image(image_path, label_path, split))
    return audits


def iter_image_paths(image_dir: Path):
    for path in image_dir.rglob("*"):
        if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS:
            yield path


def audit_image(image_path: Path, label_path: Path, split: str) -> ImageAudit:
    if not label_path.exists():
        return ImageAudit(
            split=split,
            image_path=str(image_path),
            label_path=str(label_path),
            status="missing_label",
            box_count=0,
            issues=("missing label file",),
        )

    lines = [
        line.strip()
        for line in label_path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    if not lines:
        return ImageAudit(
            split=split,
            image_path=str(image_path),
            label_path=str(label_path),
            status="negative",
            box_count=0,
            issues=(),
        )

    issues = []
    for line_number, line in enumerate(lines, start=1):
        issues.extend(validate_yolo_label_line(line, line_number))

    return ImageAudit(
        split=split,
        image_path=str(image_path),
        label_path=str(label_path),
        status="invalid_label" if issues else "positive",
        box_count=len(lines),
        issues=tuple(issues),
    )


def validate_yolo_label_line(line: str, line_number: int) -> list[str]:
    parts = line.split()
    if len(parts) != 5:
        return [f"line {line_number}: expected 5 fields, got {len(parts)}"]

    issues = []
    try:
        class_id = int(parts[0])
    except ValueError:
        issues.append(f"line {line_number}: class id must be an integer")
        class_id = 0
    if class_id < 0:
        issues.append(f"line {line_number}: class id must be non-negative")

    try:
        center_x, center_y, width, height = (float(part) for part in parts[1:])
    except ValueError:
        return issues + [f"line {line_number}: box values must be numbers"]

    values = {
        "center_x": center_x,
        "center_y": center_y,
        "width": width,
        "height": height,
    }
    for name, value in values.items():
        if not 0.0 <= value <= 1.0:
            issues.append(f"line {line_number}: {name} must be between 0 and 1")
    if width <= 0.0:
        issues.append(f"line {line_number}: width must be positive")
    if height <= 0.0:
        issues.append(f"line {line_number}: height must be positive")

    return issues


def summarize_split(split: str, audits: list[ImageAudit]) -> SplitSummary:
    split_audits = [audit for audit in audits if audit.split == split]
    return SplitSummary(
        split=split,
        images=len(split_audits),
        positives=sum(audit.status == "positive" for audit in split_audits),
        negatives=sum(audit.status == "negative" for audit in split_audits),
        missing_labels=sum(audit.status == "missing_label" for audit in split_audits),
        invalid_labels=sum(audit.status == "invalid_label" for audit in split_audits),
        boxes=sum(audit.box_count for audit in split_audits),
    )


def readiness_checks(
    summaries: tuple[SplitSummary, ...],
    *,
    min_train_positives: int,
    min_val_positives: int,
    min_negatives: int,
) -> list[str]:
    by_split = {summary.split: summary for summary in summaries}
    train = by_split.get("train", SplitSummary("train", 0, 0, 0, 0, 0, 0))
    val = by_split.get("val", SplitSummary("val", 0, 0, 0, 0, 0, 0))
    total_negatives = sum(summary.negatives for summary in summaries)
    total_missing = sum(summary.missing_labels for summary in summaries)
    total_invalid = sum(summary.invalid_labels for summary in summaries)

    notes = []
    if train.positives < min_train_positives:
        notes.append(
            f"need at least {min_train_positives} train positives; found {train.positives}"
        )
    if val.positives < min_val_positives:
        notes.append(f"need at least {min_val_positives} val positives; found {val.positives}")
    if total_negatives < min_negatives:
        notes.append(f"need at least {min_negatives} negatives; found {total_negatives}")
    if total_missing:
        notes.append(f"{total_missing} image(s) are missing label files")
    if total_invalid:
        notes.append(f"{total_invalid} image(s) have invalid labels")
    return notes


def format_summary(audit: DatasetAudit) -> str:
    columns = [
        ("split", lambda item: item.split),
        ("images", lambda item: str(item.images)),
        ("positive", lambda item: str(item.positives)),
        ("negative", lambda item: str(item.negatives)),
        ("missing", lambda item: str(item.missing_labels)),
        ("invalid", lambda item: str(item.invalid_labels)),
        ("boxes", lambda item: str(item.boxes)),
    ]
    rows = [[formatter(summary) for _, formatter in columns] for summary in audit.split_summaries]
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
    lines = [header, divider, *body]
    lines.append("")
    lines.append(f"ready_for_training: {audit.ready_for_training}")
    if audit.readiness_notes:
        lines.append("readiness_notes:")
        lines.extend(f"- {note}" for note in audit.readiness_notes)
    return "\n".join(lines)


def format_issues(audit: DatasetAudit) -> str:
    issue_lines = []
    for image_audit in audit.image_audits:
        if image_audit.issues:
            issue_lines.append(Path(image_audit.image_path).name)
            issue_lines.extend(f"  - {issue}" for issue in image_audit.issues)
    return "\n".join(issue_lines) if issue_lines else "no label issues found"


def write_report(path: Path, audit: DatasetAudit) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = [asdict(image_audit) for image_audit in audit.image_audits]
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0].keys()) if rows else [])
        if rows:
            writer.writeheader()
            writer.writerows(rows)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Audit a YOLO-format dataset.")
    parser.add_argument(
        "--dataset-root",
        default="datasets/red_ball_yolo",
        help="YOLO dataset directory.",
    )
    parser.add_argument(
        "--split",
        action="append",
        help="Split to audit. Repeatable. Defaults to train and val.",
    )
    parser.add_argument("--report-csv", help="Optional per-image CSV report path.")
    parser.add_argument("--show-issues", action="store_true", help="Print per-image issues.")
    parser.add_argument("--min-train-positives", type=int, default=40)
    parser.add_argument("--min-val-positives", type=int, default=10)
    parser.add_argument("--min-negatives", type=int, default=20)
    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()

    audit = audit_dataset(
        Path(args.dataset_root),
        splits=tuple(args.split or DEFAULT_SPLITS),
        min_train_positives=args.min_train_positives,
        min_val_positives=args.min_val_positives,
        min_negatives=args.min_negatives,
    )
    print(format_summary(audit))
    if args.show_issues:
        print("")
        print(format_issues(audit))
    if args.report_csv:
        write_report(Path(args.report_csv), audit)


if __name__ == "__main__":
    main()
