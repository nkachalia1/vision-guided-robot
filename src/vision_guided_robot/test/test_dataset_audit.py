from pathlib import Path

from vision_guided_robot.dataset_audit import (
    audit_dataset,
    audit_image,
    format_issues,
    format_summary,
    validate_yolo_label_line,
)


def touch(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b"fake image")


def write_label(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def test_validate_yolo_label_line_accepts_valid_label():
    assert validate_yolo_label_line("0 0.5 0.5 0.2 0.2", 1) == []


def test_validate_yolo_label_line_rejects_bad_values():
    issues = validate_yolo_label_line("0 1.2 0.5 0.0 0.2", 1)

    assert "line 1: center_x must be between 0 and 1" in issues
    assert "line 1: width must be positive" in issues


def test_audit_image_reports_negative_missing_positive_and_invalid(tmp_path):
    image_path = tmp_path / "images" / "train" / "red.jpg"
    label_path = tmp_path / "labels" / "train" / "red.txt"
    touch(image_path)

    missing = audit_image(image_path, label_path, "train")
    assert missing.status == "missing_label"

    write_label(label_path, "")
    negative = audit_image(image_path, label_path, "train")
    assert negative.status == "negative"

    write_label(label_path, "0 0.5 0.5 0.2 0.2\n")
    positive = audit_image(image_path, label_path, "train")
    assert positive.status == "positive"
    assert positive.box_count == 1

    write_label(label_path, "0 2.0 0.5 0.2 0.2\n")
    invalid = audit_image(image_path, label_path, "train")
    assert invalid.status == "invalid_label"
    assert invalid.issues


def test_audit_dataset_summarizes_splits_and_readiness(tmp_path):
    dataset_root = tmp_path / "dataset"
    touch(dataset_root / "images" / "train" / "red.jpg")
    touch(dataset_root / "images" / "train" / "negative.jpg")
    touch(dataset_root / "images" / "val" / "red_val.jpg")
    write_label(dataset_root / "labels" / "train" / "red.txt", "0 0.5 0.5 0.2 0.2\n")
    write_label(dataset_root / "labels" / "train" / "negative.txt", "")
    write_label(dataset_root / "labels" / "val" / "red_val.txt", "0 0.5 0.5 0.2 0.2\n")

    audit = audit_dataset(
        dataset_root,
        min_train_positives=1,
        min_val_positives=1,
        min_negatives=1,
    )

    assert audit.ready_for_training
    assert audit.split_summaries[0].split == "train"
    assert audit.split_summaries[0].positives == 1
    assert audit.split_summaries[0].negatives == 1
    assert audit.split_summaries[1].positives == 1


def test_formatters_include_readiness_notes_and_issues(tmp_path):
    dataset_root = tmp_path / "dataset"
    touch(dataset_root / "images" / "train" / "bad.jpg")
    write_label(dataset_root / "labels" / "train" / "bad.txt", "0 2.0 0.5 0.2 0.2\n")

    audit = audit_dataset(dataset_root)
    summary = format_summary(audit)
    issues = format_issues(audit)

    assert "ready_for_training: False" in summary
    assert "need at least" in summary
    assert "bad.jpg" in issues
    assert "center_x must be between 0 and 1" in issues
