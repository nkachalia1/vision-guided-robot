from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


@dataclass(frozen=True)
class ManualLabelResult:
    image_path: Path
    label_path: Path
    preview_path: Path
    bbox_xywh: tuple[int, int, int, int]
    yolo_line: str


def parse_box(text: str) -> tuple[float, float, float, float]:
    parts = text.replace(",", " ").split()
    if len(parts) != 4:
        raise ValueError("Expected four values: x,y,w,h or x1,y1,x2,y2")
    return tuple(float(part) for part in parts)


def bbox_from_xywh(
    x: float,
    y: float,
    width: float,
    height: float,
    image_width_px: int,
    image_height_px: int,
) -> tuple[int, int, int, int]:
    if width <= 0.0 or height <= 0.0:
        raise ValueError("Bounding-box width and height must be positive")
    x1 = int(round(x))
    y1 = int(round(y))
    x2 = int(round(x + width))
    y2 = int(round(y + height))
    return bbox_from_corners(x1, y1, x2, y2, image_width_px, image_height_px)


def bbox_from_corners(
    x1: float,
    y1: float,
    x2: float,
    y2: float,
    image_width_px: int,
    image_height_px: int,
) -> tuple[int, int, int, int]:
    left = int(round(min(x1, x2)))
    right = int(round(max(x1, x2)))
    top = int(round(min(y1, y2)))
    bottom = int(round(max(y1, y2)))

    left = max(0, min(image_width_px - 1, left))
    right = max(0, min(image_width_px, right))
    top = max(0, min(image_height_px - 1, top))
    bottom = max(0, min(image_height_px, bottom))

    width = right - left
    height = bottom - top
    if width <= 0 or height <= 0:
        raise ValueError("Bounding box must cover at least one pixel")
    return left, top, width, height


def yolo_line_from_bbox(
    bbox_xywh: tuple[int, int, int, int],
    image_width_px: int,
    image_height_px: int,
    *,
    class_id: int = 0,
) -> str:
    x, y, width, height = bbox_xywh
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
    return f"{class_id} " + " ".join(f"{value:.6f}" for value in values) + "\n"


def clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


def infer_label_path(image_path: Path) -> Path:
    parts = list(image_path.parts)
    if "images" in parts:
        index = parts.index("images")
        if index + 1 < len(parts):
            root = Path(*parts[:index])
            split = parts[index + 1]
            return root / "labels" / split / f"{image_path.stem}.txt"
    return image_path.with_suffix(".txt")


def infer_preview_path(image_path: Path) -> Path:
    parts = list(image_path.parts)
    if "images" in parts:
        index = parts.index("images")
        if index + 1 < len(parts):
            root = Path(*parts[:index])
            split = parts[index + 1]
            return root / "previews" / split / f"{image_path.stem}_manual_preview.png"
    return image_path.with_name(f"{image_path.stem}_manual_preview.png")


def draw_preview(
    frame: np.ndarray,
    bbox_xywh: tuple[int, int, int, int],
    *,
    label: str = "manual_label",
) -> np.ndarray:
    preview = frame.copy()
    x, y, width, height = bbox_xywh
    cv2.rectangle(preview, (x, y), (x + width, y + height), (0, 255, 0), 2)
    cv2.circle(preview, (x + width // 2, y + height // 2), 4, (0, 255, 0), -1)
    cv2.putText(
        preview,
        f"{label}: x={x} y={y} w={width} h={height}",
        (16, 32),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )
    return preview


def write_manual_label(
    image_path: Path,
    label_path: Path,
    preview_path: Path,
    bbox_xywh: tuple[int, int, int, int],
    *,
    class_id: int = 0,
) -> ManualLabelResult:
    frame = cv2.imread(str(image_path))
    if frame is None:
        raise RuntimeError(f"Could not read image: {image_path}")

    image_height, image_width = frame.shape[:2]
    yolo_line = yolo_line_from_bbox(
        bbox_xywh,
        image_width,
        image_height,
        class_id=class_id,
    )

    label_path.parent.mkdir(parents=True, exist_ok=True)
    label_path.write_text(yolo_line, encoding="utf-8")

    preview_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(preview_path), draw_preview(frame, bbox_xywh))

    return ManualLabelResult(
        image_path=image_path,
        label_path=label_path,
        preview_path=preview_path,
        bbox_xywh=bbox_xywh,
        yolo_line=yolo_line,
    )


class InteractiveBoxSelector:
    def __init__(self, frame: np.ndarray, *, display_scale: float = 1.0):
        if display_scale <= 0.0:
            raise ValueError("display_scale must be positive")
        self.frame = frame
        self.display_scale = display_scale
        self.start: tuple[int, int] | None = None
        self.end: tuple[int, int] | None = None
        self.cursor: tuple[int, int] | None = None

    def run(self, window_name: str = "manual label") -> tuple[int, int, int, int] | None:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(window_name, self._on_mouse)

        while True:
            cv2.imshow(window_name, self._display_frame())
            key = cv2.waitKey(20) & 0xFF
            if key in (ord("q"), 27):
                cv2.destroyWindow(window_name)
                return None
            if key == ord("r"):
                self.start = None
                self.end = None
            if key in (ord("s"), 13) and self.start is not None and self.end is not None:
                cv2.destroyWindow(window_name)
                height, width = self.frame.shape[:2]
                return bbox_from_corners(
                    self.start[0],
                    self.start[1],
                    self.end[0],
                    self.end[1],
                    width,
                    height,
                )

    def _on_mouse(self, event, x, y, flags, param) -> None:
        image_x = int(round(x / self.display_scale))
        image_y = int(round(y / self.display_scale))
        self.cursor = (image_x, image_y)

        if event == cv2.EVENT_LBUTTONDOWN:
            if self.start is None or self.end is not None:
                self.start = (image_x, image_y)
                self.end = None
            else:
                self.end = (image_x, image_y)
        elif event == cv2.EVENT_RBUTTONDOWN:
            self.start = None
            self.end = None

    def _display_frame(self) -> np.ndarray:
        display = self.frame.copy()
        active_end = self.end or self.cursor
        if self.start is not None and active_end is not None:
            cv2.rectangle(display, self.start, active_end, (0, 255, 0), 2)
        cv2.putText(
            display,
            "left click two corners | s/enter save | r reset | q/esc cancel",
            (16, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        if self.display_scale != 1.0:
            display = cv2.resize(
                display,
                None,
                fx=self.display_scale,
                fy=self.display_scale,
                interpolation=cv2.INTER_AREA,
            )
        return display


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Manually write one YOLO label for a dataset image."
    )
    parser.add_argument("--image", required=True, help="Dataset image path.")
    parser.add_argument(
        "--label",
        help="Output label path. Defaults to the matching labels/<split>/<image>.txt path.",
    )
    parser.add_argument(
        "--preview",
        help="Output preview path. Defaults to previews/<split>/<image>_manual_preview.png.",
    )
    parser.add_argument("--bbox", help="Pixel box as x,y,w,h.")
    parser.add_argument("--corners", help="Pixel corners as x1,y1,x2,y2.")
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="Click two box corners in an OpenCV window.",
    )
    parser.add_argument("--class-id", type=int, default=0, help="YOLO class id.")
    parser.add_argument(
        "--display-scale",
        type=float,
        default=1.0,
        help="Scale factor for the interactive display window.",
    )
    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()

    image_path = Path(args.image)
    label_path = Path(args.label) if args.label else infer_label_path(image_path)
    preview_path = Path(args.preview) if args.preview else infer_preview_path(image_path)

    frame = cv2.imread(str(image_path))
    if frame is None:
        raise RuntimeError(f"Could not read image: {image_path}")
    image_height, image_width = frame.shape[:2]

    if args.bbox:
        bbox = bbox_from_xywh(*parse_box(args.bbox), image_width, image_height)
    elif args.corners:
        bbox = bbox_from_corners(*parse_box(args.corners), image_width, image_height)
    elif args.interactive:
        selector = InteractiveBoxSelector(frame, display_scale=args.display_scale)
        bbox = selector.run()
        if bbox is None:
            raise SystemExit("Manual labeling canceled.")
    else:
        raise SystemExit("Provide --bbox, --corners, or --interactive.")

    result = write_manual_label(
        image_path,
        label_path,
        preview_path,
        bbox,
        class_id=args.class_id,
    )

    print(f"label: {result.label_path}")
    print(f"preview: {result.preview_path}")
    print(f"bbox_xywh: {result.bbox_xywh}")
    print(f"yolo: {result.yolo_line.strip()}")


if __name__ == "__main__":
    main()
