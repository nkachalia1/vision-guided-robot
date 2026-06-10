#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path


def main() -> None:
    args = parse_args()
    yaml_path = Path(args.input_yaml)
    metadata = read_simple_yaml(yaml_path)

    image_path = Path(metadata["image"])
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path

    width, height, max_value, pixels = read_pgm(image_path)
    pad_px = args.pad_px
    padded_width = width + 2 * pad_px
    padded_height = height + 2 * pad_px
    unknown = args.unknown_value

    padded = bytearray([unknown] * (padded_width * padded_height))
    for row in range(height):
        source_start = row * width
        target_start = (row + pad_px) * padded_width + pad_px
        padded[target_start : target_start + width] = pixels[source_start : source_start + width]

    resolution = float(metadata.get("resolution", "0.05"))
    origin = parse_origin(metadata.get("origin", "[0.0, 0.0, 0.0]"))
    origin[0] -= pad_px * resolution
    origin[1] -= pad_px * resolution

    output_yaml = Path(args.output_yaml)
    output_image = output_yaml.with_suffix(".pgm")
    write_pgm(output_image, padded_width, padded_height, max_value, padded)
    write_yaml(output_yaml, metadata, output_image.name, origin)

    print(f"wrote {output_image}")
    print(f"wrote {output_yaml}")
    print(f"old_size: {width}x{height}")
    print(f"new_size: {padded_width}x{padded_height}")
    print(f"new_origin: [{origin[0]:.6f}, {origin[1]:.6f}, {origin[2]:.6f}]")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Pad a Nav2 occupancy-grid PGM/YAML map.")
    parser.add_argument("input_yaml", help="Input map YAML.")
    parser.add_argument("output_yaml", help="Output padded map YAML.")
    parser.add_argument("--pad-px", type=int, default=40, help="Pixels to add on each side.")
    parser.add_argument(
        "--unknown-value",
        type=int,
        default=205,
        help="PGM value used for unknown padding. Nav2 map_saver commonly uses 205.",
    )
    return parser.parse_args()


def read_simple_yaml(path: Path) -> dict[str, str]:
    metadata: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or ":" not in stripped:
            continue
        key, value = stripped.split(":", 1)
        metadata[key.strip()] = value.strip()
    return metadata


def parse_origin(value: str) -> list[float]:
    cleaned = value.strip().strip("[]")
    parts = [part.strip() for part in cleaned.split(",")]
    if len(parts) != 3:
        raise ValueError(f"expected origin with 3 values, got: {value}")
    return [float(part) for part in parts]


def read_pgm(path: Path) -> tuple[int, int, int, bytes]:
    with path.open("rb") as file:
        magic = read_token(file)
        if magic != b"P5":
            raise ValueError(f"only binary PGM P5 is supported, got {magic!r}")
        width = int(read_token(file))
        height = int(read_token(file))
        max_value = int(read_token(file))
        if max_value > 255:
            raise ValueError("16-bit PGM files are not supported")
        pixels = file.read(width * height)
    if len(pixels) != width * height:
        raise ValueError(f"PGM pixel data is truncated: {path}")
    return width, height, max_value, pixels


def read_token(file) -> bytes:
    token = bytearray()
    while True:
        char = file.read(1)
        if not char:
            raise ValueError("unexpected EOF while reading PGM")
        if char == b"#":
            file.readline()
            continue
        if char.isspace():
            continue
        token.extend(char)
        break

    while True:
        char = file.read(1)
        if not char or char.isspace():
            break
        token.extend(char)
    return bytes(token)


def write_pgm(path: Path, width: int, height: int, max_value: int, pixels: bytes | bytearray) -> None:
    with path.open("wb") as file:
        file.write(f"P5\n{width} {height}\n{max_value}\n".encode("ascii"))
        file.write(pixels)


def write_yaml(
    path: Path,
    metadata: dict[str, str],
    image_name: str,
    origin: list[float],
) -> None:
    lines = [
        f"image: {image_name}",
        f"mode: {metadata.get('mode', 'trinary')}",
        f"resolution: {metadata.get('resolution', '0.05')}",
        f"origin: [{origin[0]:.6f}, {origin[1]:.6f}, {origin[2]:.6f}]",
        f"negate: {metadata.get('negate', '0')}",
        f"occupied_thresh: {metadata.get('occupied_thresh', '0.65')}",
        f"free_thresh: {metadata.get('free_thresh', '0.25')}",
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


if __name__ == "__main__":
    main()
