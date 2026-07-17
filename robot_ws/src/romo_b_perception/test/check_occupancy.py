#!/usr/bin/env python3
import pathlib
import subprocess
import sys


def main():
    if len(sys.argv) < 4:
        raise RuntimeError("Expected converter, input PCD, output prefix, and options")
    converter, input_pcd, output_prefix, *options = sys.argv[1:]
    subprocess.run(
        [converter, input_pcd, output_prefix, *options],
        check=True,
    )

    pgm = pathlib.Path(output_prefix + ".pgm").read_bytes()
    magic, dimensions, maximum, pixels = pgm.split(b"\n", 3)
    width, height = (int(value) for value in dimensions.split())
    assert magic == b"P5"
    assert maximum == b"255"
    assert len(pixels) == width * height
    assert {0, 205, 254}.issubset(set(pixels)), (
        "raycast map must contain occupied, unknown, and observed-free cells"
    )

    yaml_text = pathlib.Path(output_prefix + ".yaml").read_text(encoding="utf-8")
    assert "mode: trinary" in yaml_text
    assert "resolution: 0.05" in yaml_text
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
