#!/usr/bin/env python3
"""Create a ROMO-B hardware template without silently replacing calibration."""

from __future__ import annotations

import argparse
import os
import pathlib
import tempfile

import yaml


def hardware_template(adapter_serial: str, interface: str) -> dict:
    return {
        "schema_version": 1,
        "serial": {
            "device": "/dev/romo_b_pcu",
            "baud": 115200,
            "data_bits": 8,
            "parity": "none",
            "stop_bits": 1,
            "command_endian": "little",
            "adapter_serial": adapter_serial,
            "wiring": "pending",
        },
        "lidar": {
            "host_ip": "192.168.1.5",
            "lidar_ip": "pending",
            "interface": interface,
            "frame_id": "livox_frame",
            "calibrated": False,
            "transform": {
                key: 0.0 for key in ("x", "y", "z", "roll", "pitch", "yaw")
            },
        },
    }


def write_config(
    output: pathlib.Path,
    adapter_serial: str,
    interface: str,
    force: bool = False,
) -> None:
    if output.exists() and not force:
        raise FileExistsError(
            f"{output} already exists; preserve it or explicitly use --force"
        )
    output.parent.mkdir(parents=True, exist_ok=True)
    text = yaml.safe_dump(
        hardware_template(adapter_serial, interface), sort_keys=False
    )
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{output.name}.", suffix=".tmp", dir=output.parent
    )
    temporary = pathlib.Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, output)
    finally:
        if temporary.exists():
            temporary.unlink()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=pathlib.Path)
    parser.add_argument("--adapter-serial", default="pending")
    parser.add_argument("--interface", default="pending")
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    try:
        write_config(
            args.output, args.adapter_serial, args.interface, force=args.force
        )
    except FileExistsError as error:
        print(f"ERROR: {error}")
        return 1
    print(f"Generated {args.output}; complete pending fields before --apply.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
