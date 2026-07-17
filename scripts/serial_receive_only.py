#!/usr/bin/env python3
"""Validate ROMO-B PCU feedback without ever writing serial data."""

import argparse
import pathlib
import time

import serial
import yaml


FRAME_SIZE = 25
HEADER = b"STX"
TAIL = b"\r\n"


def signed_le(frame, offset):
    return int.from_bytes(frame[offset:offset + 2], "little", signed=True)


def decode(frame):
    if len(frame) != FRAME_SIZE or frame[:3] != HEADER or frame[-2:] != TAIL:
        return None
    if frame[3] > 1 or frame[4] > 1 or frame[5] > 2:
        return None
    speed_raw = [signed_le(frame, offset) for offset in range(6, 14, 2)]
    steer_raw = [signed_le(frame, offset) for offset in range(14, 22, 2)]
    if any(abs(value) > 200 for value in speed_raw):
        return None
    if any(abs(value) > 350 for value in steer_raw):
        return None
    return {
        "auto": bool(frame[3]),
        "estop": bool(frame[4]),
        "steer_mode": frame[5],
        "wheel_speed_mps": [value * 0.01 for value in speed_raw],
        "wheel_steer_deg": [value * 0.1 for value in steer_raw],
        "alive": frame[22],
    }


class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.discarded = 0

    def push(self, data):
        self.buffer.extend(data)
        decoded = []
        while len(self.buffer) >= len(HEADER):
            start = self.buffer.find(HEADER)
            if start < 0:
                keep = min(len(self.buffer), len(HEADER) - 1)
                self.discarded += len(self.buffer) - keep
                del self.buffer[:-keep]
                break
            self.discarded += start
            del self.buffer[:start]
            if len(self.buffer) < FRAME_SIZE:
                break
            frame = bytes(self.buffer[:FRAME_SIZE])
            feedback = decode(frame)
            if feedback is None:
                self.discarded += 1
                del self.buffer[0]
                continue
            decoded.append((frame, feedback))
            del self.buffer[:FRAME_SIZE]
        return decoded


def load_defaults(repo_root):
    path = repo_root / "config/local/hardware.yaml"
    if not path.is_file():
        return {}
    document = yaml.safe_load(path.read_text()) or {}
    return document.get("serial", {})


def first_by_id():
    directory = pathlib.Path("/dev/serial/by-id")
    if not directory.is_dir():
        return None
    return next(iter(sorted(directory.iterdir())), None)


def parse_args(defaults, fallback_device):
    parser = argparse.ArgumentParser(
        description="Read ROMO-B PCU feedback only; this program never calls write()."
    )
    parser.add_argument("--device", default=defaults.get("device") or fallback_device)
    parser.add_argument("--baud", type=int, default=int(defaults.get("baud", 115200)))
    parser.add_argument("--data-bits", type=int, default=int(defaults.get("data_bits", 8)))
    parser.add_argument(
        "--parity",
        choices=("none", "even", "odd"),
        default=defaults.get("parity", "none"),
    )
    parser.add_argument(
        "--stop-bits",
        type=float,
        choices=(1.0, 2.0),
        default=float(defaults.get("stop_bits", 1)),
    )
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--min-frames", type=int, default=3)
    return parser.parse_args()


def main():
    repo_root = pathlib.Path(__file__).resolve().parent.parent
    defaults = load_defaults(repo_root)
    by_id = first_by_id()
    args = parse_args(defaults, str(by_id) if by_id else None)
    if not args.device:
        raise SystemExit("ERROR: no serial device configured or detected")
    if args.duration <= 0.0 or args.min_frames <= 0:
        raise SystemExit("ERROR: duration and min-frames must be positive")

    byte_sizes = {
        5: serial.FIVEBITS,
        6: serial.SIXBITS,
        7: serial.SEVENBITS,
        8: serial.EIGHTBITS,
    }
    parities = {
        "none": serial.PARITY_NONE,
        "even": serial.PARITY_EVEN,
        "odd": serial.PARITY_ODD,
    }
    stop_bits = {1.0: serial.STOPBITS_ONE, 2.0: serial.STOPBITS_TWO}
    if args.data_bits not in byte_sizes:
        raise SystemExit("ERROR: data-bits must be 5, 6, 7, or 8")

    serial_format = f"{args.data_bits}{args.parity[0].upper()}{int(args.stop_bits)}"
    print(f"Opening {args.device} receive-only at {args.baud} {serial_format}")
    print("TX GUARANTEE: this process never invokes serial.write().")
    parser = Parser()
    received = []
    started = time.monotonic()
    try:
        with serial.Serial(
            port=args.device,
            baudrate=args.baud,
            bytesize=byte_sizes[args.data_bits],
            parity=parities[args.parity],
            stopbits=stop_bits[args.stop_bits],
            timeout=0.10,
            write_timeout=0,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
            exclusive=True,
        ) as port:
            while time.monotonic() - started < args.duration:
                block = port.read(port.in_waiting or 1)
                for frame, feedback in parser.push(block):
                    received.append((time.monotonic(), frame, feedback))
    except (OSError, serial.SerialException) as error:
        raise SystemExit(f"ERROR: cannot read {args.device}: {error}") from error

    if len(received) < args.min_frames:
        raise SystemExit(
            f"FAIL: decoded {len(received)} valid frames; need {args.min_frames}. "
            f"Discarded {parser.discarded} bytes. Check permission, data bits, and wiring."
        )
    elapsed = max(received[-1][0] - received[0][0], 1.0e-9)
    rate = (len(received) - 1) / elapsed if len(received) > 1 else 0.0
    alive_gaps = sum(
        1
        for previous, current in zip(received, received[1:])
        if (current[2]["alive"] - previous[2]["alive"]) % 256 != 1
    )
    first_frame, first = received[0][1], received[0][2]
    print(
        f"PASS: {len(received)} valid frames, approximately {rate:.2f} Hz, "
        f"discarded bytes={parser.discarded}"
    )
    print(f"First raw frame: {first_frame.hex(' ')}")
    print(
        "Decoded: "
        f"auto={first['auto']} estop={first['estop']} mode={first['steer_mode']} "
        f"speed_mps={first['wheel_speed_mps']} steer_deg={first['wheel_steer_deg']} "
        f"alive={first['alive']} alive_gaps={alive_gaps}"
    )


if __name__ == "__main__":
    main()
