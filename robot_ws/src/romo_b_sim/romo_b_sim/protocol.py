import math
import struct

HEADER = b"STX"
TAIL = b"\r\n"
COMMAND_SIZE = 13


class CommandParser:
    def __init__(self):
        self.buffer = bytearray()

    def push(self, data):
        self.buffer.extend(data)
        output = []
        while len(self.buffer) >= 3:
            start = self.buffer.find(HEADER)
            if start < 0:
                self.buffer[:] = self.buffer[-2:]
                break
            del self.buffer[:start]
            if len(self.buffer) < COMMAND_SIZE:
                break
            frame = bytes(self.buffer[:COMMAND_SIZE])
            if frame[-2:] != TAIL or frame[3] > 1 or frame[4] > 1 or frame[5] > 2:
                del self.buffer[0]
                continue
            output.append(
                {
                    "auto_mode": bool(frame[3]),
                    "estop": bool(frame[4]),
                    "steer_mode": frame[5],
                    "speed_mps": struct.unpack(">h", frame[6:8])[0] * 0.01,
                    "steer_deg": float(struct.unpack(">h", frame[8:10])[0]),
                    "alive": frame[10],
                }
            )
            del self.buffer[:COMMAND_SIZE]
        return output


def ackermann_feedback(speed_mps, steer_deg, wheelbase=0.323, track=0.390):
    if abs(steer_deg) < 1.0e-6 or abs(speed_mps) < 1.0e-6:
        return [speed_mps] * 4, [0.0] * 4
    delta = math.radians(steer_deg)
    radius = wheelbase / math.tan(delta)
    fl_angle = math.atan(wheelbase / (radius + track / 2.0))
    fr_angle = math.atan(wheelbase / (radius - track / 2.0))
    fl_radius = math.hypot(wheelbase, radius + track / 2.0)
    fr_radius = math.hypot(wheelbase, radius - track / 2.0)
    speeds = [
        speed_mps * fl_radius / abs(radius),
        speed_mps * fr_radius / abs(radius),
        speed_mps * abs(radius + track / 2.0) / abs(radius),
        speed_mps * abs(radius - track / 2.0) / abs(radius),
    ]
    return speeds, [math.degrees(fl_angle), math.degrees(fr_angle), 0.0, 0.0]


def pivot_feedback(speed_mps):
    """Mirror the ROMO-B PCU Pivot feedback convention.

    Positive command speed is clockwise: FL/RL are positive and FR/RR are
    negative. Wheel steering feedback is the fixed X pattern from the manual.
    """
    return (
        [speed_mps, -speed_mps, speed_mps, -speed_mps],
        [30.0, -30.0, -30.0, 30.0],
    )


def encode_feedback(auto_mode, estop, steer_mode, speeds, steering_deg, alive):
    frame = bytearray(25)
    frame[:3] = HEADER
    frame[3] = int(auto_mode)
    frame[4] = int(estop)
    frame[5] = int(steer_mode)
    for index, speed in enumerate(speeds):
        frame[6 + 2 * index : 8 + 2 * index] = struct.pack(
            "<h", max(-150, min(150, round(speed * 100.0)))
        )
    for index, angle in enumerate(steering_deg):
        frame[14 + 2 * index : 16 + 2 * index] = struct.pack(
            "<h", max(-300, min(300, round(angle * 10.0)))
        )
    frame[22] = alive % 256
    frame[23:] = TAIL
    return bytes(frame)
