import math

from romo_b_sim.protocol import (
    CommandParser,
    ackermann_feedback,
    encode_feedback,
    four_wis_feedback,
    pivot_feedback,
)


def test_command_parser_handles_fragments_and_signed_values():
    frame = bytes.fromhex("53 54 58 01 00 00 FF EC FF FB 02 0D 0A")
    parser = CommandParser()
    assert parser.push(b"noise" + frame[:4]) == []
    decoded = parser.push(frame[4:])
    assert len(decoded) == 1
    assert decoded[0]["speed_mps"] == -0.2
    assert decoded[0]["steer_deg"] == -5.0


def test_feedback_has_manual_layout_and_little_endian():
    frame = encode_feedback(True, False, 0, [0.5] * 4, [-10.0, -8.0, 0.0, 0.0], 7)
    assert len(frame) == 25
    assert frame[:3] == b"STX"
    assert frame[6:8] == b"\x32\x00"
    assert frame[14:16] == b"\x9c\xff"
    assert frame[22:] == b"\x07\r\n"


def test_ackermann_right_turn_has_inner_right_wheel():
    speeds, angles = ackermann_feedback(0.2, 15.0)
    assert angles[1] > angles[0] > 0.0
    assert speeds[1] < speeds[0]
    assert math.isclose(angles[2], 0.0)


def test_four_wis_uses_opposite_rear_steering():
    speeds, angles = four_wis_feedback(-0.2, 12.0)
    assert all(speed < 0.0 for speed in speeds)
    assert angles[2] == -angles[0]
    assert angles[3] == -angles[1]


def test_pivot_has_x_steering_and_opposite_wheel_speeds():
    speeds, angles = pivot_feedback(0.1)
    assert speeds == [0.1, -0.1, 0.1, -0.1]
    assert angles == [30.0, -30.0, -30.0, 30.0]
