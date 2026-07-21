import math

import pytest

from romo_b_operator_ui.model import (
    MAX_PIVOT_RATE_RADPS,
    WHEELBASE_M,
    ackermann_twist,
    four_wis_twist,
    mode_name,
    pivot_twist,
    state_name,
    uint8_value,
)


def test_ackermann_twist_uses_vehicle_wheelbase():
    linear, angular = ackermann_twist(0.2, 10.0)
    assert linear == pytest.approx(0.2)
    assert angular == pytest.approx(
        0.2 * math.tan(math.radians(10.0)) / WHEELBASE_M
    )


def test_signed_command_is_clamped_and_reverse_is_supported():
    assert ackermann_twist(2.0, 90.0)[0] == pytest.approx(1.5)
    assert ackermann_twist(-0.2, 0.0) == (-0.2, 0.0)
    assert ackermann_twist(-2.0, 0.0)[0] == pytest.approx(-1.5)


def test_four_wis_has_half_wheelbase_turning_geometry():
    linear, angular = four_wis_twist(0.2, 10.0)
    assert linear == pytest.approx(0.2)
    assert angular == pytest.approx(
        0.2 * math.tan(math.radians(10.0)) / (WHEELBASE_M * 0.5)
    )
    assert four_wis_twist(-0.2, 0.0) == (-0.2, 0.0)


def test_pivot_and_labels():
    assert pivot_twist(2.0) == (0.0, MAX_PIVOT_RATE_RADPS)
    assert mode_name(2) == "PIVOT"
    assert state_name(2) == "ARMED_AUTO"


def test_ros_uint8_accepts_integer_and_byte_representations():
    assert uint8_value(2) == 2
    assert uint8_value(b"\x00") == 0
    assert uint8_value(bytearray([3])) == 3
