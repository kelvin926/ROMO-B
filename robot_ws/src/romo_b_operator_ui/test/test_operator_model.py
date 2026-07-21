import math

import pytest

from romo_b_operator_ui.model import (
    MAX_PIVOT_RATE_RADPS,
    WHEELBASE_M,
    ackermann_twist,
    mode_name,
    pivot_twist,
    state_name,
)


def test_ackermann_twist_uses_vehicle_wheelbase():
    linear, angular = ackermann_twist(0.2, 10.0)
    assert linear == pytest.approx(0.2)
    assert angular == pytest.approx(
        0.2 * math.tan(math.radians(10.0)) / WHEELBASE_M
    )


def test_forward_command_is_clamped_and_reverse_is_rejected():
    assert ackermann_twist(2.0, 90.0)[0] == pytest.approx(0.5)
    assert ackermann_twist(-0.2, 0.0) == (0.0, 0.0)


def test_pivot_and_labels():
    assert pivot_twist(2.0) == (0.0, MAX_PIVOT_RATE_RADPS)
    assert mode_name(2) == "PIVOT"
    assert state_name(2) == "ARMED_AUTO"
