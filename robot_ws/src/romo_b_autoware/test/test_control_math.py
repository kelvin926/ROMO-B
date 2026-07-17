import math

from romo_b_autoware.control_math import (
    PathPoint,
    calculate_follow_command,
    curvature_to_twist,
)


def test_straight_path_and_final_stop():
    path = [PathPoint(float(x), 0.0, 0.2) for x in range(5)]
    command = calculate_follow_command(
        path,
        0.0,
        0.0,
        0.0,
        wheel_base=0.323,
        lookahead=0.7,
        stop_distance=0.3,
        max_speed=0.2,
        max_steer=math.radians(22.0),
    )
    assert command.speed == 0.2
    assert abs(command.steer) < 1.0e-9

    stopped = calculate_follow_command(
        path,
        4.0,
        0.0,
        0.0,
        wheel_base=0.323,
        lookahead=0.7,
        stop_distance=0.3,
        max_speed=0.2,
        max_steer=math.radians(22.0),
    )
    assert stopped.speed == 0.0


def test_left_target_and_curvature_conversion():
    path = [PathPoint(0.0, 0.0, 0.2), PathPoint(1.0, 1.0, 0.2)]
    command = calculate_follow_command(
        path,
        0.0,
        0.0,
        0.0,
        wheel_base=0.323,
        lookahead=0.7,
        stop_distance=0.2,
        max_speed=0.2,
        max_steer=math.radians(22.0),
    )
    assert 0.0 < command.steer <= math.radians(22.0)
    assert curvature_to_twist(command.speed, command.steer, 0.323) > 0.0
