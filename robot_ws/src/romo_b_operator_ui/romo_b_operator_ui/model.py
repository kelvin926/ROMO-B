import math


WHEELBASE_M = 0.323
CONTROL_TRACK_M = 0.39
MAX_SPEED_MPS = 0.5
MAX_STEER_DEG = 22.0
MAX_PIVOT_RATE_RADPS = 0.75


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, float(value)))


def ackermann_twist(speed_mps: float, steer_deg: float) -> tuple[float, float]:
    """Return a forward-only ROS Twist x/z pair for a center steering angle."""
    speed = clamp(speed_mps, 0.0, MAX_SPEED_MPS)
    steer = clamp(steer_deg, -MAX_STEER_DEG, MAX_STEER_DEG)
    if speed <= 1e-6:
        return 0.0, 0.0
    angular = speed * math.tan(math.radians(steer)) / WHEELBASE_M
    return speed, angular


def pivot_twist(rate_radps: float) -> tuple[float, float]:
    """Return a ROS-positive-CCW pivot command."""
    return 0.0, clamp(
        rate_radps, -MAX_PIVOT_RATE_RADPS, MAX_PIVOT_RATE_RADPS
    )


def mode_name(mode: int) -> str:
    return {0: "2WIS", 1: "4WIS", 2: "PIVOT"}.get(int(mode), "UNKNOWN")


def state_name(state: int) -> str:
    return {
        0: "DISCONNECTED",
        1: "CONNECTED_SAFE",
        2: "ARMED_AUTO",
        3: "E-STOP",
    }.get(int(state), "UNKNOWN")
