import math


WHEELBASE_M = 0.323
CONTROL_TRACK_M = 0.39
# ROMO-B manual section 6.1 and HLV SPEED command section 18.5 both define
# the platform command range as -1.50 .. +1.50 m/s.
MAX_SPEED_MPS = 1.5
MAX_STEER_DEG = 22.0
MAX_PIVOT_RATE_RADPS = 0.75


def uint8_value(value) -> int:
    """Normalize ROS uint8 values exposed as either int or one-byte data."""
    if isinstance(value, (bytes, bytearray, memoryview)):
        return int(value[0]) if value else 0
    return int(value)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, float(value)))


def ackermann_twist(speed_mps: float, steer_deg: float) -> tuple[float, float]:
    """Return a signed 2WIS ROS Twist for a ROS-positive-left steering angle."""
    speed = clamp(speed_mps, -MAX_SPEED_MPS, MAX_SPEED_MPS)
    steer = clamp(steer_deg, -MAX_STEER_DEG, MAX_STEER_DEG)
    if abs(speed) <= 1e-6:
        return 0.0, 0.0
    angular = speed * math.tan(math.radians(steer)) / WHEELBASE_M
    return speed, angular


def four_wis_twist(speed_mps: float, steer_deg: float) -> tuple[float, float]:
    """Return the body Twist produced by symmetric counter-phase 4WIS."""
    speed = clamp(speed_mps, -MAX_SPEED_MPS, MAX_SPEED_MPS)
    steer = clamp(steer_deg, -18.0, 18.0)
    if abs(speed) <= 1e-6:
        return 0.0, 0.0
    angular = speed * math.tan(math.radians(steer)) / (WHEELBASE_M * 0.5)
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
