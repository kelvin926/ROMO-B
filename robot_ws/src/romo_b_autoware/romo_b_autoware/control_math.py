import math
from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True)
class PathPoint:
    x: float
    y: float
    velocity: float


@dataclass(frozen=True)
class FollowCommand:
    speed: float
    steer: float
    nearest_index: int
    target_index: int
    target_x: float
    target_y: float
    goal_distance: float


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def curvature_to_twist(speed: float, steer: float, wheel_base: float) -> float:
    if wheel_base <= 0.0:
        raise ValueError("wheel_base must be positive")
    return speed * math.tan(steer) / wheel_base


def calculate_follow_command(
    path: Sequence[PathPoint],
    ego_x: float,
    ego_y: float,
    ego_yaw: float,
    *,
    wheel_base: float,
    lookahead: float,
    stop_distance: float,
    max_speed: float,
    max_steer: float,
) -> FollowCommand:
    if not path:
        raise ValueError("path is empty")
    if wheel_base <= 0.0 or lookahead <= 0.0 or stop_distance < 0.0:
        raise ValueError("invalid follower geometry")

    nearest = min(
        range(len(path)),
        key=lambda index: math.hypot(path[index].x - ego_x, path[index].y - ego_y),
    )
    target = nearest
    distance = 0.0
    while target + 1 < len(path) and distance < lookahead:
        current = path[target]
        following = path[target + 1]
        distance += math.hypot(following.x - current.x, following.y - current.y)
        target += 1

    target_point = path[target]
    alpha = wrap_angle(
        math.atan2(target_point.y - ego_y, target_point.x - ego_x) - ego_yaw
    )
    effective_lookahead = max(
        lookahead, math.hypot(target_point.x - ego_x, target_point.y - ego_y)
    )
    steer = math.atan2(
        2.0 * wheel_base * math.sin(alpha), effective_lookahead
    )
    steer = clamp(steer, -max_steer, max_steer)

    goal = path[-1]
    goal_distance = math.hypot(goal.x - ego_x, goal.y - ego_y)
    if goal_distance <= stop_distance and nearest >= max(0, len(path) - 3):
        speed = 0.0
    else:
        requested = max(0.0, target_point.velocity)
        # Reduce longitudinal speed during larger lateral corrections. This is
        # deliberately conservative for a 0.323 m wheelbase indoor platform.
        curve_scale = max(0.35, 1.0 - abs(steer) / max(max_steer, 1.0e-6) * 0.65)
        speed = clamp(requested * curve_scale, 0.0, max_speed)

    return FollowCommand(
        speed=speed,
        steer=steer,
        nearest_index=nearest,
        target_index=target,
        target_x=target_point.x,
        target_y=target_point.y,
        goal_distance=goal_distance,
    )
