import dataclasses
import math
import pathlib
from typing import Iterable, Optional

import yaml


@dataclasses.dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    yaw: Optional[float] = None


@dataclasses.dataclass(frozen=True)
class Route:
    frame_id: str
    default_speed_mps: float
    waypoints: tuple[Waypoint, ...]
    mode: str = "continuous"


def _finite(value, field):
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{field} must be finite")
    return number


def validate_route(route):
    if not route.frame_id or route.frame_id != "map":
        raise ValueError("frame_id must be 'map'")
    if route.mode != "continuous":
        raise ValueError("only continuous waypoint mode is supported")
    if not 0.0 < route.default_speed_mps <= 0.2:
        raise ValueError("default_speed_mps must be in (0.0, 0.2]")
    for index, waypoint in enumerate(route.waypoints):
        _finite(waypoint.x, f"waypoints[{index}].x")
        _finite(waypoint.y, f"waypoints[{index}].y")
        if waypoint.yaw is not None:
            _finite(waypoint.yaw, f"waypoints[{index}].yaw")
    return route


def infer_yaws(waypoints: Iterable[Waypoint]):
    points = list(waypoints)
    if not points:
        return []
    result = []
    previous_yaw = 0.0
    for index, point in enumerate(points):
        yaw = point.yaw
        if yaw is None:
            yaw = previous_yaw
            for candidate in points[index + 1 :]:
                dx = candidate.x - point.x
                dy = candidate.y - point.y
                if math.hypot(dx, dy) > 1.0e-6:
                    yaw = math.atan2(dy, dx)
                    break
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        result.append(Waypoint(point.x, point.y, yaw))
        previous_yaw = yaw
    return result


def load_route(path):
    source = pathlib.Path(path).expanduser()
    document = yaml.safe_load(source.read_text()) or {}
    if document.get("schema_version") != 1:
        raise ValueError("schema_version must be 1")
    points = []
    for item in document.get("waypoints", []):
        if not isinstance(item, dict) or "x" not in item or "y" not in item:
            raise ValueError("every waypoint requires x and y")
        points.append(
            Waypoint(
                _finite(item["x"], "x"),
                _finite(item["y"], "y"),
                None if "yaw" not in item else _finite(item["yaw"], "yaw"),
            )
        )
    route = Route(
        frame_id=str(document.get("frame_id", "")),
        default_speed_mps=_finite(document.get("default_speed_mps", 0.2), "default_speed_mps"),
        waypoints=tuple(points),
        mode=str(document.get("mode", "continuous")),
    )
    return validate_route(route)


def save_route(path, route):
    validate_route(route)
    destination = pathlib.Path(path).expanduser()
    destination.parent.mkdir(parents=True, exist_ok=True)
    points = infer_yaws(route.waypoints)
    document = {
        "schema_version": 1,
        "frame_id": route.frame_id,
        "mode": route.mode,
        "default_speed_mps": route.default_speed_mps,
        "waypoints": [
            {"x": point.x, "y": point.y, "yaw": point.yaw} for point in points
        ],
    }
    destination.write_text(yaml.safe_dump(document, sort_keys=False))
