import math
import uuid
from dataclasses import dataclass
from typing import Iterable


@dataclass
class Track:
    identifier: bytes
    x: float
    y: float
    vx: float
    vy: float
    last_seen: float
    age: int = 1


class GreedyTracker:
    """Small deterministic-association tracker for low-speed corridor obstacles."""

    def __init__(
        self,
        association_distance: float = 0.8,
        expiry: float = 0.75,
        velocity_gain: float = 0.35,
        max_velocity: float = 3.0,
    ) -> None:
        if association_distance <= 0.0 or expiry <= 0.0:
            raise ValueError("tracker distances and times must be positive")
        self.association_distance = association_distance
        self.expiry = expiry
        self.velocity_gain = velocity_gain
        self.max_velocity = max_velocity
        self.tracks: list[Track] = []

    def update(self, detections: Iterable[tuple[float, float]], stamp: float) -> list[Track]:
        points = list(detections)
        self.tracks = [track for track in self.tracks if stamp - track.last_seen <= self.expiry]
        available = set(range(len(self.tracks)))
        output: list[Track] = []

        for x, y in points:
            candidate = None
            candidate_distance = self.association_distance
            for index in available:
                track = self.tracks[index]
                predicted_x = track.x + track.vx * max(0.0, stamp - track.last_seen)
                predicted_y = track.y + track.vy * max(0.0, stamp - track.last_seen)
                distance = math.hypot(x - predicted_x, y - predicted_y)
                if distance < candidate_distance:
                    candidate = index
                    candidate_distance = distance

            if candidate is None:
                track = Track(uuid.uuid4().bytes, x, y, 0.0, 0.0, stamp)
                self.tracks.append(track)
            else:
                available.remove(candidate)
                track = self.tracks[candidate]
                dt = stamp - track.last_seen
                if dt > 1.0e-3:
                    measured_vx = (x - track.x) / dt
                    measured_vy = (y - track.y) / dt
                    magnitude = math.hypot(measured_vx, measured_vy)
                    if magnitude > self.max_velocity:
                        scale = self.max_velocity / magnitude
                        measured_vx *= scale
                        measured_vy *= scale
                    gain = self.velocity_gain
                    track.vx = (1.0 - gain) * track.vx + gain * measured_vx
                    track.vy = (1.0 - gain) * track.vy + gain * measured_vy
                track.x = x
                track.y = y
                track.last_seen = stamp
                track.age += 1
            output.append(track)

        return output
