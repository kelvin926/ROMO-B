import math

import pytest

from romo_b_waypoints.model import Route, Waypoint, infer_yaws, load_route, save_route


def test_infers_segment_yaw_and_reuses_last_direction():
    result = infer_yaws([Waypoint(0, 0), Waypoint(1, 1), Waypoint(2, 1)])
    assert result[0].yaw == pytest.approx(math.pi / 4)
    assert result[1].yaw == pytest.approx(0.0)
    assert result[2].yaw == pytest.approx(0.0)


def test_yaml_round_trip(tmp_path):
    route = Route("map", 0.2, (Waypoint(0.0, 0.0), Waypoint(1.0, 0.0)))
    path = tmp_path / "route.yaml"
    save_route(path, route)
    loaded = load_route(path)
    assert loaded.frame_id == "map"
    assert len(loaded.waypoints) == 2
    assert loaded.waypoints[0].yaw == pytest.approx(0.0)


@pytest.mark.parametrize("speed", [-0.1, 0.0, 0.51])
def test_rejects_unsafe_speed(tmp_path, speed):
    path = tmp_path / "bad.yaml"
    path.write_text(
        f"schema_version: 1\nframe_id: map\nmode: continuous\n"
        f"default_speed_mps: {speed}\nwaypoints: []\n"
    )
    with pytest.raises(ValueError):
        load_route(path)
