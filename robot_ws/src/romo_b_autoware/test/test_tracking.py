from romo_b_autoware.tracking import GreedyTracker


def test_track_identity_and_velocity():
    tracker = GreedyTracker(association_distance=1.0, velocity_gain=1.0)
    first = tracker.update([(1.0, 2.0)], 1.0)[0]
    identifier = first.identifier
    second = tracker.update([(1.2, 2.0)], 1.2)[0]
    assert second.identifier == identifier
    assert 0.9 < second.vx < 1.1


def test_expired_track_gets_new_identity():
    tracker = GreedyTracker(expiry=0.5)
    identifier = tracker.update([(0.0, 0.0)], 0.0)[0].identifier
    replacement = tracker.update([(0.0, 0.0)], 1.0)[0]
    assert replacement.identifier != identifier
