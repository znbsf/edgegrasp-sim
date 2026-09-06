from edgegrasp.action_readiness import wait_for_readiness


def test_discovery_wait_is_bounded_and_never_retries_goals():
    now = [0.]
    sleeps = []
    def pause(delay):
        sleeps.append(delay)
        now[0] += delay
    assert wait_for_readiness(lambda: now[0] >= .015, clock=lambda: now[0], pause=pause)
    assert abs(now[0] - .015) < 1e-9
    now[0] = 0.
    sleeps.clear()
    assert not wait_for_readiness(lambda: False, clock=lambda: now[0], pause=pause)
    assert abs(now[0] - .05) < 1e-9 and max(sleeps) <= .005
    sleeps.clear()
    assert wait_for_readiness(lambda: True, clock=lambda: now[0], pause=pause)
    assert sleeps == []
