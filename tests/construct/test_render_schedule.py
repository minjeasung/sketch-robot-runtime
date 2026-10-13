import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts'))


def test_physics_keeps_every_step_while_render_is_decimated():
    from construct_render_schedule import RenderSchedule
    schedule = RenderSchedule(physics_hz=100, render_hz=20)
    frames = [i for i in range(1, 101) if schedule.advance()]
    assert frames == list(range(5, 101, 5))


def test_pause_keeps_ui_responsive_and_resume_starts_a_fresh_interval():
    from construct_render_schedule import RenderSchedule
    schedule = RenderSchedule(physics_hz=100, render_hz=20)
    for _ in range(3):
        assert not schedule.advance()
    for _ in range(8):
        assert schedule.advance(playing=False)
    assert [schedule.advance() for _ in range(5)] == [False]*4 + [True]


def test_invalid_rates_fail_before_launch():
    from construct_render_schedule import RenderSchedule
    import pytest
    for rate in (0, -1, 101, float('nan')):
        with pytest.raises(ValueError):
            RenderSchedule(physics_hz=100, render_hz=rate)
