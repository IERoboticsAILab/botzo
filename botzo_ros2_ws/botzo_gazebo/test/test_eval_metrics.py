import math

import pytest

from botzo_gazebo.eval_metrics import quat_to_rpy, segment_metrics


def straight_line(vx, vy, heading, seconds=4.0, dt=0.02, z=0.2):
    out = []
    for i in range(int(seconds / dt) + 1):
        t = i * dt
        wx = vx * math.cos(heading) - vy * math.sin(heading)
        wy = vx * math.sin(heading) + vy * math.cos(heading)
        out.append((t, wx * t, wy * t, z, 0.0, 0.0, heading))
    return out


def test_speed_is_measured_in_the_heading_frame():
    m = segment_metrics(straight_line(0.2, 0.05, heading=2.0))
    assert m.vx == pytest.approx(0.2)
    assert m.vy == pytest.approx(0.05)
    assert m.wz == pytest.approx(0.0)
    assert not m.fell


def test_turning_in_place_across_the_wrap_around():
    samples = [(i * 0.1, 0.0, 0.0, 0.2, 0.0, 0.0, math.remainder(3.0 + 0.1 * i, 2 * math.pi)) for i in range(20)]
    assert segment_metrics(samples).wz == pytest.approx(1.0)


def test_tilt_and_fall_detection():
    samples = [(0.0, 0, 0, 0.2, 0.0, 0.0, 0.0), (1.0, 0, 0, 0.2, math.radians(60), 0.0, 0.0)]
    m = segment_metrics(samples)
    assert m.tilt_max == pytest.approx(60.0)
    assert m.fell


def test_quat_to_rpy_yaw():
    half = math.radians(30) / 2
    assert quat_to_rpy(0, 0, math.sin(half), math.cos(half))[2] == pytest.approx(math.radians(30))
