'''
Walking-quality metrics computed from ground-truth body poses (no ROS imports).

A sample is (t, x, y, z, roll, pitch, yaw) in the world frame, SI units.
'''

import math
from dataclasses import dataclass

FALL_TILT = math.radians(45.0)   # roll or pitch beyond this counts as a fall
FALL_HEIGHT = 0.06               # m, body origin below this counts as a fall


def quat_to_rpy(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _rms(values):
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else 0.0


@dataclass
class SegmentMetrics:
    vx: float           # m/s, mean forward speed in the body heading frame
    vy: float           # m/s, mean sideways speed
    wz: float           # rad/s, mean yaw rate
    tilt_rms: float     # deg, RMS of roll and pitch combined
    tilt_max: float     # deg
    z_mean: float       # m, body height
    z_bob: float        # m, peak-to-peak body height (up and down motion)
    fell: bool


def segment_metrics(samples):
    '''Metrics over a list of samples (at least two).'''
    t0, x0, y0, _, _, _, yaw0 = samples[0]
    t1, x1, y1, _, _, _, yaw1 = samples[-1]
    duration = t1 - t0

    # Integrate heading-frame velocity step by step, so turning while walking
    # does not smear forward speed into sideways speed.
    fwd = side = turn = 0.0
    for (_, xa, ya, _, _, _, yawa), (_, xb, yb, _, _, _, yawb) in zip(samples, samples[1:]):
        dx, dy = xb - xa, yb - ya
        c, s = math.cos(yawa), math.sin(yawa)
        fwd += c * dx + s * dy
        side += -s * dx + c * dy
        turn += _wrap(yawb - yawa)

    tilts = [math.hypot(r, p) for (_, _, _, _, r, p, _) in samples]
    zs = [z for (_, _, _, z, _, _, _) in samples]
    fell = any(t > FALL_TILT for t in tilts) or min(zs) < FALL_HEIGHT
    return SegmentMetrics(
        vx=fwd / duration, vy=side / duration, wz=turn / duration,
        tilt_rms=math.degrees(_rms(tilts)), tilt_max=math.degrees(max(tilts)),
        z_mean=sum(zs) / len(zs), z_bob=max(zs) - min(zs), fell=fell)
