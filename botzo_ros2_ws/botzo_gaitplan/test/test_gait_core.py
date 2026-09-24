import math

import pytest

from botzo_gaitplan.gait_core import LEGS, GaitParams, TrotGait, body_to_leg, leg_to_body

DT = 0.02
COXA, FEMUR, TIBIA = 2.54, 9.05, 9.85  # botzo_ik link lengths (cm)


def run(gait, seconds, cmd):
    '''Step the gait; yields (ik_targets, body_frame_feet, contacts) after every tick.'''
    for _ in range(int(round(seconds / DT))):
        targets = gait.update(DT, *cmd)
        yield targets, {leg: list(gait.feet[leg].pos) for leg in LEGS}, gait.contacts()


def advance(gait, seconds, cmd):
    for _ in run(gait, seconds, cmd):
        pass


def stance_velocity(gait, leg, seconds, cmd):
    '''Mean body-frame velocity (cm/s) of a foot over ticks where it stays on the ground.'''
    total, n = [0.0, 0.0], 0
    prev_pos, prev_contact = list(gait.feet[leg].pos), gait.contacts()[leg]
    for _, feet, contacts in run(gait, seconds, cmd):
        if prev_contact and contacts[leg]:
            total[0] += (feet[leg][0] - prev_pos[0]) / DT
            total[1] += (feet[leg][1] - prev_pos[1]) / DT
            n += 1
        prev_pos, prev_contact = feet[leg], contacts[leg]
    return total[0] / n, total[1] / n


def test_stands_still_without_command():
    gait = TrotGait()
    for targets, _, contacts in run(gait, 2.0, (0, 0, 0)):
        assert all(contacts.values())
        for leg in LEGS:
            assert targets[leg] == pytest.approx(gait.p.home(leg))
    assert not gait.state.walking


def test_leg_frame_round_trip():
    p = GaitParams()
    for leg in LEGS:
        assert body_to_leg(leg, leg_to_body(leg, (1.0, 2.0, 15.0), p), p) == pytest.approx((1.0, 2.0, 15.0))
        assert body_to_leg(leg, (*p.hip(leg), -12.0), p) == pytest.approx((0.0, 0.0, 12.0))


def test_back_left_home_is_further_out():
    '''BL home (2, 4, 16) vs BR (2, 2, 16): mirror images except BL is 2 cm further out.'''
    p = GaitParams()
    bl, br = p.neutral('bl'), p.neutral('br')
    assert bl[0] == pytest.approx(br[0])
    assert bl[1] == pytest.approx(-br[1] + 2.0)


@pytest.mark.parametrize('cmd, leg, expected', [
    ((0.1, 0, 0), 'fl', (-10.0, 0.0)),    # forward: feet slide back at 10 cm/s
    ((0, 0.1, 0), 'fr', (0.0, -10.0)),    # strafe left: feet slide right
    ((0, 0, 0.5), 'fl', None),            # yaw CCW: checked against w x p below
])
def test_stance_feet_move_opposite_to_body(cmd, leg, expected):
    gait = TrotGait()
    advance(gait, 1.0, cmd)  # let the command ramp up
    vx, vy = stance_velocity(gait, leg, 2.0, cmd)
    if expected is None:
        x, y = gait.p.neutral(leg)
        wz = cmd[2]
        expected = (wz * y, -wz * x)
    assert vx == pytest.approx(expected[0], abs=1.0)
    assert vy == pytest.approx(expected[1], abs=1.0)


def test_ik_frame_signs():
    '''Forward walking pushes IK x positive in stance; strafing left moves left feet inward.'''
    gait = TrotGait()
    advance(gait, 1.0, (0.1, 0, 0))
    before = gait.update(DT, 0.1, 0, 0)
    after = gait.update(DT, 0.1, 0, 0)
    for leg in LEGS:
        if not gait.feet[leg].swinging:
            assert after[leg][0] > before[leg][0]

    gait = TrotGait()
    advance(gait, 1.0, (0, 0.1, 0))
    before = gait.update(DT, 0, 0.1, 0)
    after = gait.update(DT, 0, 0.1, 0)
    for leg in LEGS:
        if not gait.feet[leg].swinging:
            inward = after[leg][1] < before[leg][1]
            assert inward == (leg in ('fl', 'bl'))


def test_trot_pairs_and_swing_height():
    gait = TrotGait()
    apex = 0.0
    for _, feet, contacts in run(gait, 3.0, (0.15, 0, 0)):
        assert contacts['fl'] == contacts['br']
        assert contacts['fr'] == contacts['bl']
        assert sum(contacts.values()) >= 2
        apex = max(apex, max(feet[leg][2] + gait.p.home(leg)[2] for leg in LEGS))
    # 20 ms samples of a short swing can miss the exact top by a little
    assert apex == pytest.approx(gait.p.step_height, rel=0.05)


@pytest.mark.parametrize('cmds', [
    [(0.2, 0, 0), (-0.2, 0, 0)],
    [(0, 0.2, 1.5), (0, -0.2, -1.5)],
    [(0.2, 0, 0), (0, 0, 0), (0.2, 0.2, 1.0)],
])
def test_feet_move_continuously_when_command_jumps(cmds):
    '''No teleporting feet: per-tick displacement is bounded even for step inputs.'''
    gait = TrotGait()
    # A swing covers at most one diagonal stride (max_step is per axis) plus the
    # lift in swing_time; allow twice that average speed for the curve's peak.
    # A jump would be several cm.
    stride = 2 * math.sqrt(2) * gait.p.max_step + gait.p.step_height
    max_move = 2 * (stride / gait.p.swing_time) * DT
    prev = {leg: list(gait.feet[leg].pos) for leg in LEGS}
    for cmd in cmds:
        for _, feet, _ in run(gait, 1.3, cmd):
            for leg in LEGS:
                assert math.dist(feet[leg], prev[leg]) < max_move
            prev = feet


def test_stops_within_one_cycle_and_then_holds_still():
    gait = TrotGait()
    advance(gait, 2.0, (0.3, 0.05, 0.5))
    ramp_down = 0.3 / gait.p.max_lin_acc
    advance(gait, ramp_down + gait.p.period + 2 * DT, (0, 0, 0))
    assert not gait.state.walking
    for leg in LEGS:
        nx, ny = gait.p.neutral(leg)
        assert math.hypot(gait.feet[leg].pos[0] - nx, gait.feet[leg].pos[1] - ny) < 0.5

    # Standing: no foot moves at all (no bobbing up and down).
    before = {leg: list(gait.feet[leg].pos) for leg in LEGS}
    for _, feet, contacts in run(gait, 2.0, (0, 0, 0)):
        assert all(contacts.values())
        assert feet == before


def test_targets_stay_in_ik_workspace():
    gait = TrotGait()
    for cmd in [(0.5, 0, 0), (-0.5, 0, 0), (0, 0.5, 0), (0, 0, 4.0), (0.5, 0.5, 4.0)]:
        for targets, _, _ in run(gait, 2.0, cmd):
            for leg, (x, y, z) in targets.items():
                assert z * z + y * y > COXA * COXA
                reach = math.sqrt(x * x + z * z + y * y - COXA * COXA)
                assert abs(FEMUR - TIBIA) < reach < 0.99 * (FEMUR + TIBIA)  # keep off full extension
                # swing may overshoot a little
                assert abs(x - gait.p.home(leg)[0]) <= gait.p.max_step + 1.5


def test_speed_is_limited_to_what_the_step_length_allows():
    gait = TrotGait(GaitParams(max_lin_vel=1.0))
    advance(gait, 3.0, (1.0, 0, 0))
    limit = 2 * gait.p.max_step / gait.p.stance_time
    assert gait.state.cmd[0] == pytest.approx(limit, rel=1e-3)
