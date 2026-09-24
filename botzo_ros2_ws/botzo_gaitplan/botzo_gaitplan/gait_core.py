'''
Pure-Python trot gait generator (no ROS imports, so it can be unit tested and
reused from a simulator / optimiser).

Body frame (REP-103, units cm, origin = body centre):
  x forward, y left, z up

Leg (IK) frame, as expected by botzo_ik.legIK, units cm, origin = hip:
  x: forward = negative
  y: outward = positive (left for left legs, right for right legs)
  z: down = positive (z = body height)

How the gait works
------------------
Every foot is tracked as a point in the body frame.

* Home: each leg has its own standing position, given in the IK frame
  (e.g. front (0, 2, 16), back (2, 2, 16)). A per-leg home is also where to
  compensate a leg that is mounted / calibrated slightly off.

* Stance feet are stuck to the ground, so in the body frame they move with the
  opposite of the body twist:   p <- p - (v + w x p) * dt
  This is exact for any mix of forward / strafe / yaw, and keeps foot positions
  continuous when the command changes mid-step.

* Swing feet travel from where they lifted off to a Raibert-style touchdown
  point  p_td = p_home + (v + w x p_home) * T_stance / 2  using a cubic
  Hermite curve whose end velocities match the ground (no scuffing at lift-off
  or touchdown), and a sin^2 height profile (zero vertical velocity at both
  ends -> soft landings).

* Stopping: once the command is zero, the swinging pair lands on its home spot,
  the other pair takes one last step home, and the robot stands still.
'''

import math
from dataclasses import dataclass, field

LEGS = ('fl', 'fr', 'bl', 'br')
LEFT_LEGS = ('fl', 'bl')

# Diagonal trot pairs: FL+BR swing together, FR+BL half a cycle later.
TROT_OFFSETS = {'fl': 0.0, 'br': 0.0, 'fr': 0.5, 'bl': 0.5}


@dataclass
class GaitParams:
    period: float = 0.4            # s, full gait cycle (tuned in gz sim)
    duty_factor: float = 0.55      # fraction of the cycle a foot is on the ground (>0.5: brief 4-foot support)
    step_height: float = 3.0       # cm, swing apex above the ground
    # Standing foot positions in the IK frame (x, y, z), cm. BL sits 2 cm further
    # out than BR to make up for that leg ending up under the belly.
    home_fl: tuple = (0.0, 2.0, 16.0)
    home_fr: tuple = (0.0, 2.0, 16.0)
    home_bl: tuple = (2.0, 4.0, 16.0)
    home_br: tuple = (2.0, 2.0, 16.0)
    max_step: float = 4.5          # cm, max foot travel from home (per axis)
    max_lin_vel: float = 0.35      # m/s, cmd_vel linear clamp
    max_ang_vel: float = 2.5       # rad/s, cmd_vel angular clamp
    max_lin_acc: float = 1.0       # m/s^2, command ramp
    max_ang_acc: float = 6.0       # rad/s^2, command ramp
    idle_threshold: float = 1e-3   # below this the command counts as zero
    settle_tolerance: float = 0.3  # cm, feet closer than this to home count as settled
    march_in_place: bool = False   # keep stepping with zero command
    # Hip positions in the body frame (cm): where the foot is when IK x = y = 0.
    hip_x_front: float = 9.3
    hip_x_back: float = -7.0
    hip_y: float = 5.2

    def hip(self, leg):
        x = self.hip_x_front if leg[0] == 'f' else self.hip_x_back
        y = self.hip_y if leg in LEFT_LEGS else -self.hip_y
        return (x, y)

    def home(self, leg):
        '''Home foot position in the IK frame (x, y, z).'''
        return getattr(self, f'home_{leg}')

    def neutral(self, leg):
        '''Home foot position in the body frame (x, y), cm.'''
        p = leg_to_body(leg, self.home(leg), self)
        return (p[0], p[1])

    @property
    def stance_time(self):
        return self.period * self.duty_factor

    @property
    def swing_time(self):
        return self.period * (1.0 - self.duty_factor)


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _ramp(current, target, max_delta):
    return current + _clamp(target - current, -max_delta, max_delta)


def hermite(p0, v0, p1, v1, s, duration):
    '''Cubic Hermite interpolation, s in [0, 1], velocities in units/second.'''
    s2, s3 = s * s, s * s * s
    h00 = 2 * s3 - 3 * s2 + 1
    h10 = s3 - 2 * s2 + s
    h01 = -2 * s3 + 3 * s2
    h11 = s3 - s2
    return h00 * p0 + h10 * duration * v0 + h01 * p1 + h11 * duration * v1


def swing_height(s, step_height):
    '''sin^2 profile: zero height and zero vertical velocity at s = 0 and s = 1.'''
    return step_height * math.sin(math.pi * s) ** 2


def _side(leg):
    return 1.0 if leg in LEFT_LEGS else -1.0


def body_to_leg(leg, p, params):
    '''Body-frame foot (x, y, z) -> IK-frame target (x, y, z) for botzo_ik.legIK.'''
    hx, hy = params.hip(leg)
    return (-(p[0] - hx), _side(leg) * (p[1] - hy), -p[2])


def leg_to_body(leg, q, params):
    '''IK-frame target (x, y, z) -> body-frame foot (x, y, z). Inverse of body_to_leg.'''
    hx, hy = params.hip(leg)
    return (hx - q[0], hy + _side(leg) * q[1], -q[2])


@dataclass
class _Foot:
    pos: list                      # body frame [x, y, z], cm
    swinging: bool = False
    liftoff: tuple = (0.0, 0.0)    # body-frame xy at liftoff


@dataclass
class GaitState:
    phase: float = 0.0             # [0, 1)
    walking: bool = False
    idle_handovers: int = 0        # pair hand-overs since the command went to zero
    cmd: list = field(default_factory=lambda: [0.0, 0.0, 0.0])  # ramped vx, vy (cm/s), wz (rad/s)


class TrotGait:
    def __init__(self, params=None):
        self.p = params or GaitParams()
        self.state = GaitState()
        self.feet = {}
        self.reset()

    def reset(self):
        self.state = GaitState()
        self.feet = {leg: _Foot(pos=list(leg_to_body(leg, self.p.home(leg), self.p))) for leg in LEGS}

    # ------------------------------------------------------------------ #

    def update(self, dt, vx, vy, wz):
        '''
        Advance the gait by dt seconds.
        vx, vy in m/s (body frame, REP-103), wz in rad/s (CCW positive).
        Returns {leg: (x, y, z)} targets in the IK leg frame.
        '''
        moving = self._update_command(dt, vx, vy, wz)
        self._update_phase(dt, moving)
        for leg in LEGS:
            self._update_foot(leg, dt)
        return {leg: body_to_leg(leg, self.feet[leg].pos, self.p) for leg in LEGS}

    def contacts(self):
        return {leg: not self.feet[leg].swinging for leg in LEGS}

    def leg_phase(self, leg):
        # Subtract rather than (phase + offset) % 1: with phase = 0.5 - eps the
        # sum can round up to exactly 1.0 -> 0.0 and put both pairs in swing.
        lp = self.state.phase - TROT_OFFSETS[leg]
        return lp + 1.0 if lp < 0.0 else lp

    def max_speed_scale(self, vx, vy, wz):
        '''
        Factor <= 1 that keeps every foot's stance sweep within +/- max_step.
        vx, vy in cm/s. A foot sweeps |v_ground| * T_stance during stance.
        '''
        limit = 2.0 * self.p.max_step / self.p.stance_time
        worst = 0.0
        for leg in LEGS:
            nx, ny = self.p.neutral(leg)
            worst = max(worst, math.hypot(vx - wz * ny, vy + wz * nx))
        return 1.0 if worst <= limit else limit / worst

    # --- step 1: clamp + acceleration-limit the command ----------------- #

    def _update_command(self, dt, vx, vy, wz):
        '''Ramps state.cmd toward the request. Returns True while the robot should move.'''
        p, cmd = self.p, self.state.cmd
        target = [
            _clamp(vx, -p.max_lin_vel, p.max_lin_vel) * 100.0,   # m/s -> cm/s
            _clamp(vy, -p.max_lin_vel, p.max_lin_vel) * 100.0,
            _clamp(wz, -p.max_ang_vel, p.max_ang_vel),
        ]
        k = self.max_speed_scale(*target)
        target = [t * k for t in target]

        lin_step = p.max_lin_acc * 100.0 * dt
        cmd[0] = _ramp(cmd[0], target[0], lin_step)
        cmd[1] = _ramp(cmd[1], target[1], lin_step)
        cmd[2] = _ramp(cmd[2], target[2], p.max_ang_acc * dt)

        requested = max(abs(vx), abs(vy), abs(wz)) > p.idle_threshold
        still_ramping_down = any(abs(c) > 1e-6 for c in cmd)
        return requested or still_ramping_down or p.march_in_place

    # --- step 2: advance the phase, start / stop walking ---------------- #

    def _update_phase(self, dt, moving):
        st = self.state
        if moving:
            st.walking = True
            st.idle_handovers = 0
        if not st.walking:
            return

        prev_phase = st.phase
        st.phase = (st.phase + dt / self.p.period) % 1.0
        if moving or int(prev_phase * 2) == int(st.phase * 2):
            return

        # A pair just landed with zero command. Stop as soon as every foot is
        # home; at the latest after the second hand-over, since by then both
        # pairs have taken a step with zero command.
        st.idle_handovers += 1
        if self._feet_settled() or st.idle_handovers >= 2:
            st.walking = False
            st.phase = 0.0 if st.phase < 0.5 else 0.5

    def _feet_settled(self):
        # Called at a pair hand-over, where the swinging pair is about to land,
        # so only positions matter (not the swing flag).
        tol = self.p.settle_tolerance
        for leg in LEGS:
            nx, ny = self.p.neutral(leg)
            f = self.feet[leg]
            if math.hypot(f.pos[0] - nx, f.pos[1] - ny) > tol:
                return False
        return True

    # --- step 3: move each foot ----------------------------------------- #

    def _update_foot(self, leg, dt):
        foot = self.feet[leg]
        in_swing = self.state.walking and self._in_swing(leg)
        ground_z = -self.p.home(leg)[2]

        if in_swing and not foot.swinging:        # lift-off
            foot.swinging = True
            foot.liftoff = (foot.pos[0], foot.pos[1])
        elif not in_swing and foot.swinging:      # touchdown
            foot.swinging = False
            foot.pos = [*self._touchdown_target(leg), ground_z]

        if foot.swinging:
            foot.pos = self._swing_position(leg, foot)
        else:
            gvx, gvy = self._ground_velocity(foot.pos)
            foot.pos = [foot.pos[0] + gvx * dt, foot.pos[1] + gvy * dt, ground_z]

    def _in_swing(self, leg):
        return self.leg_phase(leg) < (1.0 - self.p.duty_factor)

    def _ground_velocity(self, xy):
        '''Velocity of a ground-fixed point at body-frame xy, seen from the body (cm/s).'''
        vx, vy, wz = self.state.cmd
        # v_point = -(v + w x p),  w x p = (-wz*y, wz*x)
        return (-(vx - wz * xy[1]), -(vy + wz * xy[0]))

    def _touchdown_target(self, leg):
        nx, ny = self.p.neutral(leg)
        gvx, gvy = self._ground_velocity((nx, ny))
        # Land ahead of home by half the stance sweep so the foot ends up
        # symmetric about home over the stance phase.
        half = self.p.stance_time / 2.0
        dx = _clamp(-gvx * half, -self.p.max_step, self.p.max_step)
        dy = _clamp(-gvy * half, -self.p.max_step, self.p.max_step)
        return (nx + dx, ny + dy)

    def _swing_position(self, leg, foot):
        s = _clamp(self.leg_phase(leg) / (1.0 - self.p.duty_factor), 0.0, 1.0)
        td = self._touchdown_target(leg)
        # Start and end with the ground's velocity so the foot neither scuffs
        # at lift-off nor slams in at touchdown.
        v0 = self._ground_velocity(foot.liftoff)
        v1 = self._ground_velocity(td)
        T = self.p.swing_time
        return [
            hermite(foot.liftoff[0], v0[0], td[0], v1[0], s, T),
            hermite(foot.liftoff[1], v0[1], td[1], v1[1], s, T),
            -self.p.home(leg)[2] + swing_height(s, self.p.step_height),
        ]
