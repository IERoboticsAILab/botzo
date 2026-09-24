#!/usr/bin/env python3

'''
Gait planner node: trot gait with sine-arc swing trajectories.
Subscribes to /cmd_vel and /current_end_effectors_pos
Publishes to /target_end_effectors

Frame (relative to leg shoulder):
  X: forward = negative, backward = positive
  Y: outside = positive, inside = negative
  Z: taller (foot higher) = negative, shorter (foot lower) = positive

Trot pairs:
  Diagonal A: FL + BR swing together
  Diagonal B: FR + BL swing together

At speed=0 and yaw=0: legs bob up/down in place (no X/Y travel)
As speed increases: X travel spreads out along the direction vector
As yaw increases: legs get tangential X offsets to rotate in place

Full cycle (NUM_STEPS points, currently 10):
 Index:  0 .. SWING_STEPS-1        | SWING_STEPS .. NUM_STEPS-1
         [------  SWING  ---------] | [--------  STANCE  ----------]
          foot lifts, travels forward |  foot on ground, pushes back

SWING_RATIO controls the split. With SWING_RATIO < 0.5, each leg spends
*more* of the cycle in stance than in swing. Because the two diagonals
are still offset by exactly half the cycle (phase_a / phase_b), this
creates brief windows where BOTH diagonals are in stance at once
(all 4 feet down) instead of the gait being 2-feet-down 100% of the
time. That overlap is what stops the robot from tipping the instant a
single stance leg is slightly late or the body sways - the previous
50/50 split had zero margin for that.

Diagonal A (FL+BR):  phase_a starts at  0
Diagonal B (FR+BL):  phase_b starts at NUM_STEPS/2  <- always half a cycle apart

Neutral (home) stance is set per leg (see self.neutral in __init__) rather than
computed from one shared formula, so each leg's resting x/y/z - including any
one-off calibration offset like BL's - can be tuned independently.
'''

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from botzo_messages.msg import TargetEndEffectors, CurrentEndEffectorsPos


# Approximate robot half-length/width used to compute tangential foot velocity.
# Tune these to match your physical robot (in the same units as neutral positions).
ROBOT_HALF_LENGTH = 10.0   # front-to-back distance from center to shoulder, same unit as neutral Z (e.g. cm)
ROBOT_HALF_WIDTH  = 6.0    # left-to-right distance from center to shoulder


class GaitPlanner(Node):
    def __init__(self):
        super().__init__('gait_planner')

        self.cmd_vel_subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.current_end_effectors_subscription = self.create_subscription(CurrentEndEffectorsPos, 'current_end_effectors_pos', self.current_end_effectors_callback, 10)
        self.target_end_effectors_publisher = self.create_publisher(TargetEndEffectors, 'target_end_effectors', 10)

        self.current_end_effectors_msg = None
        self.cmd_vel_msg = None

        # ------------------------------------------------------------ #
        #  Trajectory parameters                                        #
        # ------------------------------------------------------------ #
        self.MAX_STEP_HEIGHT = 5.0   # max Z lift (subtracted, so foot goes up)
        self.MAX_STEP_LENGTH = 7.0   # max X/Y travel per half-cycle - raised from 5.0 for
                                      # a longer stride per step, on top of the faster
                                      # TIMER_PERIOD below, for more overall ground speed
        self.MAX_SPEED = 1.0         # m/s normalization
        self.MAX_YAW   = 0.5         # rad/s normalization
        self.NUM_STEPS = 10          # points per full cycle

        # Fraction of the cycle spent in SWING (foot in the air). The rest is
        # STANCE (foot planted, pushing the body along). Lower = shorter,
        # quicker swing and longer, more stable stance with 4-feet-down
        # overlap between the two diagonals. 0.5 = old behaviour (no overlap,
        # least stable). Tune this before touching timer speed if the robot
        # is still tipping.
        self.SWING_RATIO = 0.35
        self.SWING_STEPS  = max(1, round(self.NUM_STEPS * self.SWING_RATIO))
        self.STANCE_STEPS = self.NUM_STEPS - self.SWING_STEPS

        # How fast the whole node ticks. Full stride time = NUM_STEPS * TIMER_PERIOD.
        # At the original 0.1s/10 steps that was a 1.0s stride - far too slow to be
        # dynamically stable on a trot. 0.015s (~66 Hz) gives a 0.15s stride instead.
        # If your actual servo/controller loop can't keep up at that rate, back this
        # off rather than fighting it - a gait the hardware can't track is worse
        # than a slightly slower one it can.
        self.TIMER_PERIOD = 0.015

        # ------------------------------------------------------------ #
        #  Neutral (home) stance - set per leg, independently tunable   #
        # ------------------------------------------------------------ #
        # Each leg gets its OWN (x, y, z) rather than a shared formula, because
        # real hardware is never perfectly symmetric - one leg folding in
        # further than its mirror is a calibration fact, not something the
        # gait math should try to "fix" automatically.
        #
        #   x: 0 for front legs, positive (backward) for back legs - pulls the
        #      back feet out from under the body to enlarge the support polygon
        #      front-to-back and sit more of it under the (usually rear-heavy) CoM.
        #   y: positive = outside. Bumped up from 0 so all feet stand further out
        #      from the shoulder instead of almost directly under it - a wider
        #      stance is a more stable stance.
        #   z: raised from 12 -> 16 (shorter/more folded leg) to go with the wider
        #      x/y spread and keep the leg within a comfortable range of motion.
        #
        # BL specifically needed a bigger Y (4 instead of the 2 every other leg
        # uses) - without it, that leg sits tucked in under the belly instead of
        # in a proper wide stance like its BR mirror. Re-tune this one value if
        # you recalibrate the physical leg.
        self.neutral = {
            'fl': (0.0, 2.0, 16.0),
            'fr': (0.0, 2.0, 16.0),
            'bl': (2.0, 4.0, 16.0),   # y=4: calibration fix, this leg folds in more than its mirror
            'br': (2.0, 2.0, 16.0),
        }

        # Each leg's position relative to robot center (x_body, y_body).
        # Used to compute the tangential direction for yaw.
        #   x_body: positive = forward
        #   y_body: positive = left
        self.leg_pos = {
            'fl': ( ROBOT_HALF_LENGTH,  ROBOT_HALF_WIDTH),
            'fr': ( ROBOT_HALF_LENGTH, -ROBOT_HALF_WIDTH),
            'bl': (-ROBOT_HALF_LENGTH,  ROBOT_HALF_WIDTH),
            'br': (-ROBOT_HALF_LENGTH, -ROBOT_HALF_WIDTH),
        }

        # Gait phase indices (0 to NUM_STEPS-1)
        self.phase_a = 0                        # FL + BR
        self.phase_b = self.NUM_STEPS // 2      # FR + BL (always half a cycle later)

        self.create_timer(self.TIMER_PERIOD, self.walk)

    # ------------------------------------------------------------------ #
    #  Callbacks                                                           #
    # ------------------------------------------------------------------ #

    def cmd_vel_callback(self, msg):
        self.cmd_vel_msg = msg

    def current_end_effectors_callback(self, msg):
        self.current_end_effectors_msg = msg

    # ------------------------------------------------------------------ #
    #  Tangential offset for yaw                                           #
    # ------------------------------------------------------------------ #

    def yaw_offset(self, leg: str, yaw: float):
        '''
        Given a yaw rate (rad/s, positive = counter-clockwise / turn left),
        return (dx, dy) that should be added to this leg's step travel.

        Tangential velocity for a point at (px, py) rotating around Z:
          vx = -omega * py
          vy =  omega * px
        We scale it by MAX_STEP_LENGTH so it blends naturally with linear motion.
        '''
        px, py = self.leg_pos[leg]
        scale = (yaw / self.MAX_YAW) * self.MAX_STEP_LENGTH
        scale = max(-self.MAX_STEP_LENGTH, min(self.MAX_STEP_LENGTH, scale))

        # Tangential direction (normalized by distance from center)
        dist = math.sqrt(px**2 + py**2)
        if dist < 1e-6:
            return 0.0, 0.0

        # Unit tangential vector (CCW rotation)
        tx = -py / dist   # forward/backward component
        ty =  px / dist   # left/right component

        dx = tx * abs(scale) * math.copysign(1.0, yaw)
        dy = ty * abs(scale) * math.copysign(1.0, yaw)

        return dx, dy

    # ------------------------------------------------------------------ #
    #  Trajectory builder                                                  #
    # ------------------------------------------------------------------ #

    def compute_trajectory(self, dx_total, dy_total, step_height):
        '''
        Returns a list of NUM_STEPS (x, y, z) tuples for ONE leg,
        given the total X and Y travel per half-cycle (dx_total, dy_total)
        and the step height.

        Swing phase (0..SWING_STEPS-1): foot lifts and travels forward, fast.
        Stance phase (SWING_STEPS..NUM_STEPS-1): foot pushes back on ground, slower.

        Both phases are parameterized by a normalized t in [0, 1], so the
        endpoints always line up (continuous trajectory) regardless of how
        the SWING_STEPS/STANCE_STEPS split is chosen.
        '''
        trajectory = []
        for i in range(self.NUM_STEPS):
            if i < self.SWING_STEPS:
                # --- SWING PHASE ---
                t = i / (self.SWING_STEPS - 1) if self.SWING_STEPS > 1 else 0.0
                x = dx_total/2 - dx_total * t
                y = dy_total/2 - dy_total * t
                z = -step_height * math.sin(math.pi * t)  # negative = up
            else:
                # --- STANCE PHASE ---
                t = (i - self.SWING_STEPS) / (self.STANCE_STEPS - 1) if self.STANCE_STEPS > 1 else 0.0
                x = -dx_total/2 + dx_total * t
                y = -dy_total/2 + dy_total * t
                z = 0.0

            trajectory.append((x, y, z))

        return trajectory

    # ------------------------------------------------------------------ #
    #  Walk tick                                                           #
    # ------------------------------------------------------------------ #

    def walk(self):
        if self.cmd_vel_msg is None:
            self.get_logger().info('Waiting for cmd_vel...')
            return

        # --- Extract velocity components ---
        lx  = self.cmd_vel_msg.linear.x
        ly  = self.cmd_vel_msg.linear.y
        yaw = self.cmd_vel_msg.angular.z   # rad/s, positive = turn left (CCW)

        speed     = math.sqrt(lx**2 + ly**2)
        direction = math.atan2(ly, lx)  # -pi to pi

        # Step height scales with total motion (translation OR rotation)
        yaw_speed = abs(yaw) / self.MAX_YAW
        motion    = min(max(speed / self.MAX_SPEED, yaw_speed), 1.0)
        step_height = max(motion * self.MAX_STEP_HEIGHT, 1.5)  # min bob

        # Linear travel components (forward = -X in leg frame)
        step_length = min((speed / self.MAX_SPEED) * self.MAX_STEP_LENGTH, self.MAX_STEP_LENGTH)
        lin_dx = -math.cos(direction) * step_length  # negative = forward
        lin_dy =  math.sin(direction) * step_length

        self.get_logger().debug(
            f'SPEED: {speed:.2f} | DIR: {math.degrees(direction):.1f}° | YAW: {yaw:.2f} '
            f'| phase_a={self.phase_a} phase_b={self.phase_b} '
            f'| swing/stance={self.SWING_STEPS}/{self.STANCE_STEPS}'
        )

        # --- Per-leg trajectory (linear + yaw blended) ---
        def leg_traj(leg):
            ydx, ydy = self.yaw_offset(leg, yaw)
            # In leg frame: X forward = negative, so yaw contribution is negated on X
            dx_total = lin_dx - ydx
            dy_total = lin_dy + ydy
            return self.compute_trajectory(dx_total, dy_total, step_height)

        traj_fl = leg_traj('fl')
        traj_fr = leg_traj('fr')
        traj_bl = leg_traj('bl')
        traj_br = leg_traj('br')

        # --- Read current target for each leg's phase index ---
        xa_fl, ya_fl, za_fl = traj_fl[self.phase_a]
        xa_br, ya_br, za_br = traj_br[self.phase_a]   # Diagonal A shares phase_a

        xb_fr, yb_fr, zb_fr = traj_fr[self.phase_b]
        xb_bl, yb_bl, zb_bl = traj_bl[self.phase_b]   # Diagonal B shares phase_b

        # --- Build and publish target message ---
        msg = TargetEndEffectors()
        n = self.neutral

        # FL (Diagonal A)
        msg.x_fl = n['fl'][0] - xa_fl
        msg.y_fl = n['fl'][1] + ya_fl
        msg.z_fl = n['fl'][2] + za_fl

        # BR (Diagonal A)
        msg.x_br = n['br'][0] - xa_br
        msg.y_br = n['br'][1] - ya_br  # opposite Y for opposite leg
        msg.z_br = n['br'][2] + za_br

        # FR (Diagonal B)
        msg.x_fr = n['fr'][0] - xb_fr
        msg.y_fr = n['fr'][1] - yb_fr
        msg.z_fr = n['fr'][2] + zb_fr

        # BL (Diagonal B)
        msg.x_bl = n['bl'][0] - xb_bl
        msg.y_bl = n['bl'][1] + yb_bl
        msg.z_bl = n['bl'][2] + zb_bl

        self.target_end_effectors_publisher.publish(msg)

        # --- Advance phase indices ---
        self.phase_a = (self.phase_a + 1) % self.NUM_STEPS
        self.phase_b = (self.phase_b + 1) % self.NUM_STEPS


def main(args=None):
    rclpy.init(args=args)
    gait_planner = GaitPlanner()
    rclpy.spin(gait_planner)
    gait_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()