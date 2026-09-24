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

Full cycle (NUM_STEPS = 20):
 Index:  0  1  2  3  4  5  6  7  8  9 | 10 11 12 13 14 15 16 17 18 19
         [--------  SWING  ----------] | [--------  STANCE  ----------]
          foot lifts, travels forward  |  foot on ground, pushes back

Diagonal A (FL+BR):  phase_a starts at  0
Diagonal B (FR+BL):  phase_b starts at 10  ← always half a cycle apart
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

        # Trajectory parameters
        self.MAX_STEP_HEIGHT = 3.0   # max Z lift (subtracted, so foot goes up)
        self.MAX_STEP_LENGTH = 5.0   # max X/Y travel per half-cycle
        self.MAX_SPEED = 1.0         # m/s normalization
        self.MAX_YAW   = 1.0         # rad/s normalization
        self.NUM_STEPS = 20          # points per full cycle (10 swing + 10 stance)
        self.HALF = self.NUM_STEPS // 2  # = 10

        # Neutral foot positions relative to each shoulder (at rest)
        self.neutral = {
            'fl': (0.0,  0.0, 12.0),
            'fr': (0.0,  0.0, 12.0),
            'bl': (0.0,  0.0, 12.0),
            'br': (0.0,  0.0, 12.0),
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
        self.phase_a = 0           # FL + BR
        self.phase_b = self.HALF   # FR + BL

        # 10 Hz timer — each tick = one step index advance
        self.create_timer(0.1, self.walk)

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

        Swing phase (0..HALF-1): foot lifts and travels forward
        Stance phase (HALF..NUM_STEPS-1): foot pushes back on ground
        '''
        trajectory = []
        for i in range(self.NUM_STEPS):
            if i < self.HALF:
                # --- SWING PHASE ---
                t = i / (self.HALF - 1) if self.HALF > 1 else 0.0
                x = dx_total/2 - dx_total * t
                y = dy_total/2 - dy_total * t
                z = -step_height * math.sin(math.pi * t)  # negative = up
            else:
                # --- STANCE PHASE ---
                t = (i - self.HALF) / (self.HALF - 1) if self.HALF > 1 else 0.0
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
        self.get_logger().info('/cmd_vel recived')

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

        print(f'SPEED: {speed:.2f} | DIR: {math.degrees(direction):.1f}° | YAW: {yaw:.2f} | phase_a={self.phase_a} phase_b={self.phase_b}')

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