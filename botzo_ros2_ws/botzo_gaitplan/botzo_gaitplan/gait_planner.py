#!/usr/bin/env python3

'''
Gait planner node — trot gait with sine-arc swing trajectories.
Subscribes to /cmd_vel and /current_end_effectors_pos
Publishes to /target_end_effectors

Frame (relative to leg shoulder):
  X: forward = negative, backward = positive
  Y: outside = positive, inside = negative
  Z: taller (foot higher) = negative, shorter (foot lower) = positive

Trot pairs:
  Diagonal A: FL + BR swing together
  Diagonal B: FR + BL swing together

At speed=0: legs bob up/down in place (no X/Y travel)
As speed increases: X travel spreads out along the direction vector


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


class GaitPlanner(Node):
    def __init__(self):
        super().__init__('gait_planner')

        self.cmd_vel_subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.current_end_effectors_subscription = self.create_subscription(
            CurrentEndEffectorsPos, 'current_end_effectors_pos', self.current_end_effectors_callback, 10)
        self.target_end_effectors_publisher = self.create_publisher(
            TargetEndEffectors, 'target_end_effectors', 10)

        self.current_end_effectors_msg = None
        self.cmd_vel_msg = None

        # Trajectory parameters
        self.MAX_STEP_HEIGHT = 3.0   # max Z lift (subtracted, so foot goes up)
        self.MAX_STEP_LENGTH = 5.0   # max X/Y travel per half-cycle
        self.MAX_SPEED = 1.0         # m/s normalization
        self.NUM_STEPS = 20          # points per full cycle (10 swing + 10 stance)
        self.HALF = self.NUM_STEPS // 2  # = 10

        # Neutral foot positions relative to each shoulder (at rest)
        # These are your "standing" positions
        self.neutral = {
            'fl': (0.0,  0.0, 12.0),
            'fr': (0.0,  0.0, 12.0),
            'bl': (0.0,  0.0, 12.0),
            'br': (0.0,  0.0, 12.0),
        }

        # Gait phase indices (0 to NUM_STEPS-1)
        # Diagonal A (FL+BR) starts at 0, Diagonal B (FR+BL) starts at HALF
        self.phase_a = 0   # FL + BR
        self.phase_b = self.HALF  # FR + BL

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
    #  Trajectory builder                                                  #
    # ------------------------------------------------------------------ #

    def compute_trajectory(self, speed, direction):
        '''
        Returns a list of NUM_STEPS (x, z) tuples for ONE leg.
        The trajectory is split into:
          - Swing phase (indices 0..HALF-1): foot lifts and travels forward
          - Stance phase (indices HALF..NUM_STEPS-1): foot pushes back on ground

        At speed=0: step_length=0 → pure up/down bob, no X travel.
        Y offset is handled separately via direction.

        X convention: forward = negative → swing moves in -X direction
        Z convention: up = negative → foot lifts by subtracting step_height
        '''
        step_length = (speed / self.MAX_SPEED) * self.MAX_STEP_LENGTH
        step_length = min(step_length, self.MAX_STEP_LENGTH)
        step_height = (speed / self.MAX_SPEED) * self.MAX_STEP_HEIGHT
        step_height = min(step_height, self.MAX_STEP_HEIGHT)
        # Always keep a minimum bob even at speed=0
        step_height = max(step_height, 1.5)

        # X component of travel along direction (forward = -X)
        dx = -math.cos(direction) * step_length  # negative = forward
        dy =  math.sin(direction) * step_length  # lateral component

        trajectory = []
        for i in range(self.NUM_STEPS):
            if i < self.HALF:
                # --- SWING PHASE ---
                # t goes 0→1 over the swing half
                t = i / (self.HALF - 1) if self.HALF > 1 else 0.0
                # X: travels from +dx/2 (back) to -dx/2 (forward)
                x = dx/2 - dx * t           # starts behind neutral, ends ahead
                y = dy/2 - dy * t
                # Z: sine arc — lifts in the middle of swing
                z = -step_height * math.sin(math.pi * t)  # negative = up
            else:
                # --- STANCE PHASE ---
                # t goes 0→1 over the stance half
                t = (i - self.HALF) / (self.HALF - 1) if self.HALF > 1 else 0.0
                # X: pushes from -dx/2 (ahead) back to +dx/2 (behind)
                x = -dx/2 + dx * t
                y = -dy/2 + dy * t
                # Z: flat on ground
                z = 0.0

            trajectory.append((x, y, z))

        return trajectory

    # ------------------------------------------------------------------ #
    #  Walk tick                                                           #
    # ------------------------------------------------------------------ #

    def walk(self):
        if self.cmd_vel_msg is None:
            self.get_logger().info('Waiting for cmd_vel...', throttle_duration_sec=2.0)
            return

        # --- Extract speed and direction ---
        lx = self.cmd_vel_msg.linear.x
        ly = self.cmd_vel_msg.linear.y
        speed = math.sqrt(lx**2 + ly**2)
        direction = math.atan2(ly, lx)  # -pi to pi

        self.get_logger().info(
            f'SPEED: {speed:.2f} m/s | DIR: {math.degrees(direction):.1f}° | '
            f'phase_a={self.phase_a} phase_b={self.phase_b}',
            throttle_duration_sec=0.5)

        # --- Rebuild trajectory every tick (reactive to cmd_vel changes) ---
        traj = self.compute_trajectory(speed, direction)

        # --- Read current target for this phase index ---
        # Diagonal A: FL + BR  (in phase)
        xa, ya, za = traj[self.phase_a]
        # Diagonal B: FR + BL  (offset by HALF)
        xb, yb, zb = traj[self.phase_b]

        # --- Build and publish target message ---
        msg = TargetEndEffectors()

        n = self.neutral

        # FL (Diagonal A)
        msg.x_fl = n['fl'][0] + xa
        msg.y_fl = n['fl'][1] + ya
        msg.z_fl = n['fl'][2] + za

        # BR (Diagonal A — same phase as FL)
        msg.x_br = n['br'][0] + xa
        msg.y_br = n['br'][1] + ya
        msg.z_br = n['br'][2] + za

        # FR (Diagonal B)
        msg.x_fr = n['fr'][0] + xb
        msg.y_fr = n['fr'][1] + yb
        msg.z_fr = n['fr'][2] + zb

        # BL (Diagonal B — same phase as FR)
        msg.x_bl = n['bl'][0] + xb
        msg.y_bl = n['bl'][1] + yb
        msg.z_bl = n['bl'][2] + zb

        self.target_end_effectors_publisher.publish(msg)

        # --- Advance phase indices (wrap around) ---
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