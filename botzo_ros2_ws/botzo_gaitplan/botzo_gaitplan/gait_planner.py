#!/usr/bin/env python3

'''
Gait planner node: continuous-phase trot (see gait_core.py for the maths).

Subscribes: /cmd_vel (geometry_msgs/Twist, REP-103: x forward, y left, yaw CCW)
Publishes:  /target_end_effectors (botzo_messages/TargetEndEffectors, IK leg frame, cm)

Every gait_core.GaitParams field is a ROS parameter and can be changed live:
  ros2 param set /gait_planner period 0.5
  ros2 param set /gait_planner home_bl "[2.0, 4.0, 16.0]"
'''

import dataclasses

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from botzo_messages.msg import TargetEndEffectors

from botzo_gaitplan.gait_core import GaitParams, TrotGait


class GaitPlanner(Node):
    def __init__(self):
        super().__init__('gait_planner')

        self.declare_parameter('rate', 50.0)            # Hz, target publish rate
        self.declare_parameter('cmd_timeout', 0.5)      # s, zero the command if cmd_vel goes quiet
        params = GaitParams()
        for f in dataclasses.fields(GaitParams):
            default = getattr(params, f.name)
            # ROS has no tuple type: (x, y, z) homes are declared as double arrays.
            self.declare_parameter(f.name, list(default) if isinstance(default, tuple) else default)
            setattr(params, f.name, self._to_field(f.name, self.get_parameter(f.name).value))

        self.gait = TrotGait(params)
        self.add_on_set_parameters_callback(self.on_set_parameters)

        self.cmd = (0.0, 0.0, 0.0)
        self.last_cmd_time = None
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.publisher = self.create_publisher(TargetEndEffectors, 'target_end_effectors', 10)

        self.dt = 1.0 / self.get_parameter('rate').value
        self.last_tick = self.get_clock().now()
        self.create_timer(self.dt, self.tick)
        self.get_logger().info(f'gait_planner ready at {1.0 / self.dt:.0f} Hz: {params}')

    @staticmethod
    def _to_field(name, value):
        return tuple(value) if name.startswith('home_') else value

    def on_set_parameters(self, changes):
        field_names = {f.name for f in dataclasses.fields(GaitParams)}
        for p in changes:
            if p.name not in field_names:
                continue
            if p.name == 'period' and p.value <= 0.0:
                return SetParametersResult(successful=False, reason='period must be > 0')
            if p.name == 'duty_factor' and not 0.0 < p.value < 1.0:
                return SetParametersResult(successful=False, reason='duty_factor must be in (0, 1)')
            if p.name.startswith('home_') and len(p.value) != 3:
                return SetParametersResult(successful=False, reason='home_* must be [x, y, z]')
        for p in changes:
            if p.name in field_names:
                setattr(self.gait.p, p.name, self._to_field(p.name, p.value))
        if not self.gait.state.walking:
            self.gait.reset()  # standing: move the feet to the (possibly new) homes now
        return SetParametersResult(successful=True)

    def cmd_vel_callback(self, msg):
        self.cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.last_cmd_time = self.get_clock().now()

    def tick(self):
        now = self.get_clock().now()
        # Use the real elapsed time so a late timer does not slow the gait down.
        dt = min((now - self.last_tick).nanoseconds * 1e-9, 5 * self.dt)
        self.last_tick = now
        if dt <= 0.0:
            return

        cmd = self.cmd
        timeout = self.get_parameter('cmd_timeout').value
        if self.last_cmd_time is None or (now - self.last_cmd_time).nanoseconds * 1e-9 > timeout:
            cmd = (0.0, 0.0, 0.0)

        targets = self.gait.update(dt, *cmd)

        msg = TargetEndEffectors()
        for leg, (x, y, z) in targets.items():
            setattr(msg, f'x_{leg}', float(x))
            setattr(msg, f'y_{leg}', float(y))
            setattr(msg, f'z_{leg}', float(z))
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GaitPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
