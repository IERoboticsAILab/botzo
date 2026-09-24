#!/usr/bin/env python3

'''
Drive the simulated robot through a fixed set of commands and score the gait.

Needs sim.launch.py running. Uses sim time, so results do not depend on how
fast the computer is.

  ros2 run botzo_gazebo gait_eval
  ros2 run botzo_gazebo gait_eval --ros-args -p json:=/tmp/result.json
'''

import dataclasses
import json

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry

from botzo_gazebo.eval_metrics import quat_to_rpy, segment_metrics

# (name, seconds, (vx m/s, vy m/s, wz rad/s))
SCRIPT = [
    ('stand', 3.0, (0.0, 0.0, 0.0)),
    ('forward 0.15', 6.0, (0.15, 0.0, 0.0)),
    ('forward 0.30', 6.0, (0.30, 0.0, 0.0)),
    ('stop', 3.0, (0.0, 0.0, 0.0)),
    ('turn left 1.5', 5.0, (0.0, 0.0, 1.5)),
    ('strafe left 0.15', 5.0, (0.0, 0.15, 0.0)),
    ('backward 0.2', 5.0, (-0.2, 0.0, 0.0)),
    ('stop', 3.0, (0.0, 0.0, 0.0)),
]
SETTLE = 1.0  # s ignored at the start of each segment (acceleration / stopping)


class GaitEval(Node):
    def __init__(self):
        super().__init__('gait_eval', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.declare_parameter('json', '')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 50)
        self.samples = []
        self.segment = 0
        self.segment_start = None
        self.results = []
        self.create_timer(0.05, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.samples.append((self.now(), p.x, p.y, p.z, *quat_to_rpy(q.x, q.y, q.z, q.w)))

    def tick(self):
        if self.now() == 0.0:
            return  # no /clock yet
        if self.segment_start is None:
            self.segment_start = self.now()
            self.samples = []
        name, duration, cmd = SCRIPT[self.segment]
        self.cmd_pub.publish(Twist(linear=_vec(cmd[0], cmd[1]), angular=_vec(0.0, 0.0, cmd[2])))

        if self.now() - self.segment_start < duration:
            return
        steady = [s for s in self.samples if s[0] >= self.segment_start + SETTLE]
        if len(steady) >= 2:
            self.results.append((name, cmd, segment_metrics(steady)))
        self.segment += 1
        self.segment_start = None
        if self.segment == len(SCRIPT):
            self.report()
            raise SystemExit

    def report(self):
        header = f'{"segment":<18}{"cmd vx vy wz":>18}{"got vx vy wz":>20}{"tilt rms/max":>15}{"z":>7}{"bob":>7}  fell'
        lines = [header, '-' * len(header)]
        for name, cmd, m in self.results:
            lines.append(
                f'{name:<18}{cmd[0]:6.2f}{cmd[1]:6.2f}{cmd[2]:6.2f}'
                f'{m.vx:8.3f}{m.vy:6.3f}{m.wz:6.2f}'
                f'{m.tilt_rms:8.1f}/{m.tilt_max:<5.1f}{m.z_mean:7.3f}{m.z_bob * 100:6.1f}c  {"YES" if m.fell else "no"}')
        print('\n'.join(lines), flush=True)
        path = self.get_parameter('json').value
        if path:
            with open(path, 'w') as f:
                json.dump([{'segment': n, 'cmd': c, **dataclasses.asdict(m)} for n, c, m in self.results], f, indent=2)


def _vec(x=0.0, y=0.0, z=0.0):
    return Vector3(x=float(x), y=float(y), z=float(z))


def main(args=None):
    rclpy.init(args=args)
    node = GaitEval()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
