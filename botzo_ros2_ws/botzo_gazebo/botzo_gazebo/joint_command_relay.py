#!/usr/bin/env python3

'''
Forward the IK solution to the simulated servos.

botzo_ik publishes the joint angles it wants as sensor_msgs/JointState. In
simulation /joint_states belongs to the joint_state_broadcaster (measured
angles), so the IK output is remapped to /ik/joint_states and this node turns
it into the Float64MultiArray that position_controller expects, in the
controller's joint order.
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class JointCommandRelay(Node):
    def __init__(self):
        super().__init__('joint_command_relay')
        self.declare_parameter('joints', [''])
        self.joints = self.get_parameter('joints').value
        self.publisher = self.create_publisher(Float64MultiArray, 'position_controller/commands', 10)
        self.create_subscription(JointState, 'ik/joint_states', self.on_joint_states, 10)

    def on_joint_states(self, msg):
        positions = dict(zip(msg.name, msg.position))
        missing = [j for j in self.joints if j not in positions]
        if missing:
            self.get_logger().warn(f'IK message is missing joints {missing}', throttle_duration_sec=2.0)
            return
        self.publisher.publish(Float64MultiArray(data=[positions[j] for j in self.joints]))


def main(args=None):
    rclpy.init(args=args)
    node = JointCommandRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
