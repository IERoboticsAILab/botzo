#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joints)  # 10 Hz
        self.joint_state = JointState()
        '''
        NAMES:
        - LH_HAA
        - RH_HAA
        - RF_HAA
        - LF_HAA
        - LF_HFE
        - RF_HFE
        - RH_HFE
        - LH_HFE
        - LF_KFE
        - RF_KFE
        - LH_KFE
        - RH_KFE
        - RH_shank_fixed_RH_FOOT
        - LH_shank_fixed_LH_FOOT
        - LF_shank_fixed_LF_FOOT
        - RF_shank_fixed_RF_FOOT
        '''
        self.joint_state.name = ['LH_HAA', 'RH_HAA', 'RF_HAA', 'LF_HAA', 'LF_HFE', 'RF_HFE', 'RH_HFE', 'LH_HFE',
                                 'LF_KFE', 'RF_KFE', 'LH_KFE', 'RH_KFE', 'RH_shank_fixed_RH_FOOT', 'LH_shank_fixed_LH_FOOT',
                                 'LF_shank_fixed_LF_FOOT', 'RF_shank_fixed_RF_FOOT']
        self.joint_state.position = [0.0] * 16
        self.joint_state.velocity = [0.0] * 16
        self.joint_state.effort = [0.0] * 16

    def publish_joints(self):
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state.position = [1.0, 0.5, 0.0, 0.0, 1.0, 0.5, 0.0, 0.0,
                                     1.0, 0.5, 1.0, 0.5, 1.0, 1.0, 1.0, 1.0]
        self.joint_state.velocity = [1.0, 0.5, 1.0, 1.0, 1.0, 1.5, 2.5, -2.5,
                                     -2.5, -2.5, -2.5, -2.5, 0.0, 0.0, 0.0, 0.0]
        self.joint_state.effort = [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0,
                                   2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
        print(f'Publishing joint states: {self.joint_state}')
        self.publisher.publish(self.joint_state)

def main():
    rclpy.init()
    node = JointPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()