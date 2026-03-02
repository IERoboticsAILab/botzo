#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointPublisher(Node):
    def __init__(self):
        super().__init__('move_to_home')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joints)  # 10 Hz
        self.joint_state = JointState()
        '''
        NAMES:
        - BL_shoulder_joint
        - BR_shoulder_joint
        - FL_shoulder_joint
        - FR_shoulder_joint
        - FR_femur_joint
        - FR_tibia_joint
        - FL_femur_joint
        - FL_tibia_joint
        - BR_femur_joint
        - BR_tibia_joint
        - BL_femur_joint
        - BL_tibia_joint
        '''
        self.joint_state.name = ['BL_shoulder_joint', 'BR_shoulder_joint', 'FL_shoulder_joint', 'FR_shoulder_joint',
                                 'FR_femur_joint', 'FR_tibia_joint', 'FL_femur_joint', 'FL_tibia_joint',
                                 'BR_femur_joint', 'BR_tibia_joint', 'BL_femur_joint', 'BL_tibia_joint']
        self.joint_state.position = [0.0] * 12
        self.joint_state.velocity = [0.0] * 12
        self.joint_state.effort = [0.0] * 12


    def publish_joints(self):
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        # for home position: Shoulder: 85.77370457471147, Femur: 49.375728206004794, Tibia: 100.2396534509924
        shoulder_angle = 85.77370457471147 - 90
        femur_angle = 49.375728206004794 * -1
        tibia_angle = 100.2396534509924 - 90
        shoulder_angle_rad = shoulder_angle * 3.141592653589793 / 180.0
        femur_angle_rad = femur_angle * 3.141592653589793 / 180.0
        tibia_angle_rad = tibia_angle * 3.141592653589793 / 180.0
        self.joint_state.position = [shoulder_angle_rad, shoulder_angle_rad, shoulder_angle_rad, shoulder_angle_rad,
                                     femur_angle_rad, tibia_angle_rad, femur_angle_rad, tibia_angle_rad,
                                     femur_angle_rad, tibia_angle_rad, femur_angle_rad, tibia_angle_rad]        
        self.joint_state.velocity = [1.0, 0.5, 1.0, 1.0, 1.0, 1.5, 2.5, -2.5,
                                     -2.5, -2.5, -2.5, -2.5]
        self.joint_state.effort = [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0,
                                   2.0, 2.0, 2.0, 2.0]
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