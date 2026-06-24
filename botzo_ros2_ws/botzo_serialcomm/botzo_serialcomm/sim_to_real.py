#!/usr/bin/env python3

'''
Subscribe to the `/joint_states` and publish to `/real_robot_joint_states` 


Subscribe to the `/joint_states` topic (which is published by the `joint_publisher` node). 
This joint states are the ones in the RViz simularion. 
But because the servos zero's are different in the real robot from the URDF, 
the script and transform the angles in simulation in the same angles to reach the same end-effectors position in the real robot. 
The script will transform the angles and publish them to the `/real_robot_joint_states` topic. 
This way we can move the real robot according to the target end-effectors we publish to the `/target_end_effectors` topic.

RealRobotJointStates message type: sensor_msgs/msg/JointState.msg
float32 sfr
float32 ffr
float32 tfr

float32 sfl
float32 ffl
float32 tfl

float32 sbr
float32 fbr
float32 tbr

float32 sbl
float32 fbl
float32 tbl
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from botzo_messages.msg import RealRobotJointStates
import numpy as np


def rad2deg(rads):
  return [rads[0]*180/np.pi, rads[1]*180/np.pi, rads[2]*180/np.pi]


def adjust_angles_sim_to_real(current_angles_fl, current_angles_fr, current_angles_bl, current_angles_br):
  # all shoulders needs +90
  # all femurs needs * -1
  # all knees needs +90
  angles_fl = [0,0,0]
  angles_fr = [0,0,0]
  angles_bl = [0,0,0]
  angles_br = [0,0,0]

  # Shouldersì angles
  angles_fl[0] = current_angles_fl[0] + 90
  angles_fr[0] = 180 - (current_angles_fr[0] + 90)
  angles_bl[0] = 180 - (current_angles_bl[0] + 90)
  angles_br[0] = (current_angles_br[0] + 90)

  # Femur angles
  angles_fl[1] = 180 - (current_angles_fl[1] * -1)
  angles_bl[1] = 180 - (current_angles_bl[1] * -1)
  angles_fr[1] = current_angles_fr[1] * -1
  angles_br[1] = current_angles_br[1] * -1

  # Tibia angles
  theta_fl = 90 + current_angles_fl[2] # full knee_angle = np.arccos((G**2 - femur**2 - tibia**2)/(-2*femur*tibia))
  #print(f"knee angle: {theta_fl}")
  gamma_fl = 90 - (current_angles_fl[1] * -1) # full femur_angle = np.arctan2(x,D) + np.arcsin((tibia * np.sin(knee_angle)) / G)
  #print(f"femur angle: {gamma_fl}")
  x = 180 - (theta_fl + gamma_fl)
  #print(f"x; {x}")
  angles_fl[2] = 90 - x

  theta_bl = 90 + current_angles_bl[2]
  gamma_bl = 90 - (current_angles_bl[1] * -1)
  x = 180 - (theta_bl + gamma_bl)
  angles_bl[2] = 90 - x

  theta_fr = 90 + current_angles_fr[2]
  gamma_fr = 90 - (current_angles_fr[1] * -1)
  x = 180 - (theta_fr + gamma_fr)
  angles_fr[2] = 90 - x
  angles_fr[2] = 180 - angles_fr[2] # left side had opposite angles in servos (mirror)

  theta_br = 90 + current_angles_br[2]
  gamma_br = 90 - (current_angles_br[1] * -1)
  x = 180 - (theta_br + gamma_br)
  angles_br[2] = 90 - x
  angles_br[2] = 180 - angles_br[2]

  return angles_fl, angles_fr, angles_bl, angles_br



class SimToReal(Node):
    def __init__(self):
        super().__init__('sim_to_real')
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.publisher = self.create_publisher(RealRobotJointStates, '/real_robot_joint_states', 10)

    def joint_state_callback(self, msg):
        current_angles_fl = rad2deg([msg.position[2], msg.position[6], msg.position[7]])
        current_angles_fr = rad2deg([msg.position[3], msg.position[4], msg.position[5]])
        current_angles_bl = rad2deg([msg.position[0], msg.position[8], msg.position[9]])
        current_angles_br = rad2deg([msg.position[1], msg.position[10], msg.position[11]])
        angles_fl, angles_fr, angles_bl, angles_br = adjust_angles_sim_to_real(current_angles_fl, current_angles_fr, current_angles_bl, current_angles_br)

        print(f"Recived current joint states:")
        print(f"\tFL: {current_angles_fl}")
        print(f"\tFR: {current_angles_fr}")
        print(f"\tBL: {current_angles_bl}")
        print(f"\tBR: {current_angles_br}")
        print(f"Adjusted angles for real robot:")
        print(f"\tFL: {angles_fl}")
        print(f"\tFR: {angles_fr}")
        print(f"\tBL: {angles_bl}")
        print(f"\tBR: {angles_br}")
        print("\n--------------------------------------------------\n")


        # Publish the adjusted angles to the real robot joint states topic
        real_robot_joint_states_msg = RealRobotJointStates()
        real_robot_joint_states_msg.sfr = angles_fr[0]
        real_robot_joint_states_msg.ffr = angles_fr[1]
        real_robot_joint_states_msg.tfr = angles_fr[2]
        real_robot_joint_states_msg.sfl = angles_fl[0]
        real_robot_joint_states_msg.ffl = angles_fl[1]
        real_robot_joint_states_msg.tfl = angles_fl[2]
        real_robot_joint_states_msg.sbr = angles_br[0]
        real_robot_joint_states_msg.fbr = angles_br[1]
        real_robot_joint_states_msg.tbr = angles_br[2]
        real_robot_joint_states_msg.sbl = angles_bl[0]
        real_robot_joint_states_msg.fbl = angles_bl[1]
        real_robot_joint_states_msg.tbl = angles_bl[2]
        self.publisher.publish(real_robot_joint_states_msg)

def main(args=None):
    print("Starting sim to real node...")
    print("This node subscibe to /joint_state and transform them into /real_robot_joint_states")
    print("Ready:\n")
    rclpy.init(args=args)
    sim_to_real_node = SimToReal()
    rclpy.spin(sim_to_real_node)
    sim_to_real_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()