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



def deg2PWM(desire_deg_angle, coefficents):
    a, b, c = coefficents
    pulse = round((a * desire_deg_angle**2) + (b * desire_deg_angle) + c, 0)
    return pulse

def deg2PWM_set_angles(angles, coefficents_S, coefficents_F, coefficents_T):
  angles_PWM = []
  for angle in angles:
    angles_PWM.append([deg2PWM(angle[0], coefficents_S), deg2PWM(angle[1], coefficents_F), deg2PWM(angle[2], coefficents_T)])
  return angles_PWM

def rad2deg(rads):
  return [rads[0]*180/np.pi, rads[1]*180/np.pi, rads[2]*180/np.pi]
def deg2rad(deg):
  return deg*np.pi/180


def adjust_angles_sim_to_real(angles_fl, angles_fr, angles_bl, angles_br):
  # all shoulders needs +90
  # all femurs needs * -1
  # all knees needs +90
  angles_fl[0] += 90 # ok

  angles_fr[0] += 90
  angles_fr[0] = 180 - angles_fr[0]

  angles_bl[0] += 90

  angles_br[0] += 90
  angles_br[0] = 180 - angles_br[0]



  angles_fl[1] *= -1 # ok
  angles_fl[1] = 180 - angles_fl[1]

  angles_bl[1] *= -1
  angles_bl[1] = 180 - angles_bl[1]

  angles_fr[1] *= -1

  angles_br[1] *= -1



  #angles_fl[2] += 90 
  #angles_fl[2] = 180 - angles_fl[2]
  #angles_bl[2] += 90
  #angles_bl[2] = 180 - angles_bl[2]
  #angles_fr[2] += 90
  #angles_br[2] += 90

  # tetha = angles_fl[2] + 90     |    angles_fl[2] = -90 + theta + gamma     |    gamma = 180 - 90 - angles_fl[1]
  tetha_fl = angles_fl[2] + 90
  gamma_fl = 90 - angles_fl[1]
  x = 180 - (tetha_fl + gamma_fl)
  angles_fl[2] = 90 - x

  tetha_bl = angles_bl[2] + 90
  gamma_bl = 90 - angles_bl[1]
  x = 180 - (tetha_bl + gamma_bl)
  angles_bl[2] = 90 - x
  ## angles_fl[2] = 90 - tetha_fl - gamma_fl
  ##tetha_bl = angles_bl[2] + 90
  ##gamma_bl = 180 - 90 - angles_bl[1]
  ##angles_bl[2] = 90 - tetha_bl - gamma_bl
  ## tetha_fr = angles_fr[2] + 90
  ## gamma_fr = 180 - 90 - angles_fr[1]
  ## angles_fr[2] = -90 + tetha_fr + gamma_fr
  ## tetha_br = angles_br[2] + 90
  ## gamma_br = 180 - 90 - angles_br[1]
  ## angles_br[2] = -90 + tetha_br + gamma_br

  angles_fl[2] *= -1

  angles_bl[2] *= -1

  angles_fr[2] *= -1
  angles_fr[2] = 180 - angles_fr[2]

  angles_br[2] *= -1
  angles_br[2] = 180 - angles_br[2]

  return angles_fl, angles_fr, angles_bl, angles_br



class SimToReal(Node):
    def __init__(self):
        super().__init__('sim_to_real')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        self.publisher = self.create_publisher(
            RealRobotJointStates,
            '/real_robot_joint_states',
            10)

    def joint_state_callback(self, msg):
        angles_fl = rad2deg([msg.position[2], msg.position[6], msg.position[7]])
        angles_fr = rad2deg([msg.position[3], msg.position[4], msg.position[5]])
        angles_bl = rad2deg([msg.position[0], msg.position[8], msg.position[9]])
        angles_br = rad2deg([msg.position[1], msg.position[10], msg.position[11]])
        print(f"\nRECIVED CURRENT JOINT STATE:\nFL: {angles_fl}, FR: {angles_fr}, BL: {angles_bl}, BR: {angles_br}")
        angles_fl, angles_fr, angles_bl, angles_br = adjust_angles_sim_to_real(angles_fl, angles_fr, angles_bl, angles_br)
        print(f"\nADJUSTED ANGLES:\nFL: {angles_fl}, FR: {angles_fr}, BL: {angles_bl}, BR: {angles_br}")

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
    rclpy.init(args=args)
    sim_to_real_node = SimToReal()
    rclpy.spin(sim_to_real_node)
    sim_to_real_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()