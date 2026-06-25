#!/usr/bin/env python3




'''
This node subscribes to the targets positions for the 4 legs
publishes joint states to the /joint_states topic and the current end-effector positions.
TargetEndEffectors.msg:
  float32 x_fl
  float32 y_fl
  float32 z_fl

  float32 x_fr
  float32 y_fr
  float32 z_fr

  float32 x_bl
  float32 y_bl
  float32 z_bl

  float32 x_br
  float32 y_br
  float32 z_br
'''



import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from botzo_messages.msg import TargetEndEffectors, CurrentEndEffectorsPos
import numpy as np



''' HELPER FUNCTIONS '''
coxa = 3.1 # from shoulder servo to the 2 other servos in the shoulder
femur = 9.5 # from top sevo to knee
tibia = 9.8 # from knee to foot
real_femur = 9.1 # lenght of 3D printed femur
dist_focuspoint_servo_femurtibia = 2.8 # distance from focus point/pivot of the 2 servos in the shoulder

def rad2deg(rads):
  return [rads[0]*180/np.pi, rads[1]*180/np.pi, rads[2]*180/np.pi]
def deg2rad(deg):
  return deg*np.pi/180

def legIK(x,y,z): #BR
  D = np.sqrt((z**2 + y**2) - coxa**2)
  G = np.sqrt(D**2 + x**2)
  shoulder_angle = np.arctan2(y,z) + np.arctan2(D,coxa)
  knee_angle = np.arccos((G**2 - femur**2 - tibia**2)/(-2*femur*tibia))
  femur_angle = deg2rad(180) - (deg2rad(90) + np.arctan2(x,D) + np.arcsin((tibia * np.sin(knee_angle)) / G))

  return rad2deg([shoulder_angle, femur_angle, knee_angle])

def adjust_angles_to_sim(angles_fl, angles_fr, angles_bl, angles_br):
  # all shoulders needs -90
  # all femurs needs * -1
  # all knees needs -90
  angles_fl[0] -= 90
  angles_fr[0] -= 90
  angles_bl[0] -= 90
  angles_br[0] -= 90

  angles_fl[1] *= -1
  angles_fr[1] *= -1
  angles_bl[1] *= -1
  angles_br[1] *= -1

  angles_fl[2] -= 90
  angles_fr[2] -= 90
  angles_bl[2] -= 90
  angles_br[2] -= 90
  return angles_fl, angles_fr, angles_bl, angles_br




''' NODE CLASS '''
class EnfEffectorSubscriber(Node):

  def __init__(self):
    super().__init__('end_effectors_subscriber')
    self.subscription = self.create_subscription(TargetEndEffectors, 'target_end_effectors', self.listener_callback, 10)

    self.current_end_effectors_publisher = self.create_publisher(CurrentEndEffectorsPos, 'current_end_effectors_pos', 10)
    self.current_end_effectors_msg = CurrentEndEffectorsPos()

    self.publisher = self.create_publisher(JointState, '/joint_states', 10)
    self.joint_state = JointState()
    self.joint_state.name = [ 
                              'BL_shoulder_joint', 'BR_shoulder_joint', 'FL_shoulder_joint', 'FR_shoulder_joint', 
                              'FR_femur_joint', 'FR_tibia_joint', 
                              'FL_femur_joint', 'FL_tibia_joint', 
                              'BR_femur_joint', 'BR_tibia_joint', 
                              'BL_femur_joint', 'BL_tibia_joint'
                            ]
    self.joint_state.position = [0.0] * 12
    self.joint_state.velocity = [0.0] * 12
    self.joint_state.effort = [0.0] * 12

  def listener_callback(self, msg):
    target_x_fl, target_y_fl, target_z_fl = msg.x_fl, msg.y_fl, msg.z_fl
    target_x_fr, target_y_fr, target_z_fr = msg.x_fr, msg.y_fr, msg.z_fr
    target_x_bl, target_y_bl, target_z_bl = msg.x_bl, msg.y_bl, msg.z_bl
    target_x_br, target_y_br, target_z_br = msg.x_br, msg.y_br, msg.z_br

    # calculate joint angles for each leg using IK (fl_angles[0] = shoulder, fl_angles[1] = femur, fl_angles[2] = tibia)
    fl_angles = legIK(target_x_fl, target_y_fl, target_z_fl)
    fr_angles = legIK(target_x_fr, target_y_fr, target_z_fr)
    bl_angles = legIK(target_x_bl, target_y_bl, target_z_bl)
    br_angles = legIK(target_x_br, target_y_br, target_z_br)
    # adjust angles to match the simulation's coordinate system and conventions
    fl_angles, fr_angles, bl_angles, br_angles = adjust_angles_to_sim(fl_angles, fr_angles, bl_angles, br_angles)
    
    print("Received target end-effectors:")
    print("\tFL leg target: ", msg.x_fl, msg.y_fl, msg.z_fl)
    print("\tFR leg target: ", msg.x_fr, msg.y_fr, msg.z_fr)
    print("\tBL leg target: ", msg.x_bl, msg.y_bl, msg.z_bl)
    print("\tBR leg target: ", msg.x_br, msg.y_br, msg.z_br)
    print("Calculated joint angles by IK:")
    print("\tFL leg angles: ", legIK(msg.x_fl, msg.y_fl, msg.z_fl))
    print("\tFR leg angles: ", legIK(msg.x_fr, msg.y_fr, msg.z_fr))
    print("\tBL leg angles: ", legIK(msg.x_bl, msg.y_bl, msg.z_bl))
    print("\tBR leg angles: ", legIK(msg.x_br, msg.y_br, msg.z_br))
    print("Adjust angles to sim:")
    print("\tFL adjusted angles: ", fl_angles)
    print("\tFR adjusted angles: ", fr_angles)
    print("\tBL adjusted angles: ", bl_angles)
    print("\tBR adjusted angles: ", br_angles)
    print("\n--------------------------------------------------\n")

    # publish joint states
    self.joint_state.header.stamp = self.get_clock().now().to_msg()
    self.joint_state.position = [deg2rad(bl_angles[0]), deg2rad(br_angles[0]), deg2rad(fl_angles[0]), deg2rad(fr_angles[0]),
                                 deg2rad(fr_angles[1]), deg2rad(fr_angles[2]), deg2rad(fl_angles[1]), deg2rad(fl_angles[2]),
                                 deg2rad(br_angles[1]), deg2rad(br_angles[2]), deg2rad(bl_angles[1]), deg2rad(bl_angles[2])]
    self.joint_state.velocity = [1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5]
    self.joint_state.effort = [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
    self.publisher.publish(self.joint_state)

    # publish current end-effector positions
    self.current_end_effectors_msg.x_fl = target_x_fl
    self.current_end_effectors_msg.y_fl = target_y_fl
    self.current_end_effectors_msg.z_fl = target_z_fl

    self.current_end_effectors_msg.x_fr = target_x_fr
    self.current_end_effectors_msg.y_fr = target_y_fr
    self.current_end_effectors_msg.z_fr = target_z_fr

    self.current_end_effectors_msg.x_bl = target_x_bl
    self.current_end_effectors_msg.y_bl = target_y_bl
    self.current_end_effectors_msg.z_bl = target_z_bl

    self.current_end_effectors_msg.x_br = target_x_br
    self.current_end_effectors_msg.y_br = target_y_br
    self.current_end_effectors_msg.z_br = target_z_br

    self.current_end_effectors_publisher.publish(self.current_end_effectors_msg)



''' MAIN FUNCTION '''
def main(args=None):
  print("Starting IK joint publisher node...")
  print("This node subscribe to target end-effectors and publish joint states and current end-effector positions.")
  print("Ready:\n")
  rclpy.init(args=args)
  end_effectors_subscriber = EnfEffectorSubscriber()
  rclpy.spin(end_effectors_subscriber)
  end_effectors_subscriber.destroy_node()
  rclpy.shutdown()



''' EXECUTE MAIN '''
if __name__ == '__main__':
  main()