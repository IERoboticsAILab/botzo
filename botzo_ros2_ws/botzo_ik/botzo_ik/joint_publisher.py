#!/usr/bin/env python3

'''
This node get the targets positions for the 4 legs
publishes joint states to the /joint_states topic.
'''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from botzo_messages.msg import TargetEndEffectors
import numpy as np


# in cm
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
