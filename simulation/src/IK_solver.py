import numpy as np
import time

# in cm
coxa = 3.1 # from shoulder servo to the 2 other servos in the shoulder
femur = 9.5 # from top sevo to knee
tibia = 9.8 # from knee to foot
real_femur = 9.1 # lenght of 3D printed femur
dist_focuspoint_servo_femurtibia = 2.8 # distance from focus point/pivot of the 2 servos in the shoulder

''' HARD CODED TRAJECTORIES '''
home = [[0,2,16]]
sleep = [[0,2,13],[0,2,10],[0,2,8]]
jump = [[0,2,8],[-1,2,16],[0,2,9],[0,2,16]]
forward_targets_FR_BL = [
    [0,3,15],[0.5,3,15],[1,3,15],[1.5,3,15],[2,3,15],[2.5,3,15],[3,3,15],[3.5,3,15],[4,3,15],
    [4,3,14],[4,3,13],[4,3,12],
    [3,3,12],[2,3,12],[1,3,12],[0,3,12],
    [0,3,13],[0,3,14]
]
forward_targets_FL_BR = [
    [4,3,15],[4,3,14],[4,3,13],[4,3,12],
    [3,3,12],[2,3,12],[1,3,12],[0,3,12],
    [0,3,13],[0,3,14],[0,3,15],
    [0.5,3,15],[1,3,15],[1.5,3,15],[2,3,15],[2.5,3,15],[3,3,15],[3.5,3,15]
]
backward_targets_FR_BL = [
    [0,3,15],[0,3,14],[0,3,13],[0,3,12],
    [1,3,12],[2,3,12],[3,3,12],[4,3,12],
    [4,3,13],[4,3,14],[4,3,15],
    [3.5,3,15],[3,3,15],[2.5,3,15],[2,3,15],[1.5,3,15],[1,3,15],[0.5,3,15]
]
backward_targets_FL_BR = [
    [4,3,15],[3.5,3,15],[3,3,15],[2.5,3,15],[2,3,15],[1.5,3,15],[1,3,15],[0.5,3,15],
    [0,3,15],[0,3,14],[0,3,13],[0,3,12],
    [1,3,12],[2,3,12],[3,3,12],[4,3,12],
    [4,3,13],[4,3,14]
]
spin_left_targets_FR_BL = [
    [0,3,15],[0,2.5,15],[0,2,15],[0,1.5,15],[0,1,15],[0,0.5,15],[0,0,15],[0,-0.5,15],[0,-1,15],
    [0,-1,14],[0,-1,13],[0,-1,12],
    [0,0,12],[0,1,12],[0,2,12],[0,3,12],
    [0,3,13],[0,3,14]
]
spin_left_targets_FL_BR = [
    [0,3,15],[0,3,14],[0,3,13],[0,3,12],
    [0,2,12],[0,1,12],[0,0,12],[0,-1,12],
    [0,-1,13],[0,-1,14],[0,-1,15],
    [0,-0.5,15],[0,0,15],[0,0.5,15],[0,1,15],[0,1.5,15],[0,2,15],[0,2.5,15]
]
spin_right_targets_FR_BL = [
    [0,3,15],[0,3,14],[0,3,13],[0,3,12],
    [0,2,12],[0,1,12],[0,0,12],[0,-1,12],
    [0,-1,13],[0,-1,14],[0,-1,15],
    [0,-0.5,15],[0,0,15],[0,0.5,15],[0,1,15],[0,1.5,15],[0,2,15],[0,2.5,15]
]
spin_right_targets_FL_BR = [
    [0,3,15],[0,2.5,15],[0,2,15],[0,1.5,15],[0,1,15],[0,0.5,15],[0,0,15],[0,-0.5,15],[0,-1,15],
    [0,-1,14],[0,-1,13],[0,-1,12],
    [0,0,12],[0,1,12],[0,2,12],[0,3,12],
    [0,3,13],[0,3,14]
]
joint_ids = {
    "BR": {
        "shoulder": 4,
        "femur": 7,
        "knee": 8,
        "foot": 9
    },
    "FR": {
        "shoulder": 13,
        "femur": 16,
        "knee": 17,
        "foot": 18
    },
    "BL": {
        "shoulder": 24,
        "femur": 27,
        "knee": 28,
        "foot": 29
    },
    "FL": {
        "shoulder": 33,
        "femur": 36,
        "knee": 37,
        "foot": 38
    }
}

def real_sim_angle(angle, id):
  if id == 4:                 # BR_shoulder_joint
    sim_angle = angle - 90
  elif id == 7:               # BR_femur_joint
    sim_angle = 45 - angle
  elif id == 8:               # BR_knee_joint
    sim_angle = 90 - angle

  elif id == 13:              # FR_shoulder_joint
    sim_angle = angle - 90
  elif id == 16:              # FR_femur_joint
    sim_angle = 45 - angle
  elif id == 17:              # FR_knee_joint
    sim_angle = 90 - angle

  elif id == 24:              # BL_shoulder_joint
    sim_angle = angle - 90
  elif id == 27:              # BL_femur_joint
    sim_angle = 45 - angle
  elif id == 28:              # BL_knee_joint
    sim_angle = angle - 90

  elif id == 33:              # FL_shoulder_joint
    sim_angle = angle - 90
  elif id == 36:              # FL_femur_joint
    sim_angle = 45 - angle
  elif id == 37:              # FL_knee_joint
    sim_angle = 90 - angle

  else:
    sim_angle = angle
  return sim_angle

def rad2deg(rads):
  return [rads[0]*180/np.pi, rads[1]*180/np.pi, rads[2]*180/np.pi]
def deg2rad(deg):
  return deg*np.pi/180





'''
X (forward(neg), backward(pos)):    |   Y (inside(neg), outside(pos)):      |   Z (taller(neg), shorter(pos)):   |
'''

def legIK(x,y,z): #BR
  D = np.sqrt((z**2 + y**2) - coxa**2)
  G = np.sqrt(D**2 + x**2)
  shoulder_angle = np.arctan2(y,z) + np.arctan2(D,coxa)
  knee_angle = np.arccos((G**2 - femur**2 - tibia**2)/(-2*femur*tibia))
  femur_angle = deg2rad(180) - (deg2rad(90) + np.arctan2(x,D) + np.arcsin((tibia * np.sin(knee_angle)) / G))

  return rad2deg([shoulder_angle, femur_angle, knee_angle])
