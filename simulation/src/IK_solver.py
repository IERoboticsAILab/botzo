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
    "BR": { # RH
        "shoulder": 11, # RH_HAA
        "femur": 14, # RH_HFE
        "knee": 15, # RH_KFE
        "foot": 16 # RH_shank_fixed_RH_FOOT
    },
    "FR": { # RF
        "shoulder": 3, # RF_HAA
        "femur": 6, # RF_HFE
        "knee": 7, # RF_KFE
        "foot": 8 # RF_shank_fixed_RF_FOOT
    },
    "BL": { # LH
        "shoulder": 19, # LH_HAA
        "femur": 22, # LH_HFE
        "knee": 23, # LH_KFE
        "foot": 24 # LH_shank_fixed_LH_FOOT
    },
    "FL": { # LF
        "shoulder": 27, # LF_HAA
        "femur": 30, # LF_HFE
        "knee": 31, #LF_KFE
        "foot": 32 # LF_shank_fixed_LF_FOOT
    }
}

def real_sim_angle(angle, id):
  if id == 11:                # RH_HAA (shoulder) 
    sim_angle = angle - 90
  elif id == 14:               # RH_HFE (femur)
    sim_angle = 90 - angle
  elif id == 15:               # RH_KFE (knee)
    sim_angle = angle - 90

  elif id == 3:              # RF_HAA (shoulder)
    sim_angle = angle - 90
  elif id == 6:              # RF_HFE (femur)
    sim_angle = 90 - angle
  elif id == 7:              # RF_KFE (knee)
    sim_angle = angle - 90

  elif id == 19:              # LH_HAA (shoulder)
    sim_angle = angle - 90
  elif id == 22:              # LH_HFE (femur)
    sim_angle = 90 - angle
  elif id == 23:              # LH_KFE (knee)
    sim_angle = angle - 90

  elif id == 27:              # LF_HAA (shoulder)
    sim_angle = angle - 90
  elif id == 30:              # LF_HFE (femur)
    sim_angle = 90 - angle
  elif id == 31:              # LF_KFE (knee)
    sim_angle = angle - 90

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
