import threading
import msvcrt
import time
import pybullet as p
import pybullet_data
import math
import numpy as np



# ------------------- ROBOT SPECIFICS -------------------
coxa = 3.14 # from shoulder servo to the 2 other servos in the shoulder
femur = 9.05 # from top sevo to knee
tibia = 9.85 # from knee to foot
L = 7.0536 + 8.9735 # body length
W = 10.6  # body width





# ------------------- CONTROL FUNCTIONS -------------------
# X (forward(neg), backward(pos)):    |   Y (inside(neg), outside(pos)):      |   Z (taller(neg), shorter(pos)):   |

def legIK(x,y,z):
  D = np.sqrt((z**2 + y**2) - coxa**2)
  G = np.sqrt(D**2 + x**2)
  
  shoulder_angle = np.arctan2(y,z) + np.arctan2(D,coxa)
  knee_angle = np.arccos((G**2 - femur**2 - tibia**2)/(-2*femur*tibia))
  femur_angle = math.radians(180) - (math.radians(90) + np.arctan2(x,D) + np.arcsin((tibia * np.sin(knee_angle)) / G))

  return [math.degrees(shoulder_angle), math.degrees(femur_angle), math.degrees(knee_angle)]

def Rx(roll):
    """ Rotation matrix arround x (roll)
    """
    return np.matrix([[1,            0,             0, 0],
                      [0, np.cos(roll), -np.sin(roll), 0],
                      [0, np.sin(roll),  np.cos(roll), 0],
                      [0,            0,             0, 1]])

def Ry(pitch):
    """ Rotation matrix arround y (pitch)
    """
    return np.matrix([[ np.cos(pitch), 0, np.sin(pitch), 0],
                      [             0, 1,             0, 0],
                      [-np.sin(pitch), 0, np.cos(pitch), 0],
                      [             0, 0,             0, 1]])

def Rz(yaw):
    """ Rotation matrix arround z (yaw)
    """
    return np.matrix([[np.cos(yaw), -np.sin(yaw), 0, 0],
                      [np.sin(yaw),  np.cos(yaw), 0, 0],
                      [          0,            0, 1, 0],
                      [          0,            0, 0, 1]])

def RotMatrix(rotation=[0,0,0], is_radiants=True, order='xyz'):
    roll, pitch, yaw = rotation
    # Convert to radiants if input is in degrees
    if not is_radiants:
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
    rotX = Rx(roll)
    rotY = Ry(pitch)
    rotZ = Rz(yaw)
    if order == 'xyz': rotation_matrix = rotX * rotY * rotZ
    elif order == 'zyx': rotation_matrix = rotZ * rotY * rotX
    return rotation_matrix # roll, pitch, yaw rotation matrix

# ------------------- SIMULATIONS FUNCTIONS -------------------

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