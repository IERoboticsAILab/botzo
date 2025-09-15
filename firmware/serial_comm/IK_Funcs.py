import numpy as np
import serial
import time

a_SFR = 0
b_SFR = 7.378
c_SFR = 616.0
coefficents_SFR = np.array([a_SFR, b_SFR, c_SFR])

a_FFR = 0
b_FFR = 7.682
c_FFR = 578.142
coefficents_FFR = np.array([a_FFR, b_FFR, c_FFR])

a_TFR = 0
b_TFR = 7.314
c_TFR = 619.028
coefficents_TFR = np.array([a_TFR, b_TFR, c_TFR])
#------------------------

a_SFL = 0
b_SFL = 7.649
c_SFL = 638.486
coefficents_SFL = np.array([a_SFL, b_SFL, c_SFL])

a_FFL = 0
b_FFL = 7.603
c_FFL = 625.428
coefficents_FFL = np.array([a_FFL, b_FFL, c_FFL])

a_TFL = 0.001
b_TFL = 7.234
c_TFL = 550.0
coefficents_TFL = np.array([a_TFL, b_TFL, c_TFL])

#------------------------

a_SBR = 0
b_SBR = 7.514
c_SBR = 626.428
coefficents_SBR = np.array([a_SBR, b_SBR, c_SBR])

a_FBR = 0.001
b_FBR = 7.364
c_FBR = 548.742
coefficents_FBR = np.array([a_FBR, b_FBR, c_FBR])

a_TBR = 0.001
b_TBR = 7.137
c_TBR = 559.2
coefficents_TBR = np.array([a_TBR, b_TBR, c_TBR])

#------------------------

a_SBL = 0.001
b_SBL = 7.673
c_SBL = 648.857
coefficents_SBL = np.array([a_SBL, b_SBL, c_SBL])

a_FBL = 0
b_FBL = 7.704
c_FBL = 628.142
coefficents_FBL = np.array([a_FBL, b_FBL, c_FBL])

a_TBL = -0.001
b_TBL = 7.765
c_TBL = 634.285
coefficents_TBL = np.array([a_TBL, b_TBL, c_TBL])

# in cm
coxa = 3.1 # from shoulder servo to the 2 other servos in the shoulder
femur = 9.5 # from top sevo to knee
tibia = 9.8 # from knee to foot
real_femur = 9.1 # lenght of 3D printed femur
dist_focuspoint_servo_femurtibia = 2.8 # distance from focus point/pivot of the 2 servos in the shoulder

def deg2PWM(desire_deg_angle, coefficents):
    a, b, c = coefficents
    pulse = round((a * desire_deg_angle**2) + (b * desire_deg_angle) + c, 0)
    return pulse

def deg2PWM_set_angles(angles, coefficents_S, coefficents_F, coefficents_T):
  angles_PWM = []
  for angle in angles:
    angles_PWM
    angles_PWM.append([deg2PWM(angle[0], coefficents_S), deg2PWM(angle[1], coefficents_F), deg2PWM(angle[2], coefficents_T)])
  return angles_PWM

# function that take 3 angls in rad and transform them into deg
def rad2deg(rads):
  return [rads[0]*180/np.pi, rads[1]*180/np.pi, rads[2]*180/np.pi]
def deg2rad(deg):
  return deg*np.pi/180

def FR_legIK(x,y,z):
  D = np.sqrt((z**2 + y**2) - coxa**2)
  G = np.sqrt(D**2 + x**2)
  knee_angle = np.arccos((G**2 - femur**2 - tibia**2)/(-2*femur*tibia))
  shoulder_angle = np.arctan2(x,D) + np.arcsin((tibia * np.sin(knee_angle)) / G)
  adjustment = np.arccos((real_femur**2 + femur**2 - dist_focuspoint_servo_femurtibia**2) / (2 * real_femur * femur))
  #print(f"knee_angle: {knee_angle*180/np.pi}")
  #print(f"shoulder_angle: {shoulder_angle*180/np.pi}")
  #print(f"coxa: {(np.arctan2(y,z) + np.arctan2(D,coxa))*180/np.pi}")
  #print(f"adjustment: {adjustment*180/np.pi}")


  coxa_angle = deg2rad(180) - (np.arctan2(y,z) + np.arctan2(D,coxa))
  femur_angle = deg2rad(90) - (shoulder_angle + adjustment)
  tibia_angle = np.pi - knee_angle + femur_angle + adjustment

  return rad2deg([coxa_angle, femur_angle, tibia_angle])

def BR_legIK(x,y,z):
  D = np.sqrt((z**2 + y**2) - coxa**2)
  G = np.sqrt(D**2 + x**2)
  knee_angle = np.arccos((G**2 - femur**2 - tibia**2)/(-2*femur*tibia))
  shoulder_angle = np.arctan2(x,D) + np.arcsin((tibia * np.sin(knee_angle)) / G)
  adjustment = np.arccos((real_femur**2 + femur**2 - dist_focuspoint_servo_femurtibia**2) / (2 * real_femur * femur))

  coxa_angle = deg2rad(180) - (np.arctan2(y,z) + np.arctan2(D,coxa))
  femur_angle = deg2rad(90) - (shoulder_angle + adjustment)
  tibia_angle = np.pi - knee_angle + femur_angle + adjustment

  return rad2deg([coxa_angle, femur_angle, tibia_angle])

def FL_legIK(x,y,z):
  D = np.sqrt((z**2 + y**2) - coxa**2)
  G = np.sqrt(D**2 + x**2)
  knee_angle = np.arccos((G**2 - femur**2 - tibia**2)/(-2*femur*tibia))
  shoulder_angle = np.arctan2(x,D) + np.arcsin((tibia * np.sin(knee_angle)) / G)
  adjustment = np.arccos((real_femur**2 + femur**2 - dist_focuspoint_servo_femurtibia**2) / (2 * real_femur * femur))
  #print(f"knee_angle: {knee_angle*180/np.pi}")
  #print(f"shoulder_angle: {shoulder_angle*180/np.pi}")
  #print(f"coxa: {(np.arctan2(y,z) + np.arctan2(D,coxa))*180/np.pi}")
  #print(f"adjustment: {adjustment*180/np.pi}")

  coxa_angle = (deg2rad(180) - (np.arctan2(y,z) + np.arctan2(D,coxa))) - (2 * ((deg2rad(180) - (np.arctan2(y,z) + np.arctan2(D,coxa)))-deg2rad(90)))
  #coxa_angle = (np.arctan2(y,z) + np.arctan2(D,coxa))
  femur_angle = deg2rad(90) - (shoulder_angle + adjustment)
  tibia_angle = np.pi - knee_angle + femur_angle + adjustment

  femur_angle = deg2rad(180) - femur_angle
  tibia_angle = deg2rad(180) - tibia_angle

  return rad2deg([coxa_angle, femur_angle, tibia_angle])

def BL_legIK(x,y,z):
  D = np.sqrt((z**2 + y**2) - coxa**2)
  G = np.sqrt(D**2 + x**2)
  knee_angle = np.arccos((G**2 - femur**2 - tibia**2)/(-2*femur*tibia))
  shoulder_angle = np.arctan2(x,D) + np.arcsin((tibia * np.sin(knee_angle)) / G)
  adjustment = np.arccos((real_femur**2 + femur**2 - dist_focuspoint_servo_femurtibia**2) / (2 * real_femur * femur))
  #print(f"knee_angle: {knee_angle*180/np.pi}")
  #print(f"shoulder_angle: {shoulder_angle*180/np.pi}")
  #print(f"coxa: {(np.arctan2(y,z) + np.arctan2(D,coxa))*180/np.pi}")
  #print(f"adjustment: {adjustment*180/np.pi}")

  #coxa_angle = (np.arctan2(y,z) + np.arctan2(D,coxa))
  coxa_angle = ((deg2rad(180) - (np.arctan2(y,z) + np.arctan2(D,coxa))) - (2 * ((deg2rad(180) - (np.arctan2(y,z) + np.arctan2(D,coxa)))-deg2rad(90))))
  femur_angle = deg2rad(90) - (shoulder_angle + adjustment)
  tibia_angle = deg2rad(180) - (np.pi - knee_angle + femur_angle + adjustment)
  femur_angle = deg2rad(180) - femur_angle

  return rad2deg([coxa_angle, femur_angle, tibia_angle])
