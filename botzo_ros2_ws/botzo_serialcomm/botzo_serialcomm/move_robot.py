#!/usr/bin/env python3

'''
X (forward(neg), backward(pos)):    |   Y (inside(neg), outside(pos)):      |   Z (taller(neg), shorter(pos)):   |
-5,3,12,-5,3,12,-5,3,12,-5,3,12     |   0,-5,12,0,-5,12,0,-5,12,0,-5,12     |   0,2,8,0,2,8,0,2,8,0,2,8          |
-4,3,12,-4,3,12,-4,3,12,-4,3,12     |   0,-4,12,0,-4,12,0,-4,12,0,-4,12     |   0,2,9,0,2,9,0,2,9,0,2,9          |
-3,3,12,-3,3,12,-3,3,12,-3,3,12     |   0,-3,12,0,-3,12,0,-3,12,0,-3,12     |   0,2,10,0,2,10,0,2,10,0,2,10      |
-2,3,12,-2,3,12,-2,3,12,-2,3,12     |   0,-2,12,0,-2,12,0,-2,12,0,-2,12     |   0,2,11,0,2,11,0,2,11,0,2,11      |
-1,3,12,-1,3,12,-1,3,12,-1,3,12     |   0,-1,12,0,-1,12,0,-1,12,0,-1,12     |   0,2,12,0,2,12,0,2,12,0,2,12      |
0,3,12,0,3,12,0,3,12,0,3,12         |   0,0,12,0,0,12,0,0,12,0,0,12         |   0,2,13,0,2,13,0,2,13,0,2,13      |
1,3,12,1,3,12,1,3,12,1,3,12         |   0,1,12,0,1,12,0,1,12,0,1,12         |   0,2,14,0,2,14,0,2,14,0,2,14      |
2,3,12,2,3,12,2,3,12,2,3,12         |   0,2,12,0,2,12,0,2,12,0,2,12         |   0,2,15,0,2,15,0,2,15,0,2,15      |
3,3,12,3,3,12,3,3,12,3,3,12         |   0,3,12,0,3,12,0,3,12,0,3,12         |   0,2,16,0,2,16,0,2,16,0,2,16      |
4,3,12,4,3,12,4,3,12,4,3,12         |   0,4,12,0,4,12,0,4,12,0,4,12         |   0,2,17,0,2,17,0,2,17,0,2,17      |
5,3,12,5,3,12,5,3,12,5,3,12         |   0,5,12,0,5,12,0,5,12,0,5,12         |   0,2,18,0,2,18,0,2,18,0,2,18      |
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import numpy as np
import serial
import time


# Serial communication settings
SERIAL_BAUD_RATE = 500000
SERIAL_TIMEOUT = 0.1
ser = None

#-------------------------

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
  angles_bl[0] += 90
  angles_bl[0] = 180 - angles_bl[0]
  angles_br[0] += 90
  angles_br[0] = 180 - angles_br[0]

  angles_fl[1] *= -1 # ok
  angles_fl[1] = 180 - angles_fl[1]
  angles_bl[1] *= -1
  angles_bl[1] = 180 - angles_bl[1]
  angles_fr[1] *= -1
  angles_br[1] *= -1

  angles_fl[2] += 90 # tetha = angles_fl[2] + 90     |    angles_fl[2] = -90 + theta + gamma     |    gamma = 180 - 90 - angles_fl[1]
  angles_fl[2] = 180 - angles_fl[2]
  angles_bl[2] += 90
  angles_bl[2] = 180 - angles_bl[2]
  angles_fr[2] += 90
  angles_br[2] += 90
  return angles_fl, angles_fr, angles_bl, angles_br




# 1. Subscribe to /joint_states
# 2. Transform current joint states angle from sim angles to real robot angles
# 3. Transfom radinats into PWM (using calibration coefficients)
# 4. Connect to Arduino
# 5. Send angles to servos 
class SimToReal(Node):
    def __init__(self):
        super().__init__('sim_to_real')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        self.subscription  # prevent unused variable warning

    def joint_state_callback(self, msg):
        global ser
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
        angles_fl = rad2deg([msg.position[2], msg.position[6], msg.position[7]])
        angles_fr = rad2deg([msg.position[3], msg.position[4], msg.position[5]])
        angles_bl = rad2deg([msg.position[0], msg.position[8], msg.position[9]])
        angles_br = rad2deg([msg.position[1], msg.position[10], msg.position[11]])
        print(f"\nRECIVED CURRENT JOINT STATE:\nFL: {angles_fl}, FR: {angles_fr}, BL: {angles_bl}, BR: {angles_br}")
        angles_fl, angles_fr, angles_bl, angles_br = adjust_angles_sim_to_real(angles_fl, angles_fr, angles_bl, angles_br)

        # for debug purposes I set the motros all to 90 for shoulders, 45 femurs, 45 knees
        #angles_fl = [80, 45, 45]
        #angles_fr = [80, 45, 45]
        #angles_bl = [80, 45, 45]
        #angles_br = [80, 45, 45]
        
        angles_fl_PWM = deg2PWM_set_angles([angles_fl], coefficents_SFL, coefficents_FFL, coefficents_TFL)[0]
        angles_fr_PWM = deg2PWM_set_angles([angles_fr], coefficents_SFR, coefficents_FFR, coefficents_TFR)[0]
        angles_bl_PWM = deg2PWM_set_angles([angles_bl], coefficents_SBL, coefficents_FBL, coefficents_TBL)[0]
        angles_br_PWM = deg2PWM_set_angles([angles_br], coefficents_SBR, coefficents_FBR, coefficents_TBR)[0]
        print(f"\n\nSIM TO REAL ANGLES:\nFL: {angles_fl}, FR: {angles_fr}, BL: {angles_bl}, BR: {angles_br}")
        print(f"\n\nANGLES PWM:\nFL: {angles_fl_PWM}, FR: {angles_fr_PWM}, BL: {angles_bl_PWM}, BR: {angles_br_PWM}")
        if ser and ser.is_open:
            msg = f"{angles_fr_PWM[0]},{angles_fr_PWM[1]},{angles_fr_PWM[2]},{angles_fl_PWM[0]},{angles_fl_PWM[1]},{angles_fl_PWM[2]},{angles_br_PWM[0]},{angles_br_PWM[1]},{angles_br_PWM[2]},{angles_bl_PWM[0]},{angles_bl_PWM[1]},{angles_bl_PWM[2]}\n"
            ser.reset_input_buffer()
            ser.write(msg.encode('utf-8'))
            ser.flush()
        else:
            print("Serial connection to Arduino is not open. Cannot send command.")






def main(args=None):
    global ser
    try:
        # Connect to Arduino
        ser = serial.Serial('/dev/ttyACM0', SERIAL_BAUD_RATE, timeout=SERIAL_TIMEOUT)
        time.sleep(1)  # Allow time for connection
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        print(f"Connected to Arduino at {SERIAL_BAUD_RATE} baud.\n")
        ser.write(b'TEST\n')
        time.sleep(0.1)
        response = ser.readline().decode().strip()
        print(f"\n\nTest Response from Arduino: {response}\n\n")

        rclpy.init(args=args)
        sim_to_real = SimToReal()
        rclpy.spin(sim_to_real)
        sim_to_real.destroy_node()
        rclpy.shutdown()


    except serial.SerialException as e:
        print(f"Error: {e}\nCheck if the Arduino is connected to the correct port.")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial connection closed.")

if __name__ == "__main__":
    main()
