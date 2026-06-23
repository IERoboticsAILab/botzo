#!/usr/bin/env python3

'''
Subscribe to /real_robot_joint_states and pass joint to Arduino.


This node pass the joint state for the real tobot to the Arduino via serial communication. 

It subscribes to /real_robot_joint_states topic message type: botzo_messages/msg/RealRobotJointStates.msg
Recive the target angles in degrees for each joint of the robot: 
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

Transform the angles from degrees to PWM values and pass them to the Arduino via serial communication. The Arduino will then move the servos to the target angles.
'''

import rclpy
from rclpy.node import Node

from botzo_messages.msg import RealRobotJointStates

import numpy as np
import serial
import time
import struct


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


class MoveRealRobot(Node):
    def __init__(self):
        super().__init__('move_real_robot')
        self.subscription = self.create_subscription(
            RealRobotJointStates,
            '/real_robot_joint_states',
            self.listener_callback,
            10)

        # Initialize serial communication
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
        except serial.SerialException as e:
            self.get_logger().error(f'Error initializing serial communication: {e}')
            rclpy.shutdown()
        except Exception as e:
            print(f"Unexpected error: {e}")

    def listener_callback(self, msg):
        # #self.get_logger().info(f'Received joint states')
        # # Extract joint angles from the message
        # angles = [
        #     [msg.sfr, msg.ffr, msg.tfr],
        #     [msg.sfl, msg.ffl, msg.tfl],
        #     [msg.sbr, msg.fbr, msg.tbr],
        #     [msg.sbl, msg.fbl, msg.tbl]
        # ]

        # # Convert angles from degrees to PWM values
        # angles_PWM = [
        #     deg2PWM_set_angles([angles[0]], coefficents_SFR, coefficents_FFR, coefficents_TFR)[0],
        #     deg2PWM_set_angles([angles[1]], coefficents_SFL, coefficents_FFL, coefficents_TFL)[0],
        #     deg2PWM_set_angles([angles[2]], coefficents_SBR, coefficents_FBR, coefficents_TBR)[0],
        #     deg2PWM_set_angles([angles[3]], coefficents_SBL, coefficents_FBL, coefficents_TBL)[0]
        # ]

        # Send PWM values to Arduino via serial communication
        if ser and ser.is_open:
            pwm_values = [
                int(deg2PWM(msg.sfr, coefficents_SFR)),
                int(deg2PWM(msg.ffr, coefficents_FFR)),
                int(deg2PWM(msg.tfr, coefficents_TFR)),

                int(deg2PWM(msg.sfl, coefficents_SFL)),
                int(deg2PWM(msg.ffl, coefficents_FFL)),
                int(deg2PWM(msg.tfl, coefficents_TFL)),

                int(deg2PWM(msg.sbr, coefficents_SBR)),
                int(deg2PWM(msg.fbr, coefficents_FBR)),
                int(deg2PWM(msg.tbr, coefficents_TBR)),
                
                int(deg2PWM(msg.sbl, coefficents_SBL)),
                int(deg2PWM(msg.fbl, coefficents_FBL)),
                int(deg2PWM(msg.tbl, coefficents_TBL)),
            ]
            t0 = time.perf_counter()
            ser.write(struct.pack('<12H', *pwm_values)) # 12 (uint16_t) x 2 = 24 bytes       # 500000 / 10 ≈ 50000 bytes/sec
            print(time.perf_counter() - t0)
            self.get_logger().info(f'Sent PWM values: {pwm_values}')

def main(args=None):
    rclpy.init(args=args)
    move_real_robot = MoveRealRobot()
    rclpy.spin(move_real_robot)
    move_real_robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()