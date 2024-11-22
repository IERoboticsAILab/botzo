from math import pi
import numpy as np
import serial
import time

# Establish serial connection to the Arduino
ser = serial.Serial('/dev/ttyACM0', 9600)  # Adjust the port name if necessary
time.sleep(2)  # Wait for the connection to initialize

try:
    while True:
        # function that take 3 angls in rad and transform them into deg
        def rad2deg(rads):
            return [rads[0]*180/pi, rads[1]*180/pi, rads[2]*180/pi]
        def deg2rad(deg):
            return deg*pi/180
    
        def legIK(x,y,z):
            D = np.sqrt((z**2 + y**2) - coxa**2)
            G = np.sqrt(D**2 + x**2)
            tibia_angle = np.arccos((G**2 - femur**2 - tibia**2)/(-2*femur*tibia))
            femur_angle = np.arctan2(x,D) + np.arcsin((tibia * np.sin(tibia_angle)) / G)
            coxa_angle = np.arctan2(y,z) + np.arctan2(D,coxa)

            return rad2deg([coxa_angle, femur_angle, tibia_angle])
        
        
        def legIK_for_botzo(x,y,z):
            D = np.sqrt((z**2 + y**2) - coxa**2)
            G = np.sqrt(D**2 + x**2)
            knee_angle = np.arccos((G**2 - femur**2 - tibia**2)/(-2*femur*tibia))
            shoulder_angle = np.arctan2(x,D) + np.arcsin((tibia * np.sin(knee_angle)) / G)
            adjustment = np.arccos((real_femur**2 + femur**2 - dist_focuspoint_servo_femurtibia**2) / (2 * real_femur * femur))


            coxa_angle = np.arctan2(y,z) + np.arctan2(D,coxa)
            femur_angle = deg2rad(90) - (shoulder_angle + adjustment)
            tibia_angle = pi - knee_angle + adjustment + femur_angle

            return rad2deg([coxa_angle, femur_angle, tibia_angle])
        
        
        # in cm
        coxa = 1.8 # from shoulder servo to the 2 other servos in the shoulder
        femur = 9.5 # from top sevo to kneww
        tibia = 9.8 # from knee to foot
        real_femur = 9.1 # lenght of 3D printed femur
        dist_focuspoint_servo_femurtibia = 2.8 # distance from focus point/pivot of the 2 servos in the shoulder
        # target
        x,y,z=-3,0,15
        
        results = legIK_for_botzo(x,y,z)
        #data = f"{results}\n"
        data = results + [coxa, femur, tibia]
        data_str = ', '.join(map(str, data)) + '\n'
        print(f"Sending: {data.strip()}")
        ser.write(data_str.encode('utf-8'))
        time.sleep(1)
        print(f"Received: {ser.readline().decode('utf-8').strip()}")
finally:
    ser.close()
