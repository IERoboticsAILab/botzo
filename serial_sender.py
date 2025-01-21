import numpy as np
import serial
import time
from IK_Funcs import *

def main():
    
    targets_FR_BL = [
    [0,3,15],[0.5,3,15],[1,3,15],[1.5,3,15],[2,3,15],[2.5,3,15],[3,3,15],[3.5,3,15],[4,3,15],
    [4,3,14],[4,3,13],[4,3,12],
    [3,3,12],[2,3,12],[1,3,12],[0,3,12],
    [0,3,13],[0,3,14]
    ]

    targets_FL_BR = [
        [4,3,15],[4,3,14],[4,3,13],[4,3,12],
        [3,3,12],[2,3,12],[1,3,12],[0,3,12],
        [0,3,13],[0,3,14],[0,3,15],
        [0.5,3,15],[1,3,15],[1.5,3,15],[2,3,15],[2.5,3,15],[3,3,15],[3.5,3,15]
    ]
    
    FR_leg_angles = [FR_legIK(x,y,z) for x,y,z in targets_FR_BL]  # returns [coxa_deg, femur_deg, tibia_deg]
    FR_steps = deg2PWM_set_angles(FR_leg_angles, coefficents_SFR, coefficents_FFR, coefficents_TFR)
    
    FL_leg_angles = [FL_legIK(x,y,z) for x,y,z in targets_FL_BR]  # returns [coxa_deg, femur_deg, tibia_deg]
    FL_steps = deg2PWM_set_angles(FL_leg_angles, coefficents_SFL, coefficents_FFL, coefficents_TFL)
    
    BL_leg_angles = [BL_legIK(x,y,z) for x,y,z in targets_FR_BL]  # returns [coxa_deg, femur_deg, tibia_deg]
    BL_steps = deg2PWM_set_angles(BL_leg_angles, coefficents_SBL, coefficents_FBL, coefficents_TBL)
    
    BR_leg_angles = [BR_legIK(x,y,z) for x,y,z in targets_FL_BR]  # returns [coxa_deg, femur_deg, tibia_deg]
    BR_steps = deg2PWM_set_angles(BR_leg_angles, coefficents_SBR, coefficents_FBR, coefficents_TBR)
    
    # Now, for each step, send the pulses to Arduino
    # Open Serial Port to Arduino (adjust port name or baud as needed)
    ser = serial.Serial('/dev/ttyACM0', 500000, timeout=1)
    time.sleep(1)  # Wait a moment for the Arduino to reset

    print("Starting Quadruped Movement (from Pi)...")

    # Example: Send all steps in a loop
    while True:
        for i in range(len(FR_steps)):
            # Each step has [S, F, T] for each leg
            # Combine them into one message of 12 values:
            # Format: SFR, FFR, TFR, SFL, FFL, TFL, SBR, FBR, TBR, SBL, FBL, TBL
            data_str = "{},{},{},{},{},{},{},{},{},{},{},{}\n".format(
                int(FR_steps[i][0]), int(FR_steps[i][1]), int(FR_steps[i][2]),
                int(FL_steps[i][0]), int(FL_steps[i][1]), int(FL_steps[i][2]),
                int(BR_steps[i][0]), int(BR_steps[i][1]), int(BR_steps[i][2]),
                int(BL_steps[i][0]), int(BL_steps[i][1]), int(BL_steps[i][2])
            )

            # Send to Arduino
            ser.write(data_str.encode('utf-8'))
            ser.flush()
            # print(f"Sent step {i}: {data_str.strip()}")

            # Wait a bit before sending the next command
            time.sleep(0.1)

if __name__ == "__main__":
    main()
