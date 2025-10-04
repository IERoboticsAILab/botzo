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
import numpy as np
import serial
import time
from IK_Funcs import *

# Serial communication settings
SERIAL_BAUD_RATE = 500000
SERIAL_TIMEOUT = 0.1
ser = None

def move_legs_to_pos(serial_conn, coordinates):
    """
    Compute the IK for each leg, convert to PWM, and send the data to Arduino.
    Args:
        serial_conn (serial.Serial): Serial connection to Arduino.
        coordinates (list of lists): Target positions for FR, FL, BR, BL legs.
    """
    global ser
    ser = serial_conn
    FR_target, FL_target, BR_target, BL_target = coordinates
    print(*FR_target, *FL_target, *BR_target, *BL_target )

    # Compute inverse kinematics
    FR_leg_angles = [FR_legIK(FR_target[0],FR_target[1],FR_target[2])]
    FR_leg_pulses = deg2PWM_set_angles(FR_leg_angles, coefficents_SFR, coefficents_FFR, coefficents_TFR)

    FL_leg_angles = [FL_legIK(FL_target[0],FL_target[1],FL_target[2])]
    FL_leg_pulses = deg2PWM_set_angles(FL_leg_angles, coefficents_SFL, coefficents_FFL, coefficents_TFL)

    BR_leg_angles = [BR_legIK(BR_target[0],BR_target[1],BR_target[2])]
    BR_leg_pulses = deg2PWM_set_angles(BR_leg_angles, coefficents_SBR, coefficents_FBR, coefficents_TBR)

    BL_leg_angles = [BL_legIK(BL_target[0],BL_target[1],BL_target[2])]
    BL_leg_pulses = deg2PWM_set_angles(BL_leg_angles, coefficents_SBL, coefficents_FBL, coefficents_TBL)

    # Print movement details
    print(f"\nMoving to target positions: {coordinates}")
    print(f"FR -> Angles: {FR_leg_angles} -> PWM: {FR_leg_pulses}")
    print(f"FL -> Angles: {FL_leg_angles} -> PWM: {FL_leg_pulses}")
    print(f"BR -> Angles: {BR_leg_angles} -> PWM: {BR_leg_pulses}")
    print(f"BL -> Angles: {BL_leg_angles} -> PWM: {BL_leg_pulses}")

    # Format the serial message as expected by the Arduino
    fr = FR_leg_pulses[0]
    fl = FL_leg_pulses[0]
    br = BR_leg_pulses[0]
    bl = BL_leg_pulses[0]
    msg = f"{fr[0]},{fr[1]},{fr[2]},{fl[0]},{fl[1]},{fl[2]},{br[0]},{br[1]},{br[2]},{bl[0]},{bl[1]},{bl[2]}\n"

    # Send to Arduino
    ser.reset_input_buffer()
    ser.write(msg.encode('utf-8'))
    time.sleep(0.05)  # Small delay to ensure message is sent properly
    ser.flush()  # Ensure all data is sent

    # Read Arduino response to check if the message was received correctly
    response = ser.readline().decode().strip()
    #response = ser.read_until(b'\n').decode().strip()
    if response:
        print(f"Arduino Response: {response}")
    else:
        print("No response from Arduino!")

    print("Command sent!\n")

def main():
    """
    Main function to handle user input and send movement commands.
    """
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


        while True:
            try:
                # Get user input
                print("\n\nexample: 1,3,15, 1,3,15, 1,3,15, 1,3,15")
                print("example: FR_x,FR_y,FR_z, FL_x,FL_y,FL_z, BR_x,BR_y,BR_z, BL_x,BL_y,BL_z")
                raw_input_str = input("\nEnter target positions (FR_x,FR_y,FR_z, FL_x,FL_y,FL_z, BR_x,BR_y,BR_z, BL_x,BL_y,BL_z) or 'q' to quit:\n")
                if raw_input_str.lower() == 'q':
                    break  # Exit the loop

                # Parse input into a list of floats
                values = list(map(float, raw_input_str.split(',')))
                if len(values) != 12:
                    print("Invalid input! Please enter exactly 12 numbers.")
                    continue

                # Reshape input into coordinate lists for each leg
                target_positions = [values[i:i+3] for i in range(0, 12, 3)]
                move_legs_to_pos(ser, target_positions)

            except ValueError:
                print("Invalid input format! Please enter numbers separated by commas.")

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
