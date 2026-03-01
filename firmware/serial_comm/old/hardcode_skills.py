from IK_moving import *
import numpy as np
import serial
import time

# Serial communication settings
SERIAL_BAUD_RATE = 500000
SERIAL_TIMEOUT = 0.1
ser = None

def trajecotries(n): # n os the trakectory u want to do
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

    if n == 0: # HOME
        return home, home
    elif n == 1: # SLEEP
        return sleep, sleep
    elif n == 2: # JUMP
        return jump, jump
    elif n == 3: # FORWARD
        return forward_targets_FR_BL, forward_targets_FL_BR
    elif n == 4: # BACKWARD
        return backward_targets_FR_BL, backward_targets_FL_BR
    elif n == 5: # SPIN LEFT
        return spin_left_targets_FR_BL, spin_left_targets_FL_BR
    elif n == 6: # SPIN RIGHT
        return spin_right_targets_FR_BL, spin_right_targets_FL_BR
    else:
        return home, home

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
                skill = input("(q to quit) Enter skill number (0: HOME, 1: SLEEP, 2: JUMP, 3: FORWARD, 4: BACKWARD, 5: SPIN LEFT, 6: SPIN RIGHT): ")
                if skill == 'q':
                    break
                skill = int(skill)
                if skill < 0 or skill > 6:
                    print("Invalid skill number! Please enter a number between 0 and 6.")
                    continue
                trajectory_FR_BL, trajectory_FL_BR = trajecotries(skill)
                print(f"Executing skill {skill}...")
                #### for i in range(len(trajectory_FR_BL)):
                ####     # Combine trajectories into a valid input format
                ####     target_positions = [
                ####         trajectory_FR_BL[i], trajectory_FL_BR[i],
                ####         trajectory_FL_BR[i], trajectory_FR_BL[i]
                ####     ]
                ####     move_legs_to_pos(ser, target_positions)
                ####     time.sleep(0.1)
                #### # Move back to home position after execution
                #### #move_legs_to_pos(ser, [[0, 2, 16]] * 4)


                # Continuous movement for selected actions
                if skill in [3, 4, 5, 6]:  # FORWARD, BACKWARD, SPIN LEFT, SPIN RIGHT
                    print("Continuous movement started. Enter 's' to stop.")
                    while True:
                        stop_command = input("Press 's' to stop: ")
                        if stop_command.lower() == 's':
                            print("Stopping movement...")
                            break
                        else:
                            for i in range(len(trajectory_FR_BL)):
                                target_positions = [
                                    trajectory_FR_BL[i], trajectory_FL_BR[i],
                                    trajectory_FL_BR[i], trajectory_FR_BL[i]
                                ]
                                move_legs_to_pos(ser, target_positions)
                                time.sleep(0.02)
                    # Move back to home position after execution
                    move_legs_to_pos(ser, [[0, 2, 16]] * 4)

                else:
                    # Execute once for non-continuous skills
                    for i in range(len(trajectory_FR_BL)):
                        target_positions = [
                            trajectory_FR_BL[i], trajectory_FL_BR[i],
                            trajectory_FL_BR[i], trajectory_FR_BL[i]
                        ]
                        move_legs_to_pos(ser, target_positions)
                        time.sleep(0.02)

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
