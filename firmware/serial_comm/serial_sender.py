import numpy as np
import serial
import time
import socket
import json
import threading
from IK_Funcs import *

# Global variables to track movement state
current_position = [0, 3, 15]  # Start at home position
movement_in_progress = False
current_sequence = None
sequence_index = 0
ser = None  # Will be initialized in command_receiver

# Serial communication optimization settings
SERIAL_BAUD_RATE = 115200  # Higher baud rate may help if your hardware supports it
STEP_DELAY = 0.01  # Reduced delay between steps (was 0.1)
SERIAL_TIMEOUT = 0.1  # Reduced serial timeout

def is_home_position(position):
    """Check if a position is the home position"""
    return position[0] == 0 and position[1] == 3 and position[2] == 15

def process_movement(command, serial_conn):
    """
    Process movement commands and execute corresponding target sequences
    
    Args:
        command (str): The movement command ('FORWARD', 'BACKWARD', or 'ROTATE_LEFT')
        serial_conn (serial.Serial): Serial connection to Arduino
    """
    global current_position, movement_in_progress, current_sequence, sequence_index, ser
    
    # Store the serial connection
    ser = serial_conn
    
    print(f"Processing {command} movement command")
    
    # Define target coordinates for different movements
    # FORWARD movement coordinates
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
    
    # BACKWARD movement coordinates
    backward_targets_FR_BL = [
        [0,3,14],[0,3,13],[0,3,12],
        [1,3,12],[2,3,12],[3,3,12],[4,3,12],
        [4,3,13],[4,3,14],[4,3,15],
        [3.5,3,15],[3,3,15],[2.5,3,15],[2,3,15],[1.5,3,15],[1,3,15],[0.5,3,15],[0,3,15]
    ]
    
    backward_targets_FL_BR = [
        [3.5,3,15],[3,3,15],[2.5,3,15],[2,3,15],[1.5,3,15],[1,3,15],[0.5,3,15],[0,3,15],
        [0,3,14],[0,3,13],[0,3,12],
        [1,3,12],[2,3,12],[3,3,12],[4,3,12],
        [4,3,13],[4,3,14],[4,3,15]
    ]
    
    # ROTATE LEFT movement coordinates
    rotate_left_targets_FR_BL = [
        [0,5,15],[0,5,14],[0,5,13],[0,5,12],
        [0,4,12],[0,3,12],[0,2,12],[0,1,12],
        [0,1,13],[0,1,14],[0,1,15],
        [0,1.5,15],[0,2,15],[0,2.5,15],[0,3,15],[0,3.5,15],[0,4,15],[0,4.5,15]
    ]
    
    rotate_left_targets_FL_BR = [
        [0,1.5,15],[0,2,15],[0,2.5,15],[0,3,15],[0,3.5,15],[0,4,15],[0,4.5,15],[0,5,15],
        [0,5,14],[0,5,13],[0,5,12],
        [0,4,12],[0,3,12],[0,2,12],[0,1,12],
        [0,1,13],[0,1,14],[0,1,15]
    ]
    
    home_and_stop_targets_FR_BL = [0,3,15]
    home_and_stop_targets_FL_BR = [0,3,15]
    
    # Select the appropriate targets based on the command
    if command == "FORWARD":
        targets_FR_BL = forward_targets_FR_BL
        targets_FL_BR = forward_targets_FL_BR
        print("Executing FORWARD movement sequence")
    elif command == "BACKWARD":
        targets_FR_BL = backward_targets_FR_BL
        targets_FL_BR = backward_targets_FL_BR
        print("Executing BACKWARD movement sequence")
    elif command == "HOME_AND_STOP":
        targets_FR_BL = home_and_stop_targets_FR_BL
        targets_FL_BR = home_and_stop_targets_FL_BR
        print("Executing ROTATE LEFT movement sequence")
    else:
        print(f"Unknown command: {command}")
        return
    
    # Precompute all IK calculations to reduce calculation time during movement
    precomputed_pulses = []
    
    print("Precomputing IK calculations...")
    for i in range(len(targets_FR_BL)):
        current_FR_BL = targets_FR_BL[i]
        current_FL_BR = targets_FL_BR[i]
        
        # Calculate IK for each leg
        FR_leg_angles = [FR_legIK(x,y,z) for x,y,z in [current_FR_BL]]
        FR_leg_pulses = deg2PWM_set_angles(FR_leg_angles, coefficents_SFR, coefficents_FFR, coefficents_TFR)
        
        FL_leg_angles = [FL_legIK(x,y,z) for x,y,z in [current_FL_BR]]
        FL_leg_pulses = deg2PWM_set_angles(FL_leg_angles, coefficents_SFL, coefficents_FFL, coefficents_TFL)
        
        BL_leg_angles = [BL_legIK(x,y,z) for x,y,z in [current_FR_BL]]
        BL_leg_pulses = deg2PWM_set_angles(BL_leg_angles, coefficents_SBL, coefficents_FBL, coefficents_TBL)
        
        BR_leg_angles = [BR_legIK(x,y,z) for x,y,z in [current_FL_BR]]
        BR_leg_pulses = deg2PWM_set_angles(BR_leg_angles, coefficents_SBR, coefficents_FBR, coefficents_TBR)
        
        # Format the serial message
        fr = FR_leg_pulses[0]
        fl = FL_leg_pulses[0]
        br = BR_leg_pulses[0]
        bl = BL_leg_pulses[0]
        
        msg = f"{fr[0]},{fr[1]},{fr[2]},{fl[0]},{fl[1]},{fl[2]},{br[0]},{br[1]},{br[2]},{bl[0]},{bl[1]},{bl[2]}\n"
        precomputed_pulses.append(msg)
    
    # Store the sequence for use in the movement thread
    movement_data = {
        "targets_FR_BL": targets_FR_BL,
        "targets_FL_BR": targets_FL_BR,
        "precomputed_pulses": precomputed_pulses
    }
    
    # If a movement is already in progress, set the new sequence but don't start a new thread
    if movement_in_progress:
        current_sequence = movement_data
        sequence_index = 0  # Reset to start of the new sequence
        print("Updating movement sequence")
    else:
        # Start a new movement thread
        current_sequence = movement_data
        sequence_index = 0
        movement_in_progress = True
        movement_thread = threading.Thread(target=execute_movement_sequence, daemon=True)
        movement_thread.start()
        print("Started new movement sequence thread")

def execute_movement_sequence():
    """Thread function to execute movement sequences and ensure completion"""
    global current_position, movement_in_progress, current_sequence, sequence_index, ser
    
    try:
        while movement_in_progress:
            if current_sequence is None:
                time.sleep(0.01)  # Reduced sleep time for faster response
                continue
                
            # Get current targets and precomputed pulses
            targets_FR_BL = current_sequence["targets_FR_BL"]
            precomputed_pulses = current_sequence["precomputed_pulses"]
            
            # If we've reached the end of the sequence, check if we're at home position
            if sequence_index >= len(targets_FR_BL):
                # Check if the last position is the home position
                if is_home_position(targets_FR_BL[-1]):
                    print("Movement sequence completed at home position")
                    movement_in_progress = False
                    current_sequence = None
                    break
                else:
                    # We need to add steps to return to home position
                    print("Adding return to home position")
                    # Find where we are in the cycle and continue until we reach home
                    for i in range(sequence_index, len(targets_FR_BL) + sequence_index):
                        idx = i % len(targets_FR_BL)
                        if is_home_position(targets_FR_BL[idx]):
                            sequence_index = idx
                            break
                    
                    if sequence_index >= len(targets_FR_BL):
                        # If we didn't find a home position, reset to 0
                        sequence_index = 0
                    
                    continue
            
            # Get the precomputed pulse message
            msg = precomputed_pulses[sequence_index]
            
            # Store current position for tracking
            current_position = targets_FR_BL[sequence_index]
            
            # Send to Arduino - optimize serial communication
            try:
                # Clear any pending data
                ser.reset_input_buffer()
                
                # Send the command
                ser.write(msg.encode('utf-8'))
                ser.flush()  # Ensure data is sent immediately
                
                print(f"Step {sequence_index+1}/{len(targets_FR_BL)} sent")
            except serial.SerialException as e:
                print(f"Serial error: {e}")
                time.sleep(0.1)  # Wait a bit and try to continue
            
            # Increment sequence index
            sequence_index += 1
            
            # Pause between steps - reduced delay
            time.sleep(STEP_DELAY)
            
    except Exception as e:
        print(f"Error in movement execution: {e}")
        movement_in_progress = False
        current_sequence = None

def command_receiver():
    """
    Listen for incoming commands from controller_reader
    """
    # Open serial port to Arduino with optimized settings
    global ser
    try:
        ser = serial.Serial('/dev/ttyACM0', SERIAL_BAUD_RATE, timeout=SERIAL_TIMEOUT)
        time.sleep(1)  # Reduced wait time
        
        # Clear buffers at startup
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        print(f"Serial connection established at {SERIAL_BAUD_RATE} baud")
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        print("Check if the Arduino is connected and the port is correct")
        return
    
    # Create UDP socket for receiving commands
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', 12346))  # Match port in controller_reader.py
    
    print("IK_Sender started. Waiting for controller commands...")
    print("Supported commands: FORWARD, BACKWARD, ROTATE_LEFT")
    
    try:
        while True:
            # Receive command from controller
            data, addr = sock.recvfrom(1024)
            command = data.decode('utf-8')
            
            print(f"Received command: {command} from {addr}")
            
            # Process the movement command
            process_movement(command, ser)
            
    except KeyboardInterrupt:
        print("IK_Sender shutting down...")
    except Exception as e:
        print(f"Error in command receiver: {e}")
    finally:
        if ser:
            ser.close()
        sock.close()

def main():
    # Start the command receiver
    print("Starting IK_Sender with optimized serial communication...")
    print("Ready to receive button commands from controller_reader.py")
    
    try:
        command_receiver()
    except KeyboardInterrupt:
        print("IK_Sender main shutting down...")

if __name__ == "__main__":
    main()


