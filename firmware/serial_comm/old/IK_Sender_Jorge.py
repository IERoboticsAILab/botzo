import numpy as np
import serial
import time
import socket
import threading
from IK_Funcs import (
    FR_legIK,
    FL_legIK,
    BR_legIK,
    BL_legIK,
    deg2PWM_set_angles,
    coefficents_SFR,
    coefficents_FFR,
    coefficents_TFR,
    coefficents_SFL,
    coefficents_FFL,
    coefficents_TFL,
    coefficents_SBL,
    coefficents_FBL,
    coefficents_TBL,
    coefficents_SBR,
    coefficents_FBR,
    coefficents_TBR,
)

# Global variables to track movement state
current_position = [0, 3, 15]  # Start at home position
movement_in_progress = False
current_sequence = None
sequence_index = 0
ser = None  # Will be initialized in command_receiver

# Serial communication optimization settings
SERIAL_BAUD_RATE = 500000  # Match the baud rate used in IK_Sender_2.py
STEP_DELAY = 0.1  # Use the same delay as IK_Sender_2.py
SERIAL_TIMEOUT = 1  # Use the same timeout as IK_Sender_2.py


def is_home_position(position):
    """Check if a position is the home position"""
    return position[0] == 0 and position[1] == 3 and position[2] == 15


def send_ik_to_arduino(position, serial_conn):
    """
    Send a single position to the Arduino
    
    Args:
        position (list): The [x, y, z] position
        serial_conn (serial.Serial): Serial connection to Arduino
    """
    try:
        # Calculate IK for each leg (all legs go to same position)
        FR_leg_angles = [FR_legIK(position[0], position[1], position[2])]
        FR_leg_pulses = deg2PWM_set_angles(
            FR_leg_angles, coefficents_SFR, coefficents_FFR, coefficents_TFR
        )

        FL_leg_angles = [FL_legIK(position[0], position[1], position[2])]
        FL_leg_pulses = deg2PWM_set_angles(
            FL_leg_angles, coefficents_SFL, coefficents_FFL, coefficents_TFL
        )

        BL_leg_angles = [BL_legIK(position[0], position[1], position[2])]
        BL_leg_pulses = deg2PWM_set_angles(
            BL_leg_angles, coefficents_SBL, coefficents_FBL, coefficents_TBL
        )

        BR_leg_angles = [BR_legIK(position[0], position[1], position[2])]
        BR_leg_pulses = deg2PWM_set_angles(
            BR_leg_angles, coefficents_SBR, coefficents_FBR, coefficents_TBR
        )

        # Format the serial message
        fr = FR_leg_pulses[0]
        fl = FL_leg_pulses[0]
        br = BR_leg_pulses[0]
        bl = BL_leg_pulses[0]

        # Format exactly as in IK_Sender_2.py
        msg = "{},{},{},{},{},{},{},{},{},{},{},{}\n".format(
            int(fr[0]), int(fr[1]), int(fr[2]),
            int(fl[0]), int(fl[1]), int(fl[2]),
            int(br[0]), int(br[1]), int(br[2]),
            int(bl[0]), int(bl[1]), int(bl[2])
        )

        # Send to Arduino
        serial_conn.reset_input_buffer()
        serial_conn.write(msg.encode('utf-8'))
        serial_conn.flush()
        print(f"Sent position {position} to Arduino: {msg.strip()}")
        
        # Wait for movement to complete
        time.sleep(STEP_DELAY * 2)
        
    except Exception as e:
        print(f"Error sending position to Arduino: {e}")


def process_movement(command, serial_conn):
    """
    Process movement commands and execute corresponding target sequences

    Args:
        command (str): The movement command ('FORWARD', 'BACKWARD', 'ROTATE_LEFT', etc.)
        serial_conn (serial.Serial): Serial connection to Arduino
    """
    global current_position, movement_in_progress, current_sequence, sequence_index, ser

    # Store the serial connection
    ser = serial_conn

    print(f"Processing {command} movement command")

    # Handle PING command (used to check if IK_Sender is running)
    if command == "PING":
        print("Received PING command - responding with activity")
        return
        
    # Check if command contains speed information
    rotation_speed = None
    if ":" in command:
        command_parts = command.split(":")
        command = command_parts[0]
        rotation_speed = int(command_parts[1])
        print(f"Rotation speed: {rotation_speed}")

    # Handle HOME_AND_STOP command
    if command == "HOME_AND_STOP":
        print("Returning to home position and stopping")
        if not is_home_position(current_position):
            # Move to home position
            home_position = [0, 3, 15]
            send_ik_to_arduino(home_position, serial_conn)
            current_position = home_position
        movement_in_progress = False
        current_sequence = None
        sequence_index = 0
        return
        
    # Handle JUMP command
    if command == "JUMP":
        print("Executing jump sequence")
        
        # Define jump sequence
        jump_targets_FR_BL = [
            [0, 3, 15],  # Start at home position
            [0, 3, 10],  # Crouch down
            [0, 3, 18],  # Jump up
            [0, 3, 15]   # Return to home position
        ]
        
        jump_targets_FL_BR = [
            [0, 3, 15],  # Start at home position
            [0, 3, 10],  # Crouch down
            [0, 3, 18],  # Jump up
            [0, 3, 15]   # Return to home position
        ]
        
        # Use the standard movement sequence mechanism
        targets_FR_BL = jump_targets_FR_BL
        targets_FL_BR = jump_targets_FL_BR
        print("Executing JUMP movement sequence")
        
        # Continue to the standard movement processing below
    
    # Define target coordinates for different movements
    # FORWARD movement coordinates
    forward_targets_FR_BL = [
        [0, 3, 15],
        [0.5, 3, 15],
        [1, 3, 15],
        [1.5, 3, 15],
        [2, 3, 15],
        [2.5, 3, 15],
        [3, 3, 15],
        [3.5, 3, 15],
        [4, 3, 15],
        [4, 3, 14],
        [4, 3, 13],
        [4, 3, 12],
        [3, 3, 12],
        [2, 3, 12],
        [1, 3, 12],
        [0, 3, 12],
        [0, 3, 13],
        [0, 3, 14],
    ]

    forward_targets_FL_BR = [
        [4, 3, 15],
        [4, 3, 14],
        [4, 3, 13],
        [4, 3, 12],
        [3, 3, 12],
        [2, 3, 12],
        [1, 3, 12],
        [0, 3, 12],
        [0, 3, 13],
        [0, 3, 14],
        [0, 3, 15],
        [0.5, 3, 15],
        [1, 3, 15],
        [1.5, 3, 15],
        [2, 3, 15],
        [2.5, 3, 15],
        [3, 3, 15],
        [3.5, 3, 15],
    ]

    # BACKWARD movement coordinates
    backward_targets_FR_BL = [
        [0, 3, 14],
        [0, 3, 13],
        [0, 3, 12],
        [1, 3, 12],
        [2, 3, 12],
        [3, 3, 12],
        [4, 3, 12],
        [4, 3, 13],
        [4, 3, 14],
        [4, 3, 15],
        [3.5, 3, 15],
        [3, 3, 15],
        [2.5, 3, 15],
        [2, 3, 15],
        [1.5, 3, 15],
        [1, 3, 15],
        [0.5, 3, 15],
        [0, 3, 15],
    ]

    backward_targets_FL_BR = [
        [3.5, 3, 15],
        [3, 3, 15],
        [2.5, 3, 15],
        [2, 3, 15],
        [1.5, 3, 15],
        [1, 3, 15],
        [0.5, 3, 15],
        [0, 3, 15],
        [0, 3, 14],
        [0, 3, 13],
        [0, 3, 12],
        [1, 3, 12],
        [2, 3, 12],
        [3, 3, 12],
        [4, 3, 12],
        [4, 3, 13],
        [4, 3, 14],
        [4, 3, 15],
    ]

    # ROTATE LEFT movement coordinates
    rotate_left_targets_FR_BL = [
        [0, 5, 15],
        [0, 5, 14],
        [0, 5, 13],
        [0, 5, 12],
        [0, 4, 12],
        [0, 3, 12],
        [0, 2, 12],
        [0, 1, 12],
        [0, 1, 13],
        [0, 1, 14],
        [0, 1, 15],
        [0, 1.5, 15],
        [0, 2, 15],
        [0, 2.5, 15],
        [0, 3, 15],
        [0, 3.5, 15],
        [0, 4, 15],
        [0, 4.5, 15],
    ]

    rotate_left_targets_FL_BR = [
        [0, 1.5, 15],
        [0, 2, 15],
        [0, 2.5, 15],
        [0, 3, 15],
        [0, 3.5, 15],
        [0, 4, 15],
        [0, 4.5, 15],
        [0, 5, 15],
        [0, 5, 14],
        [0, 5, 13],
        [0, 5, 12],
        [0, 4, 12],
        [0, 3, 12],
        [0, 2, 12],
        [0, 1, 12],
        [0, 1, 13],
        [0, 1, 14],
        [0, 1, 15],
    ]

    # ROTATE RIGHT movement coordinates
    rotate_right_targets_FR_BL = [
        [0, 1, 15],
        [0, 1, 14],
        [0, 1, 13],
        [0, 1, 12],
        [0, 2, 12],
        [0, 3, 12],
        [0, 4, 12],
        [0, 5, 12],
        [0, 5, 13],
        [0, 5, 14],
        [0, 5, 15],
        [0, 4.5, 15],
        [0, 4, 15],
        [0, 3.5, 15],
        [0, 3, 15],
        [0, 2.5, 15],
        [0, 2, 15],
        [0, 1.5, 15],
    ]

    rotate_right_targets_FL_BR = [
        [0, 4.5, 15],
        [0, 4, 15],
        [0, 3.5, 15],
        [0, 3, 15],
        [0, 2.5, 15],
        [0, 2, 15],
        [0, 1.5, 15],
        [0, 1, 15],
        [0, 1, 14],
        [0, 1, 13],
        [0, 1, 12],
        [0, 2, 12],
        [0, 3, 12],
        [0, 4, 12],
        [0, 5, 12],
        [0, 5, 13],
        [0, 5, 14],
        [0, 5, 15],
    ]

    # HOME_AND_STOP movement coordinates (just a single position)
    home_and_stop_targets_FR_BL = [[0, 3, 15]]
    home_and_stop_targets_FL_BR = [[0, 3, 15]]

    # Select the appropriate targets based on the command
    if command == "FORWARD":
        targets_FR_BL = forward_targets_FR_BL
        targets_FL_BR = forward_targets_FL_BR
        print("Executing FORWARD movement sequence")
    elif command == "BACKWARD":
        targets_FR_BL = backward_targets_FR_BL
        targets_FL_BR = backward_targets_FL_BR
        print("Executing BACKWARD movement sequence")
    elif command == "ROTATE_LEFT":
        targets_FR_BL = rotate_left_targets_FR_BL
        targets_FL_BR = rotate_left_targets_FL_BR
        print("Executing ROTATE LEFT movement sequence")
    elif command == "ROTATE_RIGHT":
        targets_FR_BL = rotate_right_targets_FR_BL
        targets_FL_BR = rotate_right_targets_FL_BR
        print("Executing ROTATE RIGHT movement sequence")
    elif command == "HOME_AND_STOP":
        targets_FR_BL = home_and_stop_targets_FR_BL
        targets_FL_BR = home_and_stop_targets_FL_BR
        print("Executing HOME AND STOP movement sequence")
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
        FR_leg_angles = [FR_legIK(x, y, z) for x, y, z in [current_FR_BL]]
        FR_leg_pulses = deg2PWM_set_angles(
            FR_leg_angles, coefficents_SFR, coefficents_FFR, coefficents_TFR
        )

        FL_leg_angles = [FL_legIK(x, y, z) for x, y, z in [current_FL_BR]]
        FL_leg_pulses = deg2PWM_set_angles(
            FL_leg_angles, coefficents_SFL, coefficents_FFL, coefficents_TFL
        )

        BL_leg_angles = [BL_legIK(x, y, z) for x, y, z in [current_FR_BL]]
        BL_leg_pulses = deg2PWM_set_angles(
            BL_leg_angles, coefficents_SBL, coefficents_FBL, coefficents_TBL
        )

        BR_leg_angles = [BR_legIK(x, y, z) for x, y, z in [current_FL_BR]]
        BR_leg_pulses = deg2PWM_set_angles(
            BR_leg_angles, coefficents_SBR, coefficents_FBR, coefficents_TBR
        )

        # Format the serial message
        fr = FR_leg_pulses[0]
        fl = FL_leg_pulses[0]
        br = BR_leg_pulses[0]
        bl = BL_leg_pulses[0]

        # Format exactly as in IK_Sender_2.py
        msg = "{},{},{},{},{},{},{},{},{},{},{},{}\n".format(
            int(fr[0]), int(fr[1]), int(fr[2]),
            int(fl[0]), int(fl[1]), int(fl[2]),
            int(br[0]), int(br[1]), int(br[2]),
            int(bl[0]), int(bl[1]), int(bl[2])
        )
        precomputed_pulses.append(msg)

    # Store the sequence for use in the movement thread
    movement_data = {
        "targets_FR_BL": targets_FR_BL,
        "targets_FL_BR": targets_FL_BR,
        "precomputed_pulses": precomputed_pulses,
        "rotation_speed": rotation_speed  # Add rotation speed to movement data
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
        movement_thread = threading.Thread(
            target=execute_movement_sequence, daemon=True
        )
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
            
            # Get rotation speed if available
            rotation_speed = current_sequence.get("rotation_speed", None)
            
            # Calculate step delay based on rotation speed
            current_step_delay = STEP_DELAY
            if rotation_speed is not None:
                # Map rotation_speed (0-100) to a delay between 0.01 and 0.1 seconds
                # Higher speed = lower delay
                current_step_delay = 0.1 - ((rotation_speed / 100) * 0.09)
                print(f"Using step delay: {current_step_delay:.3f}s based on rotation speed: {rotation_speed}")

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
                ser.write(msg.encode("utf-8"))
                ser.flush()  # Ensure data is sent immediately

                print(f"Step {sequence_index+1}/{len(targets_FR_BL)} sent: {msg.strip()}")
            except serial.SerialException as e:
                print(f"Serial error: {e}")
                time.sleep(0.1)  # Wait a bit and try to continue

            # Increment sequence index
            sequence_index += 1

            # Pause between steps - adjusted delay based on rotation speed
            time.sleep(current_step_delay)

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
        print(f"Attempting to connect to Arduino on /dev/ttyACM0 at {SERIAL_BAUD_RATE} baud...")
        ser = serial.Serial("/dev/ttyACM0", SERIAL_BAUD_RATE, timeout=SERIAL_TIMEOUT)
        time.sleep(2)  # Wait longer for Arduino to reset, as in IK_Sender_2.py

        # Clear buffers at startup
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        print(f"Serial connection established at {SERIAL_BAUD_RATE} baud")
        print(f"Serial port details: {ser}")
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        print("Check if the Arduino is connected and the port is correct")
        print("Available ports:")
        import serial.tools.list_ports
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            print(f"  {p}")
        return

    # Create UDP socket for receiving commands
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 12346))  # Match port in controller_reader.py

    # Create a socket for sending ready notification
    ready_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    print("IK_Sender started. Waiting for controller commands...")
    print("Supported commands: FORWARD, BACKWARD, ROTATE_LEFT, ROTATE_RIGHT, JUMP, HOME_AND_STOP, PING")
    
    # Send ready notification to controller_reader
    try:
        ready_sock.sendto(b"READY", ("127.0.0.1", 12347))
        print("Sent ready notification to controller_reader")
    except Exception as e:
        print(f"Failed to send ready notification: {e}")
    finally:
        ready_sock.close()

    try:
        while True:
            # Receive command from controller
            data, addr = sock.recvfrom(1024)
            command = data.decode("utf-8")

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
