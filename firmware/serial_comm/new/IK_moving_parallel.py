import numpy as np
import serial
import time
import threading
import os
import sys
import controller_dict_parallel as controller_dict
import hardcoded_skills_parallel as skills
from imu_stabilizer import IMUStabilizer

# Add the current directory to the system path to ensure imports work correctly
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

# Import IK functions
from IK_Funcs import *

# Global variables
ser = None  # Serial connection to Arduino
current_position = [0, 3, 15]  # Start at home position

# Serial communication settings
SERIAL_BAUD_RATE = 500000 
STEP_DELAY = 0.05  # Delay between steps for reliable communication
SERIAL_TIMEOUT = 1.0  # Timeout for serial communication

# Joystick settings
JOYSTICK_THRESHOLD = 0.3  # Threshold for joystick activation

# Serial port options to try (in order of preference)
SERIAL_PORTS = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']

# IMU stabilization settings
IMU_UPDATE_HZ = 100.0
# Gain from tilt angle (deg) to leg Z correction (cm). Tune carefully.
# If the robot leans in the direction of correction, invert the sign.
ROLL_GAIN_Z_PER_DEG = -0.08  # cm per degree (negative to invert previous behavior)
PITCH_GAIN_Z_PER_DEG = 0.08
MAX_Z_CORRECTION_CM = 1.5
# Ignore tiny tilts to avoid oscillation
ROLL_PITCH_DEADBAND_DEG = 2.0
# Smoothing and slew rate limiting to avoid sudden asymmetric corrections
CORR_SMOOTH_ALPHA = 0.18  # stronger smoothing
MAX_DELTA_Z_PER_STEP_CM = 0.12
STANCE_Z_THRESHOLD = 15.5  # apply correction only when target z >= threshold (stance phase)
RATE_DAMPING_GAIN_CM_PER_DPS = 0.01  # additional damping proportional to rate

# Keep previous per-leg Z corrections for smoothing/slew
prev_leg_z_corrections = [0.0, 0.0, 0.0, 0.0]

def _apply_imu_stabilization(coordinates):
    """Apply small Z offsets to each leg based on current roll/pitch from IMU.
    coordinates: [[x,y,z], ...] for FR, FL, BR, BL
    Returns adjusted coordinates list.
    """
    global imu
    if imu is None:
        return coordinates
    try:
        ori = imu.get_orientation()
        roll = ori.get('roll', 0.0)
        pitch = ori.get('pitch', 0.0)
        rates = imu.get_rates()
        roll_rate = rates.get('roll_rate', 0.0)
        pitch_rate = rates.get('pitch_rate', 0.0)

        # Deadband to avoid constant micro-corrections
        if abs(roll) < ROLL_PITCH_DEADBAND_DEG:
            roll = 0.0
        if abs(pitch) < ROLL_PITCH_DEADBAND_DEG:
            pitch = 0.0

        # Compute Z corrections per leg to counteract tilt.
        # Map angles to Z corrections. Sign of gains determines direction.
        roll_corr = ROLL_GAIN_Z_PER_DEG * roll
        pitch_corr = PITCH_GAIN_Z_PER_DEG * pitch

        # Leg order: FR, FL, BR, BL
        # Add viscous damping based on angular rates to fight oscillation
        roll_damp = RATE_DAMPING_GAIN_CM_PER_DPS * roll_rate
        pitch_damp = RATE_DAMPING_GAIN_CM_PER_DPS * pitch_rate

        raw_corrections = [
            -roll_corr - pitch_corr - roll_damp - pitch_damp,  # FR
            +roll_corr - pitch_corr + roll_damp - pitch_damp,  # FL
            -roll_corr + pitch_corr - roll_damp + pitch_damp,  # BR
            +roll_corr + pitch_corr + roll_damp + pitch_damp   # BL
        ]

        # Apply only on stance legs to avoid disturbing swing trajectory
        # Also smooth and slew-limit per leg
        global prev_leg_z_corrections
        smoothed = prev_leg_z_corrections[:]
        applied_mask = [False, False, False, False]
        for leg_idx, target in enumerate(coordinates):
            x, y, z = target
            if z >= STANCE_Z_THRESHOLD:
                applied_mask[leg_idx] = True
                desired = max(-MAX_Z_CORRECTION_CM, min(MAX_Z_CORRECTION_CM, raw_corrections[leg_idx]))
                # Low-pass filter
                filtered = smoothed[leg_idx] + CORR_SMOOTH_ALPHA * (desired - smoothed[leg_idx])
                # Slew-rate limit
                delta = filtered - smoothed[leg_idx]
                if delta > MAX_DELTA_Z_PER_STEP_CM:
                    delta = MAX_DELTA_Z_PER_STEP_CM
                elif delta < -MAX_DELTA_Z_PER_STEP_CM:
                    delta = -MAX_DELTA_Z_PER_STEP_CM
                smoothed[leg_idx] = smoothed[leg_idx] + delta
            else:
                # During swing, don't apply correction
                smoothed[leg_idx] = 0.0

        # Enforce diagonal symmetry only when both legs of the diagonal are in stance
        # Diagonal pairs: (FR, BL) -> (0,3), (FL, BR) -> (1,2)
        for a, b in [(0, 3), (1, 2)]:
            if applied_mask[a] and applied_mask[b]:
                mean_val = 0.5 * (smoothed[a] + smoothed[b])
                smoothed[a] = mean_val
                smoothed[b] = mean_val

        # Soft zero-sum: remove only a fraction of the mean to avoid large swings
        applied_values = [smoothed[i] for i in range(4) if applied_mask[i]]
        mean_corr = sum(applied_values) / len(applied_values) if applied_values else 0.0
        mean_remove_factor = 0.7
        for i in range(4):
            if applied_mask[i]:
                smoothed[i] -= mean_remove_factor * mean_corr

        # Update state
        prev_leg_z_corrections = smoothed

        # Apply corrections
        adjusted = []
        for leg_idx, target in enumerate(coordinates):
            x, y, z = target
            if applied_mask[leg_idx]:
                z_adj = z + smoothed[leg_idx]
            else:
                z_adj = z  # never alter swing legs
            adjusted.append([x, y, z_adj])
        return adjusted
    except Exception as e:
        print(f"IMU stabilization error: {e}")
        return coordinates

def move_legs_to_pos(coordinates):
    """Move legs to the specified coordinates"""
    global ser
    
    if ser is None or not ser.is_open:
        print("ERROR: Serial connection is not open. Cannot send movement commands.")
        return
    
    # Unpack coordinates for each leg
    FR_target, FL_target, BR_target, BL_target = coordinates

    # Apply IMU-based stabilization
    coordinates = _apply_imu_stabilization([FR_target, FL_target, BR_target, BL_target])
    FR_target, FL_target, BR_target, BL_target = coordinates
    
    # Calculate IK for each leg
    try:
        # Calculate angles and pulses for each leg
        FR_leg_angles = [FR_legIK(FR_target[0], FR_target[1], FR_target[2])]
        FR_leg_pulses = deg2PWM_set_angles(FR_leg_angles, coefficents_SFR, coefficents_FFR, coefficents_TFR)
        
        FL_leg_angles = [FL_legIK(FL_target[0], FL_target[1], FL_target[2])]
        FL_leg_pulses = deg2PWM_set_angles(FL_leg_angles, coefficents_SFL, coefficents_FFL, coefficents_TFL)
        
        BL_leg_angles = [BL_legIK(BL_target[0], BL_target[1], BL_target[2])]
        BL_leg_pulses = deg2PWM_set_angles(BL_leg_angles, coefficents_SBL, coefficents_FBL, coefficents_TBL)
        
        BR_leg_angles = [BR_legIK(BR_target[0], BR_target[1], BR_target[2])]
        BR_leg_pulses = deg2PWM_set_angles(BR_leg_angles, coefficents_SBR, coefficents_FBR, coefficents_TBR)
        
        # Format the serial message
        fr = FR_leg_pulses[0]
        fl = FL_leg_pulses[0]
        br = BR_leg_pulses[0]
        bl = BL_leg_pulses[0]
        msg = f"{fr[0]},{fr[1]},{fr[2]},{fl[0]},{fl[1]},{fl[2]},{br[0]},{br[1]},{br[2]},{bl[0]},{bl[1]},{bl[2]}\n"
        
        # Send directly to Arduino
        ser.reset_input_buffer()
        ser.write(msg.encode('utf-8'))
        ser.flush()
        
    except Exception as e:
        print(f"Error calculating or sending pulse values: {e}")

def process_controller_inputs():
    """
    Process controller inputs and map them to robot actions
    """
    # Get current joystick values
    left_y = -controller_dict.get_axis_value("LEFT_STICK_Y")  # Invert Y axis so positive is forward
    right_x = controller_dict.get_axis_value("RIGHT_STICK_X")  # Positive is right
    
    # Check for one-time button presses
    if controller_dict.button_just_pressed("TRIANGLE"):
        print("Triangle pressed - Going to home position")
        skills.skill_executor.go_home()
    
    if controller_dict.button_just_pressed("SQUARE"):
        print("Square pressed - Going to sleep position")
        skills.skill_executor.go_sleep()
    
    if controller_dict.button_just_pressed("X"):
        print("X pressed - Executing jump")
        skills.skill_executor.jump()
    
    # Handle continuous movements from joysticks
    # Forward/backward movement (left joystick Y axis)
    if abs(left_y) > JOYSTICK_THRESHOLD:
        if left_y > 0 and skills.skill_executor.current_skill != 'forward':
            print("Left stick forward - Starting forward movement")
            skills.skill_executor.start_continuous_movement('forward')
        elif left_y < 0 and skills.skill_executor.current_skill != 'backward':
            print("Left stick backward - Starting backward movement")
            skills.skill_executor.start_continuous_movement('backward')
    elif skills.skill_executor.current_skill in ['forward', 'backward']:
        skills.skill_executor.stop_continuous_movement()
    
    # Spin left/right movement (right joystick X axis)
    if abs(right_x) > JOYSTICK_THRESHOLD:
        if right_x > 0 and skills.skill_executor.current_skill != 'spin_right':
            print("Right stick right - Starting spin right movement")
            skills.skill_executor.start_continuous_movement('spin_right')
        elif right_x < 0 and skills.skill_executor.current_skill != 'spin_left':
            print("Right stick left - Starting spin left movement")
            skills.skill_executor.start_continuous_movement('spin_left')
    elif skills.skill_executor.current_skill in ['spin_left', 'spin_right']:
        skills.skill_executor.stop_continuous_movement()

def setup_serial_connection():
    """Establish serial connection to the Arduino"""
    global ser
    
    for port in SERIAL_PORTS:
        try:
            print(f"Attempting to connect to serial port: {port}")
            ser = serial.Serial(port, SERIAL_BAUD_RATE, timeout=SERIAL_TIMEOUT)
            # Wait for Arduino to reset after connection
            time.sleep(2)
            # Clear buffers at startup
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            
            # Test the serial connection with a simple message
            #test_msg = "1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500\n"
            #ser.write(test_msg.encode('utf-8'))
            #ser.flush()
            #time.sleep(0.5)
            
            print(f"Serial connection established at {port}")
            return True
        except serial.SerialException as e:
            print(f"Error connecting to {port}: {e}")
            continue
    
    print("ERROR: Could not connect to any serial port. Please check Arduino connection.")
    return False

def main():
    """Main function to run the parallel control system"""
    global ser, imu
    
    print("Starting IK_moving_parallel.py - Controller-based robot control")
    
    # Ensure controller is initialized
    if not controller_dict.running:
        controller_dict.start_controller_reader()
    
    # Set up serial connection
    if not setup_serial_connection():
        return
    
    # Configure the skill executor with our move_legs function
    skills.skill_executor.set_move_legs_function(move_legs_to_pos)
    
    # Start IMU stabilizer
    imu = IMUStabilizer(update_hz=IMU_UPDATE_HZ)
    try:
        imu.start()
        # Ensure we zero reference once more after initial home pose is reached
        print("IMU stabilizer started")
    except Exception as e:
        print(f"IMU initialization failed: {e}")
        imu = None

    # Go to home position to start
    skills.skill_executor.go_home()
    # Give servos a moment to settle at home, then zero IMU reference so home is level
    if imu:
        time.sleep(0.3)
        imu.zero_reference()
        print("IMU zeroed at home pose")
    
    try:
        # Main control loop
        while True:
            # Process controller inputs
            process_controller_inputs()
            
            # Small delay to prevent high CPU usage
            time.sleep(0.01)
    
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error in main loop: {e}")
    finally:
        # Cleanup
        if skills.skill_executor.movement_in_progress:
            skills.skill_executor.stop_continuous_movement()
        
        if ser and ser.is_open:
            ser.close()
        
        controller_dict.stop_controller_reader()
        if imu:
            imu.stop()
        print("Shutdown complete")

if __name__ == "__main__":
    main() 