import numpy as np
import serial
import time
from math import cos, sin, acos, atan2, pi, sqrt, asin
import platform
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

class QuadrupedStabilizer:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200, simulation_mode=None):
        # Determine simulation mode if not specified
        if simulation_mode is None:
            simulation_mode = platform.system() != 'Linux'
        self.simulation_mode = simulation_mode
        
        # Robot dimensions (in cm)
        self.BODY_LENGTH = 27.4
        self.BODY_WIDTH = 14.0
        self.BODY_HEIGHT = 8.0
        
        # Leg segment lengths (in cm)
        self.COXA_LENGTH = 3.1        # A
        self.FEMUR_LENGTH = 9.5       # E
        self.REAL_FEMUR_LENGTH = 9.1  # E'
        self.TIBIA_LENGTH = 9.8       # F
        self.FOCUS_DIST = 2.8         # Distance from focus point to servo
        
        # Servo configuration
        self.NUM_LEGS = 4
        self.SERVOS_PER_LEG = 3
        self.SERVO_PINS = {
            'FL': [5, 6, 7],    # Front Left: shoulder, femur, tibia
            'FR': [2, 3, 4],    # Front Right
            'BL': [11, 12, 13], # Back Left
            'BR': [8, 9, 10]    # Back Right
        }
        
        # Servo limits and parameters
        self.SERVO_MIN = 0
        self.SERVO_MAX = 270  # DS3225 has 270° range
        self.SERVO_CENTER = 135  # Center position
        self.SERVO_SPEED = 0.5  # Seconds per 60 degrees
        
        # Servo angle ranges (degrees)
        self.SHOULDER_RANGE = (0, 270)   # Horizontal rotation
        self.FEMUR_RANGE = (45, 225)     # Vertical lift
        self.TIBIA_RANGE = (90, 270)     # Extension
        
        # Stabilization parameters
        self.MAX_ANGLE_CHANGE = 30  # Maximum angle change per update
        self.PITCH_COMPENSATION = 1.2  # Increased pitch response
        self.ROLL_COMPENSATION = 1.0   # Normal roll response
        self.YAW_COMPENSATION = 0.8    # Reduced yaw response
        
        # Movement ranges (in cm)
        self.X_RANGE = (-5, 5)    # Forward/backward
        self.Y_RANGE = (-3, 3)    # Left/right
        self.Z_RANGE = (-18, -12) # Height
        
        # Default standing height
        self.DEFAULT_HEIGHT = 12.5  # cm from body to ground
        
        # Debug parameters
        self.debug_mode = True
        self.last_update_time = time.time()
        self.update_count = 0
        self.avg_update_rate = 0
        
        # Leg positions relative to body center (FL, FR, BL, BR)
        self.leg_positions = [
            [ self.BODY_LENGTH/2,  self.BODY_WIDTH/2],  # Front Left
            [ self.BODY_LENGTH/2, -self.BODY_WIDTH/2],  # Front Right
            [-self.BODY_LENGTH/2,  self.BODY_WIDTH/2],  # Back Left
            [-self.BODY_LENGTH/2, -self.BODY_WIDTH/2]   # Back Right
        ]

        # Initialize visualization if in simulation mode
        if simulation_mode:
            self.init_visualization()
        
        if not simulation_mode:
            try:
                # Initialize serial connection to Arduino
                self.arduino = serial.Serial(port=port, baudrate=baudrate, timeout=1)
                time.sleep(2)  # Wait for Arduino to initialize
                print("Connected to Arduino")
                self._center_all_servos()
            except serial.SerialException as e:
                print(f"Warning: Could not connect to Arduino: {e}")
                self.simulation_mode = True
                print("Falling back to simulation mode")
                self.init_visualization()
        else:
            print("Running in simulation mode - servo commands will be printed")

    def _center_all_servos(self):
        """Center all servos on startup"""
        for leg in range(self.NUM_LEGS):
            for servo in range(self.SERVOS_PER_LEG):
                command = f"S,{leg},{servo},{self.SERVO_CENTER:.1f};"
                self.arduino.write(command.encode())
                time.sleep(0.5)  # Longer delay for initialization

    def _update_debug_stats(self):
        """Update debug statistics"""
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.update_count += 1
        
        # Calculate running average of update rate
        alpha = 0.1  # Smoothing factor
        instantaneous_rate = 1.0 / dt if dt > 0 else 0
        self.avg_update_rate = (alpha * instantaneous_rate + 
                              (1 - alpha) * self.avg_update_rate)
        
        self.last_update_time = current_time
        
        if self.debug_mode and self.update_count % 100 == 0:
            print(f"\nDebug Statistics:")
            print(f"Update Rate: {self.avg_update_rate:.1f} Hz")
            print(f"Last Update Time: {dt*1000:.1f} ms")

    def _constrain_movement(self, point):
        """Constrain movement within safe ranges"""
        x, y, z = point
        x = np.clip(x, self.X_RANGE[0], self.X_RANGE[1])
        y = np.clip(y, self.Y_RANGE[0], self.Y_RANGE[1])
        z = np.clip(z, self.Z_RANGE[0], self.Z_RANGE[1])
        return np.array([x, y, z])

    def calculate_leg_angles(self, target_point):
        """Calculate leg angles using Botzo's IK equations"""
        x, y, z = target_point
        
        # 1. Distance Calculation
        D = sqrt(z**2 + y**2 - self.COXA_LENGTH**2)
        
        # 2. G Calculation
        G = sqrt(D**2 + x**2)
        
        # 3. Knee Angle
        phi = acos((G**2 - self.FEMUR_LENGTH**2 - self.TIBIA_LENGTH**2) / 
                   (-2 * self.FEMUR_LENGTH * self.TIBIA_LENGTH))
        
        # 4. Shoulder Angle
        theta_shoulder = atan2(x, D) + asin((self.TIBIA_LENGTH * sin(phi)) / G)
        
        # 5. Coxa Angle
        psi = atan2(y, z) + atan2(D, self.COXA_LENGTH)
        
        # Angle Adjustments
        adjustment = acos((self.REAL_FEMUR_LENGTH**2 + self.FEMUR_LENGTH**2 - self.FOCUS_DIST**2) /
                         (2 * self.REAL_FEMUR_LENGTH * self.FEMUR_LENGTH))
        
        theta_femur = pi/2 - (theta_shoulder + adjustment)
        theta_tibia = pi - phi + adjustment + theta_femur
        
        return psi, theta_femur, theta_tibia

    def update_visualization(self, rotation_matrix, leg_positions_3d):
        """Update the 3D visualization"""
        self.ax.cla()
        
        # Set labels and title
        self.ax.set_xlabel('X (cm)')
        self.ax.set_ylabel('Y (cm)')
        self.ax.set_zlabel('Z (cm)')
        self.ax.set_title('Quadruped Robot Stabilization')
        
        # Set consistent view limits
        limit = max(self.BODY_LENGTH, self.BODY_WIDTH, self.DEFAULT_HEIGHT) * 1.5
        self.ax.set_xlim([-limit, limit])
        self.ax.set_ylim([-limit, limit])
        self.ax.set_zlim([-limit, limit])
        
        # Draw robot body
        body_points = np.array([
            [ self.BODY_LENGTH/2,  self.BODY_WIDTH/2, 0],
            [ self.BODY_LENGTH/2, -self.BODY_WIDTH/2, 0],
            [-self.BODY_LENGTH/2, -self.BODY_WIDTH/2, 0],
            [-self.BODY_LENGTH/2,  self.BODY_WIDTH/2, 0],
            [ self.BODY_LENGTH/2,  self.BODY_WIDTH/2, 0]
        ])
        
        # Apply rotation to body
        rotated_body = np.dot(body_points, rotation_matrix.T)
        self.ax.plot(rotated_body[:,0], rotated_body[:,1], rotated_body[:,2], 
                    'g-', linewidth=2, label='Robot Body')
        
        # Draw legs
        colors = ['r', 'b', 'y', 'm']
        leg_names = ['Front Left', 'Front Right', 'Back Left', 'Back Right']
        
        for i, (leg_pos, color, name) in enumerate(zip(leg_positions_3d, colors, leg_names)):
            mount_point = rotated_body[i]
            
            # Calculate leg segment positions
            coxa_end = mount_point + np.array([
                self.COXA_LENGTH * cos(leg_pos[0]),
                self.COXA_LENGTH * sin(leg_pos[0]),
                0
            ])
            
            femur_end = coxa_end + np.array([
                self.REAL_FEMUR_LENGTH * cos(leg_pos[1]) * cos(leg_pos[0]),
                self.REAL_FEMUR_LENGTH * cos(leg_pos[1]) * sin(leg_pos[0]),
                self.REAL_FEMUR_LENGTH * sin(leg_pos[1])
            ])
            
            tibia_end = femur_end + np.array([
                self.TIBIA_LENGTH * cos(leg_pos[2]) * cos(leg_pos[0]),
                self.TIBIA_LENGTH * cos(leg_pos[2]) * sin(leg_pos[0]),
                self.TIBIA_LENGTH * sin(leg_pos[2])
            ])
            
            # Draw leg segments
            self.ax.plot([mount_point[0], coxa_end[0]], 
                        [mount_point[1], coxa_end[1]], 
                        [mount_point[2], coxa_end[2]], 
                        color=color, linewidth=2)
            self.ax.plot([coxa_end[0], femur_end[0]], 
                        [coxa_end[1], femur_end[1]], 
                        [coxa_end[2], femur_end[2]], 
                        color=color, linewidth=2)
            self.ax.plot([femur_end[0], tibia_end[0]], 
                        [femur_end[1], tibia_end[1]], 
                        [femur_end[2], tibia_end[2]], 
                        color=color, linewidth=2)
            
            # Draw joints
            self.ax.scatter([mount_point[0], coxa_end[0], femur_end[0], tibia_end[0]],
                          [mount_point[1], coxa_end[1], femur_end[1], tibia_end[1]],
                          [mount_point[2], coxa_end[2], femur_end[2], tibia_end[2]],
                          c=color, marker='o', s=50)
        
        self.ax.legend()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def _constrain_servo_angle(self, servo_type, angle):
        """Constrain servo angle based on type and limits"""
        if servo_type == 0:  # Shoulder
            return np.clip(angle, *self.SHOULDER_RANGE)
        elif servo_type == 1:  # Femur
            return np.clip(angle, *self.FEMUR_RANGE)
        else:  # Tibia
            return np.clip(angle, *self.TIBIA_RANGE)

    def compute_leg_adjustments(self, rotation_matrix):
        """
        Convert rotation matrix to leg adjustments with enhanced stabilization
        Returns angles for all servos in each leg
        """
        # Extract Euler angles from rotation matrix
        pitch = np.arctan2(rotation_matrix[2,1], rotation_matrix[2,2])
        roll = np.arctan2(-rotation_matrix[2,0], 
                         np.sqrt(rotation_matrix[2,1]**2 + rotation_matrix[2,2]**2))
        yaw = np.arctan2(rotation_matrix[1,0], rotation_matrix[0,0])
        
        # Convert to degrees
        pitch_deg = np.degrees(pitch)
        roll_deg = np.degrees(roll)
        yaw_deg = np.degrees(yaw)
        
        adjustments = []
        leg_positions_3d = []
        
        for i, leg_pos in enumerate(self.leg_positions):
            # Base position
            x, y = leg_pos
            
            # Apply compensations
            pitch_comp = -pitch_deg * self.PITCH_COMPENSATION
            roll_comp = -roll_deg * self.ROLL_COMPENSATION
            yaw_comp = -yaw_deg * self.YAW_COMPENSATION
            
            # Adjust compensation based on leg position
            if i < 2:  # Front legs
                pitch_comp *= 1.2  # Increased compensation for front legs
            if i % 2 == 0:  # Right side legs
                roll_comp *= -1
            
            # Calculate target point with compensations
            target = np.array([
                x + pitch_comp * 0.1,  # Convert degrees to cm
                y + roll_comp * 0.1,
                -self.DEFAULT_HEIGHT + yaw_comp * 0.05
            ])
            
            # Constrain movement
            target = self._constrain_movement(target)
            
            # Transform by rotation matrix
            rotated_target = rotation_matrix.dot(target)
            
            # Calculate angles using inverse kinematics
            shoulder_angle, femur_angle, tibia_angle = self.calculate_leg_angles(rotated_target)
            
            # Store angles for visualization
            leg_positions_3d.append([shoulder_angle, femur_angle, tibia_angle])
            
            # Convert to servo angles and apply constraints
            servo_angles = [
                self._constrain_servo_angle(0, np.degrees(shoulder_angle)),
                self._constrain_servo_angle(1, np.degrees(femur_angle)),
                self._constrain_servo_angle(2, np.degrees(tibia_angle))
            ]
            
            # Limit maximum angle change per update
            if hasattr(self, 'last_angles'):
                for j in range(self.SERVOS_PER_LEG):
                    angle_diff = servo_angles[j] - self.last_angles[i][j]
                    if abs(angle_diff) > self.MAX_ANGLE_CHANGE:
                        servo_angles[j] = (self.last_angles[i][j] + 
                                        np.sign(angle_diff) * self.MAX_ANGLE_CHANGE)
            
            adjustments.append(servo_angles)
        
        # Store angles for next update
        self.last_angles = [angles.copy() for angles in adjustments]
        
        # Update debug stats
        self._update_debug_stats()
        
        # Update visualization if in simulation mode
        if self.simulation_mode:
            self.update_visualization(rotation_matrix, leg_positions_3d)
        
        return adjustments

    def send_servo_commands(self, servo_angles):
        """
        Send servo angles to Arduino with rate limiting
        servo_angles: list of [shoulder, femur, tibia] angles for each leg
        """
        for leg in range(self.NUM_LEGS):
            for servo in range(self.SERVOS_PER_LEG):
                angle = servo_angles[leg][servo]
                
                # Calculate required movement time based on angle change
                if hasattr(self, 'last_sent_angles'):
                    angle_diff = abs(angle - self.last_sent_angles[leg][servo])
                    move_time = (angle_diff / 60.0) * self.SERVO_SPEED
                else:
                    move_time = self.SERVO_SPEED
                    self.last_sent_angles = [[0] * self.SERVOS_PER_LEG 
                                           for _ in range(self.NUM_LEGS)]
                
                command = f"S,{leg},{servo},{angle:.1f};"
                
                if not self.simulation_mode:
                    self.arduino.write(command.encode())
                    time.sleep(min(move_time, 0.01))  # Limit minimum delay
                else:
                    print(f"Simulated command: {command}")
                
                self.last_sent_angles[leg][servo] = angle

    def stabilize(self, rotation_matrix):
        """
        Main stabilization function
        """
        # Compute required angles
        servo_angles = self.compute_leg_adjustments(rotation_matrix)
        
        # Send commands to servos
        self.send_servo_commands(servo_angles)
        
        return servo_angles  # Return for debugging/monitoring

def stabilize_robot(rotation_matrix):
    """
    Updated stabilize_robot function that uses the QuadrupedStabilizer
    """
    global stabilizer
    
    # Initialize stabilizer if not already done
    if 'stabilizer' not in globals():
        stabilizer = QuadrupedStabilizer()
    
    # Perform stabilization
    angles = stabilizer.stabilize(rotation_matrix)
    
    # Print debug info
    print("\nShoulder angles (degrees):")
    for i, angle in enumerate(angles):
        print(f"Leg {i}: {angle:.1f}°") 