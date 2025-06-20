import numpy as np
import serial
import time
from math import cos, sin, acos, atan2, pi, sqrt
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
        
        # Robot dimensions (in mm)
        self.BODY_LENGTH = 274  # Length between front and back legs
        self.BODY_WIDTH = 140   # Width between left and right legs
        self.LEG_LENGTH = 193   # Length of each leg segment
        
        # Servo limits (in degrees)
        self.SERVO_MIN = 0
        self.SERVO_MAX = 270  # DS3225 has 270° range
        
        # Default standing height
        self.DEFAULT_HEIGHT = 125  # mm from body to ground
        
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
            except serial.SerialException as e:
                print(f"Warning: Could not connect to Arduino: {e}")
                self.simulation_mode = True
                print("Falling back to simulation mode")
                self.init_visualization()
        else:
            print("Running in simulation mode - servo commands will be printed")

    def init_visualization(self):
        """Initialize 3D visualization"""
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        plt.ion()  # Enable interactive mode
        self.fig.show()

    def update_visualization(self, rotation_matrix, leg_positions_3d):
        """Update the 3D visualization"""
        self.ax.cla()  # Clear the current plot
        
        # Set labels and title
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Z (mm)')
        self.ax.set_title('Botzo Robot Stabilization')
        
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
        self.ax.plot(rotated_body[:,0], rotated_body[:,1], rotated_body[:,2], 'b-', linewidth=2, label='Robot Body')
        
        # Draw legs
        colors = ['r', 'g', 'b', 'y']
        leg_names = ['Front Left', 'Front Right', 'Back Left', 'Back Right']
        for i, (leg_pos, color, name) in enumerate(zip(leg_positions_3d, colors, leg_names)):
            # Draw line from body to foot
            x = [rotated_body[i,0], leg_pos[0]]
            y = [rotated_body[i,1], leg_pos[1]]
            z = [rotated_body[i,2], leg_pos[2]]
            self.ax.plot(x, y, z, f'{color}-', linewidth=2, label=name)
            # Draw foot point
            self.ax.scatter(leg_pos[0], leg_pos[1], leg_pos[2], c=color, marker='o', s=100)
        
        # Add coordinate frame
        origin = np.zeros(3)
        axis_length = 100
        axes_colors = ['r', 'g', 'b']
        axes_labels = ['X', 'Y', 'Z']
        for i, (color, label) in enumerate(zip(axes_colors, axes_labels)):
            axis = np.zeros((2, 3))
            axis[1, i] = axis_length
            rotated_axis = np.dot(axis, rotation_matrix.T)
            self.ax.plot(rotated_axis[:,0], rotated_axis[:,1], rotated_axis[:,2], 
                        f'{color}-', linewidth=1, label=f'{label}-axis')
        
        self.ax.legend()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def inverse_kinematics(self, x, y, z):
        """
        Calculate servo angles for a given leg position
        Returns: (shoulder_angle, knee_angle, ankle_angle)
        """
        # Calculate leg plane angle
        theta_base = atan2(y, x)
        
        # Calculate distance in leg plane
        L = sqrt(x*x + y*y)
        
        # Use law of cosines for knee angle
        L2 = L*L + z*z
        knee_angle = pi - acos(
            (2 * self.LEG_LENGTH * self.LEG_LENGTH - L2) / 
            (2 * self.LEG_LENGTH * self.LEG_LENGTH)
        )
        
        # Calculate ankle angle
        ankle_angle = atan2(z, L) + acos(
            (L2 + self.LEG_LENGTH * self.LEG_LENGTH - self.LEG_LENGTH * self.LEG_LENGTH) /
            (2 * self.LEG_LENGTH * sqrt(L2))
        )
        
        # Convert to degrees
        return (
            theta_base * 180/pi,
            knee_angle * 180/pi,
            ankle_angle * 180/pi
        )

    def compute_leg_adjustments(self, rotation_matrix):
        """
        Convert rotation matrix to leg height adjustments
        """
        adjustments = []
        leg_positions_3d = []
        for leg_pos in self.leg_positions:
            # Transform leg position by rotation matrix
            x, y = leg_pos
            rotated_point = rotation_matrix.dot([x, y, self.DEFAULT_HEIGHT])
            
            # Calculate new position
            new_x = rotated_point[0]
            new_y = rotated_point[1]
            new_z = rotated_point[2]
            
            # Store 3D position for visualization
            leg_positions_3d.append([new_x, new_y, new_z])
            
            # Calculate servo angles
            angles = self.inverse_kinematics(new_x, new_y, new_z)
            adjustments.append(angles)
        
        # Update visualization if in simulation mode
        if self.simulation_mode:
            self.update_visualization(rotation_matrix, leg_positions_3d)
        
        return adjustments

    def send_servo_commands(self, servo_angles):
        """
        Send servo angles to Arduino or simulate the commands
        """
        for leg in range(4):
            for servo in range(3):
                angle = servo_angles[leg][servo]
                # Constrain angle to valid range
                angle = max(self.SERVO_MIN, min(self.SERVO_MAX, angle))
                command = f"S,{leg},{servo},{angle:.1f};"
                
                if not self.simulation_mode:
                    self.arduino.write(command.encode())
                    time.sleep(0.01)  # Small delay between commands
                else:
                    # In simulation mode, just print the commands
                    print(f"Simulated command: {command}")

    def stabilize(self, rotation_matrix):
        """
        Main stabilization function
        """
        # Compute required leg adjustments
        leg_adjustments = self.compute_leg_adjustments(rotation_matrix)
        
        # Send commands to servos
        self.send_servo_commands(leg_adjustments)
        
        return leg_adjustments  # Return for debugging/monitoring

def stabilize_robot(rotation_matrix):
    """
    Updated stabilize_robot function that uses the QuadrupedStabilizer
    """
    global stabilizer
    
    # Initialize stabilizer if not already done
    if 'stabilizer' not in globals():
        stabilizer = QuadrupedStabilizer()
    
    # Perform stabilization
    adjustments = stabilizer.stabilize(rotation_matrix)
    
    # Print debug info
    print("\nLeg adjustments (degrees):")
    for i, adj in enumerate(adjustments):
        print(f"Leg {i}: Shoulder={adj[0]:.1f}°, Knee={adj[1]:.1f}°, Ankle={adj[2]:.1f}°") 