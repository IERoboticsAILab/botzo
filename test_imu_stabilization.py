import numpy as np
import time
from imu_stabilization import compute_rotation_matrix
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

def generate_test_data(t):
    """
    Generate synthetic IMU data that simulates a simple motion pattern
    t: time parameter to create varying angles
    """
    # Simulate oscillating motion
    pitch = 20 * np.sin(0.5 * t)  # Oscillate between -20 and 20 degrees
    roll = 15 * np.cos(0.3 * t)   # Oscillate between -15 and 15 degrees
    yaw = 10 * np.sin(0.2 * t)    # Oscillate between -10 and 10 degrees
    
    return pitch, roll, yaw

def create_robot_body():
    """Create vertices for robot body and legs using Botzo's actual dimensions"""
    # Robot dimensions (in cm)
    BODY_LENGTH = 27.4  # cm
    BODY_WIDTH = 14.0   # cm
    BODY_HEIGHT = 8.0   # cm
    
    # Leg segment lengths (in cm)
    COXA_LENGTH = 3.1        # A
    FEMUR_LENGTH = 9.5       # E
    REAL_FEMUR_LENGTH = 9.1  # E'
    TIBIA_LENGTH = 9.8       # F
    FOCUS_DIST = 2.8         # Distance from focus point to servo
    
    # Body vertices (main housing)
    body_front = BODY_LENGTH/2
    body_back = -BODY_LENGTH/2
    body_right = BODY_WIDTH/2
    body_left = -BODY_WIDTH/2
    body_top = BODY_HEIGHT/2
    body_bottom = -BODY_HEIGHT/2
    
    body_vertices = np.array([
        # Top face
        [body_front, body_right, body_top],
        [body_front, body_left, body_top],
        [body_back, body_left, body_top],
        [body_back, body_right, body_top],
        [body_front, body_right, body_top],  # Close the loop
    ])
    
    # Leg mounting points
    leg_mounts = np.array([
        [body_front, body_right, body_bottom],  # Front Right
        [body_front, body_left, body_bottom],   # Front Left
        [body_back, body_right, body_bottom],   # Back Right
        [body_back, body_left, body_bottom],    # Back Left
    ])
    
    return (body_vertices, leg_mounts, COXA_LENGTH, FEMUR_LENGTH, 
            REAL_FEMUR_LENGTH, TIBIA_LENGTH, FOCUS_DIST)

def calculate_leg_angles(target_point, coxa_length, femur_length, real_femur_length, 
                        tibia_length, focus_dist):
    """Calculate leg angles using Botzo's IK equations"""
    x, y, z = target_point
    
    # 1. Distance Calculation
    D = np.sqrt(z**2 + y**2 - coxa_length**2)
    
    # 2. G Calculation
    G = np.sqrt(D**2 + x**2)
    
    # 3. Knee Angle
    phi = np.arccos((G**2 - femur_length**2 - tibia_length**2) / 
                    (-2 * femur_length * tibia_length))
    
    # 4. Shoulder Angle
    theta_shoulder = np.arctan2(x, D) + np.arcsin((tibia_length * np.sin(phi)) / G)
    
    # 5. Coxa Angle
    psi = np.arctan2(y, z) + np.arctan2(D, coxa_length)
    
    # Angle Adjustments
    adjustment = np.arccos((real_femur_length**2 + femur_length**2 - focus_dist**2) /
                          (2 * real_femur_length * femur_length))
    
    theta_femur = np.pi/2 - (theta_shoulder + adjustment)
    theta_tibia = np.pi - phi + adjustment + theta_femur
    
    return psi, theta_femur, theta_tibia

def calculate_leg_position(mount_point, rotation_matrix, target_point, 
                         coxa_length, femur_length, real_femur_length, 
                         tibia_length, focus_dist):
    """Calculate actual leg segment positions"""
    # Get angles from IK
    coxa_angle, femur_angle, tibia_angle = calculate_leg_angles(
        target_point, coxa_length, femur_length, real_femur_length, 
        tibia_length, focus_dist)
    
    # Calculate positions in leg's local frame
    coxa_end = np.array([
        coxa_length * np.cos(coxa_angle),
        coxa_length * np.sin(coxa_angle),
        0
    ])
    
    femur_end = coxa_end + np.array([
        real_femur_length * np.cos(femur_angle) * np.cos(coxa_angle),
        real_femur_length * np.cos(femur_angle) * np.sin(coxa_angle),
        real_femur_length * np.sin(femur_angle)
    ])
    
    tibia_end = femur_end + np.array([
        tibia_length * np.cos(tibia_angle) * np.cos(coxa_angle),
        tibia_length * np.cos(tibia_angle) * np.sin(coxa_angle),
        tibia_length * np.sin(tibia_angle)
    ])
    
    # Transform to global frame
    coxa_global = mount_point + np.dot(rotation_matrix, coxa_end)
    femur_global = mount_point + np.dot(rotation_matrix, femur_end)
    tibia_global = mount_point + np.dot(rotation_matrix, tibia_end)
    
    return coxa_global, femur_global, tibia_global

def update(frame, ax, body_vertices, leg_mounts, coxa_length, femur_length, 
           real_femur_length, tibia_length, focus_dist):
    """Update function for animation"""
    ax.cla()
    
    # Generate new orientation data
    pitch, roll, yaw = generate_test_data(frame * 0.1)
    rotation_matrix = compute_rotation_matrix(pitch, roll, yaw)
    
    # Apply rotation to body
    rotated_body = np.dot(body_vertices, rotation_matrix.T)
    rotated_mounts = np.dot(leg_mounts, rotation_matrix.T)
    
    # Plot robot body
    ax.plot(rotated_body[:,0], rotated_body[:,1], rotated_body[:,2], 
            'g-', linewidth=2, label='Robot Body')
    
    # Plot legs
    colors = ['r', 'b', 'y', 'm']
    leg_names = ['Front Right', 'Front Left', 'Back Right', 'Back Left']
    
    # Enhanced servo visualization parameters
    servo_size = 100
    servo_alpha = 0.6
    joint_size = 50
    
    for i, (mount, color, name) in enumerate(zip(rotated_mounts, colors, leg_names)):
        # Draw servo housing at mount point (cube-like marker)
        ax.scatter(mount[0], mount[1], mount[2], 
                  c=color, marker='s', s=servo_size, alpha=servo_alpha,
                  label=f'{name} Servo')
        
        # Default target point with adjusted ranges based on leg position
        base_height = -15  # Base height in cm
        x_offset = 2 * np.sin(frame * 0.1)  # Small x oscillation
        y_offset = 2 * np.cos(frame * 0.1)  # Small y oscillation
        z_offset = -2 * np.sin(frame * 0.05)  # Height adjustment
        
        # Adjust target point based on leg position and stabilization
        if i == 0 or i == 1:  # Front legs
            x_offset *= 1.2  # Larger range for front legs
        if i % 2 == 0:  # Right side legs
            y_offset *= -1  # Mirror y movement
            
        target_point = np.array([x_offset, y_offset, base_height + z_offset])
        
        # Calculate leg positions
        coxa_pos, femur_pos, tibia_pos = calculate_leg_position(
            mount, rotation_matrix, target_point,
            coxa_length, femur_length, real_femur_length, tibia_length, focus_dist)
        
        # Draw leg segments with gradient colors
        segments = [
            (mount, coxa_pos, 'solid', 3),      # Coxa segment (thicker)
            (coxa_pos, femur_pos, 'solid', 2),  # Femur segment
            (femur_pos, tibia_pos, 'solid', 2)  # Tibia segment
        ]
        
        for (start, end, style, width) in segments:
            ax.plot([start[0], end[0]], 
                   [start[1], end[1]], 
                   [start[2], end[2]], 
                   color=color, linestyle=style, linewidth=width)
        
        # Draw joints with different sizes
        joint_positions = [coxa_pos, femur_pos, tibia_pos]
        joint_sizes = [joint_size * 1.2, joint_size, joint_size * 0.8]  # Decreasing sizes
        
        for pos, size in zip(joint_positions, joint_sizes):
            ax.scatter(pos[0], pos[1], pos[2], 
                      c=color, marker='o', s=size, alpha=0.8)
        
        # Draw foot contact point
        ax.scatter(tibia_pos[0], tibia_pos[1], tibia_pos[2],
                  c=color, marker='*', s=joint_size*1.5)
    
    # Enhanced IMU visualization
    origin = np.array([0, 0, 4])  # 4cm above body center
    axis_length = 5  # 5cm axes
    axes_colors = ['r', 'g', 'b']
    axes_labels = ['X', 'Y', 'Z']
    
    # Draw IMU housing
    imu_size = 1.5  # cm
    imu_points = np.array([
        [imu_size, imu_size, imu_size],
        [imu_size, -imu_size, imu_size],
        [-imu_size, -imu_size, imu_size],
        [-imu_size, imu_size, imu_size],
        [imu_size, imu_size, imu_size],
    ]) + origin
    
    rotated_imu = np.dot(imu_points, rotation_matrix.T)
    ax.plot(rotated_imu[:,0], rotated_imu[:,1], rotated_imu[:,2],
            'k-', linewidth=1, alpha=0.5)
    
    # Draw IMU axes with arrows
    for i, (color, label) in enumerate(zip(axes_colors, axes_labels)):
        axis = np.zeros((2, 3))
        axis[1, i] = axis_length
        rotated_axis = np.dot(axis, rotation_matrix.T) + origin
        
        # Draw axis line
        ax.plot(rotated_axis[:,0], rotated_axis[:,1], rotated_axis[:,2],
                color=color, linewidth=2, label=f'IMU {label}-axis')
        
        # Add arrow head
        arrow_length = axis_length * 0.2
        arrow_pos = rotated_axis[1] - rotated_axis[0]
        ax.quiver(rotated_axis[1,0], rotated_axis[1,1], rotated_axis[1,2],
                 arrow_pos[0], arrow_pos[1], arrow_pos[2],
                 color=color, length=arrow_length, normalize=True)
    
    # Add stabilization info to title
    ax.set_title(f'Pitch: {pitch:.1f}°, Roll: {roll:.1f}°, Yaw: {yaw:.1f}°\n'
                 f'Stabilization Active - IMU Update Rate: 100Hz')
    
    # Set view limits
    limit = 30  # 30cm view limit
    ax.set_xlim([-limit, limit])
    ax.set_ylim([-limit, limit])
    ax.set_zlim([-limit, limit])
    ax.set_xlabel('X (cm)')
    ax.set_ylabel('Y (cm)')
    ax.set_zlabel('Z (cm)')
    
    # Add grid for better depth perception
    ax.grid(True, alpha=0.3)
    
    # Optimize viewing angle
    ax.view_init(elev=30, azim=45)
    
    ax.legend(bbox_to_anchor=(1.15, 1), loc='upper right')

def visualize_rotation():
    """Create an animated visualization of the rotating robot"""
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Get robot dimensions
    dims = create_robot_body()
    
    # Create animation
    anim = FuncAnimation(
        fig, update,
        fargs=(ax,) + dims,
        frames=200,
        interval=50,
        blit=False
    )
    
    plt.show()

def test_stabilization():
    print("Starting IMU stabilization test with visualization...")
    try:
        visualize_rotation()
    except KeyboardInterrupt:
        print("\nTest stopped by user")

if __name__ == "__main__":
    test_stabilization() 