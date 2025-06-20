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
    
    # Create full body mesh for better visualization
    body_vertices = np.array([
        # Top face
        [body_front, body_right, body_top],
        [body_front, body_left, body_top],
        [body_back, body_left, body_top],
        [body_back, body_right, body_top],
        [body_front, body_right, body_top],
        # Connect to bottom face
        [body_front, body_right, body_bottom],
        # Bottom face
        [body_front, body_left, body_bottom],
        [body_back, body_left, body_bottom],
        [body_back, body_right, body_bottom],
        [body_front, body_right, body_bottom],
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

def visualize_rotation(wireframe=False, show_angles=True, show_debug=True, show_ranges=True):
    """
    Create an animated visualization of the rotating robot
    
    Parameters:
    - wireframe: Show robot in wireframe mode
    - show_angles: Display servo angles
    - show_debug: Show debug overlays
    - show_ranges: Show movement ranges
    """
    # Set up the figure with a specific size and DPI for better visibility
    plt.style.use('dark_background')  # Use dark theme for better contrast
    fig = plt.figure(figsize=(12, 8), dpi=100)
    ax = fig.add_subplot(111, projection='3d')
    
    # Configure the axes for better visibility
    ax.set_facecolor((0.1, 0.1, 0.1))  # Dark background
    ax.grid(True, linestyle='--', alpha=0.3)
    
    # Set initial view limits based on robot dimensions
    ax.set_xlim([-20, 20])
    ax.set_ylim([-20, 20])
    ax.set_zlim([-20, 20])
    
    # Set initial viewing angle
    ax.view_init(elev=25, azim=45)
    
    # Get robot dimensions
    dims = create_robot_body()
    
    # Create animation with adjusted interval for smoother motion
    anim = FuncAnimation(
        fig, update,
        fargs=(ax, wireframe, show_angles, show_debug, show_ranges) + dims,
        frames=200,
        interval=50,  # 50ms between frames = 20 FPS
        blit=False,
        init_func=lambda: None  # Prevent clearing the first frame
    )
    
    # Adjust layout to prevent text cutoff
    plt.tight_layout()
    
    # Show the plot
    plt.show()

def draw_movement_ranges(ax, mount_point, rotation_matrix, x_range, y_range, z_range, color):
    """Draw movement range box for leg"""
    # Create range box vertices
    points = np.array([
        [x_range[0], y_range[0], z_range[0]],
        [x_range[1], y_range[0], z_range[0]],
        [x_range[1], y_range[1], z_range[0]],
        [x_range[0], y_range[1], z_range[0]],
        [x_range[0], y_range[0], z_range[0]],
        # Connect to top
        [x_range[0], y_range[0], z_range[1]],
        [x_range[1], y_range[0], z_range[1]],
        [x_range[1], y_range[1], z_range[1]],
        [x_range[0], y_range[1], z_range[1]],
        [x_range[0], y_range[0], z_range[1]]
    ])
    
    # Transform points
    transformed = mount_point + np.dot(rotation_matrix, points.T).T
    
    # Draw range box
    ax.plot(transformed[:,0], transformed[:,1], transformed[:,2],
            color=color, linestyle='--', alpha=0.3)

def draw_servo_angles(ax, position, angles, color, size=0.8):
    """Draw servo angle visualization"""
    # Draw angle arcs for each servo
    for i, angle in enumerate(angles):
        # Create arc points
        t = np.linspace(0, angle, 20)
        radius = size * (3 - i) * 0.5  # Different radius for each servo
        x = radius * np.cos(t)
        y = radius * np.sin(t)
        z = np.zeros_like(t)
        points = np.vstack((x, y, z)).T
        
        # Draw arc
        ax.plot(points[:,0] + position[0],
               points[:,1] + position[1],
               points[:,2] + position[2],
               color=color, alpha=0.5)
        
        # Add angle text
        text_pos = position + np.array([
            radius * np.cos(angle/2),
            radius * np.sin(angle/2),
            0
        ])
        ax.text(text_pos[0], text_pos[1], text_pos[2],
                f'{np.degrees(angle):.1f}°',
                color=color, fontsize=8)

def draw_debug_info(ax, position, target, actual, color):
    """Draw debug information overlays"""
    # Draw line to target
    ax.plot([position[0], target[0]],
            [position[1], target[1]],
            [position[2], target[2]],
            color=color, linestyle=':', alpha=0.5)
    
    # Draw error vector
    error = actual - target
    ax.quiver(target[0], target[1], target[2],
              error[0], error[1], error[2],
              color='r', alpha=0.3)
    
    # Add error magnitude text
    error_mag = np.linalg.norm(error)
    ax.text(target[0], target[1], target[2],
            f'{error_mag:.1f}mm',
            color='r', fontsize=8)

def update(frame, ax, wireframe, show_angles, show_debug, show_ranges, 
           body_vertices, leg_mounts, coxa_length, femur_length, 
           real_femur_length, tibia_length, focus_dist):
    """Update function for animation"""
    ax.cla()
    
    # Set consistent view properties
    ax.set_xlim([-20, 20])
    ax.set_ylim([-20, 20])
    ax.set_zlim([-20, 20])
    ax.grid(True, linestyle='--', alpha=0.3)
    ax.set_xlabel('X (cm)', labelpad=10)
    ax.set_ylabel('Y (cm)', labelpad=10)
    ax.set_zlabel('Z (cm)', labelpad=10)
    
    # Generate new orientation data
    pitch, roll, yaw = generate_test_data(frame * 0.1)
    rotation_matrix = compute_rotation_matrix(pitch, roll, yaw)
    
    # Apply rotation to body
    rotated_body = np.dot(body_vertices, rotation_matrix.T)
    rotated_mounts = np.dot(leg_mounts, rotation_matrix.T)
    
    # Plot robot body
    if wireframe:
        ax.plot(rotated_body[:,0], rotated_body[:,1], rotated_body[:,2], 
                'w-', linewidth=1.5, alpha=0.8, label='Robot Body')
    else:
        ax.plot(rotated_body[:,0], rotated_body[:,1], rotated_body[:,2], 
                'w-', linewidth=2, label='Robot Body')
        # Add faces
        faces = [
            rotated_body[:4],  # Top face
            rotated_body[5:9], # Bottom face
            np.array([rotated_body[0], rotated_body[1], rotated_body[6], rotated_body[5]]),  # Front face
            np.array([rotated_body[1], rotated_body[2], rotated_body[7], rotated_body[6]]),  # Left face
            np.array([rotated_body[2], rotated_body[3], rotated_body[8], rotated_body[7]]),  # Back face
            np.array([rotated_body[3], rotated_body[0], rotated_body[5], rotated_body[8]])   # Right face
        ]
        for face in faces:
            ax.fill(face[:,0], face[:,1], face[:,2], 
                   color='w', alpha=0.2)
    
    # Plot legs with brighter colors
    colors = ['#FF4444', '#4444FF', '#FFFF44', '#FF44FF']  # Bright red, blue, yellow, magenta
    leg_names = ['Front Right', 'Front Left', 'Back Right', 'Back Left']
    
    # Enhanced servo visualization parameters
    servo_size = 100
    servo_alpha = 0.8  # Increased alpha
    joint_sizes = [100, 80, 60]  # Increased sizes for better visibility
    
    # Movement ranges
    x_range = (-5, 5)    # Forward/backward
    y_range = (-3, 3)    # Left/right
    z_range = (-18, -12) # Height
    
    for i, (mount, color, name) in enumerate(zip(rotated_mounts, colors, leg_names)):
        # Draw movement ranges if enabled
        if show_ranges:
            draw_movement_ranges(ax, mount, rotation_matrix, x_range, y_range, z_range, color)
        
        # Draw servo housing
        ax.scatter(mount[0], mount[1], mount[2], 
                  c=color, marker='s', s=servo_size, alpha=servo_alpha,
                  label=f'{name} Servos')
        
        # Calculate target point with more realistic motion
        phase = frame * 0.1 + i * np.pi/2  # Different phase for each leg
        base_height = -15
        
        # Add walking-like motion
        if i < 2:  # Front legs
            x_offset = 3 * np.sin(phase)
            y_offset = 1.5 * np.cos(phase)
            z_offset = -1 * abs(np.sin(phase))  # Lift leg during forward motion
        else:  # Back legs
            x_offset = 3 * np.sin(phase + np.pi)  # Opposite phase
            y_offset = 1.5 * np.cos(phase + np.pi)
            z_offset = -1 * abs(np.sin(phase + np.pi))
        
        target_point = np.array([x_offset, y_offset, base_height + z_offset])
        
        # Calculate leg positions
        coxa_pos, femur_pos, tibia_pos = calculate_leg_position(
            mount, rotation_matrix, target_point,
            coxa_length, femur_length, real_femur_length, tibia_length, focus_dist)
        
        # Draw leg segments
        segments = [
            (mount, coxa_pos, 'solid', 3),
            (coxa_pos, femur_pos, 'solid', 2),
            (femur_pos, tibia_pos, 'solid', 2)
        ]
        
        for (start, end, style, width) in segments:
            ax.plot([start[0], end[0]], 
                   [start[1], end[1]], 
                   [start[2], end[2]], 
                   color=color, linestyle=style, linewidth=width)
        
        # Draw joints
        positions = [coxa_pos, femur_pos, tibia_pos]
        markers = ['o', 'o', '*']
        
        for pos, size, marker in zip(positions, joint_sizes, markers):
            ax.scatter(pos[0], pos[1], pos[2], 
                      c=color, marker=marker, s=size, alpha=0.8)
        
        # Calculate angles
        angles = calculate_leg_angles(target_point, coxa_length, femur_length,
                                   real_femur_length, tibia_length, focus_dist)
        
        # Draw servo angles if enabled
        if show_angles:
            draw_servo_angles(ax, mount, angles, color)
        
        # Draw debug overlays if enabled
        if show_debug:
            # Transform target point to global frame
            global_target = mount + np.dot(rotation_matrix, target_point)
            draw_debug_info(ax, mount, global_target, tibia_pos, color)
    
    # Enhanced IMU visualization
    origin = np.array([0, 0, 4])
    axis_length = 8  # Increased length
    axes_colors = ['#FF4444', '#44FF44', '#4444FF']  # Bright RGB colors
    axes_labels = ['X', 'Y', 'Z']
    
    # Draw IMU housing with more detail
    imu_size = 2.0  # Increased size
    imu_height = 0.8  # Increased height
    imu_points = np.array([
        # Bottom face
        [imu_size, imu_size, 0],
        [imu_size, -imu_size, 0],
        [-imu_size, -imu_size, 0],
        [-imu_size, imu_size, 0],
        [imu_size, imu_size, 0],
        # Connect to top face
        [imu_size, imu_size, imu_height],
        [imu_size, -imu_size, imu_height],
        [-imu_size, -imu_size, imu_height],
        [-imu_size, imu_size, imu_height],
        [imu_size, imu_size, imu_height]
    ]) + origin
    
    rotated_imu = np.dot(imu_points, rotation_matrix.T)
    ax.plot(rotated_imu[:,0], rotated_imu[:,1], rotated_imu[:,2],
            'w-', linewidth=1.5, alpha=0.8)
    
    # Draw IMU axes with enhanced arrows
    for i, (color, label) in enumerate(zip(axes_colors, axes_labels)):
        axis = np.zeros((2, 3))
        axis[1, i] = axis_length
        rotated_axis = np.dot(axis, rotation_matrix.T) + origin
        
        # Draw axis line
        ax.plot(rotated_axis[:,0], rotated_axis[:,1], rotated_axis[:,2],
                color=color, linewidth=2.5, label=f'IMU {label}-axis')
        
        # Add arrow head
        arrow_length = axis_length * 0.2
        arrow_pos = rotated_axis[1] - rotated_axis[0]
        ax.quiver(rotated_axis[1,0], rotated_axis[1,1], rotated_axis[1,2],
                 arrow_pos[0], arrow_pos[1], arrow_pos[2],
                 color=color, length=arrow_length, normalize=True,
                 arrow_length_ratio=0.3)
        
        # Add angle text for each axis with better visibility
        euler_angles = [pitch, roll, yaw]
        ax.text(rotated_axis[1,0], rotated_axis[1,1], rotated_axis[1,2],
                f'{label}: {np.degrees(euler_angles[i]):.1f}°',
                color=color, fontsize=10, weight='bold')
    
    # Add stabilization info to title
    ax.set_title(f'Pitch: {pitch:.1f}°, Roll: {roll:.1f}°, Yaw: {yaw:.1f}°\n'
                 f'Stabilization Active - IMU Update Rate: 100Hz')
    
    # Add legend with better positioning
    ax.legend(bbox_to_anchor=(1.15, 1), loc='upper right')

def test_stabilization():
    print("Starting IMU stabilization test with visualization...")
    print("Available visualization options:")
    print("- Wireframe mode (--wireframe)")
    print("- Servo angles display (--angles)")
    print("- Debug overlays (--debug)")
    print("- Movement ranges (--ranges)")
    try:
        visualize_rotation(
            wireframe=False,  # Set to True for wireframe mode
            show_angles=True, # Show servo angles
            show_debug=True,  # Show debug overlays
            show_ranges=True  # Show movement ranges
        )
    except KeyboardInterrupt:
        print("\nTest stopped by user")

if __name__ == "__main__":
    test_stabilization() 