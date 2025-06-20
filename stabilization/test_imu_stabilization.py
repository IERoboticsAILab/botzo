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

def create_cube():
    """Create vertices and edges for a simple cube"""
    # Define the vertices of the cube
    vertices = np.array([
        [1, 1, 1], [1, 1, -1], [1, -1, 1], [1, -1, -1],
        [-1, 1, 1], [-1, 1, -1], [-1, -1, 1], [-1, -1, -1]
    ])
    
    # Define the edges connecting the vertices
    edges = [
        [0, 1], [0, 2], [0, 4], [1, 3], [1, 5], [2, 3],
        [2, 6], [3, 7], [4, 5], [4, 6], [5, 7], [6, 7]
    ]
    
    return vertices, edges

def update(frame, ax, vertices, edges):
    """Update function for animation"""
    ax.cla()  # Clear the current plot
    
    # Generate new orientation data
    pitch, roll, yaw = generate_test_data(frame * 0.1)
    rotation_matrix = compute_rotation_matrix(pitch, roll, yaw)
    
    # Apply rotation to vertices
    rotated_vertices = np.dot(vertices, rotation_matrix.T)
    
    # Plot the rotated cube
    ax.set_title(f'Pitch: {pitch:.1f}°, Roll: {roll:.1f}°, Yaw: {yaw:.1f}°')
    
    # Plot edges
    for edge in edges:
        x = [rotated_vertices[edge[0], 0], rotated_vertices[edge[1], 0]]
        y = [rotated_vertices[edge[0], 1], rotated_vertices[edge[1], 1]]
        z = [rotated_vertices[edge[0], 2], rotated_vertices[edge[1], 2]]
        ax.plot(x, y, z, 'b')
    
    # Set consistent axis limits
    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([-2, 2])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    # Add orientation indicators
    ax.plot([0, 2], [0, 0], [0, 0], 'r', label='X-axis')
    ax.plot([0, 0], [0, 2], [0, 0], 'g', label='Y-axis')
    ax.plot([0, 0], [0, 0], [0, 2], 'b', label='Z-axis')
    ax.legend()

def visualize_rotation():
    """Create an animated visualization of the rotating cube"""
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    vertices, edges = create_cube()
    
    # Create animation
    anim = FuncAnimation(
        fig, update,
        fargs=(ax, vertices, edges),
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