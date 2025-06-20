import time
import numpy as np
import math
import csv
import os
import pybullet as pb
import pybullet_data
import controls
import random
from robot import Quadrupedal
from multiprocessing import Process, Manager, Queue
from queue import Empty
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

def generate_oval_trajectory(center, x_radius, z_radius, angle, jitter_frequency=0.03, jitter_magnitude=0.01):
    """
    Generate a point on an oval trajectory with random jitter applied occasionally to simulate a more realistic movement.

    :param center: Center point of the oval [x, z]
    :param x_radius: Radius along x-axis
    :param z_radius: Radius along z-axis
    :param angle: Current angle (in radians)
    :param jitter_frequency: Probability of applying jitter at each iteration (0.0 to 1.0)
    :param jitter_magnitude: Maximum range for jitter applied to x, y, z
    :return: [x, y, z] coordinates of the point with optional jitter
    """
    # Calculate the basic oval trajectory point
    x = center[0] + x_radius * math.cos(-angle)
    y = center[1]  # Assuming the center's y position doesn't change (static on the ground)
    z = center[2] + z_radius * math.sin(-angle)

    # Randomly decide if jitter should be applied
    if random.random() < jitter_frequency:
        jitter_x = random.uniform(-jitter_magnitude, jitter_magnitude)
        jitter_y = random.uniform(-0.5, 0.5) 
        jitter_z = random.uniform(-jitter_magnitude, jitter_magnitude)

        # Apply the jitter
        x += jitter_x
        y += jitter_y
        z += jitter_z

    if random.random() < jitter_frequency:
        jitter_y = 0.5

        # Apply the jitter
        y += jitter_y

    return x, y, z

def reset_robot(robot, i):
    """
    Resets the robot to its initial position, orientation, and velocities.

    :param robot: An instance of the Quadrupedal class.
    """
    # Reset the base position and orientation
    pb.resetBasePositionAndOrientation(robot._robotId, [0, i, 0.55], [0, 0, 0, 1])
    
    # Reset the base velocity
    pb.resetBaseVelocity(robot._robotId, linearVelocity=[0, 0, 0], angularVelocity=[0, 0, 0])
    
    print("Robot has been reset!")

def run_simulation(robot_id, num_robots, use_gui, coord_queue, rev_status, max_points=500):
    """
    Run an independent simulation for each robot in a separate PyBullet client.
    Log the coordinates of each leg for plotting.
    """
    if use_gui:
        physics_client = pb.connect(pb.GUI)
    else:
        physics_client = pb.connect(pb.DIRECT) # Non-graphical version

    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    pb.setGravity(0, 0, -9.8)

    robots = []

    # Create the robot instances for the specific client
    for i in range(num_robots):
        robot = Quadrupedal(
            timeStep=1./240.,
            initialCoMheight=0.3,
            robotPATH="../assets/haruki_urdf/quadrupedal.urdf",
            startPosition=[0, i, 0.55],
            startOrientation=[0., 0., 0.],
            maxForce=12
        )
        robots.append(robot)

    # Oval trajectory parameters
    oval_center_rf = [0.2, -0.11, -0.2]  # Center of the oval path 
    oval_center_lh = [-0.2, 0.11, -0.2] 
    oval_center_lf = [0.2, 0.11, -0.2] 
    oval_center_rh = [-0.2, -0.11, -0.2]  

    initial_width = 0.1  # Radius along x-axis (horizontal stretch)
    initial_height = 0.01  # Radius along z-axis (vertical stretch)

    width_slider = pb.addUserDebugParameter("Width", 0.01, 0.5, initial_width)
    height_slider = pb.addUserDebugParameter("Height", 0.01, 0.1, initial_height)

    # Angle increment for faster or slower movement
    angle_step_slider = pb.addUserDebugParameter("Angle Step", 0.01, 1.0, 0.05)
    angle = 0.0

    # Logging variables to handle coordinate logging
    coordinates_rf = {
        'id': robot_id,
        'x_coords': [],
        'y_coords': [],
        'z_coords': []
    }

    try:
        while True:
            # Reset robot state if "R" key is pressed
            if controls.is_reset_key_pressed(): 
                for i in range(num_robots):
                    reset_robot(robots[i], i)

            if use_gui:
                # Get the angle step from the slider (how fast the robot moves)
                angle_step = pb.readUserDebugParameter(angle_step_slider)

                # Get the width and height from the sliders
                x_radius = pb.readUserDebugParameter(width_slider)
                z_radius = pb.readUserDebugParameter(height_slider)
            else:
                angle_step = 0.05
                x_radius = 0.1
                z_radius = 0.01

            angle += angle_step
            if angle > 2 * math.pi:
                angle -= 2 * math.pi

                # Set the rev_status to True after the first revolution
                rev_status[robot_id] = True
            
            for qdrp in robots:
                # Diagonal Pair 1: Front Right (RF) and Left Hind (LH)
                RF_x, RF_y, RF_z = generate_oval_trajectory(oval_center_rf, x_radius, z_radius, angle)
                LH_x, LH_y, LH_z = generate_oval_trajectory(oval_center_lh, x_radius, z_radius, angle)
                # Diagonal Pair 2: Front Left (LF) and Right Hind (RH) (phase-offset by π)
                LF_x, LF_y, LF_z = generate_oval_trajectory(oval_center_lf, x_radius, z_radius, angle + math.pi)
                RH_x, RH_y, RH_z = generate_oval_trajectory(oval_center_rh, x_radius, z_radius, angle + math.pi)

                targetPositionRF = np.array([0.2, -0.11, -0.2])  # Right Front
                targetPositionRH = np.array([-0.2, -0.11, -0.2])  # Right Hind
                targetPositionLF = np.array([0.2, 0.11, -0.2])    # Left Front
                targetPositionLH = np.array([-0.2, 0.11, -0.2])   # Left Hind
                
                # Update target positions
                targetPositionRF[0], targetPositionRF[1], targetPositionRF[2] = RF_x, RF_y, RF_z
                targetPositionLH[0], targetPositionLH[1], targetPositionLH[2] = LH_x, LH_y, LH_z
                targetPositionLF[0], targetPositionLF[1], targetPositionLF[2] = LF_x, LF_y, LF_z
                targetPositionRH[0], targetPositionRH[1], targetPositionRH[2] = RH_x, RH_y, RH_z

                # Inverse Kinematics
                RFjointPositions = qdrp.inverseKinematics(targetPositionRF, targetLeg=qdrp.legRF)
                LHjointPositions = qdrp.inverseKinematics(targetPositionLH, targetLeg=qdrp.legLH)
                LFjointPositions = qdrp.inverseKinematics(targetPositionLF, targetLeg=qdrp.legLF)
                RHjointPositions = qdrp.inverseKinematics(targetPositionRH, targetLeg=qdrp.legRH)

                # Set joint positions
                qdrp.legRF.setJointPositions(RFjointPositions)
                qdrp.legLH.setJointPositions(LHjointPositions)
                qdrp.legLF.setJointPositions(LFjointPositions)
                qdrp.legRH.setJointPositions(RHjointPositions)

                # Advance simulation
                qdrp.oneStep()

                # Log coordinates for RF leg
                coordinates_rf['x_coords'].append(RF_x)
                coordinates_rf['y_coords'].append(RF_y)
                coordinates_rf['z_coords'].append(RF_z)

                # Keep only the last max_points
                if len(coordinates_rf['x_coords']) > max_points:
                    coordinates_rf['x_coords'] = coordinates_rf['x_coords'][-max_points:]
                    coordinates_rf['y_coords'] = coordinates_rf['y_coords'][-max_points:]
                    coordinates_rf['z_coords'] = coordinates_rf['z_coords'][-max_points:]

                # Put coordinates in the queue every few iterations
                if len(coordinates_rf['x_coords']) % 10 == 0:
                    try:
                        coord_queue.put(coordinates_rf.copy(), block=False)
                    except Exception:
                        # If queue is full, just pass
                        pass

            time.sleep(1/240.)  # Maintain simulation speed

    except KeyboardInterrupt:
        print(f"Simulation {robot_id} stopped by user.")
    finally:
        pb.disconnect()

def plot_trajectories(coord_queue, num_simulations, max_points=500):
    """
    Plot the trajectories of the robots in real-time using a queue in 3D.
    """

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    lines = []
    points = []

    # Initialize plot elements for each robot
    for robot_id in range(num_simulations):
        line, = ax.plot([], [], [], label=f"Robot {robot_id} RF", lw=1)  # 3D trajectory line
        point, = ax.plot([], [], [], 'ro', markersize=2.5)  # Red dot for the latest point
        lines.append(line)
        points.append(point)

    # Initialize data stores for each robot
    robot_data = {i: {'x_coords': [], 'y_coords': [], 'z_coords': []} for i in range(num_simulations)}

    def update_robot_data():
        """
        Continuously update robot data from the queue
        """
        while True:
            try:
                # Try to get data from the queue without blocking
                data = coord_queue.get(block=False)
                robot_id = data['id']
                
                # Update the data for the specific robot
                robot_data[robot_id]['x_coords'] = data['x_coords']
                robot_data[robot_id]['y_coords'] = data['y_coords']
                robot_data[robot_id]['z_coords'] = data['z_coords']
            except Empty:
                # No data in the queue
                break

    def init():
        # Set axis labels and title
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title("Real-Time 3D Leg Trajectories")
        
        # Set reasonable axis limits
        ax.set_xlim(0.0, 0.4)
        ax.set_ylim(-0.5, 0.5)
        ax.set_zlim(-0.4, 0.0)
        
        ax.legend()
        return lines + points
    
    def update(frame):
        # Update data from queue
        update_robot_data()

        for robot_id in range(num_simulations):
            x_data = robot_data[robot_id].get('x_coords', [])
            y_data = robot_data[robot_id].get('y_coords', [])
            z_data = robot_data[robot_id].get('z_coords', [])

            if x_data and y_data and z_data:  # Check if all lists have data
                lines[robot_id].set_data_3d(x_data, y_data, z_data)  # Update 3D trajectory
                
                # Ensure the last point is a sequence
                last_x = [x_data[-1]] if not isinstance(x_data[-1], (list, np.ndarray)) else x_data[-1]
                last_y = [y_data[-1]] if not isinstance(y_data[-1], (list, np.ndarray)) else y_data[-1]
                last_z = [z_data[-1]] if not isinstance(z_data[-1], (list, np.ndarray)) else z_data[-1]
                
                points[robot_id].set_data_3d(last_x, last_y, last_z)  # Update red dot
            else:
                lines[robot_id].set_data_3d([], [], [])
                points[robot_id].set_data_3d([], [], [])
        return lines + points

    # Create the animation
    ani = FuncAnimation(fig, update, init_func=init, blit=True, interval=50)
    plt.show()


def main():
    num_robots = 1 # Number of robots per simulation
    num_simulations = 3 # Number of parallel simulations

    # Multiprocessing Queue for thread-safe coordinate logging
    coord_queue = Queue(maxsize=1000)  # Limiting queue size to prevent memory issues

    manager = Manager()
    
    # array of booleans to track when one process does its first oval revolution
    rev_status = manager.list([False for _ in range(num_simulations)])

    processes = []

    # Start time and end time for tracking
    start_time = None
    end_time = None

    # Parallel clients for each simulation
    for i in range(num_simulations):
        use_gui = (i == 0) # Only show the GUI for the first simulation
        p = Process(target=run_simulation, args=(i, num_robots, use_gui, coord_queue, rev_status))
        p.start()
        processes.append(p)

    # csv_file = "performance.csv"
    # write_headers = not os.path.exists(csv_file)
    
    try:
        # Monitoring loop for rev_status
        while True:
            # Check if all processes are False (initial state)
            if not any(rev_status) and start_time is None:
                start_time = time.time()  # Record the start time
                print("All processes are initially False. Monitoring started...")

            # Check if all processes are True (final state)
            if all(rev_status):
                if start_time is not None and end_time is None:
                    end_time = time.time()  # Record the end time
                    elapsed_time = end_time - start_time
                    print(f"All processes have completed a full revolution.")
                    print(f"Time elapsed: {elapsed_time:.2f} seconds.")

                    # with open(csv_file, mode='a', newline='') as file:
                    #     writer = csv.writer(file)
                    #     if write_headers:
                    #         writer.writerow(["Paralllel", "Serial"])
                    #         write_headers = False
                    #     writer.writerow([elapsed_time])


                    break  # Exit monitoring loop

        # Start plotting
        plot_trajectories(coord_queue, num_simulations)
    except KeyboardInterrupt:
        print("Plotting stopped by user.")
    finally:
        # Ensure all processes terminate cleanly
        for p in processes:
            p.terminate()
            p.join()

if __name__ == "__main__":
    main()