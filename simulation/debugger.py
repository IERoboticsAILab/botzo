import pybullet as p
import time
import pybullet_data
import math
from src.IK_solver import *

# Connect to PyBullet
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Optionally
p.setGravity(0, 0, -9.8)

# Load the plane and your URDF
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0, 0, 2]
cubeStartOrientation = p.getQuaternionFromEuler([math.pi, 0, 0]) # rotate the robot of 180 degrees around the x axis
robotId = p.loadURDF("../CAD_files/URDF/BOTZO_URDF_description/urdf/BOTZO_URDF.urdf", cubeStartPos, cubeStartOrientation,
                     flags=p.URDF_USE_INERTIA_FROM_FILE, globalScaling=10)

# Run the simulation loop indefinitely
try:
    while True:
        # Move joint 36 (FL_femur_joint) to 45 degrees
        joint_index = int(input("joint index:"))  # Joint index (36 in this case)
        target_angle_deg = float(input("desire angle:"))  # Target angle in degrees
        sim_angle = real_sim_angle(target_angle_deg, joint_index)
        target_angle_rad = math.radians(sim_angle)  # Convert to radians

        # Use position control to move the joint
        p.setJointMotorControl2(
            bodyUniqueId=robotId,                # Robot ID
            jointIndex=joint_index,              # Joint index (36 in this case)
            controlMode=p.POSITION_CONTROL,      # Use position control
            targetPosition=target_angle_rad,     # Target position in radians
            force=500                            # Max force applied to reach the target
            # 25 kg·cm = 25 × 9.81 × 0.01 = ~2.45 N·m
        ) # setJointMotorControl2

        for i in range(240):  # Simulate for 1 second (240 steps at 240 Hz)
            p.stepSimulation()
            time.sleep(1./240.)
            # Use position control to move the joint
            p.setJointMotorControl2(
                bodyUniqueId=robotId,                # Robot ID
                jointIndex=joint_index,              # Joint index (36 in this case)
                controlMode=p.POSITION_CONTROL,      # Use position control
                targetPosition=target_angle_rad,     # Target position in radians
                force=500                            # Max force applied to reach the target
            )

except KeyboardInterrupt:
    print("Simulation terminated.")

# Disconnect from PyBullet
p.disconnect()