import pybullet as p
import time
import pybullet_data
import math
from src.IK_solver import *

joint_ids = {
    "BR": {
        "shoulder": 4,
        "femur": 7,
        "knee": 8,
        "foot": 9
    },
    "FR": {
        "shoulder": 13,
        "femur": 16,
        "knee": 17,
        "foot": 18
    },
    "BL": {
        "shoulder": 24,
        "femur": 27,
        "knee": 28,
        "foot": 29
    },
    "FL": {
        "shoulder": 33,
        "femur": 36,
        "knee": 37,
        "foot": 38
    }
}

# Connect to PyBullet
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Optionally
p.setGravity(0, 0, -9.8)

# Load the plane and your URDF
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0, 0, 2]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF("../urdf/final_URDF.urdf", cubeStartPos, cubeStartOrientation,
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
        ) # setJointMotorControl2

        foot_side = "BR"
        foot_state = p.getLinkState(robotId, joint_ids[foot_side]["foot"])
        foot_position = foot_state[0]  # (x, y, z) coordinates of the foot
        print(f"{foot_side} Foot Position: {foot_position}")

        # Step the simulation and wait for a short time to observe movement
        for _ in range(100):  # Let the movement settle
            p.stepSimulation()
            time.sleep(1 / 240)
        time.sleep(0.5)  # Pause before switching to the next point

except KeyboardInterrupt:
    print("Simulation terminated.")

# Disconnect from PyBullet
p.disconnect()