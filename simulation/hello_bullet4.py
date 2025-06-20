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
robotId = p.loadURDF("urdf/final_URDF.urdf", cubeStartPos, cubeStartOrientation,
                     flags=p.URDF_USE_INERTIA_FROM_FILE, globalScaling=10)

# Run the simulation loop indefinitely
try:
    while True:
        # x,y,z for IK BR
        x, y, z = list(map(int, input("Enter the x, y, z coordinates for the foot of the robot: ").split()))
        s_f_t = legIK(x, y, z)
        angle_shoulder, angle_femur, angle_knee = s_f_t
        target_angle_shoulder = math.radians(real_sim_angle(angle_shoulder,joint_ids["BR"]["shoulder"]))
        target_angle_femur = math.radians(real_sim_angle(angle_femur, joint_ids["BR"]["femur"]))
        target_angle_knee = math.radians(real_sim_angle(angle_knee, joint_ids["BR"]["knee"]))
        print(f"\n\nTarget coordinate: {x,y,z} \nShoulder: {angle_shoulder}, Femur: {angle_femur}, Knee: {angle_knee}")

        # Move shoulder
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["BR"]["shoulder"], controlMode=p.POSITION_CONTROL, targetPosition=target_angle_shoulder, force=500)
        # Move femur
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["BR"]["femur"], controlMode=p.POSITION_CONTROL, targetPosition=target_angle_femur, force=500)
        # Move knee
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["BR"]["knee"], controlMode=p.POSITION_CONTROL, targetPosition=target_angle_knee, force=500)

        # Get the state of the foot
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