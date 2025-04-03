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
        for i in range(0,len(forward_targets_FR_BL)):
            print(forward_targets_FR_BL[i])
            print(forward_targets_FL_BR[i])

            # calculate front right shoulder, femur, knee angles
            FR_s_f_t = legIK(forward_targets_FR_BL[i][0], forward_targets_FR_BL[i][1], forward_targets_FR_BL[i][2])
            FR_angle_shoulder, FR_angle_femur, FR_angle_knee = FR_s_f_t
            target_FR_angle_shoulder = math.radians(real_sim_angle(FR_angle_shoulder,joint_ids["FR"]["shoulder"]))
            target_FR_angle_femur = math.radians(real_sim_angle(FR_angle_femur, joint_ids["FR"]["femur"]))
            target_FR_angle_knee = math.radians(real_sim_angle(FR_angle_knee, joint_ids["FR"]["knee"]))
            #------------------------------
            # calculate front left shoulder, femur, knee angles
            FL_s_f_t = legIK(forward_targets_FL_BR[i][0], forward_targets_FL_BR[i][1], forward_targets_FL_BR[i][2])
            FL_angle_shoulder, FL_angle_femur, FL_angle_knee = FL_s_f_t
            target_FL_angle_shoulder = math.radians(real_sim_angle(FL_angle_shoulder,joint_ids["FL"]["shoulder"]))
            target_FL_angle_femur = math.radians(real_sim_angle(FL_angle_femur, joint_ids["FL"]["femur"]))
            target_FL_angle_knee = math.radians(real_sim_angle(FL_angle_knee, joint_ids["FL"]["knee"]))
            #------------------------------
            # calculate back right shoulder, femur, knee angles
            BR_s_f_t = legIK(forward_targets_FR_BL[i][0], forward_targets_FR_BL[i][1], forward_targets_FR_BL[i][2])
            BR_angle_shoulder, BR_angle_femur, BR_angle_knee = BR_s_f_t
            target_BR_angle_shoulder = math.radians(real_sim_angle(BR_angle_shoulder,joint_ids["BR"]["shoulder"]))
            target_BR_angle_femur = math.radians(real_sim_angle(BR_angle_femur, joint_ids["BR"]["femur"]))
            target_BR_angle_knee = math.radians(real_sim_angle(BR_angle_knee, joint_ids["BR"]["knee"]))
            #------------------------------
            # calculate back left shoulder, femur, knee angles
            BL_s_f_t = legIK(forward_targets_FL_BR[i][0], forward_targets_FL_BR[i][1], forward_targets_FL_BR[i][2])
            BL_angle_shoulder, BL_angle_femur, BL_angle_knee = BL_s_f_t
            target_BL_angle_shoulder = math.radians(real_sim_angle(BL_angle_shoulder,joint_ids["BL"]["shoulder"]))
            target_BL_angle_femur = math.radians(real_sim_angle(BL_angle_femur, joint_ids["BL"]["femur"]))
            target_BL_angle_knee = math.radians(real_sim_angle(BL_angle_knee, joint_ids["BL"]["knee"]))
            #------------------------------

            # MOVE ALL JOINTS
            # Move front right shoulder
            p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["FR"]["shoulder"], controlMode=p.POSITION_CONTROL, targetPosition=target_FR_angle_shoulder, force=500)
            # Move front right femur
            p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["FR"]["femur"], controlMode=p.POSITION_CONTROL, targetPosition=target_FR_angle_femur, force=500)
            # Move front right knee
            p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["FR"]["knee"], controlMode=p.POSITION_CONTROL, targetPosition=target_FR_angle_knee, force=500)
            #------------------------------
            # Move front left shoulder
            p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["FL"]["shoulder"], controlMode=p.POSITION_CONTROL, targetPosition=target_FL_angle_shoulder, force=500)
            # Move front left femur
            p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["FL"]["femur"], controlMode=p.POSITION_CONTROL, targetPosition=target_FL_angle_femur, force=500)
            # Move front left knee
            p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["FL"]["knee"], controlMode=p.POSITION_CONTROL, targetPosition=target_FL_angle_knee, force=500)
            #------------------------------
            # Move back right shoulder
            p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["BR"]["shoulder"], controlMode=p.POSITION_CONTROL, targetPosition=target_BR_angle_shoulder, force=500)
            # Move back right femur
            p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["BR"]["femur"], controlMode=p.POSITION_CONTROL, targetPosition=target_BR_angle_femur, force=500)
            # Move back right knee
            p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["BR"]["knee"], controlMode=p.POSITION_CONTROL, targetPosition=target_BR_angle_knee, force=500)
            #------------------------------
            # Move back left shoulder
            p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["BL"]["shoulder"], controlMode=p.POSITION_CONTROL, targetPosition=target_BL_angle_shoulder, force=500)
            # Move back left femur
            p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["BL"]["femur"], controlMode=p.POSITION_CONTROL, targetPosition=target_BL_angle_femur, force=500)
            # Move back left knee
            p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["BL"]["knee"], controlMode=p.POSITION_CONTROL, targetPosition=target_BL_angle_knee, force=500)
            #------------------------------

            # Step the simulation and wait for a short time to observe movement
            for _ in range(100):  # Let the movement settle
                p.stepSimulation()
                time.sleep(1 / 240)

except KeyboardInterrupt:
    print("Simulation terminated.")

# Disconnect from PyBullet
p.disconnect()