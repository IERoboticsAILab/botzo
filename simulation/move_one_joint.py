import pybullet as p
import time
import pybullet_data
import math

joint_ids = {
    "BR": { # RH
        "shoulder": 11, # RH_HAA
        "femur": 14, # RH_HFE
        "knee": 15, # RH_KFE
        "foot": 16 # RH_shank_fixed_RH_FOOT
    },
    "FR": { # RF
        "shoulder": 3, # RF_HAA
        "femur": 6, # RF_HFE
        "knee": 7, # RF_KFE
        "foot": 8 # RF_shank_fixed_RF_FOOT
    },
    "BL": { # LH
        "shoulder": 19, # LH_HAA
        "femur": 22, # LH_HFE
        "knee": 23, # LH_KFE
        "foot": 24 # LH_shank_fixed_LH_FOOT
    },
    "FL": { # LF
        "shoulder": 27, # LF_HAA
        "femur": 30, # LF_HFE
        "knee": 31, #LF_KFE
        "foot": 32 # LF_shank_fixed_LF_FOOT
    }
}

# Connect to PyBullet
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Optionally
p.setGravity(0, 0, -10)

# Load the plane and your URDF
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0, 0, 0.1]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF("../CAD_files/URDF/BOTZO_URDF_description/urdf/BOTZO_URDF.urdf", cubeStartPos, cubeStartOrientation,
                     flags=p.URDF_USE_INERTIA_FROM_FILE)

'''
# Print joint information
num_joints = p.getNumJoints(robotId)
print(f"Number of joints in the URDF: {num_joints}\n") #-> 41

for joint_index in range(num_joints):
    joint_info = p.getJointInfo(robotId, joint_index)
    if joint_info[2] == 0:  # Skip fixed joints
        print(f"Joint {joint_index}:")
        print(f"  Name: {joint_info[1].decode('utf-8')}")
        print(f"  Type: {joint_info[2]}")  # Joint type (e.g., fixed (4), revolute (0))
        print(f"  Child Link: {joint_info[12].decode('utf-8')}")
        print(f"  Parent Frame Position: {joint_info[14]}")
        print(f"  Parent Frame Orientation: {joint_info[15]}")
        print("")
# Joint 3:
#   Name: RF_HAA
#   Child Link: RF_HIP
# Joint 6:
#   Name: RF_HFE
#   Child Link: RF_THIGH
# Joint 7:
#   Name: RF_KFE
#   Child Link: RF_SHANK
# Joint 8:
#   Name: RF_shank_fixed_RF_FOOT
#   Child Link: RF_FOOT

# Joint 11:
#   Name: RH_HAA
#   Child Link: RH_HIP
# Joint 14:
#   Name: RH_HFE
#   Child Link: RH_THIGH
# Joint 15:
#   Name: RH_KFE
#   Child Link: RH_SHANK
# Joint 16:
#   Name: RH_shank_fixed_RH_FOOT
#   Child Link: RH_FOOT

# Joint 19:
#   Name: LH_HAA
#   Child Link: LH_HIP
# Joint 22:
#   Name: LH_HFE
#   Child Link: LH_THIGH
# Joint 23:
#   Name: LH_KFE
#   Child Link: LH_SHANK
# Joint 24:
#   Name: LH_shank_fixed_LH_FOOT
#   Child Link: LH_FOOT

# Joint 27:
#   Name: LF_HAA
#   Child Link: LF_HIP
# Joint 30:
#   Name: LF_HFE
#   Child Link: LF_THIGH
# Joint 31:
#   Name: LF_KFE
#   Child Link: LF_SHANK
# Joint 32:
#   Name: LF_shank_fixed_LF_FOOT
#   Child Link: LF_FOOT
'''

while True:
    try:
        # Move joint 36 (FL_femur_joint) to 45 degrees
        joint_index = 31
        target_angle_deg = float(input("desire angle:"))  # Target angle in degrees
        target_angle_rad = math.radians(target_angle_deg)  # Convert to radians

        # Use position control to move the joint
        p.setJointMotorControl2(
            bodyUniqueId=robotId,                # Robot ID
            jointIndex=joint_index,              # Joint index (36 in this case)
            controlMode=p.POSITION_CONTROL,      # Use position control
            targetPosition=target_angle_rad,     # Target position in radians
            force=500                            # Max force applied to reach the target
        ) # setJointMotorControl2

        foot_side = "FL" # LF_shank_fixed_LF_FOOT
        foot_state = p.getLinkState(robotId, joint_ids[foot_side]["foot"])
        foot_position = foot_state[0]  # (x, y, z) coordinates of the foot
        print(f"{foot_side} Foot Position: {foot_position}")

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
            ) # setJointMotorControl2

    except KeyboardInterrupt:
        break

# Simulate for a while
for i in range(10000):
    p.stepSimulation()
    time.sleep(1./240.)

# Get base position and orientation
cubePos, cubeOrn = p.getBasePositionAndOrientation(robotId)
print("Base Position:", cubePos)
print("Base Orientation:", cubeOrn)

# Disconnect from PyBullet
p.disconnect()
