import pybullet as p
import time
import pybullet_data
import math

'''
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
'''
joint_ids = {
    "BR": { # RH
        "shoulder": 3, # RH_HAA
        "femur": 4, # RH_HFE
        "knee": 5, # RH_KFE
    },
    "FR": { # RF
        "shoulder": 9, # RF_HAA
        "femur": 10, # RF_HFE
        "knee": 11, # RF_KFE
    },
    "BL": { # LH
        "shoulder": 0, # LH_HAA
        "femur": 1, # LH_HFE
        "knee": 2, # LH_KFE
    },
    "FL": { # LF
        "shoulder": 6, # LF_HAA
        "femur": 7, # LF_HFE
        "knee": 8, #LF_KFE
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
robotId = p.loadURDF("../CAD_files/URDF/botzo.urdf", cubeStartPos, cubeStartOrientation,
                     flags=p.URDF_USE_INERTIA_FROM_FILE)


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
'''
Joint 0:
  Name: Revolute 1
  Type: 0
  Child Link: BL_shoulder

Joint 1:
  Name: Revolute 11
  Type: 0
  Child Link: BL_femur

Joint 2:
  Name: Revolute 12
  Type: 0
  Child Link: BL_tibia

Joint 3:
  Name: Revolute 2
  Type: 0
  Child Link: BR_shoulder

Joint 4:
  Name: Revolute 9
  Type: 0
  Child Link: BR_femur

Joint 5:
  Name: Revolute 10
  Type: 0
  Child Link: BR_tibia

Joint 6:
  Name: Revolute 3
  Type: 0
  Child Link: FL_shoulder

Joint 7:
  Name: Revolute 7
  Type: 0
  Child Link: FL_femur

Joint 8:
  Name: Revolute 8
  Type: 0
  Child Link: FL_tibia

Joint 9:
  Name: Revolute 4
  Type: 0
  Child Link: FR_shoulder

Joint 10:
  Name: Revolute 5
  Type: 0
  Child Link: FR_femur

Joint 11:
  Name: Revolute 6
  Type: 0
  Child Link: FR_tibia
'''

while True:
    try:
        # Move joint 36 (FL_femur_joint) to 45 degrees
        joint_index = 0
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
