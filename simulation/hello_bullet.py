import pybullet as p
import time
import pybullet_data
import math

# Connect to PyBullet
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Optionally
p.setGravity(0, 0, -10)

# Load the plane and your URDF
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0, 0, 0]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF("urdf/final_URDF.urdf", cubeStartPos, cubeStartOrientation,
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
'''

while True:
    try:
        # Move joint 36 (FL_femur_joint) to 45 degrees
        joint_index = 7
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

        p.stepSimulation()
        time.sleep(1./240.)

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
