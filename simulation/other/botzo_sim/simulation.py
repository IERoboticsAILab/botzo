import pybullet as p
import time
import pybullet_data
import math 

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10, physicsClientId=physicsClient)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,2]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
robotId = p.loadURDF("./body_urdf/body_urdf.urdf",cubeStartPos, cubeStartOrientation, flags=p.URDF_USE_INERTIA_FROM_FILE)

# # Create a fixed constraint to hold the robot in place
# fixed_position = [0, 0, 0]  # Adjust height as needed for better visualization
# p.resetBasePositionAndOrientation(robotId, fixed_position, [0, 0, 0, 1])

# Apply a constraint to keep the base at this position
constraint_id = p.createConstraint(
    parentBodyUniqueId=robotId,
    parentLinkIndex=-1,  # Base link
    childBodyUniqueId=-1,  # No child body
    childLinkIndex=-1,     # No child link
    jointType=p.JOINT_FIXED,
    jointAxis=[0, 0, 0],
    parentFramePosition=[0, 0, 0],
    childFramePosition=[0, 0, 2]
)

numJoints = p.getNumJoints(robotId)
print("Number of Joints: ", numJoints)

hip_joint = 0
knee_joint = 3
ankle_joint = 5

previous_pos = None

while True:
    for i in range (10000):
        target_position = 0.5 *math.sin(i*0.01)

        if i % 100 == 0:
            print("Target Position: ", target_position)

        # p.setJointMotorControl2(robotId, knee_joint, p.POSITION_CONTROL, targetPosition = target_position)
        p.setJointMotorControl2(robotId, ankle_joint, p.POSITION_CONTROL, targetPosition = target_position)
        p.stepSimulation()

        link_state = p.getLinkState(robotId, 6)
        current_pos = link_state[0]

        if previous_pos is not None:
            p.addUserDebugLine(previous_pos, current_pos, [1, 0, 0], 1)

        previous_pos = current_pos

        time.sleep(1./240.)



# for i in range (10000):
#     p.stepSimulation()
#     time.sleep(1./240.)
# cubePos, cubeOrn = p.getBasePositionAndOrientation(robotId)
# print(cubePos,cubeOrn)

cubePos, cubeOrn = p.getBasePositionAndOrientation(robotId)
print("Final Position: ", cubePos, cubeOrn)
p.disconnect()

