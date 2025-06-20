import pybullet as p
import time
import pybullet_data
import math 
import numpy as np

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10, physicsClientId=physicsClient)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,2]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
robotId = p.loadURDF("./simple_leg_pybullet/simple_leg_pybullet.urdf",cubeStartPos, cubeStartOrientation, flags=p.URDF_USE_INERTIA_FROM_FILE)

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


def calculate_distance(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))




numJoints = p.getNumJoints(robotId)
print("Number of Joints: ", numJoints)

shoulder_joint = 0
femur_joint = 3
tibia_joint = 5
end_effector = 6

# Getting the position of the joints
shoulder_pos = p.getLinkState(robotId, shoulder_joint)[0]
femur_pos = p.getLinkState(robotId, femur_joint)[0]
tibia_pos = p.getLinkState(robotId, tibia_joint)[0]
end_effector_pos = p.getLinkState(robotId, end_effector)[0]

# Calculating distance between the joints
coxa = calculate_distance(shoulder_pos, femur_pos) 
femur = calculate_distance(femur_pos, tibia_pos) 
tibia = calculate_distance(tibia_pos, end_effector_pos)

print("Shoulder to Femur Distance: ", coxa)
print("Femur to Tibia Distance: ", femur)
print("Tibia to End Effector Distance: ", tibia)

previous_pos = None



def legIK(x,y,z):
  D = np.sqrt((z**2 + y**2) - coxa**2)
  G = np.sqrt(D**2 + x**2)
  tibia_angle = np.arccos((G**2 - femur**2 - tibia**2)/(-2*femur*tibia))
  femur_angle = np.arctan2(x,D) + np.arcsin((tibia * np.sin(tibia_angle)) / G)
  coxa_angle = np.arctan2(y,z) + np.arctan2(D,coxa)

  return [coxa_angle, femur_angle, tibia_angle]


# # Define y and z as fixed values, and generate x-coordinates for a straight line
# y_fixed = -0.36
# z_fixed = 0.6
# x_start = 0
# x_end = 0.6
# num_points = 20  # Reduced number of points

# # Generate target points with larger steps along the x-axis
# x_values = np.linspace(x_start, x_end, num_points)
# target_points = [(x, y_fixed, z_fixed) for x in x_values]

target_points = [(0, -0.36, 0.6), (0, -0.36, 0.8), (0, -0.36, 1.0)]

while True: 
    # Simulation loop to move the joints to each target point
    for x, y, z in target_points:
        # Calculate joint angles for the given target
        joint_angles = legIK(x, y, z)
        coxa_angle, femur_angle, tibia_angle = joint_angles
        
        # Run the simulation for a certain number of steps to allow the leg to reach the target
        for i in range(500):
            # Control each joint to move towards the calculated IK angles
            p.setJointMotorControl2(robotId, shoulder_joint, p.POSITION_CONTROL, targetPosition=0)
            p.setJointMotorControl2(robotId, femur_joint, p.POSITION_CONTROL, targetPosition=femur_angle)
            p.setJointMotorControl2(robotId, tibia_joint, p.POSITION_CONTROL, targetPosition=tibia_angle)
            
            # Step the simulation
            p.stepSimulation()

            link_state = p.getLinkState(robotId, 6)
            current_pos = link_state[0]

            # if i % 100 == 0:
            #     print("Current Position: ", current_pos)

            if previous_pos is not None:
                p.addUserDebugLine(previous_pos, current_pos, [1, 0, 0], 1)

            previous_pos = current_pos

            time.sleep(1./240.)  # Control simulation speed if needed

# while True:
#     for i in range (10000):
#         target_position = 0.5 *math.sin(i*0.01)

#         # if i % 100 == 0:
#         #     print("Target Position: ", target_position)

#         # p.setJointMotorControl2(robotId, knee_joint, p.POSITION_CONTROL, targetPosition = target_position)
#         p.setJointMotorControl2(robotId, tibia_joint, p.POSITION_CONTROL, targetPosition = target_position)
#         p.stepSimulation()

#         link_state = p.getLinkState(robotId, 6)
#         current_pos = link_state[0]

#         # if i % 100 == 0:
#         #     print("Current Position: ", current_pos)

#         if previous_pos is not None:
#             p.addUserDebugLine(previous_pos, current_pos, [1, 0, 0], 1)

#         previous_pos = current_pos

#         time.sleep(1./240.)



# for i in range (10000):
#     p.stepSimulation()
#     time.sleep(1./240.)
# cubePos, cubeOrn = p.getBasePositionAndOrientation(robotId)
# print(cubePos,cubeOrn)

cubePos, cubeOrn = p.getBasePositionAndOrientation(robotId)
print("Final Position: ", cubePos, cubeOrn)
p.disconnect()

