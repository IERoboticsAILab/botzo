'''
CODE TO MOVE ROBOT IN TRANSLATIONS
UP, DOWN, LEFT, RIGHT, FORWARD, BACKWARD
ALL FOOR COORDINATED

AND

ROTATIONS
ROLL, PITCH, YAW
TILT THE BODY OF THE ROBOT


TODO:(change point of rotation)
'''

from utils.new_IK_pybullet import *


# -------------------------
# NON-BLOCKING KEY READER
# -------------------------
last_key_pressed = None

roll = 0
pitch = 0
yaw = 0

target_FL = [0,3,16]
target_FR = [0,3,16]
target_BL = [0,3,16]
target_BR = [0,3,16]
rotated_targetFR=[0,3,16]
rotated_targetFL=[0,3,16]
rotated_targetBR=[0,3,16]
rotated_targetBL=[0,3,16]


FR = [8.9, W/2, 0]
FL = [8.9, -W/2, 0]
BR = [-7, W/2, 0]
BL = [-7, -W/2, 0]

def key_listener():
    global last_key_pressed
    # translation of body comands
    print("KEYS:\n\t'w' -> forward\n\t's' -> backward\n\t'a' -> left\n\t'd' -> right\n\t''r' -> up\n\t'f' -> down")
    print("\t'i' -> roll +\n\t'k' -> roll -\n\t'j' -> pitch +\n\t'l' -> pitch -\n\t'y' -> yaw +\n\t'h' -> yaw -")
    print("\t'x' -> stop\n\t'q' -> exit\nINPUT:")
    while True:
        if msvcrt.kbhit():      # Key pressed?
            ch = msvcrt.getwch()   # Read unicode key
            last_key_pressed = ch
            print(f"\n>> Key pressed: {ch}\n")
        time.sleep(0.01)

# Start key listener thread
listener_thread = threading.Thread(target=key_listener, daemon=True)
listener_thread.start()






# -------------------------
# PYBULLET SETUP
# -------------------------
# Connect to PyBullet
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Optionally
p.setGravity(0, 0, -9.8)

# Load the plane and your URDF
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0, 0, 1.5]
#cubeStartOrientation = p.getQuaternionFromEuler([math.pi, 0, 0]) # rotate the robot of 180 degrees around the x axis
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF("../CAD_files/URDF/BOTZO_URDF_description/urdf/BOTZO_URDF.urdf", cubeStartPos, cubeStartOrientation,
                     flags=p.URDF_USE_INERTIA_FROM_FILE, globalScaling=10)

# Run the simulation loop indefinitely
rotation_step = math.radians(0.5)  # grad per step
translation_step = 0.1  # cm per step
try:
    while True:
        if last_key_pressed is not None:
            if last_key_pressed == "q":
                print("Exit command received.")
                break
            elif last_key_pressed == "w":
                target_FL[1] += translation_step  # Move forward
                target_BL[1] += translation_step  # Move forward
                target_FR[1] -= translation_step  # Move forward
                target_BR[1] -= translation_step  # Move forward
                rotated_targetFL[1] += translation_step
                rotated_targetBL[1] += translation_step
                rotated_targetFR[1] -= translation_step
                rotated_targetBR[1] -= translation_step
            elif last_key_pressed == "s":
                target_FL[1] -= translation_step  # Move backward
                target_BL[1] -= translation_step  # Move backward
                target_FR[1] += translation_step  # Move backward
                target_BR[1] += translation_step  # Move backward
                rotated_targetFL[1] -= translation_step
                rotated_targetBL[1] -= translation_step
                rotated_targetFR[1] += translation_step
                rotated_targetBR[1] += translation_step
            elif last_key_pressed == "a":
                target_FL[0] -= translation_step  # Move left
                target_BL[0] -= translation_step  # Move left
                target_FR[0] -= translation_step  # Move left
                target_BR[0] -= translation_step  # Move left
                rotated_targetFL[0] -= translation_step
                rotated_targetBL[0] -= translation_step
                rotated_targetFR[0] -= translation_step
                rotated_targetBR[0] -= translation_step
            elif last_key_pressed == "d":
                target_FL[0] += translation_step  # Move right
                target_BL[0] += translation_step  # Move right
                target_FR[0] += translation_step  # Move right
                target_BR[0] += translation_step  # Move right
                rotated_targetFL[0] += translation_step
                rotated_targetBL[0] += translation_step
                rotated_targetFR[0] += translation_step
                rotated_targetBR[0] += translation_step
            elif last_key_pressed == "r":
                target_FL[2] += translation_step  # Move up
                target_BL[2] += translation_step  # Move up
                target_FR[2] += translation_step  # Move up
                target_BR[2] += translation_step  # Move up
                rotated_targetFL[2] += translation_step
                rotated_targetBL[2] += translation_step
                rotated_targetFR[2] += translation_step
                rotated_targetBR[2] += translation_step
            elif last_key_pressed == "f":
                target_FL[2] -= translation_step  # Move down
                target_BL[2] -= translation_step  # Move down
                target_FR[2] -= translation_step  # Move down
                target_BR[2] -= translation_step  # Move down
                rotated_targetFL[2] -= translation_step
                rotated_targetBL[2] -= translation_step
                rotated_targetFR[2] -= translation_step
                rotated_targetBR[2] -= translation_step
            elif last_key_pressed == "i":
                roll += rotation_step
            elif last_key_pressed == "k":
                roll -= rotation_step
            elif last_key_pressed == "j":
                pitch += rotation_step
            elif last_key_pressed == "l":
                pitch -= rotation_step
            elif last_key_pressed == "y":
                yaw += rotation_step
            elif last_key_pressed == "h":
                yaw -= rotation_step
            elif last_key_pressed == "x":
                # Reset rotations when stopping
                #roll = 0
                #pitch = 0
                #yaw = 0
                pass
        
        if last_key_pressed in ["i", "k", "j", "l", "y", "h"]:
            target_to_bodyFR = [target_FR[0], target_FR[1], target_FR[2]]
            target_to_bodyFL = [target_FL[0], -target_FL[1], target_FL[2]]
            target_to_bodyBR = [target_BR[0], target_BR[1], target_BR[2]]
            target_to_bodyBL = [target_BL[0], -target_BL[1], target_BL[2]]

            print(f"pitch: {pitch}, roll: {roll}, yaw: {yaw}")
            R = RotMatrix(rotation=[roll, pitch, yaw], is_radiants=True, order='xyz')

            FRr = R * np.matrix([[FR[0]], [FR[1]], [FR[2]], [1]])
            FLr = R * np.matrix([[FL[0]], [FL[1]], [FL[2]], [1]])
            BRr = R * np.matrix([[BR[0]], [BR[1]], [BR[2]], [1]])
            BLr = R * np.matrix([[BL[0]], [BL[1]], [BL[2]], [1]])

            FRt = FR + target_to_bodyFR
            FLt = FL + target_to_bodyFL
            BRt = BR + target_to_bodyBR
            BLt = BL + target_to_bodyBL

            FRtr = FRr + R * np.matrix([[target_to_bodyFR[0]], [target_to_bodyFR[1]], [target_to_bodyFR[2]], [0]])
            FLtr = FLr + R * np.matrix([[target_to_bodyFL[0]], [target_to_bodyFL[1]], [target_to_bodyFL[2]], [0]])
            BRtr = BRr + R * np.matrix([[target_to_bodyBR[0]], [target_to_bodyBR[1]], [target_to_bodyBR[2]], [0]])
            BLtr = BLr + R * np.matrix([[target_to_bodyBL[0]], [target_to_bodyBL[1]], [target_to_bodyBL[2]], [0]])
            
            rotated_targetFR = [FRtr[0,0] - FR[0], FRtr[1,0] - FR[1], FRtr[2,0] - FR[2]]
            rotated_targetFL = [FLtr[0,0] - FL[0], -(FLtr[1,0] - FL[1]), FLtr[2,0] - FL[2]]
            rotated_targetBR = [BRtr[0,0] - BR[0], BRtr[1,0] - BR[1], BRtr[2,0] - BR[2]]
            rotated_targetBL = [BLtr[0,0] - BL[0], -(BLtr[1,0] - BL[1]), BLtr[2,0] - BL[2]]
            print(f"Result Rotated Target FR: {rotated_targetFR[0]}, {rotated_targetFR[1]}, {rotated_targetFR[2]}")
            print(f"Result Rotated Target FL: {rotated_targetFL[0]}, {rotated_targetFL[1]}, {rotated_targetFL[2]}")
            print(f"Result Rotated Target BR: {rotated_targetBR[0]}, {rotated_targetBR[1]}, {rotated_targetBR[2]}")
            print(f"Result Rotated Target BL: {rotated_targetBL[0]}, {rotated_targetBL[1]}, {rotated_targetBL[2]}\n\n")







            
        


        if rotated_targetFR is not None:
            rot_trans_FR = rotated_targetFR
            rot_trans_FL = rotated_targetFL
            rot_trans_BR = rotated_targetBR
            rot_trans_BL = rotated_targetBL
        else:
            rot_trans_FR = target_FR
            rot_trans_FL = target_FL
            rot_trans_BR = target_BR
            rot_trans_BL = target_BL
        # calculate front right shoulder, femur, knee angles
        FR_s_f_t = legIK(rot_trans_FR[0], rot_trans_FR[1], rot_trans_FR[2])
        FR_angle_shoulder, FR_angle_femur, FR_angle_knee = FR_s_f_t
        target_FR_angle_shoulder = math.radians(real_sim_angle(FR_angle_shoulder,joint_ids["FR"]["shoulder"]))
        target_FR_angle_femur = math.radians(real_sim_angle(FR_angle_femur, joint_ids["FR"]["femur"]))
        target_FR_angle_knee = math.radians(real_sim_angle(FR_angle_knee, joint_ids["FR"]["knee"]))
        #------------------------------
        # calculate front left shoulder, femur, knee angles
        FL_s_f_t = legIK(rot_trans_FL[0], rot_trans_FL[1], rot_trans_FL[2])
        FL_angle_shoulder, FL_angle_femur, FL_angle_knee = FL_s_f_t
        target_FL_angle_shoulder = math.radians(real_sim_angle(FL_angle_shoulder,joint_ids["FL"]["shoulder"]))
        target_FL_angle_femur = math.radians(real_sim_angle(FL_angle_femur, joint_ids["FL"]["femur"]))
        target_FL_angle_knee = math.radians(real_sim_angle(FL_angle_knee, joint_ids["FL"]["knee"]))
        #------------------------------
        # calculate back right shoulder, femur, knee angles
        BR_s_f_t = legIK(rot_trans_BR[0], rot_trans_BR[1], rot_trans_BR[2])
        BR_angle_shoulder, BR_angle_femur, BR_angle_knee = BR_s_f_t
        target_BR_angle_shoulder = math.radians(real_sim_angle(BR_angle_shoulder, joint_ids["BR"]["shoulder"]))
        target_BR_angle_femur = math.radians(real_sim_angle(BR_angle_femur, joint_ids["BR"]["femur"]))
        target_BR_angle_knee = math.radians(real_sim_angle(BR_angle_knee, joint_ids["BR"]["knee"]))
        #------------------------------
        # calculate back left shoulder, femur, knee angles
        BL_s_f_t = legIK(rot_trans_BL[0], rot_trans_BL[1], rot_trans_BL[2])
        BL_angle_shoulder, BL_angle_femur, BL_angle_knee = BL_s_f_t
        target_BL_angle_shoulder = math.radians(real_sim_angle(BL_angle_shoulder,joint_ids["BL"]["shoulder"]))
        target_BL_angle_femur = math.radians(real_sim_angle(BL_angle_femur, joint_ids["BL"]["femur"]))
        target_BL_angle_knee = math.radians(real_sim_angle(BL_angle_knee, joint_ids["BL"]["knee"]))
        #------------------------------

        # MOVE ALL JOINTS
        # Move front right shoulder
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["FR"]["shoulder"], controlMode=p.POSITION_CONTROL, targetPosition=target_FR_angle_shoulder, force=100)
        # Move front right femur
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["FR"]["femur"], controlMode=p.POSITION_CONTROL, targetPosition=target_FR_angle_femur, force=100)
        # Move front right knee
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["FR"]["knee"], controlMode=p.POSITION_CONTROL, targetPosition=target_FR_angle_knee, force=100)
        #------------------------------
        # Move front left shoulder
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["FL"]["shoulder"], controlMode=p.POSITION_CONTROL, targetPosition=target_FL_angle_shoulder, force=100)
        # Move front left femur
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["FL"]["femur"], controlMode=p.POSITION_CONTROL, targetPosition=target_FL_angle_femur, force=100)
        # Move front left knee
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["FL"]["knee"], controlMode=p.POSITION_CONTROL, targetPosition=target_FL_angle_knee, force=100)
        #------------------------------
        # Move back right shoulder
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["BR"]["shoulder"], controlMode=p.POSITION_CONTROL, targetPosition=target_BR_angle_shoulder, force=100)
        # Move back right femur
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["BR"]["femur"], controlMode=p.POSITION_CONTROL, targetPosition=target_BR_angle_femur, force=100)
        # Move back right knee
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["BR"]["knee"], controlMode=p.POSITION_CONTROL, targetPosition=target_BR_angle_knee, force=100)
        #------------------------------
        # Move back left shoulder
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["BL"]["shoulder"], controlMode=p.POSITION_CONTROL, targetPosition=target_BL_angle_shoulder, force=100)
        # Move back left femur
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["BL"]["femur"], controlMode=p.POSITION_CONTROL, targetPosition=target_BL_angle_femur, force=100)
        # Move back left knee
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["BL"]["knee"], controlMode=p.POSITION_CONTROL, targetPosition=target_BL_angle_knee, force=100)
        #------------------------------
        # set foot joint to fixed position
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["FR"]["foot"], controlMode=p.POSITION_CONTROL, targetPosition=0, force=100)
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["FL"]["foot"], controlMode=p.POSITION_CONTROL, targetPosition=0, force=100)
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["BR"]["foot"], controlMode=p.POSITION_CONTROL, targetPosition=0, force=100)
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["BL"]["foot"], controlMode=p.POSITION_CONTROL, targetPosition=0, force=100)

        # Step the simulation and wait for a short time to observe movement
        for _ in range(10):  # Let the movement settle
            p.stepSimulation()
            time.sleep(1 / 240)

except KeyboardInterrupt:
    print("Simulation terminated.")

# Disconnect from PyBullet
p.disconnect()