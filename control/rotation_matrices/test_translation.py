'''
CODE TO MOVE ROBOT IN TRANSLATIONS
UP, DOWN, LEFT, RIGHT, FORWARD, BACKWARD
ALL FOOR COORDINATED
'''


from utils.new_IK_pybullet import *
#import torch # pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu128
#device = torch.device("cuda" if torch.cuda.is_available() else "cpu")



# -------------------------
# NON-BLOCKING KEY READER
# -------------------------
last_key_pressed = None
target_L = [0,2,16]
target_R = [0,2,16]

def key_listener():
    global last_key_pressed
    # translation of body comands
    print("KEYS:\n\t'w' -> forward\n\t's' -> backward\n\t'a' -> left\n\t'd' -> right\n\t''r' -> up\n\t'f' -> down\n\t'x' -> stop\n\t'q' -> exit\nINPUT:")
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
cubeStartPos = [0, 0, 1]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF("../CAD_files/URDF/BOTZO_URDF_description/urdf/BOTZO_URDF.urdf", cubeStartPos, cubeStartOrientation,
                     flags=p.URDF_USE_INERTIA_FROM_FILE, globalScaling=10)

# Run the simulation loop indefinitely
try:
    while True:
        if last_key_pressed is not None:
            if last_key_pressed == "q":
                print("Exit command received.")
                break
            elif last_key_pressed == "w":
                target_L[1] += 0.1  # Move forward
                target_R[1] -= 0.1  # Move forward
            elif last_key_pressed == "s":
                target_L[1] -= 0.1  # Move backward
                target_R[1] += 0.1  # Move backward
            elif last_key_pressed == "a":
                target_L[0] -= 0.1  # Move left
                target_R[0] -= 0.1  # Move left
            elif last_key_pressed == "d":
                target_L[0] += 0.1  # Move right
                target_R[0] += 0.1  # Move right
            elif last_key_pressed == "r":
                target_L[2] += 0.1  # Move up
                target_R[2] += 0.1  # Move up
            elif last_key_pressed == "f":
                target_L[2] -= 0.1  # Move down
                target_R[2] -= 0.1  # Move down
            elif last_key_pressed == "x":
                target_L = target_L
                target_R = target_R



        # calculate front right shoulder, femur, knee angles
        FR_s_f_t = legIK(target_R[0], target_R[1], target_R[2])
        FR_angle_shoulder, FR_angle_femur, FR_angle_knee = FR_s_f_t
        target_FR_angle_shoulder = math.radians(real_sim_angle(FR_angle_shoulder,joint_ids["FR"]["shoulder"]))
        target_FR_angle_femur = math.radians(real_sim_angle(FR_angle_femur, joint_ids["FR"]["femur"]))
        target_FR_angle_knee = math.radians(real_sim_angle(FR_angle_knee, joint_ids["FR"]["knee"]))
        #------------------------------
        # calculate front left shoulder, femur, knee angles
        FL_s_f_t = legIK(target_L[0], target_L[1], target_L[2])
        FL_angle_shoulder, FL_angle_femur, FL_angle_knee = FL_s_f_t
        target_FL_angle_shoulder = math.radians(real_sim_angle(FL_angle_shoulder,joint_ids["FL"]["shoulder"]))
        target_FL_angle_femur = math.radians(real_sim_angle(FL_angle_femur, joint_ids["FL"]["femur"]))
        target_FL_angle_knee = math.radians(real_sim_angle(FL_angle_knee, joint_ids["FL"]["knee"]))
        #------------------------------
        # calculate back right shoulder, femur, knee angles
        BR_s_f_t = legIK(target_R[0], target_R[1], target_R[2])
        BR_angle_shoulder, BR_angle_femur, BR_angle_knee = BR_s_f_t
        target_BR_angle_shoulder = math.radians(real_sim_angle(BR_angle_shoulder, joint_ids["BR"]["shoulder"]))
        target_BR_angle_femur = math.radians(real_sim_angle(BR_angle_femur, joint_ids["BR"]["femur"]))
        target_BR_angle_knee = math.radians(real_sim_angle(BR_angle_knee, joint_ids["BR"]["knee"]))
        #------------------------------
        # calculate back left shoulder, femur, knee angles
        BL_s_f_t = legIK(target_L[0], target_L[1], target_L[2])
        BL_angle_shoulder, BL_angle_femur, BL_angle_knee = BL_s_f_t
        target_BL_angle_shoulder = math.radians(real_sim_angle(BL_angle_shoulder,joint_ids["BL"]["shoulder"]))
        target_BL_angle_femur = math.radians(real_sim_angle(BL_angle_femur, joint_ids["BL"]["femur"]))
        target_BL_angle_knee = math.radians(real_sim_angle(BL_angle_knee, joint_ids["BL"]["knee"]))
        #------------------------------

        # MOVE ALL JOINTS
        # Move front right shoulder
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["FR"]["shoulder"], controlMode=p.POSITION_CONTROL, targetPosition=target_FR_angle_shoulder, force=50)
        # Move front right femur
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["FR"]["femur"], controlMode=p.POSITION_CONTROL, targetPosition=target_FR_angle_femur, force=50)
        # Move front right knee
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["FR"]["knee"], controlMode=p.POSITION_CONTROL, targetPosition=target_FR_angle_knee, force=50)
        #------------------------------
        # Move front left shoulder
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["FL"]["shoulder"], controlMode=p.POSITION_CONTROL, targetPosition=target_FL_angle_shoulder, force=50)
        # Move front left femur
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["FL"]["femur"], controlMode=p.POSITION_CONTROL, targetPosition=target_FL_angle_femur, force=50)
        # Move front left knee
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["FL"]["knee"], controlMode=p.POSITION_CONTROL, targetPosition=target_FL_angle_knee, force=50)
        #------------------------------
        # Move back right shoulder
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["BR"]["shoulder"], controlMode=p.POSITION_CONTROL, targetPosition=target_BR_angle_shoulder, force=50)
        # Move back right femur
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["BR"]["femur"], controlMode=p.POSITION_CONTROL, targetPosition=target_BR_angle_femur, force=50)
        # Move back right knee
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["BR"]["knee"], controlMode=p.POSITION_CONTROL, targetPosition=target_BR_angle_knee, force=50)
        #------------------------------
        # Move back left shoulder
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["BL"]["shoulder"], controlMode=p.POSITION_CONTROL, targetPosition=target_BL_angle_shoulder, force=50)
        # Move back left femur
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["BL"]["femur"], controlMode=p.POSITION_CONTROL, targetPosition=target_BL_angle_femur, force=50)
        # Move back left knee
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["BL"]["knee"], controlMode=p.POSITION_CONTROL, targetPosition=target_BL_angle_knee, force=50)
        #------------------------------
        # set foot joint to fixed position
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["FR"]["foot"], controlMode=p.POSITION_CONTROL, targetPosition=0, force=50)
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["FL"]["foot"], controlMode=p.POSITION_CONTROL, targetPosition=0, force=50)
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["BR"]["foot"], controlMode=p.POSITION_CONTROL, targetPosition=0, force=50)
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=joint_ids["BL"]["foot"], controlMode=p.POSITION_CONTROL, targetPosition=0, force=50)

        # Step the simulation and wait for a short time to observe movement
        for _ in range(10):  # Let the movement settle
            p.stepSimulation()
            time.sleep(1 / 240)

except KeyboardInterrupt:
    print("Simulation terminated.")

# Disconnect from PyBullet
p.disconnect()