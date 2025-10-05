import pybullet as p
import time
import pybullet_data

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)

planeId = p.loadURDF("plane.urdf")
startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,0])
robotId = p.loadURDF("../CAD_files/URDF/BOTZO_URDF_description/urdf/BOTZO_URDF.urdf", startPos, startOrientation,
                     flags=p.URDF_USE_INERTIA_FROM_FILE)


for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)
    
cubePos, cubeOrn = p.getBasePositionAndOrientation(robotId)
print(cubePos,cubeOrn)
p.disconnect()