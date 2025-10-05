# Botzo Simulation

![leg](../assets/gifs/digital_twin_botzo_urdf.gif)

To create a digital twin of our real robot we built a URDF file from the CAD model in Fusion 360 (for more details see [here](https://ieroboticsailab.github.io/botzo/build/cad/urdf/)). A URDF file is a format used in robotics to describe the physical properties of a robot, including its geometry, links and meshes positions. It allows for simulation and visualization of the robot in simulation environments.

The goal will be to create an exact twin of botzo in a digital space, allowing us to test algorithms, simulate movements, and visualize the robot's behavior without needing the physical hardware. This is particularly useful for testing and development purposes.

Our final aim is to train and teach botzo to perform tasks using Reinforcement Learning in simulator engines such as Pybullet + Mujoco or IsaacLab. This will allow Botzo to learn how to stabilize his walking, climb stairs and even perform tricks like jumping or dancing, all in a safe and controlled environment before applying the learned behaviors to the physical robot. 

## `simulation/` folder

!!! note

    We recommend to create a virtual environment to run the code in this folder, and install PyBullet and Numpy.

- `utils/`: Contains a script used by the simulations scripts to import our Inverse Kinematics algorithm, which is used to calculate the angles of the motors based on the desired position of the robot's feet. Using the exact same algorithm used in the real robot, we can ensure that the simulation behaves similarly to the physical robot. (find [here](https://github.com/IERoboticsAILab/botzo/tree/main/simulation/utils))

- PyBullet scripts:
  - `spawn_botzo.py`: Load URDF, basic script of PyBullet (find [here](https://github.com/IERoboticsAILab/botzo/tree/main/simulation/spawn_botzo.py))
  - `move_one_joint.py`: Just to showcase how we access and move joints in Pybullet. So in this script we move one joint of botzo (find [here](https://github.com/IERoboticsAILab/botzo/tree/main/simulation/move_one_joint.py))
  - `debugger.py`: A script that allow us to move specific joints defined by the user. It is used to find the joint zeros, so we create a [function](https://github.com/IERoboticsAILab/botzo/blob/main/simulation/utils/IK_solver.py#L90) that transform simulation joint to real joint position (find [here](https://github.com/IERoboticsAILab/botzo/tree/main/simulation/debugger.py))
  - `IK_one_leg.py`: A script to test and visualize the Inverse Kinematics algorithm for one leg of botzo in PyBullet. It allows us to see how the IK algorithm calculates the joint angles needed to position the leg's foot at a desired location in 3D space. (find [here](https://github.com/IERoboticsAILab/botzo/tree/main/simulation/IK_one_leg.py))
  - `digital_twin.py`: The main script to simulate the full botzo robot in PyBullet using the URDF file. It incorporates the Inverse Kinematics algorithm to control the robot's movements, allowing us to test and visualize how botzo would behave in a digital environment. (find [here](https://github.com/IERoboticsAILab/botzo/blob/main/simulation/digital_twin.py))

---

[BOTZO SIMULATION REPO](https://github.com/IERoboticsAILab/botzo/tree/main/simulation)