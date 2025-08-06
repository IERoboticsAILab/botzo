# Botzo Simulation

To create a digital twin of our robot we built a URDF file from the CAD model in Fusion 360 (for more details see [here](https://github.com/IERoboticsAILab/botzo/tree/main/CAD_files/URDF%20file)). A URDF file is a format used in robotics to describe the physical properties of a robot, including its geometry, links and meshes positions. It allows for simulation and visualization of the robot in simulation environments.

The goal will be to create an exact twin of botzo in a digital space, allowing us to test algorithms, simulate movements, and visualize the robot's behavior without needing the physical hardware. This is particularly useful for testing and development purposes.

Our final aim is to train and teach botzo to perform tasks using Reinforcement Learning in simulator engines such as Pybullet + Mujoco or IsaacLab. This will allow Botzo to learn how to stabilize his walking, climb stairs and even perform tricks like jumping or dancing, all in a safe and controlled environment before applying the learned behaviors to the physical robot. For more informations about RL, see in [/reinforcement_learning/README.md](https://github.com/IERoboticsAILab/botzo/tree/main/simulation/reinforcement_learning).

## `simulation/` folder

!!! note

    We recommend to create a virtual environment to run the code in this folder, and install PyBullet and other dependencies.

- `meshes/`: Contains the meshes used and called by the URDF file.
- `urdf/`: Contains the URDF file of Botzo, which describes the robot's physical properties and structure.
- `reinforcement_learning/`: Contains the code and scripts related to Reinforcement Learning, where we will implement algorithms to train Botzo in the IsaacLab and IsaacSim simulators.
- `src/`: Contains a script used by the simulations scripts to import our Inverse Kinematics algorithm, which is used to calculate the angles of the motors based on the desired position of the robot's feet. Using the exact same algorithm used in the real robot, we can ensure that the simulation behaves similarly to the physical robot.

- PyBullet scripts:
  - `basic.py`: Load URDF, basic script of PyBullet
  - `hello_bullet.py`, `hello_bullet2.py`, `hello_bullet3.py`, `hello_bullet4.py`: These scripts are used to test the URDF file and the robot's movements in PyBullet. They demonstrate how to load the URDF file, set up the simulation environment, and control the robot's movements. This scripts where usefull for us to understand how to interact with PyBullet adn are great step by step examples to learn how to use the library.
  - `hello_bullet5.py`: After calibrating the zeros of each motor, we can pass coordinates and make Botzo walk. Achiving the desire goal of making a Digital Twin of botzo the help us to devellop new algorithms in a safe and controlled environment.

![leg](../assets/gifs/pybullet_digitaltwin.gif)
