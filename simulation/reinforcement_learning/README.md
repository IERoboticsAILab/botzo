<div align="center">
<h1>BOTZO 🐾</h1>

**`The good boy quadruped robot :)`**

<p align="center">
    <a href="https://www.instagram.com/botzo.ie/" target="_blank" rel="noopener noreferrer">
        <img alt="Instagram" src="https://img.shields.io/badge/Instagram-%232C3454.svg?style=for-the-badge&logo=Instagram&logoColor=white" />
    </a>
    <a href="" target="_blank" rel="noopener noreferrer">
        <img alt="LinkedIn" src="https://img.shields.io/badge/Youtube-%232C3454.svg?style=for-the-badge&logo=Youtube&logoColor=white" />
    </a>
    <a href="mailto:botzoteam@gmail.com">
        <img alt="Gmail" src="https://img.shields.io/badge/Gmail-2c3454?style=for-the-badge&logo=gmail&logoColor=white" />
    </a>
    <a href="">
        <img alt="Views" src="https://komarev.com/ghpvc/?username=botzo&color=blue&style=for-the-badge&abbreviated=true" />
    </a>

more about botzo [here](https://github.com/IERoboticsAILab/botzo)

for specific Isaac RL Botzo documentation [here](https://ieroboticsailab.github.io/botzo/learn/reinforcement-learning/)

![results]()

</p>
</div>

File structure:

-- BOTZO_URDF
    |-- Looks
    |-- joints
        |-- LF_HAA
        |-- LF_HFE
        |-- LF_KFE
        |-- LF_shank_fixed_LF_FOOT
        |-- LH_HAA
        |-- LH_HFE
        |-- LH_KFE
        |-- LH_shank_fixed_LH_FOOT
        |-- RF_HAA
        |-- RF_HFE
        |-- RF_KFE
        |-- RF_shank_fixed_RF_FOOT
        |-- RH_HAA
        |-- RH_HFE
        |-- RH_KFE
        |-- RH_shank_fixed_RH_FOOT
    |-- base
        |-- ...
        |-- imu_link
        |-- ...
    |-- LF_HIP
    |-- LF_THIGH
    |-- LF_SHANK
    |-- LF_FOOT
    |-- LH_HIP
    |-- LH_THIGH
    |-- LH_SHANK
    |-- LH_FOOT
    |-- RF_HIP
    |-- RF_THIGH
    |-- RF_SHANK
    |-- RF_FOOT
    |-- RH_HIP
    |-- RH_THIGH
    |-- RH_SHANK
    |-- RH_FOOT

1. Create USD file of botzo for Isaac Lab: 
    a. Open Isaac Sim  `cd C:\isaacsim\` > `.\isaac-sim.bat`
    b. File > Import > Select botzo urdf file
    c. Select **Movable base**
    d. It will automatically create a folder with the USD file and all the parts

2. Create Botzo Isaac Lab asset
    a. Go to `IsaacLab\IsaacLab\source\isaaclab_assets\isaaclab_assets\robots`
    b. Create new python file named `botzo.py`
    c. Set up the botzo loading used to easily load the botzo asset in Isaac Lab
    d. For debugging purposes use isaac sim example to load a USD file

3. Test on Isaac Lab
    a. `conda activate env_isaaclab`
    b. `cd C:\Users\grego\Desktop\GRINGO\IsaacLab\IsaacLab`
    c. `python .\scripts\tutorials\01_assets\add_new_robot.py`, etc... *try all examples*

4. Train robot dog to walk using RL
    a. Go to `IsaacLab\IsaacLab\source\isaaclab_tasks\isaaclab_tasks\manager_based\locomotion`
    b. Create your custom robot config in `isaaclab_tasks\isaaclab_tasks\manager_based\locomotion\velocity\config\<custom_robot_name>` (Copy and paste from another robot and modify as needed observations, rewards and hyperparameters)
    c. Train using: `python .\scripts\reinforcement_learning\skrl\train.py --task=Isaac-Velocity-Flat-Botzo-v0 --headless --video`
    d. Monitor training with tensorboard: `(env_isaaclab) PS C:\Users\grego\Desktop\GRINGO\IsaacLab\IsaacLab> tensorboard --logdir .\logs\`