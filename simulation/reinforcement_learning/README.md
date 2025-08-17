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

</p>

</div>

more [here](https://github.com/IERoboticsAILab/botzo)

# Reinforcement Learning
In this repo we will explore how we teach botzo to walk

## Resources

- Genesis [link](https://genesis-world.readthedocs.io/en/latest/user_guide/index.html)
- Basics [link](https://youtu.be/f6LkEQsXGF8?si=eqC7IzNiBTZROUgm)
- theconstrucsim course Intermediate Generative AI for Robotics [link](https://app.theconstruct.ai/courses/intermediate-generative-ai-for-robotics-276/)
- theconstrucsim course Mastering Reinforcement Learning for Robotics [link](https://app.theconstruct.ai/courses/mastering-reinforcement-learning-for-robotics-286/)
- [Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/index.html)

# Isaac Sim

<p align="center">
  <img src="https://github.com/IERoboticsAILab/botzo/blob/main/media_assests/botzo_btw_friends.png" width="350"/>
</p>
<p align="center"><em>Botzo between the giants & friends</em></p>


Computer specifications:
| Component | Specification |
|-----------|---------------|
| RAM       | 64GB          |
| GPU       | RTX 4090      |
| VRAM      | 16GB          |
| OS        | Windows 11    |
| Storage   | 1TB           |

>_Check system requirements compatibility and satisfaction with [this](https://download.isaacsim.omniverse.nvidia.com/isaac-sim-comp-check%404.5.0-rc.6%2Brelease.675.f1cca148.gl.windows-x86_64.release.zip)_

<img src="https://raw.githubusercontent.com/IERoboticsAILab/botzo/main/media_assests/isaac_compatability_check.png" alt="ours" width="250"/>

Download [IsaacSim](https://download.isaacsim.omniverse.nvidia.com/isaac-sim-standalone%404.5.0-rc.36%2Brelease.19112.f59b3005.gl.windows-x86_64.release.zip) for your OS

```shell
mkdir C:\isaacsim
cd %USERPROFILE%/Downloads
tar -xvzf "isaac-sim-standalone@4.5.0-rc.36+release.19112.f59b3005.gl.windows-x86_64.release.zip" -C C:\isaacsim
cd C:\isaacsim
post_install.bat
isaac-sim.selector.bat
```
> [!NOTE] 
>The Isaac Sim app can be run directly via command line with `isaac-sim.bat`. Start new empty simulation.

> [!NOTE] 
>_extension_examples_ and _standalone_examples_ folder for the tutorials (ex: `.\python.bat .\standalone_examples\api\isaacsim.asset.importer.urdf\urdf_import.py`)

### Getting started with Isaac Sim

For more tutorials: Documentation [here](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/introduction/quickstart_index.html#isaac-sim-intro-quickstart-series)

#### Import URDF

`.\python.bat ..\Users\grego\Desktop\GRINGO\botzo\botzo\simulation\reinforcment_learning\standalone_examples\import_botzo_urdf.py`

And then save it as a USD file. This file is used to import and manipulate your robot in Isaac Lab using python APIs. 

#### Import USD

`.\python.bat ..\Users\grego\Desktop\GRINGO\botzo\botzo\simulation\reinforcment_learning\standalone_examples\load_botzo_usd.py --usd_path "C:\Users\grego\Desktop\GRINGO\botzo\botzo\simulation\reinforcment_learning\botzo_USD\botzo_USD.usd"`

![result](https://github.com/IERoboticsAILab/botzo/blob/main/media_assests/isaac_sim_botzo.png)

<br>

<br>

# Isaac Lab

Isaac Lab is a collection of tools and libraries for building and simulating robots in Isaac Sim. It provides a set of APIs for creating robots, sensors, and environments, as well as tools for training and deploying reinforcement learning agents.

After the installation of Isaac Sim and Isaac Lab, you run examples such as:

```shell
(env_isaaclab) C:\Users\grego\Desktop\GRINGO\IsaacLab\IsaacLab>python scripts\tutorials\03_envs\create_quadruped_base_env.py
```

![example](https://github.com/IERoboticsAILab/botzo/blob/main/media_assests/isaaclab.gif)


## Create your robot Configuration

To import your custom robot it is recomended to create a new Robot configuration python script.

Go to isaaclab/source/isaaclab_assets/robots and create a new python file with the name of your robot, for example `botzo.py`. Here we will import the USD file, and set robot properties.


<details>
<summary><b> `botzo.py` Code Robot configuration script </b></summary>

```python

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

stiffness = 2000.0     # N⋅m/rad - Very stiff for standing
damping = 200.0        # N⋅m⋅s/rad - High damping for stability
effort_limit_sim = 4.0 # N⋅m - Allow higher than rated for simulation
velocity_limit_sim = 10.0 # rad/s - Slightly higher for responsiveness

stiffness *= 40
damping *= 15
effort_limit_sim *= 25
velocity_limit_sim *= 10

BOTZO_CONFIG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="C:\\Users\\grego\\Desktop\\GRINGO\\botzo\\botzo\\simulation\\reinforcement_learning\\botzo_USD\\botzo_USD.usd",
        scale=(0.3, 0.3, 0.3),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "BL_shoulder_joint": 0.0,
            "BL_femur_joint": 0.0,
            "BL_knee_joint": 0.0,

            "BR_shoulder_joint": 0.0,
            "BR_femur_joint": 0.0,
            "Revolute_43": 0.0,

            "FL_shoulder_joint": 0.0,
            "FL_femur_joint": 0.0,
            "FL_knee_joint": 0.0,

            "FR_shoulder_joint": 0.0,
            "FR_femur_joint": 0.0,
            "FR_knee_joint": 0.0,
        },
        pos=(0.0, 0.0, 0.25),
        # rotate 90 degrees around the x-axis
        rot=(-0.7071, 0.0, 0.0, 0.7071),  # Quaternion for 90 degrees rotation around x-axis
        # rotate upside down
        #rot=(0.0, 1.0, 0.0, 0.0),
    ),
    actuators={
        "BL_shoulder_act": ImplicitActuatorCfg(joint_names_expr=["BL_shoulder_joint"],effort_limit_sim=effort_limit_sim, velocity_limit_sim=velocity_limit_sim, stiffness=stiffness, damping=damping),
        "BL_femur_act": ImplicitActuatorCfg(joint_names_expr=["BL_femur_joint"], effort_limit_sim=effort_limit_sim, velocity_limit_sim=velocity_limit_sim, stiffness=stiffness, damping=damping),
        "BL_knee_act": ImplicitActuatorCfg(joint_names_expr=["BL_knee_joint"], effort_limit_sim=effort_limit_sim, velocity_limit_sim=velocity_limit_sim, stiffness=stiffness, damping=damping),
        "BR_shoulder_act": ImplicitActuatorCfg(joint_names_expr=["BR_shoulder_joint"], effort_limit_sim=effort_limit_sim, velocity_limit_sim=velocity_limit_sim, stiffness=stiffness, damping=damping),
        "BR_femur_act": ImplicitActuatorCfg(joint_names_expr=["BR_femur_joint"], effort_limit_sim=effort_limit_sim, velocity_limit_sim=velocity_limit_sim, stiffness=stiffness, damping=damping),
        "Revolute_43_act": ImplicitActuatorCfg(joint_names_expr=["Revolute_43"], effort_limit_sim=effort_limit_sim, velocity_limit_sim=velocity_limit_sim, stiffness=stiffness, damping=damping),
        "FL_shoulder_act": ImplicitActuatorCfg(joint_names_expr=["FL_shoulder_joint"], effort_limit_sim=effort_limit_sim, velocity_limit_sim=velocity_limit_sim, stiffness=stiffness, damping=damping),
        "FL_femur_act": ImplicitActuatorCfg(joint_names_expr=["FL_femur_joint"], effort_limit_sim=effort_limit_sim, velocity_limit_sim=velocity_limit_sim, stiffness=stiffness, damping=damping),
        "FL_knee_act": ImplicitActuatorCfg(joint_names_expr=["FL_knee_joint"], effort_limit_sim=effort_limit_sim, velocity_limit_sim=velocity_limit_sim, stiffness=stiffness, damping=damping),
        "FR_shoulder_act": ImplicitActuatorCfg(joint_names_expr=["FR_shoulder_joint"], effort_limit_sim=effort_limit_sim, velocity_limit_sim=velocity_limit_sim, stiffness=stiffness, damping=damping),
        "FR_femur_act": ImplicitActuatorCfg(joint_names_expr=["FR_femur_joint"], effort_limit_sim=effort_limit_sim, velocity_limit_sim=velocity_limit_sim, stiffness=stiffness, damping=damping),
        "FR_knee_act": ImplicitActuatorCfg(joint_names_expr=["FR_knee_joint"], effort_limit_sim=effort_limit_sim, velocity_limit_sim=velocity_limit_sim, stiffness=stiffness, damping=damping),
    },
)

```

</details>

<br>

<details>
<summary><b> `dofrobot.py` Code Robot configuration script </b></summary>

```python

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

DOFBOT_CONFIG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/Dofbot/dofbot.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "joint1": 0.0,
            "joint2": 0.0,
            "joint3": 0.0,
            "joint4": 0.0,
        },
        pos=(0.25, -0.25, 0.0),
    ),
    actuators={
        "front_joints": ImplicitActuatorCfg(
            joint_names_expr=["joint[1-2]"],
            effort_limit_sim=100.0,
            velocity_limit_sim=100.0,
            stiffness=10000.0,
            damping=100.0,
        ),
        "joint3_act": ImplicitActuatorCfg(
            joint_names_expr=["joint3"],
            effort_limit_sim=100.0,
            velocity_limit_sim=100.0,
            stiffness=10000.0,
            damping=100.0,
        ),
        "joint4_act": ImplicitActuatorCfg(
            joint_names_expr=["joint4"],
            effort_limit_sim=100.0,
            velocity_limit_sim=100.0,
            stiffness=10000.0,
            damping=100.0,
        ),
    },
)

```

</details>

<br>

<details>
<summary><b> `jetbot.py` Code Robot configuration script </b></summary>

```python

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

JETBOT_CONFIG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/Jetbot/jetbot.usd"),
    actuators={"wheel_acts": ImplicitActuatorCfg(joint_names_expr=[".*"], damping=None, stiffness=None)},
)

```

</details>




### Use your robot configuration

Now that we configured our robot, we can simply import it and use it in any python file we want

1. Import robot config: `from isaaclab_assets.robots.botzo import BOTZO_CONFIG`
2. Spawn robot: `Botzo = BOTZO_CONFIG.replace(prim_path="{ENV_REGEX_NS}/Botzo")`
3. Now you can move and interact with it

#### Our Examples:

1. `python scripts\demos\quadrupeds.py`

<details>
<summary><b> Spawn various robots </b></summary>

```python
# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
This script demonstrates different legged robots.

.. code-block:: bash

    # Usage
    ./isaaclab.sh -p scripts/demos/quadrupeds.py

"""

"""Launch Isaac Sim Simulator first."""

import argparse

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="This script demonstrates different legged robots.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import numpy as np
import torch

import isaacsim.core.utils.prims as prim_utils

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation

##
# Pre-defined configs
##
from isaaclab_assets.robots.anymal import ANYMAL_B_CFG, ANYMAL_C_CFG, ANYMAL_D_CFG  # isort:skip
from isaaclab_assets.robots.spot import SPOT_CFG  # isort:skip
from isaaclab_assets.robots.unitree import UNITREE_A1_CFG, UNITREE_GO1_CFG, UNITREE_GO2_CFG  # isort:skip
from isaaclab_assets.robots.botzo import BOTZO_CONFIG  # isort:skip


def define_origins(num_origins: int, spacing: float) -> list[list[float]]:
    """Defines the origins of the scene."""
    # create tensor based on number of environments
    env_origins = torch.zeros(num_origins, 3)
    # create a grid of origins
    num_cols = np.floor(np.sqrt(num_origins))
    num_rows = np.ceil(num_origins / num_cols)
    xx, yy = torch.meshgrid(torch.arange(num_rows), torch.arange(num_cols), indexing="xy")
    env_origins[:, 0] = spacing * xx.flatten()[:num_origins] - spacing * (num_rows - 1) / 2
    env_origins[:, 1] = spacing * yy.flatten()[:num_origins] - spacing * (num_cols - 1) / 2
    env_origins[:, 2] = 0.0
    # return the origins
    return env_origins.tolist()


def design_scene() -> tuple[dict, list[list[float]]]:
    """Designs the scene."""
    # Ground-plane
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg)
    # Lights
    cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/Light", cfg)

    # Create separate groups called "Origin1", "Origin2", "Origin3"
    # Each group will have a mount and a robot on top of it
    origins = define_origins(num_origins=8, spacing=1.25)

    # Origin 1 with Anymal B
    prim_utils.create_prim("/World/Origin1", "Xform", translation=origins[0])
    # -- Robot
    anymal_b = Articulation(ANYMAL_B_CFG.replace(prim_path="/World/Origin1/Robot"))

    # Origin 2 with Anymal C
    prim_utils.create_prim("/World/Origin2", "Xform", translation=origins[1])
    # -- Robot
    anymal_c = Articulation(ANYMAL_C_CFG.replace(prim_path="/World/Origin2/Robot"))

    # Origin 3 with Anymal D
    prim_utils.create_prim("/World/Origin3", "Xform", translation=origins[2])
    # -- Robot
    anymal_d = Articulation(ANYMAL_D_CFG.replace(prim_path="/World/Origin3/Robot"))

    # Origin 4 with Unitree A1
    prim_utils.create_prim("/World/Origin4", "Xform", translation=origins[3])
    # -- Robot
    unitree_a1 = Articulation(UNITREE_A1_CFG.replace(prim_path="/World/Origin4/Robot"))

    # Origin 5 with Unitree Go1
    prim_utils.create_prim("/World/Origin5", "Xform", translation=origins[4])
    # -- Robot
    unitree_go1 = Articulation(UNITREE_GO1_CFG.replace(prim_path="/World/Origin5/Robot"))

    # Origin 6 with Unitree Go2
    prim_utils.create_prim("/World/Origin6", "Xform", translation=origins[5])
    # -- Robot
    unitree_go2 = Articulation(UNITREE_GO2_CFG.replace(prim_path="/World/Origin6/Robot"))

    # Origin 7 with Boston Dynamics Spot
    prim_utils.create_prim("/World/Origin7", "Xform", translation=origins[6])
    # -- Robot
    spot = Articulation(SPOT_CFG.replace(prim_path="/World/Origin7/Robot"))

    # Origin 8 with Botzo
    prim_utils.create_prim("/World/Origin8", "Xform", translation=origins[7])
    # -- Robot
    botzo = Articulation(BOTZO_CONFIG.replace(prim_path="/World/Origin8/Robot"))

    # return the scene information
    scene_entities = {
        "anymal_b": anymal_b,
        "anymal_c": anymal_c,
        "anymal_d": anymal_d,
        "unitree_a1": unitree_a1,
        "unitree_go1": unitree_go1,
        "unitree_go2": unitree_go2,
        "spot": spot,
        "botzo": botzo,
    }
    return scene_entities, origins


def run_simulator(sim: sim_utils.SimulationContext, entities: dict[str, Articulation], origins: torch.Tensor):
    """Runs the simulation loop."""
    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0
    # Simulate physics
    while simulation_app.is_running():
        # reset
        if count % 200 == 0:
            # reset counters
            sim_time = 0.0
            count = 0
            # reset robots
            for index, robot in enumerate(entities.values()):
                # root state
                root_state = robot.data.default_root_state.clone()
                root_state[:, :3] += origins[index]
                robot.write_root_pose_to_sim(root_state[:, :7])
                robot.write_root_velocity_to_sim(root_state[:, 7:])
                # joint state
                joint_pos, joint_vel = robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()
                robot.write_joint_state_to_sim(joint_pos, joint_vel)
                # reset the internal state
                robot.reset()
            print("[INFO]: Resetting robots state...")
        # apply default actions to the quadrupedal robots
        for robot in entities.values():
            # generate random joint positions
            joint_pos_target = robot.data.default_joint_pos + torch.randn_like(robot.data.joint_pos) * 0.1
            # apply action to the robot
            robot.set_joint_position_target(joint_pos_target)
            # write data to sim
            robot.write_data_to_sim()
        # perform step
        sim.step()
        # update sim-time
        sim_time += sim_dt
        count += 1
        # update buffers
        for robot in entities.values():
            robot.update(sim_dt)


def main():
    """Main function."""

    # Initialize the simulation context
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=0.01))
    # Set main camera
    sim.set_camera_view(eye=[2.5, 2.5, 2.5], target=[0.0, 0.0, 0.0])
    # design scene
    scene_entities, scene_origins = design_scene()
    scene_origins = torch.tensor(scene_origins, device=sim.device)
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # Run the simulator
    run_simulator(sim, scene_entities, scene_origins)


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()

```

</details>

2. `python scripts\tutorials\01_assets\add_new_robot.py`

<details>
<summary><b> Spawn various robots and move them </b></summary>

```python
# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(
    description="This script demonstrates adding a custom robot to an Isaac Lab environment."
)
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import AssetBaseCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR


from isaaclab_assets.robots.botzo import BOTZO_CONFIG # source\isaaclab_assets\isaaclab_assets\robots\botzo.py

JETBOT_CONFIG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/Jetbot/jetbot.usd"),
    actuators={"wheel_acts": ImplicitActuatorCfg(joint_names_expr=[".*"], damping=None, stiffness=None)},
)

DOFBOT_CONFIG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/Dofbot/dofbot.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "joint1": 0.0,
            "joint2": 0.0,
            "joint3": 0.0,
            "joint4": 0.0,
        },
        pos=(0.25, -0.25, 0.0),
    ),
    actuators={
        "front_joints": ImplicitActuatorCfg(
            joint_names_expr=["joint[1-2]"],
            effort_limit_sim=100.0,
            velocity_limit_sim=100.0,
            stiffness=10000.0,
            damping=100.0,
        ),
        "joint3_act": ImplicitActuatorCfg(
            joint_names_expr=["joint3"],
            effort_limit_sim=100.0,
            velocity_limit_sim=100.0,
            stiffness=10000.0,
            damping=100.0,
        ),
        "joint4_act": ImplicitActuatorCfg(
            joint_names_expr=["joint4"],
            effort_limit_sim=100.0,
            velocity_limit_sim=100.0,
            stiffness=10000.0,
            damping=100.0,
        ),
    },
)


class NewRobotsSceneCfg(InteractiveSceneCfg):
    """Designs the scene."""

    # Ground-plane
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )

    # robot
    Jetbot = JETBOT_CONFIG.replace(prim_path="{ENV_REGEX_NS}/Jetbot")
    Dofbot = DOFBOT_CONFIG.replace(prim_path="{ENV_REGEX_NS}/Dofbot")
    Botzo = BOTZO_CONFIG.replace(prim_path="{ENV_REGEX_NS}/Botzo")


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0

    while simulation_app.is_running():
        # reset
        if count % 500 == 0:
            # reset counters
            count = 0
            # reset the scene entities to their initial positions offset by the environment origins
            root_jetbot_state = scene["Jetbot"].data.default_root_state.clone()
            root_jetbot_state[:, :3] += scene.env_origins
            root_dofbot_state = scene["Dofbot"].data.default_root_state.clone()
            root_dofbot_state[:, :3] += scene.env_origins
            robot_botzo_state = scene["Botzo"].data.default_root_state.clone()
            robot_botzo_state[:, :3] += scene.env_origins

            # copy the default root state to the sim for the jetbot's orientation and velocity
            scene["Jetbot"].write_root_pose_to_sim(root_jetbot_state[:, :7])
            scene["Jetbot"].write_root_velocity_to_sim(root_jetbot_state[:, 7:])
            scene["Dofbot"].write_root_pose_to_sim(root_dofbot_state[:, :7])
            scene["Dofbot"].write_root_velocity_to_sim(root_dofbot_state[:, 7:])
            scene["Botzo"].write_root_pose_to_sim(robot_botzo_state[:, :7])
            scene["Botzo"].write_root_velocity_to_sim(robot_botzo_state[:, 7:])

            # copy the default joint states to the sim
            joint_pos, joint_vel = (
                scene["Jetbot"].data.default_joint_pos.clone(),
                scene["Jetbot"].data.default_joint_vel.clone(),
            )
            scene["Jetbot"].write_joint_state_to_sim(joint_pos, joint_vel)
            joint_pos, joint_vel = (
                scene["Dofbot"].data.default_joint_pos.clone(),
                scene["Dofbot"].data.default_joint_vel.clone(),
            )
            scene["Dofbot"].write_joint_state_to_sim(joint_pos, joint_vel)
            joint_pos, joint_vel = (
                scene["Botzo"].data.default_joint_pos.clone(),
                scene["Botzo"].data.default_joint_vel.clone(),
            )
            scene["Botzo"].write_joint_state_to_sim(joint_pos, joint_vel)
            # clear internal buffers
            scene.reset()
            print("[INFO]: Resetting Jetbot and Dofbot state...")

        # drive around
        if count % 100 < 75:
            # Drive straight by setting equal wheel velocities
            action = torch.Tensor([[10.0, 10.0]])
        else:
            # Turn by applying different velocities
            action = torch.Tensor([[5.0, -5.0]])

        scene["Jetbot"].set_joint_velocity_target(action)

        # wave
        wave_action = scene["Dofbot"].data.default_joint_pos
        wave_action[:, 0:4] = 0.25 * np.sin(2 * np.pi * 0.5 * sim_time)
        scene["Dofbot"].set_joint_position_target(wave_action)
        # move botzo
        botzo_action = scene["Botzo"].data.default_joint_pos
        botzo_action[:, 0:3] = 0.25 * np.sin(2 * np.pi * 0.5 * sim_time)
        botzo_action[:, 3:6] = 0.25 * np.cos(2 * np.pi * 0.5 * sim_time)
        scene["Botzo"].set_joint_position_target(botzo_action)

        scene.write_data_to_sim()
        sim.step()
        sim_time += sim_dt
        count += 1
        scene.update(sim_dt)


def main():
    """Main function."""
    # Initialize the simulation context
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)

    sim.set_camera_view([3.5, 0.0, 3.2], [0.0, 0.0, 0.5])
    # design scene
    scene_cfg = NewRobotsSceneCfg(args_cli.num_envs, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # Run the simulator
    run_simulator(sim, scene)


if __name__ == "__main__":
    main()
    simulation_app.close()

```

</details>

3. `python scripts\tutorials\01_assets\botzo_move_IK.py `

<details>
<summary><b> Create digital twin of botzo and move it with our IK </b></summary>

```python
import argparse

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(
    description="This script demonstrates adding a custom robot to an Isaac Lab environment."
)
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import AssetBaseCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR


from isaaclab_assets.robots.botzo import BOTZO_CONFIG # source\isaaclab_assets\isaaclab_assets\robots\botzo.py
from botzo_IK_solver import *
import math


class NewRobotsSceneCfg(InteractiveSceneCfg):
    """Designs the scene."""

    # Ground-plane
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )

    # robot
    Botzo = BOTZO_CONFIG.replace(prim_path="{ENV_REGEX_NS}/Botzo")


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0

    idx = 0

    while simulation_app.is_running():
        # reset
        if count % 500 == 0:
            # reset counters
            count = 0
            # reset the scene entities to their initial positions offset by the environment origins
            robot_botzo_state = scene["Botzo"].data.default_root_state.clone()
            robot_botzo_state[:, :3] += scene.env_origins

            # copy the default root state to the sim for the jetbot's orientation and velocity
            scene["Botzo"].write_root_pose_to_sim(robot_botzo_state[:, :7])
            scene["Botzo"].write_root_velocity_to_sim(robot_botzo_state[:, 7:])

            # copy the default joint states to the sim
            joint_pos, joint_vel = (
                scene["Botzo"].data.default_joint_pos.clone(),
                scene["Botzo"].data.default_joint_vel.clone(),
            )
            scene["Botzo"].write_joint_state_to_sim(joint_pos, joint_vel)
            # clear internal buffers
            scene.reset()
            print("[INFO]: Resetting Botzo state...")



        # move botzo
        '''
        tensor([[
                0  -  BL_shoulder, 
                1  -  BR_shoulder, 
                2  -  FL_shoulder, 
                3  -  FR_shoulder, 

                4  -  BL_femur, 
                5  -  BR_femur, 
                6  -  FL_femur, 
                7  -  FR_femur, 

                8  -  BL_knee, 
                9  -  BR_knee, 
                10 -  FL_knee, 
                11 -  FR_knee
                ]], device='cuda:0')
        '''
        botzo_action = scene["Botzo"].data.default_joint_pos
        # print the difference btw current pos and target pos
        #print(f"{'='*10}\nCurrent: {scene['Botzo'].data.joint_pos},\nTarget: {botzo_action}\n{'='*10}\n\n\n")
        tolerance = math.radians(5.0)
        if not torch.all(torch.abs(scene["Botzo"].data.joint_pos - botzo_action) < tolerance):
            #print("position not reached")
            pass
        else:
            #print("position reached, pass to new target")
            # calculate angles
            FR_s_f_t = legIK(forward_targets_FR_BL[idx][0], forward_targets_FR_BL[idx][1], forward_targets_FR_BL[idx][2])
            FR_angle_shoulder, FR_angle_femur, FR_angle_knee = FR_s_f_t
            target_FR_angle_shoulder = math.radians(real_sim_angle(FR_angle_shoulder,joint_ids["FR"]["shoulder"]))
            target_FR_angle_femur = math.radians(real_sim_angle(FR_angle_femur, joint_ids["FR"]["femur"]))
            target_FR_angle_knee = math.radians(real_sim_angle(FR_angle_knee, joint_ids["FR"]["knee"]))
            FL_s_f_t = legIK(forward_targets_FL_BR[idx][0], forward_targets_FL_BR[idx][1], forward_targets_FL_BR[idx][2])
            FL_angle_shoulder, FL_angle_femur, FL_angle_knee = FL_s_f_t
            target_FL_angle_shoulder = math.radians(real_sim_angle(FL_angle_shoulder,joint_ids["FL"]["shoulder"]))
            target_FL_angle_femur = math.radians(real_sim_angle(FL_angle_femur, joint_ids["FL"]["femur"]))
            target_FL_angle_knee = math.radians(real_sim_angle(FL_angle_knee, joint_ids["FL"]["knee"]))
            BR_s_f_t = legIK(forward_targets_FL_BR[idx][0], forward_targets_FL_BR[idx][1], forward_targets_FL_BR[idx][2])
            BR_angle_shoulder, BR_angle_femur, BR_angle_knee = BR_s_f_t
            target_BR_angle_shoulder = math.radians(real_sim_angle(BR_angle_shoulder,joint_ids["BR"]["shoulder"]))
            target_BR_angle_femur = math.radians(real_sim_angle(BR_angle_femur, joint_ids["BR"]["femur"]))
            target_BR_angle_knee = math.radians(real_sim_angle(BR_angle_knee, joint_ids["BR"]["knee"]))
            BL_s_f_t = legIK(forward_targets_FR_BL[idx][0], forward_targets_FR_BL[idx][1], forward_targets_FR_BL[idx][2])
            BL_angle_shoulder, BL_angle_femur, BL_angle_knee = BL_s_f_t
            target_BL_angle_shoulder = math.radians(real_sim_angle(BL_angle_shoulder,joint_ids["BL"]["shoulder"]))
            target_BL_angle_femur = math.radians(real_sim_angle(BL_angle_femur, joint_ids["BL"]["femur"]))
            target_BL_angle_knee = math.radians(real_sim_angle(BL_angle_knee, joint_ids["BL"]["knee"]))
            
            if idx >= len(forward_targets_FR_BL)-1:
                idx = 0  # Reset index if it exceeds the number of targets
            else:
                idx += 1
        # set the angles to the botzo action
        botzo_action[:, 0] = target_BL_angle_shoulder
        botzo_action[:, 1] = target_BR_angle_shoulder
        botzo_action[:, 2] = target_FL_angle_shoulder
        botzo_action[:, 3] = target_FR_angle_shoulder
        botzo_action[:, 4] = target_BL_angle_femur
        botzo_action[:, 5] = target_BR_angle_femur
        botzo_action[:, 6] = target_FL_angle_femur
        botzo_action[:, 7] = target_FR_angle_femur
        botzo_action[:, 8] = target_BL_angle_knee
        botzo_action[:, 9] = target_BR_angle_knee
        botzo_action[:, 10] = target_FL_angle_knee
        botzo_action[:, 11] = target_FR_angle_knee
        # write the action to the sim
        scene["Botzo"].set_joint_position_target(botzo_action)



        scene.write_data_to_sim()
        sim.step()
        sim_time += sim_dt
        count += 1
        scene.update(sim_dt)


def main():
    """Main function."""
    # Initialize the simulation context
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)

    sim.set_camera_view([3.5, 0.0, 3.2], [0.0, 0.0, 0.5])
    # design scene
    scene_cfg = NewRobotsSceneCfg(args_cli.num_envs, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # Run the simulator
    run_simulator(sim, scene)


if __name__ == "__main__":
    main()
    simulation_app.close()

```

</details>


4. `python scripts\tutorials\05_controllers\botzo_diff_ik.py`

<details>
<summary><b> Create digital twin of botzo and move it with NVIDIA IK tools </b></summary>

```python
work in progress
```

</details>





## Isaac Lab RL templates

By having the robot config file setup, now we can create a RL agent environment template, that will contain all the necessary scripts to run different RL algorithm to teach our robot how to perfom our desire task.

This termplates can be found in the `isaaclab/source/isaaclab_tasks` directory.

They are divided into direct and manager based (both with their pros and cons (check isaac lab documentation for more infos)).

Each template is designed to be easily customizable for different robot configurations and tasks. They contain agents, that refer to the RL algorithm that we will use to train our robot to perform our desire task. This algorithm are taken form Stable Baselines3 or Skrl.

Then we have the **environment configuration** files, which define the specific settings and parameters for each environment. These files are crucial for ensuring that the RL agent can effectively interact with the simulation and learn from it (**environment properties**).  And the **environment** files are also responsible for defining the observation and action spaces for the RL agent (**reset, get reward, observations, etc...**).

<details>
<summary><b>Jetbot RL example</b></summary>


Isaac Lab Documentation [here](https://isaac-sim.github.io/IsaacLab/main/source/setup/walkthrough/index.html)

### Train Jetbot to drive forward

![example](https://github.com/IERoboticsAILab/botzo/blob/main/media_assests/isaac_lab_train_jetbot.gif)

```shell
isaaclab.bat --new
python scripts\environments\list_envs.py      # find for the task/project/template just created
# if you don't find it try this comand:  python -m pip install -e source\isaaclab_tasks
python scripts\reinforcement_learning\skrl\train.py --task=Isaac-Jetbot-Marl-Direct-v0
python scripts\reinforcement_learning\skrl\play.py --task=Isaac-Jetbot-Marl-Direct-v0
```

### Train Jetbot to follow commands (Controller with RL)

Objective: now start modifying our observations and rewards in order to train a policy to act as a controller for the Jetbot. As a user, we would like to be able to specify the desired direction for the Jetbot to drive, and have the wheels turn such that the robot drives in that specified direction as fast as possible.


1. Create template:

```shell
isaaclab.bat --new # will create new task such as: Generating 'Isaac-Jetbot-Controller-Direct-v0'
```

2. Check the task:

```shell
python scripts\environments\list_envs.py
```

3. create the logic for setting commands for each Jetbot on the stage. Each command will be a unit vector, and we need one for every clone of the robot on the stage, which means a tensor of shape [num_envs, 3]. And setup visualizations, so we can more easily tell what the policy is doing during training and inference. So we define two arrow VisualizationMarkers: one to represent the “forward” direction of the robot, and one to represent the command direction. When the policy is fully trained, these arrows should be aligned! 

4. Add arrow visualization to the `*_env.py`:

```python
from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
import isaaclab.utils.math as math_utils

def define_markers() -> VisualizationMarkers:
    """Define markers with various different shapes."""
    marker_cfg = VisualizationMarkersCfg(
        prim_path="/Visuals/myMarkers",
        markers={
                "forward": sim_utils.UsdFileCfg(
                    usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/arrow_x.usd",
                    scale=(0.25, 0.25, 0.5),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 1.0)),
                ),
                "command": sim_utils.UsdFileCfg(
                    usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/arrow_x.usd",
                    scale=(0.25, 0.25, 0.5),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),
                ),
        },
    )
    return VisualizationMarkers(cfg=marker_cfg)
```

5. expand the initialization and setup steps to construct the data we need for tracking the commands as well as the marker positions and rotations. Replace the contents of _setup_scene with the following

```python
def _setup_scene(self):
    self.robot = Articulation(self.cfg.robot_cfg)
    # add ground plane
    spawn_ground_plane(prim_path="/World/ground", cfg=GroundPlaneCfg())
    # clone and replicate
    self.scene.clone_environments(copy_from_source=False)
    # add articulation to scene
    self.scene.articulations["robot"] = self.robot
    # add lights
    light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
    light_cfg.func("/World/Light", light_cfg)

    self.visualization_markers = define_markers()

    # setting aside useful variables for later
    self.up_dir = torch.tensor([0.0, 0.0, 1.0]).cuda()
    self.yaws = torch.zeros((self.cfg.scene.num_envs, 1)).cuda()
    self.commands = torch.randn((self.cfg.scene.num_envs, 3)).cuda()
    self.commands[:,-1] = 0.0
    self.commands = self.commands/torch.linalg.norm(self.commands, dim=1, keepdim=True)

    # offsets to account for atan range and keep things on [-pi, pi]
    ratio = self.commands[:,1]/(self.commands[:,0]+1E-8)
    gzero = torch.where(self.commands > 0, True, False)
    lzero = torch.where(self.commands < 0, True, False)
    plus = lzero[:,0]*gzero[:,1]
    minus = lzero[:,0]*lzero[:,1]
    offsets = torch.pi*plus - torch.pi*minus
    self.yaws = torch.atan(ratio).reshape(-1,1) + offsets.reshape(-1,1)

    self.marker_locations = torch.zeros((self.cfg.scene.num_envs, 3)).cuda()
    self.marker_offset = torch.zeros((self.cfg.scene.num_envs, 3)).cuda()
    self.marker_offset[:,-1] = 0.5
    self.forward_marker_orientations = torch.zeros((self.cfg.scene.num_envs, 4)).cuda()
    self.command_marker_orientations = torch.zeros((self.cfg.scene.num_envs, 4)).cuda()
```

<img src="https://isaac-sim.github.io/IsaacLab/main/_images/walkthrough_training_vectors.svg" alt="quat_calc" width="300"/>

6. Next we have the method for actually visualizing the markers. Remember, these markers aren’t scene entities! We need to “draw” them whenever we want to see them.

```python
def _visualize_markers(self):
    # get marker locations and orientations
    self.marker_locations = self.robot.data.root_pos_w
    self.forward_marker_orientations = self.robot.data.root_quat_w
    self.command_marker_orientations = math_utils.quat_from_angle_axis(self.yaws, self.up_dir).squeeze()

    # offset markers so they are above the jetbot
    loc = self.marker_locations + self.marker_offset
    loc = torch.vstack((loc, loc))
    rots = torch.vstack((self.forward_marker_orientations, self.command_marker_orientations))

    # render the markers
    all_envs = torch.arange(self.cfg.scene.num_envs)
    indices = torch.hstack((torch.zeros_like(all_envs), torch.ones_like(all_envs)))
    self.visualization_markers.visualize(loc, rots, marker_indices=indices)
```

7. See result for now: `python scripts\reinforcement_learning\skrl\train.py --task=Isaac-Jetbot-Controller-Direct-v0`

8. Observations

```python
def _get_observations(self) -> dict:
    self.velocity = self.robot.data.root_com_vel_w
    self.forwards = math_utils.quat_apply(self.robot.data.root_link_quat_w, self.robot.data.FORWARD_VEC_B)
    obs = torch.hstack((self.velocity, self.commands))
    observations = {"policy": obs}
    return observations
```

9. Reward (When the robot is behaving as desired, it will be driving at full speed in the direction of the command. If we reward both “driving forward” and “alignment to the command”, then maximizing that combined signal)

```python
def _get_rewards(self) -> torch.Tensor:
    forward_reward = self.robot.data.root_com_lin_vel_b[:,0].reshape(-1,1)
    alignment_reward = torch.sum(self.forwards * self.commands, dim=-1, keepdim=True)
    total_reward = forward_reward + alignment_reward
    return total_reward
```

10. Test this setup: `python scripts\reinforcement_learning\skrl\train.py --task=Isaac-Jetbot-Controller-Direct-v0`

11. **Reward and Observation Tuning**

    a. keep the observation space as small as possible (reduce the number parameters & training time) 
    b. reduce and simplify the reward function as much as possible

    **SO**: encode our alignment to the command and our forward speed.

12. New observations

```python
def _get_observations(self) -> dict:
    self.velocity = self.robot.data.root_com_vel_w
    self.forwards = math_utils.quat_apply(self.robot.data.root_link_quat_w, self.robot.data.FORWARD_VEC_B)

    dot = torch.sum(self.forwards * self.commands, dim=-1, keepdim=True)
    cross = torch.cross(self.forwards, self.commands, dim=-1)[:,-1].reshape(-1,1)
    forward_speed = self.robot.data.root_com_lin_vel_b[:,0].reshape(-1,1)
    obs = torch.hstack((dot, cross, forward_speed))

    observations = {"policy": obs}
    return observations
```

13. The dot or inner product tells us how aligned two vectors are as a single scalar quantity


    **SO**: 
        
        Before: we are rewarding driving forward and being aligned to the command by adding them together, so our agent can be reward for driving forward OR being aligned to the command.

        Now: learn to drive in the direction of the command, we should only reward the agent driving forward AND being aligned. Logical AND suggests multiplication and therefore the following reward function:

14. New reward function

```python
def _get_rewards(self) -> torch.Tensor:
    forward_reward = self.robot.data.root_com_lin_vel_b[:,0].reshape(-1,1)
    alignment_reward = torch.sum(self.forwards * self.commands, dim=-1, keepdim=True)
    total_reward = forward_reward*alignment_reward
    return total_reward
```

15. Jetbots have learned to drive in reverse if the command is pointed behind them. (degenerate solutions)
16. New reward to avoid neg *neg = positive rewards but bad behavior

```python
def _get_rewards(self) -> torch.Tensor:
    forward_reward = self.robot.data.root_com_lin_vel_b[:,0].reshape(-1,1)
    alignment_reward = torch.sum(self.forwards * self.commands, dim=-1, keepdim=True)
    total_reward = forward_reward*torch.exp(alignment_reward)
    return total_reward
```

10. Train: `python scripts\reinforcement_learning\skrl\train.py --task=Isaac-Jetbot-Controller-Direct-v0`
11. Play: `python scripts\reinforcement_learning\skrl\play.py --task=Isaac-Jetbot-Controller-Direct-v0`

![result](https://github.com/IERoboticsAILab/botzo/blob/main/media_assests/jetbot_controller_learn.gif)

</details>

## Train Flags

```python
# add argparse arguments
parser = argparse.ArgumentParser(description="Train an RL agent with Stable-Baselines3.")
parser.add_argument("--video", action="store_true", default=False, help="Record videos during training.")
parser.add_argument("--video_length", type=int, default=200, help="Length of the recorded video (in steps).")
parser.add_argument("--video_interval", type=int, default=2000, help="Interval between video recordings (in steps).")
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")
parser.add_argument("--log_interval", type=int, default=100_000, help="Log data every n timesteps.")
parser.add_argument("--checkpoint", type=str, default=None, help="Continue the training from checkpoint.")
parser.add_argument("--max_iterations", type=int, default=None, help="RL Policy training iterations.")
```

`python scripts\reinforcement_learning\skrl\train.py --task=Isaac-Jetbot-Controller-Direct-v0` or `isaaclab.bat scripts\reinforcement_learning\skrl\train.py --task=Isaac-Jetbot-Controller-Direct-v0` or (RANDOM AGENT) `./isaaclab.sh -p scripts/environments/random_agent.py --task Isaac-Cartpole-v0 --num_envs 32`

- Add number of environments to train in parallel: `--num_envs 10`
- Add training mode: 

    - `--headless` (simulation is not rendered during training)
    - `--num_envs 64 --headless --video`: `--enable_cameras` which enables off-screen rendering. Additionally, we pass the flag `--video` which records a video of the agent’s behavior during training. (The videos are saved to the `logs/sb3/Isaac-Cartpole-v0/<run-dir>/videos/train directory`.)
    - ignore the `--headless` to see the simulation rendered in a window and manipulate it (slow down the training process since the simulation is rendered on the screen)

- Play with last checkpoint: `python scripts\reinforcement_learning\skrl\play.py --task=Isaac-Jetbot-Controller-Direct-v0 --num_envs 32 --use_last_checkpoint` (You can also specify a specific checkpoint by passing the `--checkpoint` flag)

EXAMPLE:

`isaaclab.bat -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Velocity-Rough-H1-v0 --headless`

`isaaclab.bat -p scripts/reinforcement_learning/rsl_rl/play.py --task Isaac-Velocity-Rough-H1-v0 --num_envs 64 --checkpoint logs/rsl_rl/h1_rough/EXPERIMENT_NAME/POLICY_FILE.pt`

`isaaclab.bat -p scripts/tutorials/03_envs/policy_inference_in_usd.py --checkpoint logs/rsl_rl/h1_rough/EXPERIMENT_NAME/exported/policy.pt`

<br>

<br>

### BOTZO RL TEMPLATE

I created my own project using Isaac Lab template:

```shell
./isaaclab.sh --new

(env_isaaclab) C:\Users\grego\Desktop\GRINGO\IsaacLab\IsaacLab>python -m pip install -e source\isaaclab_tasks

(env_isaaclab) C:\Users\grego\Desktop\GRINGO\IsaacLab\IsaacLab>python scripts\environments\list_envs.py
```
Result: 
`|   7    | Isaac-Botzo-Direct-v0                                         | isaaclab_tasks.direct.botzo.botzo_env:BotzoEnv                                                        | isaaclab_tasks.direct.botzo.botzo_env_cfg:BotzoEnvCfg`

Now I can use my environment to train:
```shell
(env_isaaclab) C:\Users\grego\Desktop\GRINGO\IsaacLab\IsaacLab>python scripts\reinforcement_learning\skrl\train.py --task=Isaac-Botzo-Direct-v0
```

by default, this should start a cartpole training environment.

Let the training finish and then run the following command to see the trained policy in action!

```shell
python scripts\reinforcement_learning\skrl\play.py --task=Isaac-Botzo-Direct-v0
```

Notice that you did not need to specify the path for the checkpoint file! This is because Isaac Lab handles much of the minute details like checkpoint saving, loading, and logging. In this case, the train.py script will create two directories: logs and output, which are used as the default output directories for tasks run by this project.