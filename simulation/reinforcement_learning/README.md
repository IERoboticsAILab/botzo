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

Computer specifications:
| Component | Specification |
|-----------|---------------|
| RAM       | 64GB          |
| GPU       | RTX 4090      |
| VRAM      | 16GB          |
| OS        | Windows 11    |
| Storage   | 1TB           |

>_Check system requirements compatibility and satisfaction with [this](https://download.isaacsim.omniverse.nvidia.com/isaac-sim-comp-check%404.5.0-rc.6%2Brelease.675.f1cca148.gl.windows-x86_64.release.zip)_

<img src="https://raw.githubusercontent.com/IERoboticsAILab/botzo/main/media_assests/isaac_compatability_check.png" alt="ours" width="200"/>

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

Documentation [here](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/introduction/quickstart_index.html#isaac-sim-intro-quickstart-series)

#### Tutorial

<details>
<summary><b>Create Cube</b></summary>

<details>
<summary>GUI</summary>

1. `isaac-sim.selector.bat`: Load scene
2. create a new scene: `File > New`
3. Add ground plane: `Create > Physics > Ground Plane`
4. Add lightr source: `Create > Lights > Distant Light`
5. Add visual cube: `Create > Shapes > Cube` (_has no physics attached (no collisions, no mass). If press play the cube doesn't move_)
6. Add physics and collision propreties:
    
    a. find the object (“/World/Cube”) on the stage tree and highlight it

    b. go to the Property panel on the bottom right

    c. click on the Add button and select Physics on the dropdown menu

    d. select Rigid Body with Colliders Preset to add both phyiscs and collision meshes to the object.

    e. press play button
</details>

<details>
<summary>Extension</summary>

1. Start Isaac Sim with `isaac-sim.bat`
2. Open the Extension Manager: `Window > Script Editor`
3. Add ground plane:
```python
from isaacsim.core.api.objects.ground_plane import GroundPlane
GroundPlane(prim_path="/World/GroundPlane", z_position=0)
```
4. Press the `Run` button to execute the code.
5. Press `Tab` and add another script tab.
6. Add light source:
```python
from pxr import Sdf, UsdLux
stage = omni.usd.get_context().get_stage()
distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
distantLight.CreateIntensityAttr(300)
```
7. Add visual cube (no physics):
```python
import numpy as np
from isaacsim.core.api.objects import VisualCuboid
VisualCuboid(
prim_path="/visual_cube",
name="visual_cube",
position=np.array([0, 0.5, 0.5]),
size=0.3,
color=np.array([255, 255, 0]),
)
VisualCuboid(
prim_path="/test_cube",
name="test_cube",
position=np.array([0, -0.5, 0.5]),
size=0.3,
color=np.array([0, 255, 255]),
)
```
8. Add physics propreties cube:
```python
import numpy as np
from isaacsim.core.api.objects import DynamicCuboid

DynamicCuboid(
prim_path="/dynamic_cube",
name="dynamic_cube",
position=np.array([0, -1.0, 1.0]),
scale=np.array([0.6, 0.5, 0.2]),
size=1.0,
color=np.array([255, 0, 0]),
)
```
9. Move, Rotate and Scale:
```python
import numpy as np
from isaacsim.core.prims import XFormPrim

translate_offset = np.array([[1.5,1.2,1.0]])
orientation_offset = np.array([[0.7,0.7,0,1]])     # note this is in radians
scale = np.array([[1,1.5,0.2]])

stage = omni.usd.get_context().get_stage()
cube_in_coreapi = XFormPrim(prim_paths_expr="/test_cube")
cube_in_coreapi.set_world_poses(translate_offset, orientation_offset)
cube_in_coreapi.set_local_scales(scale)
```


</details>

<details>
<summary>Standalone Python</summary>

Script: `standalone_examples/tutorials/getting_started.py` (Code [here](https://github.com/IERoboticsAILab/botzo/blob/main/simulation/reinforcment_learning/src/getting_started.py))

Run: `python.bat standalone_examples\tutorials\getting_started.py` OR `.\python.bat ..\Users\$HOME$\botzo\botzo\simulation\reinforcment_learning\src\getting_started.py`


1. Add Ground plane
    ```python
    from isaacsim.core.api.objects.ground_plane import GroundPlane
    GroundPlane(prim_path="/World/GroundPlane", z_position=0)
    ```
1. Add Light Source
    ```python
    from pxr import Sdf, UsdLux
    stage = omni.usd.get_context().get_stage()
    distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
    distantLight.CreateIntensityAttr(300)
    ```
1. Add Visual Cube
    ```python
    import numpy as np
    from isaacsim.core.api.objects import VisualCuboid
    VisualCuboid(
       prim_path="/visual_cube",
       name="visual_cube",
       position=np.array([0, 0.5, 0.5]),
       size=0.3,
       color=np.array([255, 255, 0]),
    )
    ```
1. Add physics propreties by turning it into "RigidPrim"
    ```python
    from isaacsim.core.prims import RigidPrim
    RigidPrim("/visual_cube")
    ```
1. Add Collision Propreties
    ```python
    from isaacsim.core.prims import GeometryPrim
    prim = GeometryPrim("/visual_cube")
    prim.apply_collision_apis()
    ```
1. Move, Rotate and Scale
    ```python
    import numpy as np
    from isaacsim.core.prims.xform_prim import XformPrim
    from isaacsim.core.prims.prim import Prim

    translate_offset = np.array([[1.5,-0.2,1.0]])
    rotate_offset = np.array([[90,-90,180]])
    scale = np.array([[1,1.5,0.2]])

    cube_in_coreapi = XformPrim(Prim(prim_paths_expr="/test_cube"))
    cube_in_coreapi.set_world_poses(translate_offset, rotate_offset)
    cube_in_coreapi.set_scales(scale)
    ```

</details>
</details>

---


<details>
<summary><b>Move robot</b></summary>

<details>
<summary>GUI</summary>

1. New empty stage: `File > New Stage`
2. Add robot: `Create > Robot > Franka Emika Panda Arm`
3. Use Physics Inspector to examine the robot’s joint properties: `Tools > Physics > Physics Inspector`
4. Open the graph generator: `Tools > Robotics > Omnigraph Controllers > Joint Position`
5. In the newly appeared `Articulation Position Controller Inputs` popup window, click Add for the `Robot Prim` field, select Franka as the Target.
6. Click `OK` to generate the graph.
7. To move the robot, you need to change the values in the `JointCommandArray` node inside the Position_Controller graph.
8. You can do this by either selecting the node on the Stage tree, or selecting the node in the graph editor. Both will open up lead to the Properties panel showing the joint command values
9. Press Play first to start the simulation, then type or slide the values with name starting with `input` to see the robot move
10. To visualize the generated graph, open an graph editor window: `Window > Graph Editors > Action Graph`

</details>

<details>
<summary>Extension</summary>

_to be done_

</details>

<details>
<summary>Standalone Python</summary>

Script: `standalone_examples/tutorials/getting_started_robot.py` (Code [here](https://github.com/IERoboticsAILab/botzo/blob/main/simulation/reinforcment_learning/src/getting_started_robot.py))

Run: `python.bat standalone_examples\tutorials\getting_started_robot.py` OR `.\python.bat ..\Users\grego\Desktop\GRINGO\botzo\botzo\simulation\reinforcment_learning\src\getting_started_robot.py`

import necessary modules, add the ground plane, set the camera angle, and add two robots

1. Start simulation
    ```python
    from isaacsim import SimulationApp
    simulation_app = SimulationApp({"headless": False})  # start the simulation app, with GUI open
    ```
2. **"World"** object (Controls everything about this virtual world, such as physics and rendering stepping, and holding object handles) 
    ```python
    my_world = World(stage_units_in_meters=1.0)
    ```

    stepping function my_world.step() is called every iteration
    ```python
    my_world.step(render=True)
    ```
3. Move robot
    ```python
    arm.set_joint_positions([[-1.5, 0.0, 0.0, -1.5, 0.0, 1.5, 0.5, 0.04, 0.04]])
    ```
    AND
    ```python
    car.set_joint_velocities([[1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]])
    car_joint_positions = car.get_joint_positions()
    print("car joint positions:", car_joint_positions)
    ```

</details>
</details>

### Import URDF

`.\python.bat ..\Users\grego\Desktop\GRINGO\botzo\botzo\simulation\reinforcment_learning\standalone_examples\import_botzo_urdf.py`

### Import USD

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

```shell
(env_isaaclab) C:\Users\grego\Desktop\GRINGO\IsaacLab\IsaacLab>python scripts\tutorials\01_assets\add_new_robot.py
```
![example](https://github.com/IERoboticsAILab/botzo/blob/main/media_assests/add_new_robot.png)

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