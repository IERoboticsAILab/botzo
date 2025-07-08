# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})  # start the simulation app, with GUI open

import sys

import carb
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path

# Import IK solver
import sys
import os
import math
# Add the path to simulation/src
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../src')))
from IK_solver import *

# preparing the scene
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()  # add ground plane
set_camera_view(
    eye=[5.0, 0.0, 1.5], target=[0.00, 0.00, 1.00], camera_prim_path="/OmniverseKit_Persp"
)  # set camera view

# Add botzo USD
usd_path = "C:\\Users\\grego\\Desktop\\GRINGO\\botzo\\botzo\\simulation\\reinforcment_learning\\botzo_USD\\botzo_USD.usd"
add_reference_to_stage(usd_path=usd_path, prim_path="/World/final_URDF")  # add robot to stage
botzo = Articulation(prim_paths_expr="/World/final_URDF", name="my_botzo")  # create an articulation object

# set the initial poses of the arm and the car so they don't collide BEFORE the simulation starts
botzo.set_world_poses(positions=np.array([[0.0, 1.0, 0.0]]) / get_stage_units())

# initialize the world
my_world.reset()
while my_world.is_playing():
    for i in range(0,len(forward_targets_FR_BL)):
        #print(f"Cordinate to achive: {forward_targets_FR_BL[i]}")
        # calculate front right shoulder, femur, knee angles
        FR_s_f_t = legIK(forward_targets_FR_BL[i][0], forward_targets_FR_BL[i][1], forward_targets_FR_BL[i][2])
        FR_angle_shoulder, FR_angle_femur, FR_angle_knee = FR_s_f_t
        target_FR_angle_shoulder = math.radians(real_sim_angle(FR_angle_shoulder,joint_ids["FR"]["shoulder"]))
        target_FR_angle_femur = math.radians(real_sim_angle(FR_angle_femur, joint_ids["FR"]["femur"]))
        target_FR_angle_knee = math.radians(real_sim_angle(FR_angle_knee, joint_ids["FR"]["knee"]))
        #------------------------------
        # calculate front left shoulder, femur, knee angles
        FL_s_f_t = legIK(forward_targets_FL_BR[i][0], forward_targets_FL_BR[i][1], forward_targets_FL_BR[i][2])
        FL_angle_shoulder, FL_angle_femur, FL_angle_knee = FL_s_f_t
        target_FL_angle_shoulder = math.radians(real_sim_angle(FL_angle_shoulder,joint_ids["FL"]["shoulder"]))
        target_FL_angle_femur = math.radians(real_sim_angle(FL_angle_femur, joint_ids["FL"]["femur"]))
        target_FL_angle_knee = math.radians(real_sim_angle(FL_angle_knee, joint_ids["FL"]["knee"]))
        #------------------------------
        # calculate back right shoulder, femur, knee angles
        BR_s_f_t = legIK(forward_targets_FL_BR[i][0], forward_targets_FL_BR[i][1], forward_targets_FL_BR[i][2])
        BR_angle_shoulder, BR_angle_femur, BR_angle_knee = BR_s_f_t
        target_BR_angle_shoulder = math.radians(real_sim_angle(BR_angle_shoulder,joint_ids["BR"]["shoulder"]))
        target_BR_angle_femur = math.radians(real_sim_angle(BR_angle_femur, joint_ids["BR"]["femur"]))
        target_BR_angle_knee = math.radians(real_sim_angle(BR_angle_knee, joint_ids["BR"]["knee"]))
        #------------------------------
        # calculate back left shoulder, femur, knee angles
        BL_s_f_t = legIK(forward_targets_FR_BL[i][0], forward_targets_FR_BL[i][1], forward_targets_FR_BL[i][2])
        BL_angle_shoulder, BL_angle_femur, BL_angle_knee = BL_s_f_t
        target_BL_angle_shoulder = math.radians(real_sim_angle(BL_angle_shoulder,joint_ids["BL"]["shoulder"]))
        target_BL_angle_femur = math.radians(real_sim_angle(BL_angle_femur, joint_ids["BL"]["femur"]))
        target_BL_angle_knee = math.radians(real_sim_angle(BL_angle_knee, joint_ids["BL"]["knee"]))
        #------------------------------

        #print("target_FR_angle_shoulder:", target_FR_angle_shoulder)
        # Set joint positions for each leg
        botzo.set_joint_positions([[target_BL_angle_femur, target_BL_angle_knee, target_BL_angle_shoulder,
                                target_FL_angle_femur, target_FL_angle_knee, target_FL_angle_shoulder,
                                target_BR_angle_femur, target_BR_angle_knee, target_BR_angle_shoulder,
                                target_FR_angle_femur, target_FR_angle_knee, target_FR_angle_shoulder]])
        for j in range(100):
            # step the simulation, both rendering and physics
            my_world.step(render=True)
    # print the joint positions of the car at every physics step
    botzo_joint_positions = botzo.get_joint_positions()
    print("botzo joint positions:", botzo_joint_positions)

simulation_app.close()