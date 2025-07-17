# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim import SimulationApp

# URDF import, configuration and simulation sample
kit = SimulationApp({"renderer": "RaytracedLighting", "headless": False})
import omni.kit.commands
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.extensions import get_extension_path_from_name
from pxr import Gf, PhysxSchema, Sdf, UsdLux, UsdPhysics




import sys
import os
import math
# Add the path to simulation/src
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../src')))

from IK_solver import *




# Setting up import configuration:
status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
import_config.merge_fixed_joints = False
import_config.convex_decomp = False
import_config.import_inertia_tensor = True
import_config.fix_base = False
import_config.distance_scale = 6.0

# Import URDF file:
status, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path="C:\\Users\\grego\\Desktop\\GRINGO\\botzo\\botzo\\simulation\\urdf\\final_URDF.urdf",
    import_config=import_config,
    get_articulation_root=True,
)
# Get stage handle
stage = omni.usd.get_context().get_stage()

# Enable physics
scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
# Set gravity
scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
scene.CreateGravityMagnitudeAttr().Set(9.81) # (0.0) no gravity
# Set solver settings
PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/physicsScene"))
physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(stage, "/physicsScene")
physxSceneAPI.CreateEnableCCDAttr(True)
physxSceneAPI.CreateEnableStabilizationAttr(True)
physxSceneAPI.CreateEnableGPUDynamicsAttr(True)
#physxSceneAPI.CreateBroadphaseTypeAttr("MBP")
#physxSceneAPI.CreateSolverTypeAttr("TGS")

# Add ground plane
omni.kit.commands.execute(
    "AddGroundPlaneCommand",
    stage=stage,
    planePath="/groundPlane",
    axis="Z",
    size=1500.0,
    position=Gf.Vec3f(0, 0, -0.50),
    color=Gf.Vec3f(0.5),
)

# Add lighting
distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
distantLight.CreateIntensityAttr(500)

# Start simulation
omni.timeline.get_timeline_interface().play()

# perform one simulation step so physics is loaded and dynamic control works.
kit.update()
art = Articulation("/final_URDF")
art.set_world_pose(position=Gf.Vec3f(0.0, 0.0, 1.0))  # Lift to 1m height
art.initialize()
joints_ids = {
    "BR": {
        "shoulder": art.get_dof_index("BR_shoulder_joint"),
        "femur": art.get_dof_index("BR_femur_joint"),
        "knee": art.get_dof_index("Revolute_43")
    },
    "FR": {
        "shoulder": art.get_dof_index("FR_shoulder_joint"),
        "femur": art.get_dof_index("FR_femur_joint"),
        "knee": art.get_dof_index("FR_knee_joint")
    },
    "BL": {
        "shoulder": art.get_dof_index("BL_shoulder_joint"),
        "femur": art.get_dof_index("BL_femur_joint"),
        "knee": art.get_dof_index("BL_knee_joint")
    },
    "FL": {
        "shoulder": art.get_dof_index("FL_shoulder_joint"),
        "femur": art.get_dof_index("FL_femur_joint"),
        "knee": art.get_dof_index("FL_knee_joint")
    }
}
joint_ids = {
    "BR": {
        "shoulder": 4,
        "femur": 7,
        "knee": 8,
        "foot": 9
    },
    "FR": {
        "shoulder": 13,
        "femur": 16,
        "knee": 17,
        "foot": 18
    },
    "BL": {
        "shoulder": 24,
        "femur": 27,
        "knee": 28,
        "foot": 29
    },
    "FL": {
        "shoulder": 33,
        "femur": 36,
        "knee": 37,
        "foot": 38
    }
}

# perform simulation
for frame in range(1000):
    for i in range(0,len(forward_targets_FR_BL)):
        print(forward_targets_FR_BL[i])
        print(forward_targets_FL_BR[i])
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

        # Set joint positions for each leg
        art.set_joint_positions([[target_FR_angle_shoulder]], joint_indices=[joints_ids["FR"]["shoulder"]])
        art.set_joint_positions([[target_FR_angle_femur]], joint_indices=[joints_ids["FR"]["femur"]])
        art.set_joint_positions([[target_FR_angle_knee]], joint_indices=[joints_ids["FR"]["knee"]])
        art.set_joint_positions([[target_FL_angle_shoulder]], joint_indices=[joints_ids["FL"]["shoulder"]])
        art.set_joint_positions([[target_FL_angle_femur]], joint_indices=[joints_ids["FL"]["femur"]])
        art.set_joint_positions([[target_FL_angle_knee]], joint_indices=[joints_ids["FL"]["knee"]])
        art.set_joint_positions([[target_BR_angle_shoulder]], joint_indices=[joints_ids["BR"]["shoulder"]])
        art.set_joint_positions([[target_BR_angle_femur]], joint_indices=[joints_ids["BR"]["femur"]])
        art.set_joint_positions([[target_BR_angle_knee]], joint_indices=[joints_ids["BR"]["knee"]])
        art.set_joint_positions([[target_BL_angle_shoulder]], joint_indices=[joints_ids["BL"]["shoulder"]])
        art.set_joint_positions([[target_BL_angle_femur]], joint_indices=[joints_ids["BL"]["femur"]])
        art.set_joint_positions([[target_BL_angle_knee]], joint_indices=[joints_ids["BL"]["knee"]])
        kit.update()

# Shutdown and exit
omni.timeline.get_timeline_interface().stop()
kit.close()
