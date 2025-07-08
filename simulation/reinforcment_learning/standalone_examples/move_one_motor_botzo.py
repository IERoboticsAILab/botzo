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
scene.CreateGravityMagnitudeAttr().Set(9.81)
# Set solver settings
PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/physicsScene"))
physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(stage, "/physicsScene")
physxSceneAPI.CreateEnableCCDAttr(True)
physxSceneAPI.CreateEnableStabilizationAttr(True)
physxSceneAPI.CreateEnableGPUDynamicsAttr(False)
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
art.initialize()
dof_ptr = art.get_dof_index("BR_shoulder_joint")


print(f"\n\n\n\n---------------\nImporting URDF file: {prim_path}")  #Importing URDF file: /final_URDF/base_link
print(f"Import status: {status}")           #Import status: True
print(f"Number of DOFs: {art}\n-------------\n\n\n\n")  #Memory address

# perform simulation
for frame in range(1000):
    art.set_joint_positions([[-1.5]], joint_indices=[dof_ptr])
    kit.update()

# Shutdown and exit
omni.timeline.get_timeline_interface().stop()
kit.close()
