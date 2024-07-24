# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import numpy as np
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid

import omni
import SyntheticToolkit
from SyntheticToolkit.core.dynamic_objects import DynamicObject
import SyntheticToolkit.utils.omni_utils as util
from SyntheticToolkit.Sensors.rig import Rig
from pxr import Gf, Sdf, Semantics, UsdGeom

from omni.isaac.core.prims import XFormPrim
my_world = World(stage_units_in_meters=1.0)


fancy_cube = my_world.scene.add(
    DynamicCuboid(
        prim_path="/World/random_cube",
        name="fancy_cube",
        position=np.array([0, 0, 1.0]),
        scale=np.array([0.5015, 0.5015, 0.5015]),
        color=np.array([0, 0, 1.0]),
    )
)
stage = omni.usd.get_context().get_stage()
path = "http://localhost:34080/omniverse://localhost/NVIDIA/Assets/Characters/Reallusion/Worker/Worker.usd"

cube1 = DynamicObject(
    [5, 0, 1],
    [0, 0, 0, 0],
    [
        0.01,
        0.01,
        0.01,
    ],
    "worker",
    "/World",
    stage,
    usd_path=path,
)
path = "http://localhost:34080/omniverse://localhost/NVIDIA/Assets/Characters/Reallusion/Orc/Orc.usd"

path = "/home/jon/Documents/isaacsim/ov-characters3dpack-01-100.1.2/Assets/Characters/Reallusion/Debra/Debra.usd"
path = "http://localhost:34080/omniverse://localhost/NVIDIA/Demos/WarehousePhysics/Assets/Props/Forklift/forklift.usd"
cube = DynamicObject(
    [0, 0, 1],
    [0, 0, 0, 0],
    [
        0.01,
        0.01,
        0.01,
    ],
    "orc",
    "/World",
    stage,
    usd_path=path,
)

prim = XFormPrim(
    name="rig",
    prim_path="/World/rig",
)
_rig = Rig(
    [10, 0, 1],
    [0, 0, 0, 0],
    [
        0.01,
        0.01,
        0.01,
    ],
    "rig222",
    "/World",
    stage,
)
_rig.create_rig_from_file("/home/jon/Documents/buzz_backup/sensors.json")

my_world.scene.add_default_ground_plane()


wp = [[10, 0, 1], [5, 0, 1], [10, 10, 1]]

cube.reset_waypoints(wp)
cube1.reset_waypoints(wp)
_rig.reset_waypoints(wp)


# === camera tests

# layer1 = Sdf.Layer.CreateAnonymous()
# stage.GetRootLayer().subLayerPaths.append(layer1.identifier)
# layer2 =Sdf.Layer.CreateAnonymous()
#
# stage.GetRootLayer().subLayerPaths.append(layer2.identifier)
# # Assign objects to layers
# object1 = stage.GetPrimAtPath("/World/worker")
# object2 = stage.GetPrimAtPath("/World/orc")
#
#
# reference1 = Sdf.Reference(assetPath=layer1.identifier)
# reference2 = Sdf.Reference(assetPath=layer2.identifier)
# # Add references to the objects
# object1.GetReferences().AddReference(reference1)
# object2.GetReferences().AddReference(reference2)
#
# # Configure camera visibility (assuming cameras exist)
# camera1 = stage.GetPrimAtPath("/World/worker/Camera")
# camera2 = stage.GetPrimAtPath("/World/orc/Camera")
#
# # UsdGeom.XformCommonAPI(camera1).SetLayerMask(layer1)
# # UsdGeom.XformCommonAPI(camera2).SetLayerMask(layer2)
#
# UsdGeom.Imageable(object1).GetVisibilityAttr().Set(UsdGeom.Tokens.invisible, layer=layer2)
# UsdGeom.Imageable(object2).GetVisibilityAttr().Set(UsdGeom.Tokens.invisible, layer=layer1)
while simulation_app.is_running():
    my_world.step(render=True)
    count = 0

    while my_world.is_playing():
        count += 1
        cube.move(1)
        _rig.move(1)
        print(cube.get_translate())
        # cube1.move(1)

        my_world.step(render=True)
        if count > 600:
            cube.reset_waypoints(wp)
            count = 0
            print("resetting")
        # cube.apply_veloc([random.randint(0,4),random.randint(0,4),random.randint(0,4)],[0,0,0])

simulation_app.close()
