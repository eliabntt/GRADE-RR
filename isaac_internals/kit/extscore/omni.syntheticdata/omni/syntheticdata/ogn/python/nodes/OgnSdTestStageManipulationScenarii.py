# Copyright (c) 2021-2022, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.usd
import omni.graph.core as og
from pxr import Gf, Semantics, UsdGeom
import numpy as np

class OgnSdTestStageManipulationScenarii:

    _prim_names = ["Sphere", "Capsule", "Plane", "Torus", "Cube", "Cone"]
    _sem_types = ["type", "class", "genre"]
    _sem_labels = ["sphere", "capsule", "plane", "torus", "cube", "ball", "cone"]

    @staticmethod
    def add_semantics(prim, semantic_label, semantic_type="class"):
        sem = Semantics.SemanticsAPI.Apply(prim, "Semantics")
        sem.CreateSemanticTypeAttr()
        sem.CreateSemanticDataAttr()
        sem.GetSemanticTypeAttr().Set(semantic_type)
        sem.GetSemanticDataAttr().Set(semantic_label)

    @staticmethod
    def get_random_transform(rng):
        tf = np.eye(4)
        tf[:3, :3] = Gf.Matrix3d(Gf.Rotation(rng.rand(3).tolist(), rng.rand(3).tolist()))
        tf[3, :3] = rng.rand(3).tolist()
        return Gf.Matrix4d(tf)

    @staticmethod
    def compute(db) -> bool:

        usd_context = omni.usd.get_context()
        stage = usd_context.get_stage()
        if not stage:
            return False

        rng = np.random.default_rng(db.inputs.randomSeed + ((db.state.frameNumber * 23) % 1579))
            
        world_prim = stage.GetPrimAtPath(db.inputs.worldPrimPath)
        if not world_prim:
            world_prim = stage.DefinePrim(db.inputs.worldPrimPath)
            if world_prim:
                world_xform_prim = UsdGeom.Xformable(world_prim) if world_prim else None
                if world_xform_prim:
                    world_xform_prim.AddTransformOp().Set(OgnSdTestStageManipulationScenarii.get_random_transform(rng))
        if not world_prim:
            return False    
        
        db.state.frameNumber += 1

        num_manipulations = rng.randint(0, 3)
        for manip_index in range(num_manipulations):

            prims = world_prim.GetChildren()
            prims.append(world_prim)

            prim = rng.choice(prims)
            if not prim :
                continue
            
            manipulation = rng.randint(0, 38)
            
            if (manipulation < 11):
                """create a new children prim"""
                prim_name = rng.choice(OgnSdTestStageManipulationScenarii._prim_names)
                prim_path = prim.GetPath().pathString + "/" + prim_name + "_" + str(db.state.frameNumber) + "_" + str(manip_index)
                new_prim = stage.DefinePrim(prim_path, prim_name)
                new_prim_color_attr = new_prim.GetAttribute("primvars:displayColor") if new_prim else None
                if new_prim_color_attr:
                    new_prim_color_attr.Set([rng.rand(3).tolist()])
                xform_prim = UsdGeom.Xformable(new_prim) if new_prim else None
                if xform_prim:
                    xform_prim.AddScaleOp().Set((175.0*rng.random(), 175.0*rng.random(), 175.0*rng.random()))
                    xform_prim.AddTransformOp().Set(OgnSdTestStageManipulationScenarii.get_random_transform(rng))

            elif (manipulation >= 11) and (manipulation <12):
                 """remove the prim"""
                 stage.RemovePrim(prim.GetPath())

            elif (manipulation >=12) and (manipulation <23):
                """move the prim"""
                xform_prim = UsdGeom.Xformable(prim)
                if xform_prim:
                    xform_prim.ClearXformOpOrder()
                    xform_prim.AddTransformOp().Set(OgnSdTestStageManipulationScenarii.get_random_transform(rng))

            elif (manipulation >=23) and (manipulation < 31):
                """add semantic to the prim"""
                OgnSdTestStageManipulationScenarii.add_semantics(prim, rng.choice(OgnSdTestStageManipulationScenarii._sem_labels), rng.choice(OgnSdTestStageManipulationScenarii._sem_types))

            elif (manipulation >=31) and (manipulation < 39):
                """change color of the prim"""
                prim_color_attr = prim.GetAttribute("primvars:displayColor")
                if prim_color_attr:
                    prim_color_attr.Set([rng.rand(3).tolist()])

        return True
