# Copyright (c) 2018-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.client
import omni.kit
import omni.usd

import asyncio
import os
from pxr import UsdGeom, Gf, Tf

import random
import sys

from .globals import *


def file_exists_on_omni(file_path):
    result, _ = omni.client.stat(file_path)
    if result == omni.client.Result.OK:
        return True

    return False


async def create_folder_on_omni(folder_path):
    if not file_exists_on_omni(folder_path):
        result = await omni.client.create_folder_async(folder_path)
        return result == omni.client.Result.OK


async def convert(in_file, out_file):
    # This import causes conflicts when global?
    import omni.kit.asset_converter as assetimport

    # Folders must be created first through usd_ext of omni won't be able to create the files creted in them in the current session.
    out_folder = out_file[0 : out_file.rfind("/") + 1]

    # only call create_folder_on_omni if it's connected to an omni server
    if out_file.startswith("omniverse://"):
        await create_folder_on_omni(out_folder + "materials")

    def progress_callback(progress, total_steps):
        pass

    converter_context = omni.kit.asset_converter.AssetConverterContext()
    # setup converter and flags
    converter_context.as_shapenet = True
    converter_context.single_mesh = True
    instance = omni.kit.asset_converter.get_instance()
    task = instance.create_converter_task(in_file, out_file, progress_callback, converter_context)

    success = True
    while True:
        success = await task.wait_until_finished()
        if not success:
            await asyncio.sleep(0.1)
        else:
            break
    return success


# This is the main entry point for any function that wants to add a shape to the scene.
# Care must be taken when running this on a seperate thread from the main thread because
# it calls c++ modules from python which hold the GIL.
def addShapePrim(
    omniverseServer, synsetId, modelId, pos, rot, scale, auto_add_physics, use_convex_decomp, do_not_place=False
):
    # allow for random ids
    shapenet_db = get_database()
    if synsetId == None or synsetId == "random":
        synsetId = random.choice(list(shapenet_db))

    if modelId == None or modelId == "random":
        modelId = random.choice(list(shapenet_db[synsetId]))

    # use shapenet v2 for models
    # Get the local file system path and the omni server path
    local_folder = get_local_shape_loc() + "/" + synsetId + "/" + modelId + "/"
    local_path = local_folder + "models/model_normalized.obj"
    local_modified_path = local_folder + "models/modified/model.obj"

    global g_omni_shape_loc
    omni_shape_loc = "omniverse://" + omniverseServer + g_omni_shape_loc

    (result, entry) = asyncio.new_event_loop().run_until_complete(omni.client.stat_async(omni_shape_loc))
    if not result == omni.client.Result.OK:
        print("Saving converted files locally since omniverse server is not connected")
        omni_shape_loc = get_local_shape_loc() + "/local-converted-USDs"

    omni_path = (
        omni_shape_loc + "/n" + synsetId + "/i" + modelId + "/"
    )  # don't forget to add the name at the end and .usd
    omni_modified_path = omni_shape_loc + "/n" + synsetId + "/i" + modelId + "/modified/"

    stage = omni.usd.get_context().get_stage()
    if not stage:
        return "ERROR Could not get the stage."

    # Get the name of the shapenet object reference in the stage if it exists
    # (i.e. it has been added already and is used in another location on the stage).
    synsetID_path = g_root_usd_namespace_path + "/n" + synsetId
    over_path = synsetID_path + "/i" + modelId

    # Get the name of the instance we will add with the transform, this is the actual visible prim
    # instance of the reference to the omniverse file which was converted to local disk after
    global g_shapenet_db
    g_shapenet_db = get_database()
    if g_shapenet_db == None:
        shape_name = ""
        print("Please create an Shapenet ID Database with the menu.")
    else:
        shape_name = Tf.MakeValidIdentifier(g_shapenet_db[synsetId][modelId][4])
    if shape_name == "":
        shape_name = "empty_shape_name"
    prim_path = str(stage.GetDefaultPrim().GetPath()) + "/" + shape_name
    prim_path_len = len(prim_path)
    shape_name_len = len(shape_name)

    # if there is only one instance, we don't add a _# postfix, but if there is multiple, then the second instance
    # starts with a _1 postfix, and further additions increase that number.
    insta_count = 0
    while stage.GetPrimAtPath(prim_path):
        insta_count += 1
        prim_path = f"{prim_path[:prim_path_len]}_{insta_count}"
        shape_name = f"{shape_name[:shape_name_len]}_{insta_count}"

    omni_path = omni_path + shape_name + ".usd"
    omni_modified_path = omni_modified_path + shape_name + ".usd"
    # If the prim refernce to the omnivers file is not already on
    # the stage the stage we will need to add it.
    place_later = False
    if not stage.GetPrimAtPath(over_path):
        print(f"-Shapenet is adding {shape_name} to the stage for the first time.")
        # If the files does not already exist in omniverse we will have to add it there
        # with our automatic conversion of the original shapenet file.
        # We need to check if the modified file is on disk, so if it's not on the omni server it will
        # be added there even if the non modified one already exists on omni.
        if os.path.exists(local_modified_path) or file_exists_on_omni(omni_modified_path):
            omni_path = omni_modified_path
        if not file_exists_on_omni(omni_path):
            # If the original omniverse file does not exist locally, we will have to pull
            # it from Stanford's shapenet database on the web.
            if os.path.exists(local_modified_path):
                local_path = local_modified_path
                omni_path = omni_modified_path
            if not os.path.exists(local_path):
                # Pull the shapenet files to the local drive for conversion to omni:usd
                no_model_message = f"The file does not exist at {local_path}, are you sure you have the env var SHAPENET_LOCAL_DIR set and the shapnet database downloaded to it?"
                print(no_model_message)
                return f"ERROR {no_model_message}"
            # Add The file to omniverse here, if you add them asyncronously, then you have to do the
            # rest of the scene adding later.
            print(f"---Converting {shape_name}...")
            status = asyncio.get_event_loop().run_until_complete(convert(local_path, omni_path))
            if not status:
                return f"ERROR OmniConverterStatus is {status}"
            print(f"---Added to Omniverse as {omni_path}.")

        # Add the over reference of the omni file to the stage here.
        print(f"----Adding over of {over_path} to stage.")
        if not do_not_place and not place_later:
            over = stage.OverridePrim(over_path)
            over.GetReferences().AddReference(omni_path)

    # Add the instance of the shape here.
    if not do_not_place and not place_later:
        prim = stage.DefinePrim(prim_path, "Xform")
        prim.GetReferences().AddInternalReference(over_path)

        # shapenet v2 models are normalized to 1 meter, and rotated -90deg on the x axis.
        metersPerUnit = UsdGeom.GetStageMetersPerUnit(stage)
        scaled_scale = scale / metersPerUnit
        rot = Gf.Rotation(Gf.Vec3d(1, 0, 0), 90) * rot
        addobject_fn(prim.GetPath(), pos, rot, scaled_scale)

        # add physics
        if auto_add_physics:
            from omni.physx.scripts import utils

            print("Adding PHYSICS to ShapeNet model")
            shape_approximation = "convexHull"
            if use_convex_decomp:
                shape_approximation = "convexDecomposition"
            utils.setRigidBody(prim, shape_approximation, False)

        return prim

    return None


def get_min_max_vert(obj_file_name):
    min_x = min_y = min_z = sys.float_info.max
    max_x = max_y = max_z = -sys.float_info.max
    with open(obj_file_name, "r") as fi:
        for ln in fi:
            if ln.startswith("v "):
                vx = float(ln[2:].partition(" ")[0])
                vy = float(ln[2:].partition(" ")[2].partition(" ")[0])
                vz = float(ln[2:].partition(" ")[2].partition(" ")[2])
                min_x = min(min_x, vx)
                min_y = min(min_y, vy)
                min_z = min(min_z, vz)
                max_x = max(max_x, vx)
                max_y = max(max_y, vy)
                max_z = max(max_z, vz)
    return Gf.Vec3f(min_x, min_y, min_z), Gf.Vec3f(max_x, max_y, max_z)


# Got this From Lou Rohan... Thanks Lou!
# objectpath - path in omniverse - omni:/Projects/Siggraph2019/AtticWorkflow/Props/table_cloth/table_cloth.usd
# objectname - name you want it to be called in the stage
# xform - Gf.Matrix4d
def addobject_fn(path, position, rotation, scale):
    # The original model was translated by the centroid, and scaled to be normalized by the length of the
    # hypotenuse of the bbox
    translate_mtx = Gf.Matrix4d()
    rotate_mtx = Gf.Matrix4d()
    scale_mtx = Gf.Matrix4d()

    translate_mtx.SetTranslate(position)  # centroid/metersPerUnit)
    rotate_mtx.SetRotate(rotation)
    scale_mtx = scale_mtx.SetScale(scale)
    transform_matrix = scale_mtx * rotate_mtx * translate_mtx

    omni.kit.commands.execute("TransformPrimCommand", path=path, new_transform_matrix=transform_matrix)
