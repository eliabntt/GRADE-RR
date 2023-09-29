import asyncio
import carb
import ipdb
import json
import ntpath
import numpy as np
import os
import pickle as pkl
from PIL import Image
from pyquaternion import Quaternion
import scipy.spatial.transform as tf
from stl import mesh
import time
import trimesh
from typing import Dict, Optional, Union

# ros
import rospy, rosgraph
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String

# omni
import omni.isaac.shapenet as shapenet
import omni.kit
from omni.isaac import RangeSensorSchema
from omni.isaac.core import SimulationContext, PhysicsContext
import omni.replicator.core as rep
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.carb import set_carb_setting
from omni.isaac.core.utils.extensions import enable_extension, disable_extension
from omni.isaac.core.utils.stage import is_stage_loading, set_stage_up_axis
from omni.isaac.dynamic_control import _dynamic_control
import omni.isaac.IsaacSensorSchema as IsaacSensorSchema
from omni.isaac.synthetic_recorder import extension_custom
from omni.physxcommands import SetStaticColliderCommand, RemoveStaticColliderCommand
from pxr import UsdGeom, Gf, Usd, UsdSkel, AnimationSchema, Semantics, UsdPhysics, Sdf, UsdShade
from pxr.Usd import Prim

# 2022 edits
import omni.graph.core as og
from omni.isaac.core_nodes.scripts.utils import set_target_prims

def add_semantics(prim: Prim, semantic_label: str):
  """
  Adds semantic labels to the prim.
  prim: the prim to add the semantic label to
  semantic_label: the semantic label to add
  """
  if not prim.HasAPI(Semantics.SemanticsAPI):
    sem = Semantics.SemanticsAPI.Apply(prim, "Semantics")
    sem.CreateSemanticTypeAttr()
    sem.CreateSemanticDataAttr()
  else:
    sem = Semantics.SemanticsAPI.Get(prim, "Semantics")
  sem.GetSemanticTypeAttr().Set("class")
  sem.GetSemanticDataAttr().Set(str(semantic_label))

def correct_paths(parent_name: str):
  """
  Helper function to correct the paths of the world's materials (as they come from Windows).

  parent_name: the prim path of the father.
  """
  stage = omni.usd.get_context().get_stage()

  for prim in stage.Traverse():
    shader_path = prim.GetPath()
    if parent_name.lower() in str(shader_path).lower():
      if prim.GetTypeName().lower() == "mesh":
        prim.GetProperty('doubleSided').Set(False)
      if prim.GetTypeName().lower() == "shader":
        try:
          change_shader_path(shader_path)
        except:
          print(f"Error changing shader of in {shader_path}")
          time.sleep(5)

def change_shader_path(shader_path: str):
    """
    Changes the shader path of the material.
    material_path: the prim path to the material collection (e.g. "/World/my_robot_0/materials, /World/home/materials")
    """
    stage = omni.usd.get_context().get_stage()
    shader = stage.GetPrimAtPath(shader_path)
    if 'inputs:diffuse_texture' in shader.GetPropertyNames():
      old_path = str(shader.GetAttribute('inputs:diffuse_texture').Get().resolvedPath)
      new_path = old_path.replace("@", "")
      # print(f"Changing path {old_path}")
      if "something" in old_path or "P:" in old_path:
        new_path = old_path.replace(ntpath.sep, os.sep).replace('P:/', '').replace("@", "")
      elif "somethingelse" in old_path.lower():
        splitted = old_path.split(ntpath.sep)
        tmp_path = ""
        for i in splitted:
          tmp_path += i + ntpath.sep
          if "something" in i:
            break
        tmp_path = tmp_path.replace(ntpath.sep, os.sep)
        new_path = old_path.replace(ntpath.sep, os.sep).replace(tmp_path, '').replace(
          "@", "")
      shader.GetAttribute('inputs:diffuse_texture').Set(new_path)

    if 'inputs:reflectionroughness_texture' in shader.GetPropertyNames():
      old_path = str(shader.GetAttribute('inputs:reflectionroughness_texture').Get().resolvedPath)
      new_path = old_path.replace("@", "")
      # print(f"Changing path {old_path}")
      if "something" in old_path or "P:" in old_path:
          new_path = old_path.replace(ntpath.sep, os.sep).replace('P:/', '').replace("@", "")
      elif "somethingelse" in old_path.lower():
          splitted = old_path.split(ntpath.sep)
          tmp_path = ""
          for i in splitted:
              tmp_path += i + ntpath.sep
              if "something" in i:
                  break
          tmp_path = tmp_path.replace(ntpath.sep, os.sep)
          new_path = old_path.replace(ntpath.sep, os.sep).replace(tmp_path, '').replace(
              "@", "")
      shader.GetAttribute('inputs:reflectionroughness_texture').Set(new_path)

def set_colliders(path_main_asset: str, value: bool):
  """
  It takes a path to a main asset, and a boolean value, and sets the physics:collisionEnabled attribute to the boolean
  value for all children of the main asset. This effectively enable or disable collisions.

  :param path_main_asset: The path to the main asset in the USD file
  :type path_main_asset: str
  :param value: bool
  :type value: bool
  """
  stage = omni.usd.get_context().get_stage()
  for j in stage.GetPrimAtPath(path_main_asset).GetAllChildren():
    for i in j.GetAllChildren():
      if "physics:collisionEnabled" in i.GetPropertyNames():
        if i.GetProperty("physics:collisionEnabled").Get() == value:
          continue
        i.GetProperty("physics:collisionEnabled").Set(value)

def add_colliders(path_main_asset: str):
  """
  Adds the colliders to the main asset. This allows the object to have collisions or not (if supported).
  Return True if the colliders were added, False otherwise.

  path_main_asset: the path of the prim asset whose childs need to be processed
  """
  stage = omni.usd.get_context().get_stage()
  fres = True

  for prim in stage.Traverse():
    prim_path = prim.GetPath()
    if path_main_asset.lower() in str(prim_path).lower():
      if prim.GetTypeName().lower() == "mesh" or prim.GetTypeName().lower() == "xform":
        res, _ = SetStaticColliderCommand.execute(str(prim.GetPath()))
        fres = res and fres
  return fres

def process_semantics(parent_name: str, name_to_label: str = None):
  """
  Processes the semantics of the world.
  In case the name_to_label is specified (not coming from Front3D), it will be set to the name_to_label param.

  parent_name: the prim path of the father.
  label: the eventual label to give to the set of assets
  """
  for prim in omni.usd.get_context().get_stage().Traverse():

    primpath = prim.GetPath()
    if parent_name.lower() in str(primpath).lower():
      if prim.GetTypeName().lower() == "mesh" or prim.GetTypeName().lower() == "xform":
        if name_to_label == None:
          # tmp = prim.GetAttribute('userProperties:category_id')
          tmp = prim.GetAttribute('userProperties:semantic')
          if tmp.Get() != None:
            add_semantics(prim, str(tmp.Get()))
        else:
          add_semantics(prim, name_to_label)

def randomize_and_fix_lights(config: dict, rng: np.random.default_rng, parent_name: str, z_lim, meters_per_unit,
                             is_rtx: bool = False):
  """
  Randomize the lights within an environment

  config: the configuration dict with the parameters and enabled/disabled config for intensity/color
  rng: global rng
  parent_name: parent whose childs need to be considered to change the lights
  """
  stage = omni.usd.get_context().get_stage()
  if not (config["intensity"] or config["color"]):
    return
  min_int = config.get("intensity_interval", 0.0)[0]
  max_int = config.get("intensity_interval", 1.0)[1]
  for prim in stage.Traverse():
    path = prim.GetPath()
    if parent_name.lower() in str(path).lower():
      if "light" in prim.GetTypeName().lower():
        if "environment" in str(path).lower():
          continue
        if config["intensity"]:
          prim.GetAttribute('intensity').Set(rng.uniform(low=min_int, high=max_int))
        if config["color"]:
          col = rng.random(size=3)
          prim.GetAttribute('color').Set(Gf.Vec3f(col[0], col[1], col[2]))
        if not is_rtx:
          prim.GetAttribute('diffuse').Set(4)
          prim.GetAttribute('specular').Set(4)

        # FIXME no actual check I'm not moving other stuff. but this should work based on the "existance" of segmentation info and that lights on its own does not have a translation attribute
        z_lamp = omni.usd.get_world_transform_matrix(prim)[3, 2] * meters_per_unit
        if z_lamp > z_lim - 0.08:
          diff = z_lamp - z_lim - 0.08
          while not prim.HasAttribute('xformOp:translate'):
            prim = prim.GetParent()
          #            while (not "semantic:Semantics:params:semanticData" in parent.GetPropertyNames()):
          #                parent = parent.GetParent()
          p_lamp = prim.GetAttribute('xformOp:translate').Get()
          p_lamp[2] -= diff
          prim.GetAttribute('xformOp:translate').Set(p_lamp)
        # move the light if it is too high

def randomize_roughness(config: dict, rng: np.random.default_rng, parent_name: str):
  """
  Randomize the roughness (reflectivity) of assets within an environment

  config: the configuration dict with the parameters and enabled/disabled config for intensity/color
  rng: global rng
  parent_name: parent whose childs need to be considered to change the lights
  """
  stage = omni.usd.get_context().get_stage()
  if not (config["enabled"]):
    return
  min_int = config.get("intensity_interval", 0.0)[0]
  max_int = config.get("intensity_interval", 1.0)[1]
  for prim in stage.Traverse():
    path = prim.GetPath()
    if parent_name.lower() in str(path).lower():
      if prim.GetTypeName().lower() == "material" or prim.GetTypeName().lower() == "shader":
        if "inputs:RoughnessMin" in prim.GetPropertyNames():
          val = rng.uniform(low=min_int, high=max_int)
          prim.GetAttribute('inputs:RoughnessMin').Set(val)
          prim.GetAttribute('inputs:RoughnessMax').Set(val)

def get_area(polygon):
  """
  Computes the area of a polygon.
  """
  x = polygon[:, 0]
  y = polygon[:, 1]
  return .5 * np.absolute(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))

def change_prim_collision(enable, prim_path):
  for j in omni.usd.get_context().get_stage().Traverse():
    if str(j.GetPath()).startswith(prim_path):
      if 'physics:collisionEnabled' in j.GetPropertyNames():
        omni.kit.commands.execute('ChangeProperty',
                                  prop_path=Sdf.Path(str(j.GetPath())+'.physics:collisionEnabled'),
                                  value=enable,
                                  prev=None)

def change_collision_at_path(enable, paths=['/my_robot_0/camera_link/Cube.physics:collisionEnabled','/my_robot_0/yaw_link/visuals.physics:collisionEnabled']):
  """
  It enables or disables collisions for the paths

  :param enable: True or False
  """
  for path in paths:
    omni.kit.commands.execute('ChangeProperty',
                              prop_path=Sdf.Path(path),
                              value=enable,
                              prev=None)

def add_translate_anim(prim_path: str, pos: Gf.Vec3d, time: float = 0.0):
  """
  Add a goal location at a given timecode. The object will EVENTUALLY move there with a smooth movement.

  prim_path: the path of the asset to be moved
  pos: the final position
  time: the time in FRAME
  """
  omni.kit.commands.execute('ChangePropertyCommand',
                            prop_path=prim_path + '.xformOp:translate',
                            value=pos,
                            prev=Gf.Vec3d(0, 0, 0),
                            type_to_create_if_not_exist=UsdGeom.XformOp.TypeTranslate,
                            timecode=Usd.TimeCode(time))

def add_rotation_anim(prim_path: str, rot: list, time: float = 0.0, use_double=False):
  """
  Add a goal rotation at a given timecode. The object will EVENTUALLY move there with a smooth movement.

  EXPECT ROT IN RAD!

  prim_path: the path of the asset to be moved
  rot: the final position
  time: the time in FRAME
  """
  rot = np.array(rot) * 180 / np.pi
  quat = (
      Gf.Rotation(Gf.Vec3d.XAxis(), rot[0])
      * Gf.Rotation(Gf.Vec3d.YAxis(), rot[1])
      * Gf.Rotation(Gf.Vec3d.ZAxis(), rot[2])
  )
  omni.kit.commands.execute('ChangePropertyCommand',
                            prop_path=prim_path + ".xformOp:orient",
                            value=Gf.Quatf(quat.GetQuat()) if not use_double else Gf.Quatd(quat.GetQuat()),
                            prev=Gf.Quatf(0, 0, 0, 1) if not use_double else Gf.Quatd(0, 0, 0, 1),
                            type_to_create_if_not_exist=UsdGeom.XformOp.TypeOrient,
                            timecode=Usd.TimeCode(time))

def inf_helper(y: np.array):
  """Helper to handle indices and logical indices of NaNs.
  Input:
          - y, 1d numpy array with possible NaNs
  Output:
          - nans, logical indices of NaNs
          - index, a function, with signature indices= index(logical_indices),
              to convert logical indices of NaNs to 'equivalent' indices
  """
  return np.isinf(y), lambda z: z.nonzero()[0]

def position_object(environment, type: int, objects: list = [], ob_stl_paths: list = [], reset: bool = False,
                    max_collisions: int = 200):
  """
  type = 0 -> camera z_lim = [0.8 - 1.8] using camera stl
  type = 1 -> humans z_lim = [0 - 0] using human stl
  type = 2 -> shapenet z_lim = [0 - 1.8] using camera stl
  type = 3 -> origin z_lim = [0 - 0] using camera stl
  note: when min == max we apply a small offset to the max to address shifts in the z-axis to allow small collisions.
   However, the result will be still published at the wanted height.

  envionment: the environment object
  type: see above
  objects: the list of objects to be placed
  ob_stl_paths: the corresponding stls
  reset: if the collision checker need to be resetted forcefully
  """
  # thih import will work if you compile our https://github.com/eliabntt/moveit_based_collision_checker_and_placement/tree/main
  # and you add the source catkin command to isaac_X_X/setup_python_env.sh
  from collision_check.srv import *
  if environment.env_stl_path == None:
    print(
      "No stl is being loaded for the environment, please pre-fix all objects locations or implement your own strategy")
    environment.env_stl_path = ""
  print("Wait for service")
  rospy.wait_for_service("/fake/collision_checker/check")
  print("Service loaded")
  try:
    check_collision = rospy.ServiceProxy("/fake/collision_checker/check", collision_check_srv)
    req = collision_check_srvRequest()
    req.env_stl_path = environment.env_stl_path
    req.env_polygon = environment.env_polygon
    req.reset = reset
    if type == 1:
      for ob in objects:
        req.ob_names.append(ob)
      req.ob_stl_paths = ob_stl_paths
    req.is_cam = True if type != 1 else False
    min_z = (0.8 + environment.env_limits[2]) if type == 0 else environment.env_limits[2]
    max_z = environment.env_limits[2] if (type == 1 or type == 3) else min(1.8 + environment.env_limits[2],
                                                                           environment.env_limits[5] - 0.5)
    if type == 4:
      min_z = environment.env_limits[2]
      max_z = environment.env_limits[2]
    has_forced_z = -1
    if min_z == max_z:
      max_z += 0.5
      has_forced_z = min_z
    req.min_limits = [environment.env_limits[0] + 0.5, environment.env_limits[1] + 0.5, min_z]
    req.max_limits = [environment.env_limits[3] - 0.5, environment.env_limits[4] - 0.5, max_z]
    req.limit_collision = 0 if type != 1 else max_collisions
    req.forced_z = has_forced_z
    res = check_collision.call(req)
    if has_forced_z != -1:
      res.z = [min(has_forced_z, z) for z in res.z]
    return np.array(res.x) - environment.shifts[0], np.array(res.y) - environment.shifts[1], np.array(res.z) - \
           environment.shifts[2], res.yaw
  except rospy.ServiceException as e:
    print("Service call failed: %s" % e)
    return [-1] * len(objects), [-1] * len(objects), [-1] * len(objects), [0] * len(objects)

def set_scale(prim: Prim, scale: float = 1.0):
  """
  Set the scale of a Prim

  prim: the prim
  scale: the scale
  """
  prop_names = prim.GetPropertyNames()
  if "xformOp:scale" not in prop_names:
    xformable = UsdGeom.Xformable(prim)
    xform_op_scale = xformable.AddXformOp(UsdGeom.XformOp.TypeScale, UsdGeom.XformOp.PrecisionDouble, "")
  else:
    xform_op_scale = UsdGeom.XformOp(prim.GetAttribute("xformOp:scale"))
  xform_op_scale.Set(Gf.Vec3d([scale, scale, scale]))

def clear_properties(path: str):
  """
  The function clears all the POSE properties of the given prim.
  This is to ensure a consistent way of setting those properties for different objects.

  This should be called with ALL loaded objects so that we have consistent xformOp:trans/Orient
  """
  current_position, current_orientation = XFormPrim(path).get_world_pose()

def set_translate(prim: Prim, new_loc: list):
  """
  prim: must be prim type, the prim to be moved
  new_loc: list [x-y-z] for the single prim
  """
  properties = prim.GetPropertyNames()
  if "xformOp:translate" in properties:
    translate_attr = prim.GetAttribute("xformOp:translate")
    translate_attr.Set(Gf.Vec3d(new_loc))
  elif "xformOp:transform" in properties:
    transform_attr = prim.GetAttribute("xformOp:transform")
    matrix = prim.GetAttribute("xformOp:transform").Get()
    matrix.SetTranslateOnly(Gf.Vec3d(new_loc))
    transform_attr.Set(matrix)
  else:
    xform = UsdGeom.Xformable(prim)
    xform_op = xform.AddXformOp(UsdGeom.XformOp.TypeTranslate, UsdGeom.XformOp.PrecisionDouble, "")
    xform_op.Set(Gf.Vec3d(new_loc))

def set_rotate(prim: XFormPrim, rot: list):
  """
  expects rot in rad

  prim: The prim to be rotated
  rot: roll-pitch-yaw in RAD
  """
  properties = prim.GetPropertyNames()
  rot = np.array(rot) * 180 / np.pi
  quat = (
      Gf.Rotation(Gf.Vec3d.XAxis(), rot[0])
      * Gf.Rotation(Gf.Vec3d.YAxis(), rot[1])
      * Gf.Rotation(Gf.Vec3d.ZAxis(), rot[2])
  )
  if "xformOp:orient" in properties:
    rotation = prim.GetAttribute("xformOp:orient")
    rotation.Set(Gf.Quatd(quat.GetQuat()))
  else:
    xform = UsdGeom.Xformable(prim)
    xform_op = xform.AddXformOp(UsdGeom.XformOp.TypeOrient, UsdGeom.XformOp.PrecisionDouble, "")
    xform_op.Set(Gf.Quatd(quat.GetQuat()))

def dynamic_control_interface():
  """
  This is placed here as the extension is not loaded in the main script.
  """
  return _dynamic_control.acquire_dynamic_control_interface()

def reload_references(path):
  """
  It reloads all the references and payloads of a given prim

  :param path: The path to the prim you want to reload references for
  """
  stage = omni.usd.get_context().get_stage()
  prim_list = []
  for j in stage.GetPrimAtPath(path).GetAllChildren():
    prim_list.append(j)
  layers = set()
  for prim in prim_list:
    for (ref, intro_layer) in omni.usd.get_composed_references_from_prim(prim):
      layer = Sdf.Find(intro_layer.ComputeAbsolutePath(ref.assetPath)) if ref.assetPath else None
      if layer:
        layers.add(layer)
    for (ref, intro_layer) in omni.usd.get_composed_payloads_from_prim(prim):
      layer = Sdf.Find(intro_layer.ComputeAbsolutePath(ref.assetPath)) if ref.assetPath else None
      if layer:
        layers.add(layer)
  for l in layers:
    l.Reload(force=True)

def teleport(path, loc, rot):
  """
  It teleports the object at the given path to the given location and rotation

  :param path: The path to the object you want to teleport
  :param loc: (x, y, z)
  :param rot: (x, y, z, w)
  """
  omni.kit.commands.execute(
    "IsaacSimTeleportPrim",
    prim_path=path,
    translation=(loc[0], loc[1], loc[2]),
    rotation=(rot[0], rot[1], rot[2], rot[3]),
  )

def toggle_dynamic_objects(dynamic_prims: list, status: bool):
  """
  It toggles the visibility of the dynamic objects in the scene

  :param dynamic_prims: a list of prims that you want to toggle
  :type dynamic_prims: list
  """
  # print("Toggling environment...")
  for _ in range(3):
    for prim in dynamic_prims:
      imageable = UsdGeom.Imageable(prim)
      if status:
        imageable.MakeVisible()
      else:
        imageable.MakeInvisible()
      imageable = []

def reset_physics(timeline, simulation_context):
    timeline.stop()
    simulation_context.reset()
    timeline.play()