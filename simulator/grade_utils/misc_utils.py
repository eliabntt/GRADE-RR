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

def get_area(polygon):
	"""
	Computes the area of a polygon.
	:param polygon: np.ndarray of shape (N, 2) representing the vertices of the polygon
	:return: float, area of the polygon
	"""
	x = polygon[:, 0]
	y = polygon[:, 1]
	return 0.5 * np.abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))


# ros
try:
	try:
		import rclpy
		from rclpy.node import Node
		from geometry_msgs.msg import PoseStamped, Point
		from nav_msgs.msg import Odometry
		from sensor_msgs.msg import Imu
		from std_msgs.msg import String
		ROS2_AVAILABLE = True
	except ImportError:
		rclpy = None
		Node = None
		PoseStamped = None
		Point = None
		Odometry = None
		Imu = None
		String = None
		ROS2_AVAILABLE = False
except ImportError:
	rclpy = None
	Node = None
	PoseStamped = None
	Point = None
	Odometry = None
	Imu = None
	String = None
	ROS2_AVAILABLE = False

shapenet = None
shapenet = None
try:
	import omni.isaac.shapenet as shapenet
except Exception as e:
	print("[INFO] omni.isaac.shapenet extension not available or failed to import:", e)
	shapenet = None

import omni.kit
from omni.isaac import RangeSensorSchema
from omni.isaac.core import SimulationContext, PhysicsContext
import omni.replicator.core as rep
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.utils.carb import set_carb_setting
from isaacsim.core.utils.extensions import enable_extension, disable_extension
from isaacsim.core.utils.stage import is_stage_loading, set_stage_up_axis
from omni.isaac.dynamic_control import _dynamic_control
try:
	import omni.isaac.IsaacSensorSchema as IsaacSensorSchema
except ImportError:
	IsaacSensorSchema = None
try:
	from omni.isaac.synthetic_recorder import extension_custom
except ModuleNotFoundError:
	extension_custom = None
try:
	from omni.physxcommands import SetStaticColliderCommand, RemoveStaticColliderCommand
except ModuleNotFoundError:
	SetStaticColliderCommand = None
	RemoveStaticColliderCommand = None
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
	try:
		if not prim.HasAPI(Semantics.SemanticsAPI):
			sem = Semantics.SemanticsAPI.Apply(prim, "Semantics")
			sem.CreateSemanticTypeAttr()
			sem.CreateSemanticDataAttr()
		else:
			sem = Semantics.SemanticsAPI.Get(prim, "Semantics")
		sem.GetSemanticTypeAttr().Set("class")
		sem.GetSemanticDataAttr().Set(str(semantic_label))
	except Exception:
		# Some legacy assets contain malformed semantic attrs (empty typeName).
		# Skip semantic tagging for those prims to keep simulation running.
		return False
	return True

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


def set_scale(prim: Prim, scale: float = 1.0):
	"""
	Set uniform scale on a prim.
	"""
	if prim is None or not prim.IsValid():
		return
	prop_names = prim.GetPropertyNames()
	if "xformOp:scale" not in prop_names:
		xformable = UsdGeom.Xformable(prim)
		xform_op_scale = xformable.AddXformOp(UsdGeom.XformOp.TypeScale, UsdGeom.XformOp.PrecisionDouble, "")
	else:
		xform_op_scale = UsdGeom.XformOp(prim.GetAttribute("xformOp:scale"))
	xform_op_scale.Set(Gf.Vec3d([scale, scale, scale]))


def clear_properties(path: str):
	"""
	Ensure a prim has translate/orient/scale xform ops in a stable order.
	"""
	stage = omni.usd.get_context().get_stage()
	prim = stage.GetPrimAtPath(path)
	if prim is None or not prim.IsValid():
		return

	xformable = UsdGeom.Xformable(prim)
	properties = prim.GetPropertyNames()

	if "xformOp:translate" in properties:
		t_op = UsdGeom.XformOp(prim.GetAttribute("xformOp:translate"))
	else:
		t_op = xformable.AddXformOp(UsdGeom.XformOp.TypeTranslate, UsdGeom.XformOp.PrecisionDouble, "")
		t_op.Set(Gf.Vec3d(0.0, 0.0, 0.0))

	if "xformOp:orient" in properties:
		o_op = UsdGeom.XformOp(prim.GetAttribute("xformOp:orient"))
	else:
		o_op = xformable.AddXformOp(UsdGeom.XformOp.TypeOrient, UsdGeom.XformOp.PrecisionDouble, "")
		o_op.Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))

	if "xformOp:scale" in properties:
		s_op = UsdGeom.XformOp(prim.GetAttribute("xformOp:scale"))
	else:
		s_op = xformable.AddXformOp(UsdGeom.XformOp.TypeScale, UsdGeom.XformOp.PrecisionDouble, "")
		s_op.Set(Gf.Vec3d(1.0, 1.0, 1.0))

	try:
		xformable.SetXformOpOrder([t_op, o_op, s_op])
	except Exception:
		pass


def set_translate(prim: Prim, new_loc: list):
	"""
	Set prim translation.
	"""
	if prim is None or not prim.IsValid():
		return
	properties = prim.GetPropertyNames()
	if "xformOp:translate" in properties:
		translate_attr = prim.GetAttribute("xformOp:translate")
		translate_attr.Set(Gf.Vec3d(new_loc))
	elif "xformOp:transform" in properties:
		transform_attr = prim.GetAttribute("xformOp:transform")
		matrix = transform_attr.Get()
		matrix.SetTranslateOnly(Gf.Vec3d(new_loc))
		transform_attr.Set(matrix)
	else:
		xform = UsdGeom.Xformable(prim)
		xform_op = xform.AddXformOp(UsdGeom.XformOp.TypeTranslate, UsdGeom.XformOp.PrecisionDouble, "")
		xform_op.Set(Gf.Vec3d(new_loc))


def set_rotate(prim: Prim, rot: list):
	"""
	Set prim rotation using roll-pitch-yaw in radians.
	"""
	if prim is None or not prim.IsValid():
		return
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


def add_translate_anim(prim_path: str, pos: Gf.Vec3d, time: float = 0.0):
	"""
	Add translate keyframe animation.
	"""
	omni.kit.commands.execute(
		"ChangePropertyCommand",
		prop_path=prim_path + ".xformOp:translate",
		value=pos,
		prev=Gf.Vec3d(0, 0, 0),
		type_to_create_if_not_exist=UsdGeom.XformOp.TypeTranslate,
		timecode=Usd.TimeCode(time),
	)


def add_rotation_anim(prim_path: str, rot: list, time: float = 0.0, use_double=False):
	"""
	Add rotation keyframe animation. `rot` is expected in degrees.
	"""
	quat = (
		Gf.Rotation(Gf.Vec3d.XAxis(), rot[0])
		* Gf.Rotation(Gf.Vec3d.YAxis(), rot[1])
		* Gf.Rotation(Gf.Vec3d.ZAxis(), rot[2])
	)
	rotation_value = Gf.Quatd(quat.GetQuat()) if use_double else Gf.Quatf(quat.GetQuat())
	omni.kit.commands.execute(
		"ChangePropertyCommand",
		prop_path=prim_path + ".xformOp:orient",
		value=rotation_value,
		prev=Gf.Quatf(1, 0, 0, 0),
		type_to_create_if_not_exist=UsdGeom.XformOp.TypeOrient,
		timecode=Usd.TimeCode(time),
	)


def change_prim_collision(enable, prim_path):
	"""
	Toggle collision enabled for all prims under `prim_path`.
	"""
	stage = omni.usd.get_context().get_stage()
	for prim in stage.Traverse():
		path = str(prim.GetPath())
		if not path.startswith(prim_path):
			continue
		if "physics:collisionEnabled" in prim.GetPropertyNames():
			try:
				omni.kit.commands.execute(
					"ChangeProperty",
					prop_path=Sdf.Path(path + ".physics:collisionEnabled"),
					value=bool(enable),
					prev=None,
				)
			except Exception:
				try:
					prim.GetAttribute("physics:collisionEnabled").Set(bool(enable))
				except Exception:
					pass


def dynamic_control_interface():
	"""
	Acquire dynamic control interface.
	"""
	try:
		return _dynamic_control.acquire_dynamic_control_interface()
	except Exception:
		return None


def reload_references(path):
	"""
	Best-effort reload of references for a prim.
	"""
	stage = omni.usd.get_context().get_stage()
	prim = stage.GetPrimAtPath(path)
	if prim and prim.IsValid():
		try:
			prim.GetReferences().ClearReferences()
		except Exception:
			pass


def teleport(path, loc, rot):
	"""
	Teleport a prim to `loc` and quaternion `rot` (x, y, z, w).
	"""
	try:
		omni.kit.commands.execute(
			"IsaacSimTeleportPrim",
			prim_path=path,
			translation=(loc[0], loc[1], loc[2]),
			rotation=(rot[0], rot[1], rot[2], rot[3]),
		)
		return
	except Exception:
		pass

	stage = omni.usd.get_context().get_stage()
	prim = stage.GetPrimAtPath(path)
	if prim is None or not prim.IsValid():
		return
	set_translate(prim, loc)
	quat = Gf.Quatd(float(rot[3]), Gf.Vec3d(float(rot[0]), float(rot[1]), float(rot[2])))
	if "xformOp:orient" in prim.GetPropertyNames():
		prim.GetAttribute("xformOp:orient").Set(quat)
	else:
		xform = UsdGeom.Xformable(prim)
		xform_op = xform.AddXformOp(UsdGeom.XformOp.TypeOrient, UsdGeom.XformOp.PrecisionDouble, "")
		xform_op.Set(quat)


def toggle_dynamic_objects(dynamic_prims: list, status: bool):
	"""
	Toggle visibility for dynamic objects.
	"""
	for _ in range(2):
		for prim in dynamic_prims:
			if prim is None:
				continue
			imageable = UsdGeom.Imageable(prim)
			if status:
				imageable.MakeVisible()
			else:
				imageable.MakeInvisible()


def reset_physics(timeline, simulation_context):
	timeline.stop()
	simulation_context.reset()
	timeline.play()
