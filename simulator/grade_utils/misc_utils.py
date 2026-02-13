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

# shapenet
try:
	import omni.isaac.shapenet as shapenet
except ImportError:
	shapenet = None

import omni.kit
from omni.isaac import RangeSensorSchema
from omni.isaac.core import SimulationContext, PhysicsContext
import omni.replicator.core as rep
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.carb import set_carb_setting
from omni.isaac.core.utils.extensions import enable_extension, disable_extension
from omni.isaac.core.utils.stage import is_stage_loading, set_stage_up_axis
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
