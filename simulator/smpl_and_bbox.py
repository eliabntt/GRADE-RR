import argparse
import carb
import confuse
import ipdb
import math
import numpy as np
import os
import roslaunch
import rospy
import scipy.spatial.transform as tf
import sys
import time
import traceback
import trimesh
import yaml
from omni.isaac.kit import SimulationApp
from time import sleep

from omni.syntheticdata import sensors, helpers as sensors, generic_helper_lib
def get_obj_pose(time):
	"""Get pose of all objects with a semantic label.
	"""
	stage = omni.usd.get_context().get_stage()
	mappings = generic_helper_lib.get_instance_mappings()
	pose = []
	for m in mappings:
		prim_path = m[1]
		prim = stage.GetPrimAtPath(prim_path)
		prim_tf = omni.usd.get_world_transform_matrix(prim, time)
		pose.append((str(prim_path), m[2], str(m[3]), np.array(prim_tf)))
	return pose

def boolean_string(s):
	if s.lower() not in {'false', 'true'}:
		raise ValueError('Not a valid boolean string')
	return s.lower() == 'true'


"""
Exported information will have the shape of
[[prim_asset_path, bbox] [prim_asset_path,skel] [prim_asset_path, init_tf, init_rot]]
prim_asset_path is string of the asset in the simulation.
It will be processed in order so expect groups of human,cloth --- possibly reversed

All is output in WORLD frame. Please check the notes regarding projection in camera frame.

bbox will be of shape (ef, 8, 3) if only one bbox is saved or (ef, 2, 8, 3) if both are saved

ef will be either the last animated frame (given the simulated environment) or the last frame of the animations + 1

if you need to access the bbox of the mesh after that just use [-1]

skel is the smpl skeleton info

use the flags below to export only the skeleton, only the garments or only the body or any combination

init_rot is the same of the info file
init_tf is equal, except that here we account for the small vertical translation that is added to meshes very close to the ground
-- this was a bug during the data generation which actually has very little influence (< 0.1 cm in vertical displacement)
-- the design choice was to save the placement value and then have always a way to recover the eventual vertical displacement which is anyway based on a rule (check human_utils.py:move_humans_to_ground)
everything is in meters

NOTE: We start writing images from timeline.frame = 1 (1/fps) since the "forward_timeline" call has been placed _before_ the publishing
"""
try:
	parser = argparse.ArgumentParser(description="Get Bounding Boxes")
	parser.add_argument("--experiment_folder", type=str,
	                    help="The experiment folder with the USD file and the info file")
	parser.add_argument("--body", type=boolean_string, default=True, help="When true process the bodies")
	parser.add_argument("--garments", type=boolean_string, default=True, help="When true process the garments")
	parser.add_argument("--base_path", type=str, default="my_human_", help="Human prim base path")
	parser.add_argument("--headless", type=boolean_string, default=False, help="Whether run this headless or not")
	parser.add_argument("--write", type=boolean_string, default=True, help="Whether to write results")
	parser.add_argument("--both", type=boolean_string, default=False,
	                    help="Whether to write both vertex types -- preference in code is both - fast - slow")
	parser.add_argument("--fast", type=boolean_string, default=True,
	                    help="Whether to write only the axis-aligned box or the oriented one")
	parser.add_argument("--only_exp", type=boolean_string, default=True,
	                    help="Whether to export only the experiment (considering the reverse strategy) or the whole sequences")
	parser.add_argument("--get_skel", type=boolean_string, default=True, help="Whether to include the skeleton info")
	parser.add_argument("--skel_root", type=str, default="avg_root",
	                    help="This is a recognizable last part of the root of the skeleton prim, in our case _avg_root "
	                         + "It will process ONLY the path of which the last part is this root")
	parser.add_argument("--correct_poses", type=boolean_string, default=False)

	args, unknown = parser.parse_known_args()
	config = confuse.Configuration("BoundingBoxes", __name__)
	config.set_args(args)

	exp_info = np.load(os.path.join(config["experiment_folder"].get(), "experiment_info.npy"), allow_pickle=True)
	exp_info = exp_info.item()

	CONFIG = {"display_options": 3286, "width": 1280, "height": 720, "headless": config["headless"].get()}
	kit = SimulationApp(launch_config=CONFIG, experience=f"{os.environ['EXP_PATH']}/omni.isaac.sim.python.kit")

	import utils.misc_utils
	from utils.misc_utils import *
	from utils.robot_utils import *
	from utils.simulation_utils import *
	from utils.objects_utils import *
	from utils.environment_utils import *
	from utils.human_utils import *

	simulation_environment_setup()
	local_file_prefix = "my-computer://"
	omni.usd.get_context().open_stage(local_file_prefix + config["experiment_folder"].get() + "/loaded_stage.usd", None)
	kit.update()
	kit.update()

	print("Loading stage...")
	while is_stage_loading():
		kit.update()
	print("Loading Complete")

	context = omni.usd.get_context()
	stage = context.get_stage()
	set_stage_up_axis("Z")

	simulation_context = SimulationContext(physics_dt=1.0 / exp_info["config"]["physics_hz"].get(),
	                                       rendering_dt=1.0 / exp_info["config"]["render_hz"].get(),
	                                       stage_units_in_meters=0.01)
	simulation_context.start_simulation()
	meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)
	set_raytracing_settings(exp_info["config"]["physics_hz"].get())
	timeline = setup_timeline(exp_info["config"])

	base_path = config["base_path"].get()

	fast, both, slow = False, False, False
	if config["both"].get():
		both = True
	elif config["fast"].get():
		fast = True
	else:
		slow = True

	get_skel = config["get_skel"]

	only_exp = config["only_exp"].get()

	humans_info = exp_info["humans"]
	write = config["write"].get()
	if write:
		results = []
	stime = time.time()

	helper_list_global = []
	helper_list_skel = []

	skel_root = config["skel_root"].get()

	smpl_info_path = ""
	for prim in stage.Traverse():
		prim_path = str(prim.GetPath()).lower()

		if base_path in prim_path:
			if (get_skel and skel_root in prim_path and prim_path[:prim_path.find(skel_root)] not in helper_list_skel) or \
					(str(prim.GetTypeName()).lower() == "mesh" and "points" in prim.GetPropertyNames()):
				print(f"Processing {prim}")
				parent = prim.GetParent()
				refs = omni.usd.get_composed_references_from_prim(parent)
				while len(refs) == 0:
					parent = parent.GetParent()
					refs = omni.usd.get_composed_references_from_prim(parent)
				human_global_path = str(omni.usd.get_composed_references_from_prim(parent)[0][0].assetPath)
				human_global_path = human_global_path[len(local_file_prefix):]
				index = humans_info['folders'].index(human_global_path[:-3] + "stl")

				init_tf = np.array(parent.GetAttribute("xformOp:translate").Get())
				init_rot = parent.GetAttribute("xformOp:orient").Get()
				init_rot = np.array([init_rot.GetImaginary()[0], init_rot.GetImaginary()[1], init_rot.GetImaginary()[2],
				                     init_rot.GetReal()])
				init_rot_mat = tf.Rotation.from_quat(init_rot).as_matrix()

				if write and str(parent.GetPath()) not in helper_list_global:
					results.append([str(parent.GetPath()), init_tf, init_rot])
					helper_list_global.append(str(parent.GetPath()))

				if human_global_path[:-3] + "pkl" != smpl_info_path:
					smpl_info_path = human_global_path[:-3] + "pkl"
					smpl_anim_info = pkl.load(open(smpl_info_path, 'rb'))
					smpl_info = smpl_anim_info["info"]
					r = smpl_info["zrot"]
					rot_mat = tf.Rotation.from_euler('z', r).as_matrix()

				ef = int(math.ceil(smpl_anim_info["ef"] * exp_info["config"]["fps"].get() / 24))
				if only_exp:
					ef = min(ef, int(math.ceil(
						exp_info["config"]["experiment_length"].get() / exp_info['reversing_timeline_ratio'])))
				if (get_skel and skel_root in prim_path):
					helper_list_skel.append(prim_path[:prim_path.find(skel_root)])
					skeleton, joint_token = AnimationSchema.SkelJoint(prim).GetJoint()
					skel_cache = UsdSkel.Cache()
					skel_query = skel_cache.GetSkelQuery(UsdSkel.Skeleton(skeleton.GetPrim()))
					xfCache = UsdGeom.XformCache()
					skeleton_info = np.empty((ef, 3), dtype=object)
					for i in range(0, ef):
						xfCache.SetTime(i)
						transforms = skel_query.ComputeJointWorldTransforms(xfCache)
						translates, rotations, scales = UsdSkel.DecomposeTransforms(transforms)
						skeleton_info[i] = [np.array(translates) * meters_per_unit, np.array(rotations),
						                    np.array(scales) * meters_per_unit]
					if write:
						results.append([str(prim.GetPath()), np.array(skeleton_info)])
				else:
					points = UsdGeom.PointBased(prim)
					if both:
						bounds = np.zeros((ef, 2, 8, 3))
					else:
						bounds = np.zeros((ef, 8, 3))
					for i in range(0, ef):
						points_in_mesh = points.ComputePointsAtTime(i, Usd.TimeCode(i))
						points_in_mesh = np.array(points_in_mesh)
						# bound = points.ComputeWorldBound(i, "default")
						# for j in range(8):
						#    print(bound.ComputeAlignedRange().GetCorner(j))
						points_in_mesh = ((points_in_mesh @ rot_mat.T @ init_rot_mat.T) + init_tf * meters_per_unit)
						# normals = prim.GetAttribute("normals").Get(i)
						# normals = np.array(normals)

						mymesh = trimesh.PointCloud(points_in_mesh)
						if fast:
							temp_bounds = mymesh.bounding_box.vertices
						elif slow:
							temp_bounds = mymesh.bounding_box_oriented.vertices
						elif both:
							temp_bounds = [mymesh.bounding_box.vertices, mymesh.bounding_box_oriented.vertices]

						bounds[i] = temp_bounds
					if write:
						results.append([str(prim.GetPath()), bounds])

	results = np.array(results, dtype=object)
	print(f"etime {time.time() - stime}")
	if write:
		np.save(os.path.join(config["experiment_folder"].get(), "bboxes.npy"), results)
except:
	extype, value, tb = sys.exc_info()
	traceback.print_exc()

	import ipdb

	ipdb.set_trace()
finally:

	simulation_context.stop()
	try:
		kit.close()
	except:
		pass
