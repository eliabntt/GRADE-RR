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
	parser.add_argument("--base_human_path", type=str, default="my_human_", help="Human prim base path")
	parser.add_argument("--headless", type=boolean_string, default=False, help="Whether run this headless or not")
	parser.add_argument("--write", type=boolean_string, default=True, help="Whether to write results")
	parser.add_argument("--both", type=boolean_string, default=False,
	                    help="Whether to write both vertex types -- preference in code is both - fast - slow")
	parser.add_argument("--fast", type=boolean_string, default=True,
	                    help="Whether to write only the axis-aligned box or the oriented one")
	parser.add_argument("--get_skel", type=boolean_string, default=True, help="Whether to include the skeleton info")
	parser.add_argument("--skel_root", type=str, default="avg_root",
	                    help="This is a recognizable last part of the root of the skeleton prim, in our case _avg_root "
	                         + "It will process ONLY the path of which the last part is this root")
	parser.add_argument("--correct_poses", type=boolean_string, default=True)
	parser.add_argument("--old_poses", type=str, default='')
	parser.add_argument("--decimate", type=int, default=0, help="Decimate the mesh by this factor")
	parser.add_argument("--output_dir_humans", type=str)
	parser.add_argument("--output_dir_poses", type=str)

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
	from omni.syntheticdata import sensors, helpers


	def get_obj_poses(time,mappings = []):
		"""Get pose of all objects with a semantic label.
		"""
		stage = omni.usd.get_context().get_stage()
		if len(mappings) == 0:
			mappings = helpers.get_instance_mappings()
		pose = []
		for m in mappings:
			prim_path = m[1]
			prim = stage.GetPrimAtPath(prim_path)
			prim_tf = omni.usd.get_world_transform_matrix(prim, time)
			pose.append((str(prim_path), m[2], str(m[3]), np.array(prim_tf)))
		return pose

	simulation_environment_setup(need_ros=False)
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

	base_human_path = config["base_human_path"].get()

	fast, both, slow = False, False, False
	if config["both"].get():
		both = True
	elif config["fast"].get():
		fast = True
	else:
		slow = True

	get_skel = config["get_skel"]

	human_prims = [x for x in stage.GetPrimAtPath('/').GetAllChildren() if base_human_path in x.GetName()]
	humans_info = exp_info["humans"]
	for prim in stage.Traverse():
		if "human" in prim.GetName():
			imageable = UsdGeom.Imageable(prim)
			imageable.MakeVisible()

	for id, folder in enumerate(humans_info['folders']):
		# if folder does not exist, remove the _with_cache from ids, folder
		if not os.path.exists(folder):
			humans_info['ids'][id] = humans_info['ids'][id].replace("_with_cache", "")
			humans_info['folders'][id] = humans_info['folders'][id].replace("_with_cache", "")

			human_prim = human_prims[id]
			human_global_path = str(omni.usd.get_composed_references_from_prim(human_prim)[0][0].assetPath)
			human_global_path = human_global_path.replace("_with_cache", "")
			human_prim.GetReferences().SetReferences([Sdf.Reference(assetPath=human_global_path)])

	for _ in range(100):
		kit.update()

	write = config["write"].get()
	if write:
		results = []

	decimate = config["decimate"].get()

	helper_list_global = []
	helper_list_skel = []

	skel_root = config["skel_root"].get()

	smpl_info_path = ""
	experiment_length = exp_info["config"]["experiment_length"].get()
	reversing_timeline_ratio = exp_info['reversing_timeline_ratio']
	fn = 0
	forward = True
	cnt_reversal = 1

	mapping = []
	if config["correct_poses"].get():
		if config["old_poses"].get() != '':
			old_poses = np.load(config["old_poses"].get(), allow_pickle=True)
			for i in range(len(old_poses)):
				mapping.append([i, old_poses[i][0], old_poses[i][1], old_poses[i][2]])
		else:
			print("Using local mapping")
			sleep(10)

	i = 0

	out_dir_humans = config["output_dir_humans"].get()
	out_dir_poses = config["output_dir_poses"].get()
	for folder in [out_dir_humans, out_dir_poses]:
		if not os.path.exists(folder):
			os.makedirs(folder)

	for simulation_step in range(experiment_length+1):
		fn += 1
		stime = time.time()

		if simulation_step < (experiment_length / reversing_timeline_ratio) * (
				cnt_reversal):
			forward = True
			timeline.forward_one_frame() # or you can advance time directly here as it was done previously, note that you need to remove the "+ratio_camera" above
		else:
			if simulation_step >= ((experiment_length - 1) / reversing_timeline_ratio) * (
					cnt_reversal + 1) or \
					(timeline.get_current_time() - 1 / timeline.get_time_codes_per_seconds()) < 0:
				cnt_reversal += 2
				forward = True
				timeline.forward_one_frame()
			else:
				forward = False
				timeline.rewind_one_frame()
		i = i + 1 if forward else i - 1

		print(f"Processing frame {fn} which correspond to fram number {i}-th in the timeline at ctim {timeline.get_current_time()}")
		print("You can check the time with the information in Vieport/camera/i-th.npy. Field 'ctime'.")
		results = {'bbox3d':{},'skel':{},'verts':{},'init_pose':{}}
		if config["correct_poses"].get():
			poses = get_obj_poses(Usd.TimeCode(i),mapping)
			if write:
				try:
					np.save(os.path.join(out_dir_poses, f"{fn}.npy"), poses)
				except:
					print("Error saving poses")
					import ipdb; ipdb.set_trace()

		for prim in stage.Traverse():
			prim_path = str(prim.GetPath()).lower()

			if base_human_path in prim_path:
				if (get_skel and skel_root in prim_path and prim_path[:prim_path.find(skel_root)] not in helper_list_skel) or \
						(str(prim.GetTypeName()).lower() == "mesh" and "points" in prim.GetPropertyNames()):
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

					if write and str(parent.GetPath()):
						results['init_pose'][str(parent.GetPath())] = [init_tf, init_rot]

					if human_global_path[:-3] + "pkl" != smpl_info_path:
						smpl_info_path = human_global_path[:-3] + "pkl"
						# if the path does not exist
						smpl_anim_info = pkl.load(open(smpl_info_path, 'rb'))
						smpl_info = smpl_anim_info["info"]
						if 'zrot' in smpl_info.keys():
							r = smpl_info["zrot"]
						else:
							r = smpl_info['poses'][0, :3][2]
						rot_mat = tf.Rotation.from_euler('z', r).as_matrix()

					if (get_skel and skel_root in prim_path):
						helper_list_skel.append(prim_path[:prim_path.find(skel_root)])
						skeleton, joint_token = AnimationSchema.SkelJoint(prim).GetJoint()
						skel_cache = UsdSkel.Cache()
						skel_query = skel_cache.GetSkelQuery(UsdSkel.Skeleton(skeleton.GetPrim()))
						xfCache = UsdGeom.XformCache()
						skeleton_info = np.empty((1, 3), dtype=object)
						xfCache.SetTime(i)
						transforms = skel_query.ComputeJointWorldTransforms(xfCache)
						translates, rotations, scales = UsdSkel.DecomposeTransforms(transforms)
						skeleton_info[0] = [np.array(translates) * meters_per_unit, np.array(rotations),
						                    np.array(scales) * meters_per_unit]

						if write:
							results['skel'][str(prim.GetPath())] = np.array(skeleton_info)
					else:
						points = UsdGeom.PointBased(prim)
						if both:
							bounds = np.zeros((1, 2, 8, 3))
						else:
							bounds = np.zeros((1, 8, 3))

						points_in_mesh = points.ComputePointsAtTime(i, Usd.TimeCode(i))
						points_in_mesh = np.array(points_in_mesh)
						# bound = points.ComputeWorldBound(i, "default")
						# for j in range(8):
						#    print(bound.ComputeAlignedRange().GetCorner(j))
						points_in_mesh = ((points_in_mesh @ rot_mat.T @ init_rot_mat.T) + init_tf * meters_per_unit)
						# normals = prim.GetAttribute("normals").Get(i)
						# normals = np.array(normals)
						results['verts'][str(prim.GetPath())] = points_in_mesh

						oldmesh = trimesh.PointCloud(points_in_mesh)

						# decimate points_in_mesh
						if decimate > 1:
							points_in_mesh = points_in_mesh[::decimate]
						mymesh = trimesh.PointCloud(points_in_mesh)
						if fast:
							temp_bounds = oldmesh.bounding_box.vertices
						elif slow:
							temp_bounds = mymesh.bounding_box_oriented.vertices
						elif both:
							temp_bounds = [oldmesh.bounding_box.vertices, mymesh.bounding_box_oriented.vertices]

						if write:
							if both:
								results['bbox3d'][str(prim.GetPath())] = {}
								results['bbox3d'][str(prim.GetPath())]['aligned'] = np.array(temp_bounds[0])
								results['bbox3d'][str(prim.GetPath())]['oriented'] = np.array(temp_bounds[1])
							else:
								results['bbox3d'][str(prim.GetPath())] = np.array(temp_bounds)
		humans = []
		# for each results['bbox3d'] get the human (/my_human_x) and combine its bounding boxes
		dic_keys = list(results['bbox3d'].keys())
		for key in dic_keys:
			newkey = key[:key[1:].find("/")+1]
			if newkey not in humans:
				humans.append(newkey)
			if newkey not in results['bbox3d'].keys():
				results['bbox3d'][newkey] = results['bbox3d'][key]
			else:
				# extend
				if both:
					results['bbox3d'][newkey]['aligned'] = np.concatenate(
						(results['bbox3d'][newkey]['aligned'], results['bbox3d'][key]['aligned']))
					results['bbox3d'][newkey]['oriented'] = np.concatenate(
						(results['bbox3d'][newkey]['oriented'], results['bbox3d'][key]['oriented']))
				else:
					results['bbox3d'][newkey] = np.concatenate((results['bbox3d'][newkey], results['bbox3d'][key]))
		# merge the boxes
		for key in humans:
			if both:
				points_in_mesh = results['bbox3d'][key]['aligned']
				mymesh = trimesh.PointCloud(points_in_mesh)
				temp_bounds = mymesh.bounding_box.vertices
				results['bbox3d'][key]['aligned'] = np.array(temp_bounds)
				points_in_mesh = results['bbox3d'][key]['oriented']
				mymesh = trimesh.PointCloud(points_in_mesh)
				temp_bounds = mymesh.bounding_box_oriented.vertices
				results['bbox3d'][key]['oriented'] = np.array(temp_bounds)
			else:
				if slow:
					points_in_mesh = results['bbox3d'][key]
					mymesh = trimesh.PointCloud(points_in_mesh)
					temp_bounds = mymesh.bounding_box_oriented.vertices
					results['bbox3d'][key] = np.array(temp_bounds)
				else:
					points_in_mesh = results['bbox3d'][key]
					mymesh = trimesh.PointCloud(points_in_mesh)
					temp_bounds = mymesh.bounding_box.vertices
					results['bbox3d'][key] = np.array(temp_bounds)

		results = np.array(results, dtype=object)
		print(f"etime {time.time() - stime}")
		if write:
			try:
				np.save(os.path.join(config["output_dir_humans"].get(), f"{fn}.npy"), results)
			except:
				import ipdb; ipdb.set_trace()
				np.save(os.path.join(config["output_dir_humans"].get(), f"{fn}.npy"), results)
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
