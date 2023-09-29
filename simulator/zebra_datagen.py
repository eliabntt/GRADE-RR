import argparse
import carb
import confuse
import ipdb
import numpy as np
import os
import sys
import time
import traceback
import yaml
from omni.isaac.kit import SimulationApp
from time import sleep


def boolean_string(s):
	if s.lower() not in {'false', 'true'}:
		raise ValueError('Not a valid boolean string')
	return s.lower() == 'true'


def compute_points(skel_root_path, prim, ef, stage):
	usdSkelRoot = UsdSkel.Root.Get(stage, skel_root_path)
	UsdSkel.BakeSkinning(usdSkelRoot, Gf.Interval(0, ef))

	prim = UsdGeom.PointBased(prim)
	xformCache = UsdGeom.XformCache()
	final_points = np.zeros((ef, len(prim.GetPointsAttr().Get()), 3))

	for prim in Usd.PrimRange(usdSkelRoot.GetPrim()):
		if prim.GetTypeName() != "Mesh":
			continue
		localToWorld = xformCache.GetLocalToWorldTransform(prim)
		for time in range(ef):
			points = UsdGeom.Mesh(prim).GetPointsAttr().Get(time)
			for index in range(len(points)):
				points[index] = localToWorld.Transform(points[index])
			points = np.array(points)
			final_points[time] = points
	return final_points


def randomize_floor_position(floor_data, floor_translation, scale, meters_per_unit, env_name, rng):
	floor_points = np.zeros((len(floor_data), 3))
	if env_name == "Windmills":
		yaw = np.deg2rad(-155)
		rot = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
		floor_translation = np.matmul(floor_translation, rot)

	if env_name == "L_Terrain":
		meters_per_unit = 1

	for i in range(len(floor_data)):
		floor_points[i, 0] = floor_data[i][0] * scale[0] * meters_per_unit + floor_translation[0] * meters_per_unit
		floor_points[i, 1] = floor_data[i][1] * scale[1] * meters_per_unit + floor_translation[1] * meters_per_unit
		floor_points[i, 2] = floor_data[i][2] * scale[2] * meters_per_unit + floor_translation[2] * meters_per_unit

	if env_name == "L_Terrain":
		meters_per_unit = 0.01

	max_floor_x = max(floor_points[:, 0])
	min_floor_x = min(floor_points[:, 0])
	max_floor_y = max(floor_points[:, 1])
	min_floor_y = min(floor_points[:, 1])

	if env_name == "Windmills":
		min_floor_x = -112
		max_floor_x = 161
		min_floor_y = -209
		max_floor_y = 63
		rows = np.where((floor_points[:, 0] > min_floor_x) & (floor_points[:, 0] < max_floor_x) & (floor_points[:, 1] > min_floor_y) & (floor_points[:, 1] < max_floor_y))[0]
		floor_points = floor_points[rows]

	rows = []

	while (len(rows) == 0):
		size_x = rng.integers(40, 120)
		size_y = rng.integers(40, 120)

		# get all floor_points within a size x size square randomly centered
		min_x = rng.uniform(min(floor_points[:, 0]), max(floor_points[:, 0]))
		max_x = min_x + min(size_x, max(floor_points[:, 0]) - min(floor_points[:, 0]))
		while max_x > max(floor_points[:, 0]):
			min_x = rng.uniform(min(floor_points[:, 0]), max(floor_points[:, 0]))
			max_x = min_x + min(size_x, max(floor_points[:, 0]) - min(floor_points[:, 0]))

		min_y = rng.uniform(min(floor_points[:, 1]), max(floor_points[:, 1]))
		max_y = min_y + min(size_y, max(floor_points[:, 1]) - min(floor_points[:, 1]))
		while max_y > max(floor_points[:, 1]):
			min_y = rng.uniform(min(floor_points[:, 1]), max(floor_points[:, 1]))
			max_y = min_y + min(size_y, max(floor_points[:, 1]) - min(floor_points[:, 1]))
		# FIXME this is just an approximation which MAY NOT WORK ALWAYS!
		rows = np.where((min_x <= floor_points[:,0]) & (floor_points[:,0] <= max_x) & (floor_points[:,1]<=max_y) & (floor_points[:,1]>= min_y))[0]
	floor_points = floor_points[rows]

	shape = (len(np.unique(floor_points[:, 0])), -1, 3)
	floor_points = floor_points.reshape(shape)
	if (floor_points[0, 1, 0] - floor_points[0, 0, 0]) > 1:
		zoom_factor = int(floor_points[0, 1, 0] - floor_points[0, 0, 0])
		import scipy.ndimage.interpolation as interpolation

		floor_points = interpolation.zoom(floor_points, (zoom_factor, zoom_factor, 1))

	return floor_points, max_floor_x, min_floor_x, max_floor_y, min_floor_y

try:
	parser = argparse.ArgumentParser(description="Dynamic Worlds Simulator")
	parser.add_argument("--config_file", type=str, default="config.yaml")
	parser.add_argument("--headless", type=boolean_string, default=True, help="Wheter to run it in headless mode or not")
	parser.add_argument("--rtx_mode", type=boolean_string, default=False,
	                    help="Use rtx when True, use path tracing when False")
	parser.add_argument("--record", type=boolean_string, default=False, help="Writing data to the disk")
	parser.add_argument("--debug_vis", type=boolean_string, default=False,
	                    help="When true continuosly loop the rendering")
	parser.add_argument("--neverending", type=boolean_string, default=False, help="Never stop the main loop")
	parser.add_argument("--fix_env", type=str, default="",
	                    help="leave it empty to have a random env, fix it to use a fixed one. Useful for loop processing")

	args, unknown = parser.parse_known_args()
	config = confuse.Configuration("DynamicWorlds", __name__)
	config.set_file(args.config_file)
	config.set_args(args)
	can_start = True

	CONFIG = {"display_options": 3286, "width": 1280, "height": 720, "headless": config["headless"].get()}
	kit = SimulationApp(launch_config=CONFIG, experience=f"{os.environ['EXP_PATH']}/omni.isaac.sim.python.kit")

	# Cannot move before SimApp is launched
	import utils.misc_utils
	from utils.misc_utils import *
	from utils.robot_utils import *
	from utils.simulation_utils import *
	from utils.environment_utils import *
	from pxr import UsdGeom, UsdLux, Gf, Vt, UsdPhysics, PhysxSchema, Usd, UsdShade, Sdf, UsdSkel

	simulation_environment_setup(need_ros=False)

	all_env_names = ["Bliss", "Forest", "Grasslands", "Iceland", "L_Terrain", "Meadow",
	                 "Moorlands", "Nature_1", 'Nature_2', "Savana", "Windmills", "Woodland"]
	ground_area_name = ["Landscape_1", "Landscape_1", "Landscape_1", "Landscape_0", "Terrain_5", "Landscape_0",
	                    "Landscape_2", "Ground", "Ground", "Landscape_1", "Landscape_0", "Landscape_1"]

	need_sky = [True] * len(all_env_names)
	env_id = all_env_names.index(config["fix_env"].get())

	rng = np.random.default_rng()
	rng_state = np.random.get_state()

	local_file_prefix = ""

	# setup environment variables
	environment = environment(config, rng, local_file_prefix)

	out_dir = os.path.join(config['out_folder'].get(), environment.env_name)
	out_dir_npy = os.path.join(config['out_folder_npy'].get(), environment.env_name)
	if not os.path.exists(out_dir):
		os.makedirs(out_dir)

	omni.usd.get_context().open_stage(local_file_prefix + config["base_env_path"].get(), None)

	# Wait two frames so that stage starts loading
	kit.update()
	kit.update()

	print("Loading stage...")
	while is_stage_loading():
		kit.update()
	print("Loading Complete")

	context = omni.usd.get_context()
	stage = context.get_stage()
	set_stage_up_axis("Z")

	omni.kit.commands.execute("DeletePrimsCommand", paths=["/World/GroundPlane"])

	# do this AFTER loading the world
	simulation_context = SimulationContext(physics_dt=1.0 / config["physics_hz"].get(),
	                                       rendering_dt=1.0 / config["render_hz"].get(),
	                                       stage_units_in_meters=0.01)
	simulation_context.initialize_physics()

	simulation_context.play()
	simulation_context.stop()

	kit.update()
	meters_per_unit = 0.01

	# use rtx while setting up!
	set_raytracing_settings(config["physics_hz"].get())
	env_prim_path = environment.load_and_center(config["env_prim_path"].get())
	process_semantics(config["env_prim_path"].get(), "World")

	if all_env_names[env_id] == "L_Terrain":
		set_scale(stage.GetPrimAtPath(f"/World/home"), 100)

	while is_stage_loading():
		kit.update()

	floor_data = stage.GetPrimAtPath(f"/World/home/{ground_area_name[env_id]}/{ground_area_name[env_id]}").GetProperty(
		'points').Get()
	floor_translation = np.array(stage.GetPrimAtPath(f"/World/home/{ground_area_name[env_id]}").GetProperty(
		'xformOp:translate').Get())
	scale = np.array(stage.GetPrimAtPath(f"/World/home/{ground_area_name[env_id]}").GetProperty("xformOp:scale").Get())
	# i need to consider that z has a bounding box and that the position is on the top corner

	for _ in range(50):
		simulation_context.render()

	floor_points, max_floor_x, min_floor_x, max_floor_y, min_floor_y = randomize_floor_position(floor_data,
	                                                                                            floor_translation, scale,
	                                                                                            meters_per_unit, all_env_names[env_id], rng)

	add_semantics(stage.GetPrimAtPath("/World/home"), "world")
	# set timeline of the experiment
	timeline = setup_timeline(config)

	viewport_window_list = []
	dynamic_prims = []

	first = True
	simulation_context.stop()

	simulation_context.play()
	for _ in range(10):
		simulation_context.step()
	_dc = dynamic_control_interface()

	print("Loading robots..")
	robot_base_prim_path = config["robot_base_prim_path"].get()
	usd_robot_path = str(config["usd_robot_path"].get())
	old_h_ap = []
	old_v_ap = []
	simulation_context.stop()

	for n in range(config["num_robots"].get()):
		import_robot(robot_base_prim_path, n, usd_robot_path, local_file_prefix)
		change_prim_collision(False, robot_base_prim_path + str(n))
		set_drone_joints_init_loc(robot_base_prim_path + str(n), [0, 0, 0], [0, 0, 0], 10e15)
		kit.update()


	for n in range(config["num_robots"].get()):
		add_npy_viewport(viewport_window_list, robot_base_prim_path, n, old_h_ap, old_v_ap, config,simulation_context, tot_num_ros_cam=0)
	kit.update()

	for _ in range(5):
		simulation_context.render()

	for index, cam in enumerate(viewport_window_list):
		camera = stage.GetPrimAtPath(cam.get_active_camera())
		camera.GetAttribute("horizontalAperture").Set(old_h_ap[index])
		camera.GetAttribute("verticalAperture").Set(old_v_ap[index])
	print("Loading robot complete")

	print("Loading zebras..")
	zebra_anims_loc = config["zebra_anims_loc"].get()
	# get a list of .usd file in the folder
	import glob

	zebra_files = glob.glob(f"{zebra_anims_loc}/*.usd")

	from utils.zebra_utils import *
	from omni.kit.window.sequencer.scripts import sequencer_drop_controller

	_, sequence = omni.kit.commands.execute("SequencerCreateSequenceCommand")
	sequence_path = sequence.GetPrim().GetPath()
	kit.update()

	zebra_anim_names = ["Attack", "Attack01", "Attack02", "Eating", "Gallop", "Hit_Back", "Hit_Front", "Hit_Left",
	                    "Hit_Right", "Idle", "Idle2", "Idle3", "Idle4", "Jump", "Tarsus", "Trot", "Walkback"]
	zebra_seq_lengths = [27, 54, 32, 133, 12, 15, 17, 20, 15, 48, 72, 119, 201, 43, 29, 24, 27]
	zebra_mesh_paths = [
		"Attack/Zebra_motions/African_Animal___Zebra/_Object_Pivot_Node_/Object_6/Object_45/Zebra_SHP2_0_Zebra_Mat_0_001",
		"Attack01/Zebra_motions/African_Animal___Zebra/_Object_Pivot_Node_/Object_6/Object_45/Zebra_SHP2_0_Zebra_Mat_0_001",
		"Attack02/Zebra_motions/African_Animal___Zebra/_Object_Pivot_Node_/Object_6/Object_45/Zebra_SHP2_0_Zebra_Mat_0_001",
		"Eating/Zebra_motions/African_Animal___Zebra/_Object_Pivot_Node_/Object_6/Object_45/Zebra_SHP2_0_Zebra_Mat_0",
		"Gallop/Zebra_motions/African_Animal___Zebra/_Object_Pivot_Node_/Object_6/Object_45/Zebra_SHP2_0_Zebra_Mat_0",
		"Hit_Back/Zebra_motions/African_Animal___Zebra/_Object_Pivot_Node_/Object_6/Object_45/Zebra_SHP2_0_Zebra_Mat_0",
		"Hit_Front/Zebra_motions/African_Animal___Zebra/_Object_Pivot_Node_/Object_6/Object_45/Zebra_SHP2_0_Zebra_Mat_0",
		"Hit_Left/Zebra_motions/African_Animal___Zebra/_Object_Pivot_Node_/Object_6/Object_45/Zebra_SHP2_0_Zebra_Mat_0",
		"Hit_Right/Zebra_motions/African_Animal___Zebra/_Object_Pivot_Node_/Object_6/Object_45/Zebra_SHP2_0_Zebra_Mat_0",
		"Idle/Zebra_motions/African_Animal___Zebra/_Object_Pivot_Node_/Object_6/Object_45/Zebra_SHP2_0_Zebra_Mat_0",
		"Idle2/Zebra_motions/African_Animal___Zebra/_Object_Pivot_Node_/Object_6/Object_45/Zebra_SHP2_0_Zebra_Mat_0",
		"Idle3/Zebra_motions/African_Animal___Zebra/_Object_Pivot_Node_/Object_6/Object_45/Zebra_SHP2_0_Zebra_Mat_0",
		"Idle4/Zebra_motions/African_Animal___Zebra/_Object_Pivot_Node_/Object_6/Object_45/Zebra_SHP2_0_Zebra_Mat_0",
		"Jump/Zebra_motions/African_Animal___Zebra/_Object_Pivot_Node_/Object_6/Object_45/Zebra_SHP2_0_Zebra_Mat_0",
		"Tarsus/Zebra_motions/African_Animal___Zebra/_Object_Pivot_Node_/Object_6/Object_45/Zebra_SHP2_0_Zebra_Mat_0",
		"Trot/Zebra_motions/African_Animal___Zebra/_Object_Pivot_Node_/Object_6/Object_45/Zebra_SHP2_0_Zebra_Mat_0",
		"Walkback/Zebra_motions/African_Animal___Zebra/_Object_Pivot_Node_/Object_6/Object_45/Zebra_SHP2_0_Zebra_Mat_0"]

	zebra_info = {}
	for i, v in enumerate(zebra_anim_names):
		zebra_info[v] = {"path": zebra_mesh_paths[i], "length": zebra_seq_lengths[i], "mesh_path": zebra_mesh_paths[i]}

	for zebra_file in zebra_files:
		if not os.path.exists(zebra_file[:-4] + "_points.npy"):
			zebra_name = zebra_file.split("/")[-1].split(".")[0]
			zebra_index = zebra_anim_names.index(zebra_name)
			zebra_path = load_zebra("/zebra_", zebra_index, zebra_file)
			kit.update()
			kit.update()

			zebra_name = zebra_file.split("/")[-1].split(".")[0]
			zebra_index = zebra_anim_names.index(zebra_name)
			prim = stage.GetPrimAtPath(zebra_path + zebra_mesh_paths[zebra_index][len(zebra_name):])
			skel_root_path = zebra_path + "/Zebra_motions/African_Animal___Zebra/_Object_Pivot_Node_/Object_6"
			points = compute_points(skel_root_path, prim, zebra_seq_lengths[zebra_index], stage) * meters_per_unit
			np.save(zebra_file[:-4] + "_points.npy", points)
			zebra_info[zebra_name]["points"] = points
			omni.kit.commands.execute("DeletePrimsCommand", paths=[zebra_path])
		else:
			zebra_name = zebra_file.split("/")[-1].split(".")[0]
			zebra_index = zebra_anim_names.index(zebra_name)
			zebra_info[zebra_name]["points"] = np.load(zebra_file[:-4] + "_points.npy")

	max_anim_length = max(zebra_seq_lengths)

	# IT IS OF CRUCIAL IMPORTANCE THAT AFTER THIS POINT THE RENDER GETS DONE WITH THE SLEEPING CALL! OTHERWISE PATH TRACING SPP WILL GET RUINED
	if (config["rtx_mode"].get()):
		set_raytracing_settings(config["physics_hz"].get())
	else:
		set_pathtracing_settings(config["physics_hz"].get())

	omni.usd.get_context().get_selection().set_selected_prim_paths([], False)

	for _ in range(5):
		simulation_context.step(render=False)
		sleeping(simulation_context, viewport_window_list, config["rtx_mode"].get())
	timeline.set_current_time(0)
	simulation_step = 0  # this is NOT the frame, this is the "step" (related to physics_hz)

	my_recorder = recorder_setup(config['_recorder_settings'].get(), out_dir_npy, config['record'].get(), 0)

	timeline.set_current_time(0)  # set to 0 to be sure that the first frame is recorded
	timeline.set_auto_update(False)

	# two times, this will ensure that totalSpp is reached
	sleeping(simulation_context, viewport_window_list, config["rtx_mode"].get())
	sleeping(simulation_context, viewport_window_list, config["rtx_mode"].get())

	my_recorder._enable_record = False

	exp_len = config["anim_exp_len"].get()

	my_recorder._enable_record = False
	sleeping(simulation_context, viewport_window_list, config["rtx_mode"].get())
	if config["rtx_mode"].get():
		my_recorder._update()

	hidden_position = [min_floor_x / meters_per_unit, min_floor_y / meters_per_unit, -10e5]
	all_zebras = preload_all_zebras(config, rng, zebra_files, zebra_info, simulation_context, sequencer_drop_controller,
	                         max_anim_length, hidden_position)
	substep = 3

	simulation_context.play()
	import ipdb; ipdb.set_trace()

	while kit.is_running():
		if simulation_step > 0:
			for zebra in all_zebras:
				set_translate(stage.GetPrimAtPath(zebra), list(hidden_position))

		floor_points, max_floor_x, min_floor_x, max_floor_y, min_floor_y = randomize_floor_position(floor_data,
		                                                                                            floor_translation,
		                                                                                            scale,
		                                                                                            meters_per_unit,
		                                                                                            all_env_names[env_id], rng)
		frame_info = place_zebras(all_zebras, rng, floor_points, meters_per_unit, hidden_position, config, max_anim_length,
		                          zebra_info)
		for c_substep in range(substep):
			average_zebra_x = 0
			average_zebra_y = 0
			average_zebra_z = 0
			max_zebra_x = -1e10
			max_zebra_y = -1e10
			min_zebra_x = 1e10
			min_zebra_y = 1e10

			counter = 0
			for prim in frame_info:
				if "zebra" in prim:
					average_zebra_x += frame_info[prim]["position"][0]
					average_zebra_y += frame_info[prim]["position"][1]
					average_zebra_z += frame_info[prim]["position"][2]
					max_zebra_x = max(max_zebra_x, frame_info[prim]["position"][0])
					max_zebra_y = max(max_zebra_y, frame_info[prim]["position"][1])
					min_zebra_x = min(min_zebra_x, frame_info[prim]["position"][0])
					min_zebra_y = min(min_zebra_y, frame_info[prim]["position"][1])
					counter += 1
			average_zebra_x /= counter
			average_zebra_y /= counter
			average_zebra_z /= counter
			delta_x = max_zebra_x - min_zebra_x
			delta_y = max_zebra_y - min_zebra_y
			used_x = []
			used_y = []
			used_z = []
			for n in range(config["num_robots"].get()):
				safe = False
				while not safe:
					# -100 + 100
					random_x = rng.uniform(average_zebra_x - delta_x/2 - 5, average_zebra_x + delta_x/2 + 5)
					# keep random_x within max_floor_x min_floor_x
					random_x = max(random_x, min_floor_x)
					random_x = min(random_x, max_floor_x)

					random_y = rng.uniform(average_zebra_y - delta_y/2 -5, average_zebra_y + delta_y/2 + 5)
					# keep random_y within max_floor_y min_floor_y
					random_y = max(random_y, min_floor_y)
					random_y = min(random_y, max_floor_y)

					random_z = rng.uniform(average_zebra_z + 5, average_zebra_z + 20)
					if len(used_x) > 0:
						for i in range(len(used_x)):
							safe = True
							if np.sqrt((used_x[i] - random_x) ** 2 + (used_y[i] - random_y) ** 2 + (used_z[i] - random_z) ** 2) < .5:
								safe = False
								break
					else:
						safe = True
					if safe:
						used_x.append(random_x)
						used_y.append(random_y)
						used_z.append(random_z)

				# get angle between robot and average_zebra
				angle = np.arctan2(average_zebra_y - random_y, average_zebra_x - random_x)
				# randomize yaw +- 30 degrees
				yaw = rng.uniform(-np.pi / 6, np.pi / 6) + angle

				# randomize yaw +- 15 degrees
				yaw = rng.uniform(-np.pi / 12, np.pi / 12) + angle

				# get pitch + 15 degrees (camera already pitched)
				# with a weight based on the average zebra location
				pitch = - np.arctan2(average_zebra_z - random_z, np.sqrt(
					(average_zebra_x - random_x) ** 2 + (average_zebra_y - random_y) ** 2))

				# roll minimal -10, 10 degrees
				roll = rng.uniform(-np.pi / 18, np.pi / 18)
				rot = Rotation.from_euler('xyz', [roll, pitch, yaw])
				teleport(robot_base_prim_path + str(n),
				         [random_x / meters_per_unit, random_y / meters_per_unit, random_z / meters_per_unit],
				         rot.as_quat())

				frame_info[f"{robot_base_prim_path}{n}"] = {"position": [random_x, random_y, random_z],
				                                            "rotation": [roll, pitch, yaw]}
			simulation_context.step(render=False)
			simulation_context.step(render=False)

			for _ in range(3):
				simulation_context.step(render=False)
				simulation_context.render()

			sleep(0.5)
			# two frames with the same animation point
			# todo fix the time
			import ipdb;

			ipdb.set_trace()

			timeline.set_current_time(max_anim_length / timeline.get_time_codes_per_seconds())
			if need_sky[env_id]:
				# with probability 0.9 during day hours
				stage.GetPrimAtPath("/World/Looks/SkyMaterial/Shader").GetAttribute("inputs:SunPositionFromTOD").Set(True)
				if rng.uniform() < 0.9:
					stage.GetPrimAtPath("/World/Looks/SkyMaterial/Shader").GetAttribute("inputs:TimeOfDay").Set(
						rng.uniform(5, 20))
				else:
					if rng.uniform() < 0.5:
						stage.GetPrimAtPath("/World/Looks/SkyMaterial/Shader").GetAttribute("inputs:TimeOfDay").Set(
							rng.uniform(0, 5))
					else:
						stage.GetPrimAtPath("/World/Looks/SkyMaterial/Shader").GetAttribute("inputs:TimeOfDay").Set(
							rng.uniform(20, 24))
			print("Publishing cameras...")
			my_recorder._enable_record = True
			frame_info["step"] = simulation_step
			frame_info["substep"] = c_substep
			pub_try_cnt = 0
			success_pub = False
			while not success_pub and pub_try_cnt < 3:
				try:
					pub_and_write_images(simulation_context, viewport_window_list, [],
					                     config["rtx_mode"].get(), my_recorder)
					success_pub = True
				except:
					print("Error publishing camera")
					pub_try_cnt += 1
					import ipdb; ipdb.set_trace()
					# simulation_context.stop()
					# simulation_context.play()
					sleep(0.5)
					simulation_context.render()
					simulation_context.render()
			if not success_pub:
				frame_info["error"] = True
			else:
				frame_info["error"] = False

			np.save(out_dir_npy + f"/frame_{simulation_step}_{c_substep}.npy", frame_info)
			simulation_context.stop()
			# clips = [f"/World/Sequence{k}{k}_Clip" for k in frame_info.keys() if k.startswith("/zebra")]
			# remove targets from clips
			# for clip in clips:
			# 	relationship = stage.GetPrimAtPath(clip).GetProperty("animation")
			# 	relationship.RemoveTarget(relationship.GetTargets()[0])
			# 	relationship = stage.GetPrimAtPath(clip).GetProperty("assetPrim")
			# 	asset = relationship.GetTargets()[0]
			# 	relationship.RemoveTarget(asset)

			# omni.kit.commands.execute("DeletePrimsCommand",
			#                           paths=clips)

			# omni.kit.commands.execute("DeletePrimsCommand",
			#                           paths=
			#                           [f"/World/Sequence{k}" for k in frame_info.keys() if k.startswith("/zebra")])

			# omni.kit.commands.execute("DeletePrimsCommand", paths=[k for k in frame_info.keys() if k.startswith("/zebra")])
			timeline.set_current_time(0)

			my_recorder._counter += 1
		simulation_step += 1
		if simulation_step >= exp_len:
			break
except:
	extype, value, tb = sys.exc_info()
	traceback.print_exc()
	ipdb.post_mortem(tb)
finally:
	simulation_context.stop()
	try:
		kit.close()
	except:
		pass
