# 1. Start Isaac Sim first
import os as _os_bootstrap
from datetime import datetime as _dt_bootstrap
try:
	with open("/tmp/grade_rr_zebra_bootstrap.log", "a", encoding="utf-8") as _f_bootstrap:
		_f_bootstrap.write(f"[{_dt_bootstrap.now().isoformat()}] zebra_datagen.py bootstrap reached\n")
except Exception:
	pass

import isaacsim
from isaacsim import SimulationApp
# conda activate env_isaaclab && /home/jschwenkbeck/miniforge3/envs/env_isaaclab/bin/python /home/jschwenkbeck/Documents/GRADE/GRADE-RR/simulator/zebra_datagen.py --config_file /home/jschwenkbeck/Documents/GRADE/GRADE-RR/simulator/configs/config_zebra_datagen.yaml --headless False
import argparse
import confuse
import numpy as np
import os
import sys
from datetime import datetime

# Ensure 'simulator' directory is in sys.path for robust imports
simulator_dir = os.path.dirname(os.path.abspath(__file__))
if simulator_dir not in sys.path:
	sys.path.insert(0, simulator_dir)
import time
import traceback
import yaml
from scipy.spatial.transform import Rotation
from time import sleep


def _heartbeat(msg):
	timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
	line = f"[{timestamp}] zebra_datagen: {msg}"
	print(line, flush=True)
	try:
		carb.log_warn(line)
	except Exception:
		pass
	try:
		with open("/tmp/grade_rr_zebra_datagen.log", "a", encoding="utf-8") as f:
			f.write(line + "\n")
	except Exception:
		pass


def _patch_property_window_for_headless():
	"""Guard known property-window callback issue in headless Isaac Sim 5.1."""
	try:
		import omni.kit.window.property.window as _prop_window_mod
		_orig = _prop_window_mod.PropertyWindow.save_scroll_pos

		def _safe_save_scroll_pos(self, *args, **kwargs):
			frame = getattr(self, "properties_frame", None)
			if frame is None:
				return
			if getattr(frame, "scroll_y_max", None) is None or getattr(frame, "scroll_y", None) is None:
				return
			try:
				return _orig(self, *args, **kwargs)
			except AttributeError as e:
				if "scroll_y_max" in str(e):
					return
				raise

		_prop_window_mod.PropertyWindow.save_scroll_pos = _safe_save_scroll_pos
	except Exception:
		pass


_heartbeat("module import completed")

# /home/jschwenkbeck/miniforge3/envs/env_isaaclab/bin/python /home/jschwenkbeck/Documents/GRADE/GRADE-RR/simulator/zebra_datagen.py --config_file /home/jschwenkbeck/Documents/GRADE/GRADE-RR/simulator/configs/config_zebra_datagen.yaml --headless False --fix_env env_base_flat

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
	_heartbeat("entered main try block")


	parser = argparse.ArgumentParser(description="Dynamic Worlds Simulator")
	parser.add_argument("--config_file", type=str, default="config.yaml")
	parser.add_argument("--headless", type=boolean_string, default=True, help="Wheter to run it in headless mode or not")
	parser.add_argument("--rtx_mode", type=boolean_string, default=True,
						help="Use rtx when True, use path tracing when False")
	parser.add_argument("--record", type=boolean_string, default=False, help="Writing data to the disk")
	parser.add_argument("--debug_vis", type=boolean_string, default=False,
						help="When true continuosly loop the rendering")
	parser.add_argument("--neverending", type=boolean_string, default=False, help="Never stop the main loop")
	parser.add_argument("--mode", type=str, default="datagen", choices=["datagen", "roam"],
						help="datagen runs the full dataset pipeline; roam keeps the scene interactive and lightweight")
	parser.add_argument("--fix_env", type=str, default="",
						help="leave it empty to have a random env, fix it to use a fixed one. Useful for loop processing")

	args, unknown = parser.parse_known_args()
	_heartbeat(f"args parsed config_file={args.config_file} headless={args.headless} fix_env={args.fix_env}")
	cli_argv = set(sys.argv[1:])

	# Print the contents of the config file before loading with confuse
	import yaml as _yaml
	try:
		with open(args.config_file, 'r') as f:
			config_contents = f.read()
		print("[DEBUG] Raw config.yaml contents:\n" + config_contents)
		config_yaml = _yaml.safe_load(config_contents)
		print("[DEBUG] Parsed YAML keys:", list(config_yaml.keys()) if config_yaml else "EMPTY OR INVALID")
		# If the config only has an 'include' key, resolve it and use that file instead
		if config_yaml and list(config_yaml.keys()) == ["include"]:
			include_path = config_yaml["include"]
			print(f"[DEBUG] Resolving include: {include_path}")
			# If the include path is relative, resolve relative to the current config file
			import os
			if not os.path.isabs(include_path):
				include_path = os.path.join(os.path.dirname(args.config_file), include_path)
			print(f"[DEBUG] Using included config file: {include_path}")
			args.config_file = include_path
			with open(args.config_file, 'r') as f:
				config_contents = f.read()
			print("[DEBUG] Included config.yaml contents:\n" + config_contents)
			config_yaml = _yaml.safe_load(config_contents)
			print("[DEBUG] Included YAML keys:", list(config_yaml.keys()) if config_yaml else "EMPTY OR INVALID")
	except Exception as e:
		print(f"[ERROR] Could not read or parse config file: {e}")

	config = confuse.Configuration("DynamicWorlds", __name__)
	config.set_file(args.config_file)
	# Avoid overriding file values with parser defaults when a flag was not explicitly passed.
	# In particular, --fix_env default "" would mask config-file fix_env.
	if "--fix_env" not in cli_argv:
		try:
			delattr(args, "fix_env")
		except Exception:
			pass
	config.set_args(args)
	can_start = True
	interactive_preview_mode = args.mode == "roam"
	preview_settings_rate = 60 if interactive_preview_mode else config["physics_hz"].get()
	preview_stage_renders = 5 if interactive_preview_mode else 50
	preview_robot_renders = 1 if interactive_preview_mode else 5
	preview_substep = 1 if interactive_preview_mode else 3
	preview_inner_renders = 1 if interactive_preview_mode else 3
	preview_sleep_s = 0.0 if interactive_preview_mode else 0.5

	CONFIG = {"display_options": 3286, "width": 1280, "height": 720, "headless": config["headless"].get()}
	simulation_app = SimulationApp(launch_config=CONFIG)
	kit = simulation_app
	_heartbeat("SimulationApp launched from config")
	if interactive_preview_mode:
		_heartbeat("roam mode enabled: reduced render load and no data capture")

	import carb
	import omni
	import omni.client
	if config["headless"].get():
		_patch_property_window_for_headless()
	cloud_path = "http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1"
	omni.client.set_alias("omniverse://localhost", cloud_path)
	omni.client.set_alias("omniverse://localhost/NVIDIA", f"{cloud_path}/NVIDIA")
	if hasattr(omni.client, "mount"):
		try:
			omni.client.mount(
				"omniverse://localhost",
				cloud_path,
			)
			omni.client.mount(
				"omniverse://localhost/NVIDIA",
				f"{cloud_path}/NVIDIA",
			)
		except Exception as e:
			carb.log_warn(f"omni.client.mount unavailable/failed, relying on set_alias only: {e}")

	settings = carb.settings.get_settings()
	mdl_paths = settings.get("/rtx/materialDb/mdlSearchPaths") or []
	if cloud_path not in mdl_paths:
		mdl_paths.append(cloud_path)
	if f"{cloud_path}/NVIDIA/Materials" not in mdl_paths:
		mdl_paths.append(f"{cloud_path}/NVIDIA/Materials")
	if f"{cloud_path}/NVIDIA/Assets/Skies" not in mdl_paths:
		mdl_paths.append(f"{cloud_path}/NVIDIA/Assets/Skies")
	settings.set("/rtx/materialDb/mdlSearchPaths", mdl_paths)

	# Cannot move before SimApp is launched
	import grade_utils.misc_utils
	from grade_utils.misc_utils import *
	from grade_utils.robot_utils import *
	from grade_utils.simulation_utils import *
	from grade_utils.environment_utils import *
	from pxr import UsdGeom, UsdLux, Gf, Vt, UsdPhysics, PhysxSchema, Usd, UsdShade, Sdf, UsdSkel

	def _focus_editor_camera_on_target(target_pos_m):
		"""Best-effort: move default editor camera to look at a target in meters."""
		camera_candidates = ["/OmniverseKit_Persp", "/World/Camera", "/OmniverseKit_Top"]
		cam_prim = None
		for p in camera_candidates:
			prim = stage.GetPrimAtPath(p)
			if prim and prim.IsValid():
				cam_prim = prim
				break
		if cam_prim is None:
			return

		tx = float(target_pos_m[0]) / meters_per_unit
		ty = float(target_pos_m[1]) / meters_per_unit
		tz = float(target_pos_m[2]) / meters_per_unit
		cam_pos = np.array([tx - (8.0 / meters_per_unit), ty - (8.0 / meters_per_unit), tz + (4.0 / meters_per_unit)])

		delta_x = tx - cam_pos[0]
		delta_y = ty - cam_pos[1]
		delta_z = tz - cam_pos[2]
		yaw = np.arctan2(delta_y, delta_x)
		pitch = -np.arctan2(delta_z, max(1e-6, np.sqrt(delta_x * delta_x + delta_y * delta_y)))

		set_translate(cam_prim, [float(cam_pos[0]), float(cam_pos[1]), float(cam_pos[2])])
		set_rotate(cam_prim, [0.0, float(pitch), float(yaw)])

	# Defer extension setup until after stage load to avoid loader-time instability.
	# (Some optional extensions can interfere with stage reopen/load in Isaac Sim 5.1.)
	deferred_extension_setup = (False, True, False)

	all_env_names = ["Bliss", "Forest", "Grasslands", "Iceland", "L_Terrain", "Meadow",
	                 "Moorlands", "Nature_1", 'Nature_2', "Savana", "Windmills", "Woodland"]
	ground_area_name = ["Landscape_1", "Landscape_1", "Landscape_1", "Landscape_0", "Terrain_5", "Landscape_0",
	                    "Landscape_2", "Ground", "Ground", "Landscape_1", "Landscape_0", "Landscape_1"]

	need_sky = [True] * len(all_env_names)
	fix_env_value = config["fix_env"].get()
	if fix_env_value and fix_env_value in all_env_names:
		env_id = all_env_names.index(fix_env_value)
	else:
		# fallback: pick a random environment if not specified or invalid
		import random
		env_id = random.randint(0, len(all_env_names) - 1)
		print(f"[INFO] fix_env not set or invalid, using random environment: {all_env_names[env_id]}")

	rng = np.random.default_rng()
	rng_state = np.random.get_state()

	local_file_prefix = ""

	# setup environment variables
	import os
	abs_config_path = os.path.abspath(args.config_file)
	print(f"[DEBUG] Config file used: {args.config_file}")
	print(f"[DEBUG] Absolute config path: {abs_config_path}")
	print(f"[DEBUG] Config file exists: {os.path.exists(abs_config_path)}")
	print(f"[DEBUG] Config keys: {list(config.keys())}")
	environment = environment(config, rng, local_file_prefix)
	base_env_name = os.path.splitext(os.path.basename(config["base_env_path"].get()))[0]
	if base_env_name in all_env_names:
		env_id = all_env_names.index(base_env_name)
		print(f"[INFO] Using base_env_path environment '{base_env_name}' (env_id={env_id})")
	elif environment.env_name in all_env_names:
		env_id = all_env_names.index(environment.env_name)
		print(f"[INFO] Using loaded environment '{environment.env_name}' (env_id={env_id})")
	else:
		print(
			f"[WARN] Loaded environment '{environment.env_name}' not in known list; "
			f"keeping env_id={env_id} ({all_env_names[env_id]})"
		)
	requested_env = config["fix_env"].get()
	if requested_env and environment.env_name != requested_env:
		env_root = os.path.abspath(config["env_path"].get())
		available_envs = []
		if os.path.isdir(env_root):
			available_envs = sorted([os.path.splitext(f)[0] for f in os.listdir(env_root) if f.endswith(".usd")])
		raise FileNotFoundError(
			f"Requested --fix_env={requested_env} was not found in env_path={env_root}. "
			f"Loaded '{environment.env_name}' instead. Available .usd environments: {available_envs}"
		)

	out_dir = os.path.join(config['out_folder'].get(), environment.env_name)
	out_dir_npy = os.path.join(config['out_folder_npy'].get(), environment.env_name)
	if not os.path.exists(out_dir):
		os.makedirs(out_dir)

	omni.usd.get_context().open_stage(local_file_prefix + config["base_env_path"].get(), None)
	_heartbeat(f"open_stage requested path={local_file_prefix + config['base_env_path'].get()}")

	# Wait multiple frames so that stage starts loading  
	if 'kit' in globals():
		for i in range(10):
			_heartbeat(f"update frame {i+1}/10")
			kit.update()
	elif 'simulation_app' in globals():
		for i in range(10):
			_heartbeat(f"update frame {i+1}/10")
			simulation_app.update()
	else:
		print("[ERROR] Neither 'kit' nor 'simulation_app' is defined. Cannot update simulation.")

	print("Loading stage...")
	_heartbeat("starting stage load wait loop")
	load_timeout_secs = 120
	start_time = time.time()
	while is_stage_loading() and (time.time() - start_time) < load_timeout_secs:
		time.sleep(0.1)
		try:
			kit.update()
		except Exception as e:
			_heartbeat(f"Exception during update: {e}")
			break
	_heartbeat(f"Loading Complete (elapsed: {time.time() - start_time:.1f}s)")

	context = omni.usd.get_context()
	stage = context.get_stage()
	set_stage_up_axis("Z")

	# Enable optional extensions only after stage is loaded.
	need_ros_ext, need_seq_ext, need_shapenet_ext = deferred_extension_setup
	simulation_environment_setup(
		need_ros=need_ros_ext,
		need_sequencer=need_seq_ext,
		need_shapenet=need_shapenet_ext,
	)

	if stage.GetPrimAtPath("/World/GroundPlane").IsValid():
		omni.kit.commands.execute("DeletePrimsCommand", paths=["/World/GroundPlane"])

	# do this AFTER loading the world
	simulation_context = SimulationContext(physics_dt=1.0 / config["physics_hz"].get(),
	                                       rendering_dt=1.0 / config["render_hz"].get(),
	                                       stage_units_in_meters=0.01)
	simulation_context.initialize_physics()

	simulation_context.play()
	try:
		simulation_context.stop()
	except NameError:
		print("[ERROR] simulation_context is not defined at shutdown.")

	kit.update()
	meters_per_unit = 0.01

	# use rtx while setting up!
	set_raytracing_settings(preview_settings_rate)
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

	for _ in range(preview_stage_renders):
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

	for _ in range(preview_robot_renders):
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
	if len(zebra_files) == 0:
		raise FileNotFoundError(
			f"No zebra animation USD files found in zebra_anims_loc={zebra_anims_loc}. "
			"Expected files like Attack.usd, Gallop.usd, Idle.usd, ..."
		)

	from grade_utils.zebra_utils import *
	sequence_path = ""
	if config["headless"].get() or interactive_preview_mode:
		sequencer_drop_controller = None
		if interactive_preview_mode:
			_heartbeat("roam mode: skipping sequencer clip authoring for zebras")
		else:
			_heartbeat("headless mode: skipping sequencer clip authoring for zebras")
	else:
		try:
			from omni.kit.window.sequencer.scripts import sequencer_drop_controller
			_heartbeat("using window sequencer drop controller")
		except Exception as _sequencer_import_error:
			sequencer_drop_controller = None
			_heartbeat(f"window sequencer unavailable, using fallback sequencer commands: {_sequencer_import_error}")

		seq_ok, sequence = omni.kit.commands.execute("SequencerCreateSequenceCommand")
		if (not seq_ok) or sequence is None:
			raise RuntimeError(
				"Failed to create Sequencer sequence. Ensure sequencer extensions are available "
				"(omni.kit.sequencer.core and omni.kit.sequencer.usd)."
			)
		sequence_path = sequence.GetPrim().GetPath().pathString
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
		set_raytracing_settings(preview_settings_rate)
	else:
		set_pathtracing_settings(preview_settings_rate)

	if not config["headless"].get():
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
	                         max_anim_length, hidden_position, sequence_path)
	substep = preview_substep

	simulation_context.play()

	while kit.is_running():
		if interactive_preview_mode:
			if simulation_step == 0:
				frame_info_preview = place_zebras(
					all_zebras,
					rng,
					floor_points,
					meters_per_unit,
					hidden_position,
					config,
					max_anim_length,
					zebra_info,
				)
				zebra_keys = [k for k in frame_info_preview.keys() if k.startswith("/zebra")]
				if len(zebra_keys) > 0:
					try:
						_focus_editor_camera_on_target(frame_info_preview[zebra_keys[0]]["position"])
						_heartbeat(f"preview camera focused on {zebra_keys[0]}")
					except Exception as e:
						_heartbeat(f"preview camera focus failed: {e}")
				_heartbeat("interactive roam mode active: use viewport navigation freely")
				simulation_step = 1
			simulation_context.step(render=False)
			simulation_context.render()
			continue

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

			for _ in range(preview_inner_renders):
				simulation_context.step(render=False)
				simulation_context.render()

			if preview_sleep_s > 0:
				sleep(preview_sleep_s)
			# two frames with the same animation point
			# todo fix the time

			timeline.set_current_time(max_anim_length / timeline.get_time_codes_per_seconds())
			if need_sky[env_id]:
				sky_shader = stage.GetPrimAtPath("/World/Looks/SkyMaterial/Shader")
				if sky_shader and sky_shader.IsValid():
					# with probability 0.9 during day hours
					sun_attr = sky_shader.GetAttribute("inputs:SunPositionFromTOD")
					if sun_attr:
						sun_attr.Set(True)
					tod_attr = sky_shader.GetAttribute("inputs:TimeOfDay")
					if tod_attr:
						if rng.uniform() < 0.9:
							tod_attr.Set(rng.uniform(5, 20))
						else:
							if rng.uniform() < 0.5:
								tod_attr.Set(rng.uniform(0, 5))
							else:
								tod_attr.Set(rng.uniform(20, 24))
			if config["record"].get():
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
			else:
				frame_info["error"] = False
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
	traceback.print_exc()
	raise
finally:
	if 'simulation_context' in globals() or 'simulation_context' in locals():
		try:
			simulation_context.stop()
		except Exception:
			pass
	try:
		kit.close()
	except:
		pass
