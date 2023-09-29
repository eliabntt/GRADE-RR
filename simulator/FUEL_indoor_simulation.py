import argparse
import carb
import confuse
import ipdb
import numpy as np
import os
import roslaunch
import rospy
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


try:
	parser = argparse.ArgumentParser(description="Dynamic Worlds Simulator")
	parser.add_argument("--config_file", type=str, default="config.yaml")
	parser.add_argument("--headless", type=boolean_string, default=True, help="Wheter to run it in headless mode or not")
	parser.add_argument("--rtx_mode", type=boolean_string, default=False,
	                    help="Use rtx when True, use path tracing when False")
	parser.add_argument("--record", type=boolean_string, default=True, help="Writing data to the disk")
	parser.add_argument("--debug_vis", type=boolean_string, default=False,
	                    help="When true continuosly loop the rendering")
	parser.add_argument("--neverending", type=boolean_string, default=False, help="Never stop the main loop")
	parser.add_argument("--fix_env", type=str, default="",
	                    help="leave it empty to have a random env, fix it to use a fixed one. Useful for loop processing")

	args, unknown = parser.parse_known_args()
	config = confuse.Configuration("DynamicWorlds", __name__)
	config.set_file(args.config_file)
	config.set_args(args)
	os.environ["SHAPENET_LOCAL_DIR"] = config["shapenet_local_dir"].get()
	experiment_length = config["experiment_length"].get()
	can_start = True

	CONFIG = {"display_options": 3286, "width": 1280, "height": 720, "headless": config["headless"].get()}
	kit = SimulationApp(launch_config=CONFIG, experience=f"{os.environ['EXP_PATH']}/omni.isaac.sim.python.kit")

	# Cannot move before SimApp is launched
	import utils.misc_utils
	from utils.misc_utils import *
	from utils.robot_utils import *
	from utils.simulation_utils import *
	from utils.objects_utils import *
	from utils.environment_utils import *
	from utils.human_utils import *

	def monitor_movement(msg, args):
		global second_start
		global last_check_time
		global c_pose
		global old_pose
		global rng
		global env_prim_path

		wait_time = rospy.Duration(1)

		index, environment = args[0], args[1]
		if second_start and rospy.Time.now() > last_check_time + wait_time:
			last_check_time = rospy.Time.now()

			diff_x = abs(old_pose[index][0] - c_pose[index][0]) ** 2
			diff_y = abs(old_pose[index][1] - c_pose[index][1]) ** 2
			diff_z = abs(old_pose[index][2] - c_pose[index][2]) ** 2
			dist = (diff_x + diff_y + diff_z) ** 0.5
			if (dist) < 0.1:
				my_pose = PoseStamped()
				if (rng.uniform() > .9):
					x, y, z, yaw = position_object(environment, type=0)
					x = x[0]
					y = y[0]
					z = z[0]
					yaw = yaw[0] + rng.uniform(0, 2 * np.pi)
				else:
					yaw = get_robot_yaw(c_pose[index][0], c_pose[index][1], c_pose[index][2],
					                    environment.env_mesh, environment.shifts)
					x = c_pose[index][0] + 0.2 * np.cos(yaw)
					y = c_pose[index][1] + 0.2 * np.sin(yaw)
					z = c_pose[index][2]
					yaw += rng.uniform(0, 2 * np.pi)

				my_pose.pose.position.x = x
				my_pose.pose.position.y = y
				my_pose.pose.position.z = z
				rot = np.array(yaw) * 180 / np.pi
				quat = (
						Gf.Rotation(Gf.Vec3d.XAxis(), 0)
						* Gf.Rotation(Gf.Vec3d.YAxis(), 0)
						* Gf.Rotation(Gf.Vec3d.ZAxis(), rot)
				).GetQuat()
				my_pose.pose.orientation.x = quat.imaginary[0]
				my_pose.pose.orientation.y = quat.imaginary[1]
				my_pose.pose.orientation.z = quat.imaginary[2]
				my_pose.pose.orientation.w = quat.real
				print(
					f"Publishing random goal since robot {index} stuck [{x},{y},{z}, {yaw} ({yaw * 180 / 3.14})].")
				my_pose.header.frame_id = "world"
				my_pose.header.stamp = rospy.Time.now()
				movement_monitor_pubs[index].publish(my_pose)
				if (dist) < 0.05:
					set_colliders(env_prim_path, True)
			else:
				old_pose[index] = c_pose[index]
				set_colliders(env_prim_path, True)
	def autostart_exploration(msg, index):
		global first_start
		global second_start
		global can_start
		global can_change_second_start
		global last_pub_time

		if (msg.data == "PUB_FIRST_360"):
			can_change_second_start = True

		wait_time = rospy.Duration(0, 500000000) if second_start else rospy.Duration(1)
		if (msg.data == "WAIT_TRIGGER" or (
				msg.data == "PUB_360" and not second_start) and rospy.Time.now() > last_pub_time + wait_time):
			if can_start:
				if not first_start:
					first_start = True
				elif can_change_second_start:
					second_start = True
					print("Exploration will start at the end of this movement")

			default_pose = PoseStamped()
			default_pose.header.frame_id = "world"
			default_pose.header.stamp = rospy.Time.now()
			start_explorer_pubs[index].publish(default_pose)
			last_pub_time = rospy.Time.now()
	def publish_random_goal(msg, args):
		global last_pub_time
		global first_start
		global second_start
		global can_start
		global can_change_second_start

		index, environment = args[0], args[1]
		if (msg.data == "PUB_FIRST_360"):
			can_change_second_start = True

		if (msg.data == "WAIT_TRIGGER" or (
				msg.data == "PUB_360" and not second_start) and rospy.Time.now() > last_pub_time + rospy.Duration(0,
		                                                                                                      500000000)):
			if can_start:
				if not first_start:
					first_start = True
				elif can_change_second_start:
					second_start = True

			my_pose = PoseStamped()
			x, y, z, yaw = position_object(environment, type=0)
			my_pose.pose.position.x = x[0]
			my_pose.pose.position.y = y[0]
			my_pose.pose.position.z = z[0]
			rot = np.array(yaw[0]) * 180 / np.pi
			quat = (
					Gf.Rotation(Gf.Vec3d.XAxis(), 0)
					* Gf.Rotation(Gf.Vec3d.YAxis(), 0)
					* Gf.Rotation(Gf.Vec3d.ZAxis(), rot)
			).GetQuat()
			my_pose.pose.orientation.x = quat.imaginary[0]
			my_pose.pose.orientation.y = quat.imaginary[1]
			my_pose.pose.orientation.z = quat.imaginary[2]
			my_pose.pose.orientation.w = quat.real
			print(f"Publishing random goal [{x[0]},{y[0]},{z[0]}, {yaw[0]} ({yaw[0] * 180 / 3.14})] for robot {index}")
			my_pose.header.frame_id = "fixing_manual"
			my_pose.header.stamp = rospy.Time.now()
			send_waypoint_pubs[index].publish(my_pose)
			last_pub_time = rospy.Time.now()

	simulation_environment_setup()
	# set timeline of the experiment
	timeline = setup_timeline(config)

	rospy.init_node("my_isaac_ros_app", anonymous=True, disable_signals=True, log_level=rospy.ERROR)
	starting_pub = rospy.Publisher('starting_experiment', String)

	rng = np.random.default_rng()
	rng_state = np.random.get_state()

	local_file_prefix = "" # if something is broken try my-computer://

	# setup environment variables
	meters_per_unit = config["meters_per_unit"].get()
	environment = environment(config, rng, local_file_prefix, meters_per_unit)

	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
	out_dir = os.path.join(config['out_folder'].get(), environment.env_name)
	out_dir_npy = os.path.join(config['out_folder_npy'].get(), environment.env_name)
	if not os.path.exists(out_dir):
		os.makedirs(out_dir)

	os.environ["ROS_LOG_DIR"] = out_dir

	roslaunch.configure_logging(uuid)
	launch_files = ros_launchers_setup(roslaunch, environment.env_limits_shifted, config)
	parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files, force_log=True)

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

	# do this AFTER loading the world
	simulation_context = SimulationContext(physics_dt=1.0 / config["physics_hz"].get(),
	                                       rendering_dt=1.0 / config["render_hz"].get(),
	                                       stage_units_in_meters=meters_per_unit, backend='torch')
	simulation_context.initialize_physics()
	physx_interface = omni.physx.acquire_physx_interface()
	physx_interface.start_simulation()

	_clock_graph = add_clock()  # add ROS clock

	simulation_context.play()
	for _ in range(10):
		simulation_context.step()
		og.Controller.evaluate_sync(_clock_graph)
		last_pub_time = rospy.Time.now()
	simulation_context.stop()

	# fixme IDK why this is necessary sometimes
	try:
		parent.start()
	except:
		print("Failed to start roslaunch, retry")
		try:
			parent.start()
		except:
			print("Failed to start roslaunch, exit")
			exit(1)
	print("ros node launched")

	kit.update()

	# use rtx while setting up!
	set_raytracing_settings(config["physics_hz"].get())

	env_prim_path = environment.load_and_center(config["env_prim_path"].get())
	process_semantics(config["env_prim_path"].get())

	randomize_and_fix_lights(config["_random_light"].get(), rng, env_prim_path, environment.env_limits[-1] - 0.2,
	                         meters_per_unit, is_rtx=config["rtx_mode"].get())
	randomize_roughness(config["_random_roughness"].get(), rng, env_prim_path)

	ros_camera_list = []
	ros_transform_components = []  # list of tf and joint components, one (of each) for each robot
	viewport_window_list = []
	dynamic_prims = []
	imus_handle_list = []
	robot_odom_frames = []
	robot_imu_frames = []
	camera_pose_frames = []
	imu_pubs = []
	odom_pubs = []
	cam_pose_pubs = []

	first = True

	simulation_context.play()
	for _ in range(100):
		og.Controller.evaluate_sync(_clock_graph)
		simulation_context.step()
		last_pub_time = rospy.Time.now()
	simulation_context.stop()

	print("Generating map...")
	if add_colliders(env_prim_path):
		simulation_context.play()
		x, y, z, yaw = position_object(environment, type=3)
		environment.generate_map(out_dir, origin=[x[0], y[0], 0])
		for _ in range(10):
			simulation_context.step()
		timeline.set_current_time(0)  # set to 0 to be sure that the first frame is recorded
	else:
		simulation_context.play()
		for _ in range(10):
			simulation_context.step()
		print("Error generating collisions", file=sys.stderr)
	simulation_context.play()
	_dc = dynamic_control_interface()

	print("Loading robots..")

	from omni.isaac.sensor import _sensor
	_is = _sensor.acquire_imu_sensor_interface()

	robot_base_prim_path = config["robot_base_prim_path"].get()
	usd_robot_path = str(config["usd_robot_path"].get())
	c_pose = []
	old_pose = []
	old_h_ap = []
	old_v_ap = []
	lidars = []
	simulation_context.stop()
	for n in range(config["num_robots"].get()):
		import_robot(robot_base_prim_path, n, usd_robot_path, local_file_prefix)
		x, y, z, yaw = get_valid_robot_location(environment, first)

		set_drone_joints_init_loc(f"{robot_base_prim_path}{n}", [x / meters_per_unit, y / meters_per_unit, z / meters_per_unit], [0,0,yaw],
		           (environment.env_limits[5]) / meters_per_unit, 0.3/meters_per_unit, irotate=config["is_iRotate"].get())

		c_pose.append([x, y, z])
		old_pose.append([x, y, z])

		# todo make a comment about this and the number of cameras
		add_ros_components(robot_base_prim_path, n, ros_transform_components, ros_camera_list, viewport_window_list,
		                   camera_pose_frames, cam_pose_pubs, imu_pubs, robot_imu_frames,
		                   robot_odom_frames, odom_pubs, lidars,
		                   dynamic_prims, config, old_h_ap, old_v_ap, _is, simulation_context, _clock_graph)
		kit.update()
		first = False

	for n in range(config["num_robots"].get()):
		add_npy_viewport(viewport_window_list, robot_base_prim_path, n, old_h_ap, old_v_ap, config, simulation_context,
		                 config["num_robots"].get() * 1)

	for _ in range(50):
		simulation_context.render()
	print("Loading robot complete")

	print("WARNING: CAMERA APERTURE MANUAL SET NO LONGER WORKS, NEEDS TO BE FIXED BY NVIDIA!!!!")
	time.sleep(5)

	# # legacy code
	# for index, cam in enumerate(viewport_window_list):
	# 	camera = stage.GetPrimAtPath(cam.get_active_camera())
	# 	camera.GetAttribute("horizontalAperture").Set(old_h_ap[index])
	# 	camera.GetAttribute("verticalAperture").Set(old_v_ap[index])
	print("Starting FSM - setting up topics...")
	start_explorer_pubs = []
	send_waypoint_pubs = []
	movement_monitor_pubs = []
	for index, _ in enumerate(robot_odom_frames):
		print("Waiting for fsm to start for robot {}".format(index))
		my_topic = f"{robot_base_prim_path}{index}/exploration_node/fsm_exploration/state"
		if config["autonomous"].get():
			rospy.Subscriber(my_topic, String, callback=autostart_exploration, callback_args=index)
			start_explorer_pubs.append(
				rospy.Publisher(f"{robot_base_prim_path}{index}/traj_start_trigger", PoseStamped, queue_size=10))
		else:
			rospy.Subscriber(my_topic, String, callback=publish_random_goal, callback_args=(index, environment))
			send_waypoint_pubs.append(
				rospy.Publisher(f"{robot_base_prim_path}{index}/exploration_node/manual_goal", PoseStamped,
				                queue_size=10))
		rospy.Subscriber(my_topic, String, callback=monitor_movement, callback_args=(index, environment))
		movement_monitor_pubs.append(
			rospy.Publisher(f"{robot_base_prim_path}{index}/command/pose", PoseStamped, queue_size=10))
		print("fsm management for robot {} setted up".format(index))
	print("FSM setted up")

	print("Loading humans..")
	my_humans = []
	my_humans_heights = []
	human_export_folder = config["human_path"].get()
	human_folders = os.listdir(human_export_folder)
	tot_area = 0
	areas = []
	initial_dynamics = len(dynamic_prims)
	used_ob_stl_paths = []

	## todo cycle to complete area, need to update the service probably
	n = 0
	human_anim_len = []
	added_prims = []
	human_base_prim_path = config["human_base_prim_path"].get()
	while n < rng.integers(7, 1 + max(7, config["num_humans"].get())):
		anim_len = 0
		# the animation needs to be shorter than config["max_anim_len"].get() and longer than 0/min_len
		while anim_len < max(config["min_human_anim_len"].get(), 0) or anim_len > config["max_human_anim_len"].get():
			folder = rng.choice(human_folders)
			while "old_textures" in folder:
				folder = rng.choice(human_folders)
			random_name = rng.choice(os.listdir(os.path.join(human_export_folder, folder)))
			asset_path = local_file_prefix + os.path.join(human_export_folder, folder, random_name,
			                                              random_name + ".usd")
			tmp_pkl = pkl.load(open(os.path.join(human_export_folder, folder, random_name, random_name + ".pkl"), 'rb'))
			anim_len = tmp_pkl['ef']
		print("Loading human {} from {}".format(random_name, folder))
		used_ob_stl_paths.append(os.path.join(human_export_folder, folder, random_name, random_name + ".stl"))
		human_anim_len.append(tmp_pkl['ef'])
		if "verts" in tmp_pkl.keys():
			my_humans_heights.append(tmp_pkl['verts'][:, :, 2])
		else:
			my_humans_heights.append(None)
		my_humans.append(random_name)
		load_human(human_base_prim_path, n, asset_path, dynamic_prims, added_prims)
		stl_path = os.path.join(human_export_folder, folder, random_name, random_name + ".stl")
		this_mesh = mesh.Mesh.from_file(stl_path)
		areas.append((this_mesh.x.max() - this_mesh.x.min()) * (this_mesh.y.max() - this_mesh.y.min()))
		tot_area += areas[-1]
		n += 1

	x, y, z, yaw = position_object(environment, type=1, objects=my_humans, ob_stl_paths=used_ob_stl_paths, max_collisions=int(config["allow_collision"].get()))
	to_be_removed = []
	human_prim_list = []
	body_origins = []
	for n, human in enumerate(my_humans):
		if z[n] < 0:
			to_be_removed.append(n)
			tot_area -= areas[n]
		else:
			set_translate(stage.GetPrimAtPath(f"{human_base_prim_path}{n}"),
			              [x[n] / meters_per_unit, y[n] / meters_per_unit, z[n] / meters_per_unit])
			set_scale(stage.GetPrimAtPath(f"{human_base_prim_path}{n}"), 1 / meters_per_unit)
			set_rotate(stage.GetPrimAtPath(f"{human_base_prim_path}{n}"), [0, 0, yaw[n]])
			human_prim_list.append(f"{human_base_prim_path}{n}")
			body_origins.append([x[n], y[n], z[n], yaw[n]])
	if len(to_be_removed) > 0:
		print("Removing humans that are out of the environment")
		to_be_removed.reverse()
		cumsum = np.cumsum(added_prims)
		for n in to_be_removed:
			my_humans.pop(n)
			used_ob_stl_paths.pop(n)
			my_humans_heights.pop(n)
			for _ in range(added_prims[n]):
				if n > 0:
					dynamic_prims.pop(cumsum[n - 1] + initial_dynamics)
				else:
					dynamic_prims.pop(initial_dynamics)
			human_anim_len.pop(n)
		omni.kit.commands.execute("DeletePrimsCommand", paths=[f"{human_base_prim_path}{n}" for n in to_be_removed])
	print("Loading human complete")
	google_ob_used, shapenet_ob_used = load_objects(config, environment, rng, dynamic_prims, 1/meters_per_unit)

	# IT IS OF CRUCIAL IMPORTANCE THAT AFTER THIS POINT THE RENDER GETS DONE WITH THE SLEEPING CALL! OTHERWISE PATH TRACING SPP WILL GET RUINED
	if (config["rtx_mode"].get()):
		set_raytracing_settings(config["physics_hz"].get())
	else:
		set_pathtracing_settings(config["physics_hz"].get())

	omni.usd.get_context().get_selection().clear_selected_prim_paths()
	omni.usd.get_context().get_selection().set_selected_prim_paths([], False)

	for _ in range(5):
		simulation_context.step(render=False)
		sleeping(simulation_context, viewport_window_list, raytracing=config["rtx_mode"].get())
	timeline.set_current_time(0)
	simulation_step = 0  # this is NOT the frame, this is the "step" (related to physics_hz)

	my_recorder = recorder_setup(config['_recorder_settings'].get(), out_dir_npy, config['record'].get(), skip_cameras=1)

	simulation_context.stop()
	timeline.set_current_time(0)  # set to 0 to be sure that the first frame is recorded
	timeline.set_auto_update(False)
	for _ in range(5):
		kit.update()
	simulation_context.play()
	timeline.set_auto_update(False)

	first_start = False
	second_start = False
	can_change_second_start = False

	# two times, this will ensure that totalSpp is reached
	sleeping(simulation_context, viewport_window_list, raytracing=config["rtx_mode"].get())
	sleeping(simulation_context, viewport_window_list, raytracing=config["rtx_mode"].get())

	last_pub_time = rospy.Time.now()
	last_check_time = rospy.Time.now()

	if config['debug_vis'].get():
		cnt = 0
		while 1:
			cnt += 1
			if cnt % 10000 == 0:
				import ipdb
			ipdb.set_trace()
			print("Debug vis")
			sleeping(simulation_context, viewport_window_list, raytracing=config["rtx_mode"].get())

	reversing_timeline_ratio = compute_timeline_ratio(human_anim_len, config["reverse_strategy"].get(),
	                                                  experiment_length)

	print(f"The reversing ratio is {reversing_timeline_ratio}.\n"
	      f"This implies that that every {experiment_length / reversing_timeline_ratio} frames we reverse the animations")
	cnt_reversal = 1
	# example
	# exp length: 600, ratio: 4
	# forward 0-150, 151-300 backward, 300-450 forward, 450-600 backward (so 4 slots)
	# exp length: 1200, ratio: 4
	# forward 0-300, 301-600 backward, 601-900 forward, 901-1200 backward (so 4 slots)
	ratio_camera = config["ratio_camera"].get()
	ratio_odom = config["ratio_odom"].get()
	ratio_tf = config["ratio_tf"].get()
	starting_to_pub = False
	my_recorder._enable_record = False
	status = True

	while kit.is_running():
		# NOTE EVERYTHING THAT NEEDS TO BE RENDERED NEEDS TO BE MOVED AFTER THE TIMELINE UPDATE CONSISTENTLY
		if can_start:
			last_check_time = rospy.Time.now()
			if second_start:
				if config['record'].get():
					sleeping(simulation_context, viewport_window_list, raytracing=config["rtx_mode"].get())
					my_recorder._update()
					sleeping(simulation_context, viewport_window_list, raytracing=config["rtx_mode"].get())
				starting_to_pub = True
				timeline.set_current_time(min(- 1 / (config["physics_hz"].get() / ratio_camera),
				                              -abs(config["bootstrap_exploration"].get())))
				simulation_step = int(timeline.get_current_time() * config["physics_hz"].get()) - 1
				# reset_physics(timeline, simulation_context)
				print("Bootstrap started")
				can_start = False
		simulation_step += 1
		if starting_to_pub and simulation_step == 0:
			timeline.set_current_time(0)
			# reset_physics(timeline, simulation_context)
			move_humans_to_ground(my_humans_heights, human_prim_list, simulation_step / ratio_camera, meters_per_unit,
			                      config["max_distance_human_ground"].get())
			print("Starting recording NOW!")
			msg = String("starting")
			starting_pub.publish(msg)
			starting_to_pub = False
			time.sleep(0.5)
			if config['record'].get():
				my_recorder._enable_record = True
				last_check_time = rospy.Time.now()

		if (config["_random_light"].get()["during_experiment"]):
			if (simulation_step % config["_random_light"].get()["n-frames"] == 0):
				# fixme todo smooth change, idea get max-min and time window
				randomize_and_fix_lights(config["_random_light"].get(), rng, env_prim_path, environment.env_limits[-1],
				                         environment.meters_per_unit, is_rtx=config["rtx_mode"].get())

		# step the physics
		simulation_context.step(render=False)

		# get the current time in ROS
		print("Clocking...")
		og.Controller.evaluate_sync(_clock_graph)
		time.sleep(0.1)
		ctime = timeline.get_current_time()
		simulation_context.render()
		timeline.set_current_time(ctime)

		# publish IMU
		print("Publishing IMU...")
		pub_imu(_is, imu_pubs, robot_imu_frames, meters_per_unit)

		# publish joint status (ca 120 Hz)
		if simulation_step % ratio_tf == 0:
			print("Publishing joint/tf status...")
			for component in ros_transform_components:
				og.Controller.set(og.Controller.attribute(f"{component}/OnImpulseEvent.state:enableImpulse"), True)

		# publish odometry (60 hz)
		if simulation_step % ratio_odom == 0:
			print("Publishing odometry...")
			c_pose, _ = pub_odom(robot_odom_frames, odom_pubs, _dc, meters_per_unit)
			pub_cam_pose(camera_pose_frames, cam_pose_pubs, _dc, meters_per_unit)

		# we consider ratio_camera to forward the animation.
		# If you want it different ratio_animation < ratio_camera to avoid
		# two frames with the same animation point
		if second_start:
			if simulation_step % ratio_camera == 0:
				if my_recorder._enable_record:
					# update the image counter externally so that we can use it in the recorder and all images have the same index
					my_recorder._counter += 1
				if simulation_step / ratio_camera < (experiment_length / reversing_timeline_ratio) * (
						cnt_reversal):
					timeline.forward_one_frame()
				else:
					if simulation_step / ratio_camera >= ((experiment_length - 1) / reversing_timeline_ratio) * (
							cnt_reversal + 1) or \
							(timeline.get_current_time() - 1 / timeline.get_time_codes_per_seconds()) < 0:
						cnt_reversal += 2
						timeline.forward_one_frame()
					else:
						timeline.rewind_one_frame()

		if simulation_step % ratio_camera == 0:
			for lidar in lidars:
				og.Controller.attribute(lidar+".inputs:step").set(1)
			ctime = timeline.get_current_time()
			simulation_context.render()
			timeline.set_current_time(ctime)
			for lidar in lidars:
				og.Controller.attribute(lidar+".inputs:step").set(0)

		# publish camera (30 hz)
		if simulation_step % ratio_camera == 0:
			ctime = timeline.get_current_time()
			print("Publishing cameras...")

			# FIRST ONE WRITTEN IS AT 1/30 on the timeline
			pub_and_write_images(simulation_context, viewport_window_list, ros_camera_list, config["rtx_mode"].get(), my_recorder, second_start)
			timeline.set_current_time(ctime)

		if simulation_step % ratio_camera == 0 and simulation_step / ratio_camera == experiment_length \
				and not config["neverending"].get():
			print("End of experiment!!!")
			simulation_context.pause()
			if my_recorder.data_writer is not None:
				my_recorder.data_writer.stop_threads()
			timeline.set_current_time(0)
			context.save_as_stage(os.path.join(out_dir, "loaded_stage.usd"))
			experiment_info = {}
			experiment_info["config"] = config
			experiment_info["reversing_timeline_ratio"] = reversing_timeline_ratio
			experiment_info["humans"] = {}
			experiment_info["humans"]["ids"] = my_humans
			experiment_info["humans"]["folders"] = used_ob_stl_paths
			experiment_info["humans"]["origins"] = body_origins  # x y z yaw
			experiment_info["google_obs"] = google_ob_used
			experiment_info["shapenet_obs"] = shapenet_ob_used
			experiment_info["environment"] = {}
			experiment_info["environment"]["id"] = environment.env_name
			experiment_info["environment"]["folder"] = environment.env_path
			experiment_info["environment"]["shifts"] = environment.shifts
			experiment_info["rng_state"] = rng_state
			np.save(os.path.join(out_dir, "experiment_info.npy"), experiment_info)
			break
except:
	extype, value, tb = sys.exc_info()
	traceback.print_exc()
	ipdb.post_mortem(tb)
finally:
	for pub in odom_pubs:
		pub.unregister()
	for pub in imu_pubs:
		pub.unregister()
	for pub in cam_pose_pubs:
		pub.unregister()
	for pub in start_explorer_pubs:
		pub.unregister()
	for pub in send_waypoint_pubs:
		pub.unregister()
	parent.shutdown()
	rospy.signal_shutdown("my_simulation complete")
	simulation_context.stop()
	try:
		kit.close()
	except:
		pass
