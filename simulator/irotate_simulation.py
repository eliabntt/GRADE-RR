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
	experiment_length = config["experiment_length"].get()
	can_start = True

	CONFIG = {"display_options": 3286, "width": 1280, "height": 720, "headless": config["headless"].get()}
	kit = SimulationApp(launch_config=CONFIG, experience=f"{os.environ['EXP_PATH']}/omni.isaac.sim.python.kit")

	# Cannot move before SimApp is launched
	import utils.misc_utils
	from utils.misc_utils import *
	from utils.robot_utils import *
	from utils.simulation_utils import *
	from utils.environment_utils import *

	simulation_environment_setup()

	rospy.init_node("my_isaac_ros_app", anonymous=True, disable_signals=True, log_level=rospy.ERROR)
	starting_pub = rospy.Publisher('starting_experiment', String)

	rng = np.random.default_rng()
	rng_state = np.random.get_state()

	local_file_prefix = "my-computer://"

	# setup environment variables
	environment = environment(config, rng, local_file_prefix)

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
	                                       stage_units_in_meters=0.01)
	simulation_context.start_simulation()

	add_clock()  # add ROS clock

	simulation_context.play()
	for _ in range(100):
		omni.kit.commands.execute("RosBridgeTickComponent", path="/ROS_Clock")
		simulation_context.step()
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
	meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)

	# use rtx while setting up!
	set_raytracing_settings(config["physics_hz"].get())
	env_prim_path = environment.load_and_center(config["env_prim_path"].get())
	process_semantics(config["env_prim_path"].get())

	randomize_and_fix_lights(config["_random_light"].get(), rng, env_prim_path, environment.env_limits[-1] - 0.2,
	                         environment.meters_per_unit, is_rtx=config["rtx_mode"].get())
	randomize_roughness(config["_random_roughness"].get(), rng, env_prim_path)

	# set timeline of the experiment
	timeline = setup_timeline(config)

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
	camera_odom_pubs = []
	camera_odom_frames = []
	lidar_components = []
	first = True

	imu_sensor, imu_props = setup_imu_sensor(config)

	simulation_context.play()
	for _ in range(100):
		omni.kit.commands.execute("RosBridgeTickComponent", path="/ROS_Clock")
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
	robot_base_prim_path = config["robot_base_prim_path"].get()
	usd_robot_path = str(config["usd_robot_path"].get())
	c_pose = []
	old_pose = []
	old_h_ap = []
	old_v_ap = []
	for n in range(config["num_robots"].get()):
		simulation_context.stop()
		import_robot(robot_base_prim_path, n, usd_robot_path, local_file_prefix)
		x, y, z, yaw = 0, 0, 0, 0
		simulation_context.stop()
		set_drone_joints_init_loc(f"{robot_base_prim_path}{n}", [x / meters_per_unit, y / meters_per_unit, z / meters_per_unit],
		           [0, 0, yaw],
		           (environment.env_limits[5]) / meters_per_unit, irotate=config["is_iRotate"].get())
		c_pose.append([x, y, z])
		old_pose.append([x, y, z])
		kit.update()
		simulation_context.play()
		kit.update()
		add_ros_components(robot_base_prim_path, n, ros_transform_components, ros_camera_list, viewport_window_list,
		                   camera_pose_frames, cam_pose_pubs, imus_handle_list, imu_pubs, robot_imu_frames,
		                   robot_odom_frames, odom_pubs,
		                   dynamic_prims, config, imu_sensor, imu_props, old_h_ap, old_v_ap, config["is_iRotate"].get())

		add_irotate_ros_components(camera_odom_frames, camera_odom_pubs, lidar_components, robot_base_prim_path, n)
		kit.update()
		first = False

	for n in range(config["num_robots"].get()):
		add_npy_viewport(viewport_window_list, robot_base_prim_path, n, old_h_ap, old_v_ap, config,
		                 config["num_robots"].get() * 1)
	kit.update()

	for _ in range(50):
		simulation_context.render()

	print("Loading robot complete")
	for index, cam in enumerate(viewport_window_list):
		camera = stage.GetPrimAtPath(cam.get_active_camera())
		camera.GetAttribute("horizontalAperture").Set(old_h_ap[index])
		camera.GetAttribute("verticalAperture").Set(old_v_ap[index])
	# setup manual ticks for all components (just to be sure)
	# IMU not necessary as it is NOT a ROS component itself
	for component in ros_camera_list:
		omni.kit.commands.execute("RosBridgeTickComponent", path=str(component.GetPath()))
	for component in ros_transform_components:
		omni.kit.commands.execute("RosBridgeTickComponent", path=str(component.GetPath()))

	#    IT IS OF CRUCIAL IMPORTANCE THAT AFTER THIS POINT THE RENDER GETS DONE WITH THE SLEEPING CALL! OTHERWISE PATH TRACING SPP WILL GET RUINED
	if (config["rtx_mode"].get()):
		set_raytracing_settings(config["physics_hz"].get())
	else:
		set_pathtracing_settings(config["physics_hz"].get())

	omni.usd.get_context().get_selection().set_selected_prim_paths([], False)

	simulation_context.stop()
	simulation_context.play()
	for _ in range(5):
		simulation_context.step(render=False)
		sleeping(simulation_context, viewport_window_list, raytracing=config["rtx_mode"].get())
	timeline.set_current_time(0)
	simulation_step = 0  # this is NOT the frame, this is the "step" (related to physics_hz)

	my_recorder = recorder_setup(config['_recorder_settings'].get(), out_dir_npy, config['record'].get())

	timeline.set_current_time(0)  # set to 0 to be sure that the first frame is recorded
	timeline.set_auto_update(False)

	omni.kit.commands.execute("RosBridgeUseSimTime", use_sim_time=True)
	omni.kit.commands.execute("RosBridgeUsePhysicsStepSimTime", use_physics_step_sim_time=True)

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
			print("DEBUGGING VIS")
			sleeping(simulation_context, viewport_window_list, raytracing=config["rtx_mode"].get())
			for i, cam in enumerate(ros_camera_list):
				omni.kit.commands.execute("RosBridgeTickComponent", path=str(cam.GetPath()))

	reversing_timeline_ratio = 1

	print(
		f"The reversing ratio is {reversing_timeline_ratio}.\n"
		f"This implies that that every {experiment_length / reversing_timeline_ratio} frames we reverse the animations")
	cnt_reversal = 1

	ratio_camera = config["ratio_camera"].get()
	ratio_odom = config["ratio_odom"].get()
	ratio_tf = config["ratio_tf"].get()
	starting_to_pub = False
	my_recorder._enable_record = False
	second_start = False
	while kit.is_running():
		if can_start:
			last_check_time = rospy.Time.now()
			if config['record'].get():
				sleeping(simulation_context, viewport_window_list, raytracing=config["rtx_mode"].get())
				my_recorder._update()
				sleeping(simulation_context, viewport_window_list, raytracing=config["rtx_mode"].get())
			starting_to_pub = True
			timeline.set_current_time(min(- 1 / (config["physics_hz"].get() / ratio_camera),
			                              -abs(config["bootstrap_exploration"].get())))
			simulation_step = int(timeline.get_current_time() * config["physics_hz"].get()) - 1
			print("Bootstrap started")
			can_start = False
			second_start = True
		simulation_step += 1
		if starting_to_pub and simulation_step == 0:
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
		omni.kit.commands.execute("RosBridgeTickComponent", path="/ROS_Clock")
		time.sleep(0.2)

		# publish IMU
		print("Publishing IMU...")
		pub_imu(imus_handle_list, imu_sensor, imu_pubs, robot_imu_frames, meters_per_unit)

		# publish joint status (ca 120 Hz)
		if simulation_step % ratio_tf == 0:
			print("Publishing joint/tf status...")
			for component in ros_transform_components:
				omni.kit.commands.execute("RosBridgeTickComponent", path=str(component.GetPath()))

		# publish odometry (60 hz)
		if simulation_step % ratio_odom == 0:
			print("Publishing odometry...")
			pub_cam_pose(camera_pose_frames, cam_pose_pubs, _dc, meters_per_unit)
			c_pose, _ = pub_odom(camera_odom_frames, camera_odom_pubs, _dc, meters_per_unit, robot_odom_frames)
			c_pose, _ = pub_odom(robot_odom_frames, odom_pubs, _dc, meters_per_unit)
			for component in lidar_components:
				omni.kit.commands.execute("RosBridgeTickComponent", path=str(component.GetPath()))
		# we consider ratio_camera to forward the animation.
		# If you want it different ratio_animation < ratio_camera to avoid
		# two frames with the same animation point
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

		# publish camera (30 hz)
		if simulation_step % ratio_camera == 0:
			print("Publishing cameras...")
			# getting skel pose for each joint
			# get_skeleton_info(meters_per_unit, body_origins, body_list)
			# FIRST ONE WRITTEN IS AT 1/30 on the timeline
			pub_and_write_images(simulation_context, viewport_window_list,
			                     ros_camera_list, config["rtx_mode"].get(),
								 my_recorder, second_start)

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
# ipdb.post_mortem(tb)
finally:
	for pub in odom_pubs:
		pub.unregister()
	for pub in imu_pubs:
		pub.unregister()
	for pub in cam_pose_pubs:
		pub.unregister()
	parent.shutdown()
	rospy.signal_shutdown("my_simulation complete")
	simulation_context.stop()
	try:
		kit.close()
	except:
		pass
