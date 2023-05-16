import argparse
import carb
import confuse
import cv2
import ipdb
import math
import numpy as np
import os
import rosbag
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
Suppose you want a stereo camera
And to have optical flow
And LiDAR (not fully supported yet) of the experiments.
This is a way in which you can re-process your info and get the results.

Suggestion: teleport is much more precise (sub mm difference). Working with velocities is fisy

This code is a bit hard-coded as it is a demonstration code.
"""
try:
	parser = argparse.ArgumentParser(description="Get Bounding Boxes")
	parser.add_argument("--experiment_folder", type=str,
	                    help="The experiment folder with the USD file and the info file")
	parser.add_argument("--headless", type=boolean_string, default=False, help="Whether run this headless or not")
	parser.add_argument("--write", type=boolean_string, default=False, help="Whether to write new cameras results")
	parser.add_argument("--write_flow", type=boolean_string, default=True, help="Whether to write optical flow")
	parser.add_argument("--write_normals", type=boolean_string, default=True, help="Whether to write normals")
	parser.add_argument("--use_teleport", type=boolean_string, default=True,
	                    help="Whether to use teleport or force joint vel, both have adv and disadv")
	parser.add_argument("--use_reindex", type=boolean_string, default=False, help="Whether to use reindexed bags")
	parser.add_argument("--bag_basename", type=str, default="7659a6c9-9fc7-4be5-bc93-5b202ff2a22b")
	parser.add_argument("--out_folder_npy", type=str, default='additional_data')
	parser.add_argument("--bag_subpath", type=str, default="")

	args, unknown = parser.parse_known_args()
	config = confuse.Configuration("NewSensor", __name__)
	config.set_args(args)

	exp_info = np.load(os.path.join(config["experiment_folder"].get(), "experiment_info.npy"), allow_pickle=True)
	exp_info = exp_info.item()

	poses_path = os.path.join(config["experiment_folder"].get(), "Viewport0", "camera")
	write_flow = config["write_flow"].get()
	write_normals = config["write_normals"].get()
	write = config["write"].get()

	CONFIG = {"display_options": 3286, "width": 1280, "height": 720, "headless": config["headless"].get()}
	kit = SimulationApp(launch_config=CONFIG, experience=f"{os.environ['EXP_PATH']}/omni.isaac.sim.python.kit")

	import utils.misc_utils
	from utils.misc_utils import *
	from utils.robot_utils import *
	from utils.simulation_utils import *
	from utils.objects_utils import *
	from utils.environment_utils import *
	from utils.human_utils import *

	environment_setup()
	rospy.init_node("new_sensor_publisher", anonymous=True, disable_signals=True, log_level=rospy.ERROR)
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

	simulation_context = SimulationContext(physics_dt=1.0 / exp_info["config"]["physics_hz"].get(),
	                                       rendering_dt=1.0 / exp_info["config"]["render_hz"].get(),
	                                       stage_units_in_meters=0.01)
	simulation_context.start_simulation()
	meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)
	set_raytracing_settings(exp_info["config"]["physics_hz"].get())
	timeline = setup_timeline(exp_info["config"])

	reversing_timeline_ratio = exp_info['reversing_timeline_ratio']
	experiment_length = exp_info['config']['experiment_length'].get()
	ratio_camera = exp_info['config']['ratio_camera'].get()
	cnt_reversal = 1
	simulation_context.stop()

	### here we add the new camera to the robot. It will be located 5 cm to the right w.r.t. the original one
	old_h_ape = []
	old_v_ape = []
	viewport_window_list = []
	ros_camera_list = []
	# omni.kit.commands.execute('CopyPrim',
	#                           path_from='/my_robot_0/camera_link/Camera',
	#                           path_to='/my_robot_0/camera_link/Camera_stereo',
	#                           exclusive_select=False)
	#
	# set_translate(stage.GetPrimAtPath('/my_robot_0/camera_link/Camera_stereo'), [0.5, 0, 0])
	# component, viewport = add_camera_and_viewport("/my_robot_0/camera_link",
	#                                               exp_info["config"]["robot_sensor_size"].get(), old_h_ape, old_v_ape,
	#                                               config["headless"].get(), 0, 0, "Camera_stereo")
	# ros_camera_list.append(component)
	# viewport_window_list.append(viewport)
	#
	# omni.kit.commands.execute('CopyPrim',
	#                           path_from='/my_robot_0/camera_link/Camera_npy',
	#                           path_to='/my_robot_0/camera_link/Camera_npy_stereo',
	#                           exclusive_select=False)
	#
	# set_translate(stage.GetPrimAtPath('/my_robot_0/camera_link/Camera_npy_stereo'), [0.5, 0, 0])
	viewport, _ = create_viewport("/my_robot_0/camera_link/Camera", config["headless"].get(), 0,
	                           exp_info["config"]["npy_sensor_size"].get(), old_h_ape, old_v_ape)
	viewport_window_list.append(viewport)
	is_rtx = exp_info["config"]["rtx_mode"].get()
	if is_rtx:
		set_raytracing_settings(exp_info["config"]["physics_hz"].get())
	else:
		set_pathtracing_settings(exp_info["config"]["physics_hz"].get())

	simulation_context.play()
	for _ in range(5): simulation_context.render()
	old_v_ape = [2.32] * len(old_v_ape)  # todo this is harcoded
	for index, cam in enumerate(viewport_window_list):
		simulation_context.step(render=False)
		simulation_context.render()
		camera = stage.GetPrimAtPath(cam.get_active_camera())
		camera.GetAttribute("horizontalAperture").Set(old_h_ape[index])
		camera.GetAttribute("verticalAperture").Set(old_v_ape[index])
	simulation_context.stop()
	# lidar_prim = add_3d_lidar("/my_robot_0/yaw_link")
	# set_translate(stage.GetPrimAtPath('/my_robot_0/yaw_link/Lidar'), [0, 0, -5])

	cnt_tf = -1
	use_teleport = config["use_teleport"].get()
	use_reindex = config["use_reindex"].get()
	id_bag = 0
	bag_path = os.path.join(config["experiment_folder"].get(), config['bag_subpath'].get(),
	                        f"{config['bag_basename'].get()}_{id_bag}.bag")
	joint_order = ['x_joint', 'y_joint', 'z_joint', 'roll_joint', 'pitch_joint', 'yaw_joint']
	joint_position = []
	joint_velocity = []
	joint_time = []
	robot_pose = []
	started = use_reindex
	while os.path.exists(bag_path):
		bag = rosbag.Bag(bag_path)
		for topic, msg, t in bag.read_messages(
				topics=["/my_robot_0/joint_states", "/my_robot_0/odom", "/starting_experiment"]):
			if not started:
				if topic == "/starting_experiment":
					started = True
					continue
				else:
					continue
			if 'joint' in topic:
				joint_position.append(msg.position)
				joint_velocity.append(msg.velocity)
				joint_time.append(msg.header.stamp)
			else:
				robot_pose.append([msg.pose.pose.position, msg.pose.pose.orientation])
		id_bag += 1
		bag_path = os.path.join(config["experiment_folder"].get(), config['bag_subpath'].get(),
		                        f"{config['bag_basename'].get()}_{id_bag}.bag")
	if len(joint_position) == 0:
		print("No bag found")
		sys.exit(-1)
	ratio_tf = exp_info['config']['ratio_tf'].get()
	init_x, init_y, init_z, init_roll, init_pitch, init_yaw = get_robot_joint_init_loc('/my_robot_0')
	init_pos = np.array([init_x, init_y, init_z])
	init_rot = np.array([init_roll, init_pitch, init_yaw])
	simulation_context.play()
	robot_collisions(False)
	kit.update()
	simulation_context.stop()
	move_robot('/my_robot_0', [0, 0, 0], [0,0,0], 0)
	kit.update()
	simulation_context.play()
	for _ in range(5):
		simulation_context.step(render=False)
		simulation_context.render()

	timeline.set_auto_update(False)

	timeline.set_current_time(min(- 1 / (exp_info['config']["physics_hz"].get() / ratio_camera),
	                              -abs(exp_info['config']["bootstrap_exploration"].get())))
	simulation_step = int(timeline.get_current_time() * exp_info['config']["physics_hz"].get()) - 1

	out_dir_npy = os.path.join(config['experiment_folder'].get(), config['out_folder_npy'].get())
	if write_flow:
		_tmp = extension_custom.MyRecorder()
		_tmp.on_startup()
		_settings = _tmp.get_default_settings()
		_settings["rgb"]["enabled"] = False
		_settings["motion-vector"]["enabled"] = write_flow
		_settings["motion-vector"]["colorize"] = False
		_settings["motion-vector"]["npy"] = True
		my_recorder_flow = recorder_setup(_settings, True, out_dir_npy, True, 0)
		my_recorder_flow._enable_record = False

	if write_normals:
		_tmp = extension_custom.MyRecorder()
		_tmp.on_startup()
		_settings = _tmp.get_default_settings()
		_settings["rgb"]["enabled"] = True
		_settings["normals"]["enabled"] = write_normals
		_settings["motion-vector"]["colorize"] = False
		_settings["motion-vector"]["npy"] = True
		my_recorder_normals = recorder_setup(_settings, True, out_dir_npy, True, 0)
		my_recorder_normals._enable_record = False

	if write:
		_tmp = exp_info['config']['_recorder_settings'].get()
		_tmp["depth"]["enabled"] = False
		_tmp["depthLinear"]["enabled"] = False
		_tmp["semantic"]["enabled"] = False
		_tmp["normals"]["enabled"] = False
		_tmp["bbox_2d_loose"]["enabled"] = False
		_tmp["bbox_2d_tight"]["enabled"] = False
		_tmp["bbox_3d"]["enabled"] = False

		my_recorder = recorder_setup(_tmp, out_dir_npy, True, 0)
		my_recorder._enable_record = False

	# how to hide dynamic content
	dynamicprims = [stage.GetPrimAtPath(f"/my_human_{i}") for i in range(12)]
	for prim in stage.GetPrimAtPath("/World").GetChildren()[6:]:
		dynamicprims.append(prim)
	toggle_dynamic_objects(dynamicprims, False)

	forward = True
	while kit.is_running():
		simulation_step += 1
		if simulation_step == 0:
			simulation_context.play()
			_dc = dynamic_control_interface()
			handle = _dc.get_rigid_body('/my_robot_0/yaw_link')
			if not use_teleport:
				art = _dc.get_articulation('/my_robot_0')
				body = _dc.get_articulation_root_body(art)
				joints = []
				_dc.wake_up_articulation(art)
				for joint in joint_order:
					joints.append(_dc.find_articulation_dof(art, joint))

			robot_collisions(True)
			omni.kit.commands.execute("RosBridgeUseSimTime", use_sim_time=True)
			omni.kit.commands.execute("RosBridgeUsePhysicsStepSimTime", use_physics_step_sim_time=True)
			omni.kit.commands.execute("RosBridgeTickComponent", path="/ROS_Clock")  # need to initialize
			prev_time = timeline.get_current_time() + 7 / 240 * (
						simulation_step == 0)  # to account for an "issue" during generation

			if write:
				my_recorder._update()
				my_recorder._enable_record = True
			if write_flow:
				my_recorder_flow._update()
				my_recorder_flow._enable_record = True
			if write_normals:
				my_recorder_normals._update()
				my_recorder_normals._enable_record = True

			simulation_context.render()
			simulation_context.render()

		if simulation_step < 0:
			simulation_context.step(render=False)
			if (simulation_step % ratio_camera == 0):
				timeline.forward_one_frame()
			continue

		if use_teleport:
			if simulation_step % ratio_tf == 0:
				cnt_tf += 1
				teleport("/my_robot_0", np.array(joint_position[cnt_tf][:3]) / meters_per_unit + init_pos
				         , tf.Rotation.from_euler('XYZ', joint_position[cnt_tf][3:] + init_rot).as_quat())
				if (simulation_step % (ratio_tf * 2) == 0):  # odm is published half the rate of the tf
					myp = _dc.get_rigid_body_pose(handle)
					print(
						f"pose diff {np.array(_dc.get_rigid_body_pose(handle).p) / 100 - np.array([robot_pose[int(cnt_tf / 2)][0].x, robot_pose[int(cnt_tf / 2)][0].y, robot_pose[int(cnt_tf / 2)][0].z])}")
			else:
				vel = np.array(joint_velocity[
					               cnt_tf])  # or average position between the two, or use the IMU to interpolate also which has 240 hz
				pos = (np.array(joint_position[cnt_tf][:3]) + vel[:3] * 1 / 240) / meters_per_unit + init_pos
				ori = (np.array(joint_position[cnt_tf][3:]) + vel[3:] * 1 / 240) + init_rot
				teleport("/my_robot_0", pos, tf.Rotation.from_euler('XYZ', ori).as_quat())
		else:
			_dc.wake_up_articulation(art)
			if simulation_step % ratio_tf == 0:
				cnt_tf += 1
				vel = np.array(joint_velocity[cnt_tf])
				if cnt_tf == 0:
					teleport("/my_robot_0", np.array(joint_position[cnt_tf][:3]) / meters_per_unit + init_pos
					         , tf.Rotation.from_euler('XYZ', joint_position[cnt_tf][3:] + init_rot - vel[3:] * 1 / 240).as_quat())

				_dc.set_rigid_body_linear_velocity(body, vel[:3] / meters_per_unit)
				_dc.set_rigid_body_angular_velocity(body, vel[3:])

				if (simulation_step % (ratio_tf * 2) == 0):
					myp = _dc.get_rigid_body_pose(handle)
					print(
						f"pose diff {np.array(_dc.get_rigid_body_pose(handle).p) / 100 - np.array([robot_pose[int(cnt_tf / 2)][0].x, robot_pose[int(cnt_tf / 2)][0].y, robot_pose[int(cnt_tf / 2)][0].z])}")
		simulation_context.render()
		# you can either do this or the old way. This changes the location of objects/animations in substeps such that
		# sensors can eventually perceive it. Note that "forward" is set on the camera loop as it was set in the main
		# generation loop. This means that you "rewind" only at the -eight frame.
		timeline.set_current_time(prev_time + 1 / 240 * (1 if forward else -1))
		prev_time = timeline.get_current_time()

		simulation_context.step(render=False)

		print("Clocking...")
		# NOTE THAT THIS MIGHT GET CONFUSING -- reindexing/retiming is needed for sure
		omni.kit.commands.execute("RosBridgeTickComponent", path="/ROS_Clock")
		if simulation_step == 0:
			omni.kit.commands.execute("RosBridgeTickComponent", path="/ROS_Clock")
		time.sleep(0.2)


		if simulation_step % ratio_camera == 0:
			if (simulation_step + ratio_camera) / ratio_camera < (experiment_length / reversing_timeline_ratio) * (
					cnt_reversal):
				forward = True
				# timeline.forward_one_frame() # or you can advance time directly here as it was done previously, note that you need to remove the "+ratio_camera" above
			else:
				if (simulation_step + ratio_camera) / ratio_camera >= ((experiment_length - 1) / reversing_timeline_ratio) * (
						cnt_reversal + 1) or \
						(timeline.get_current_time() - 1 / timeline.get_time_codes_per_seconds()) < 0:
					cnt_reversal += 2
					forward = True
					# timeline.forward_one_frame() # note that you need to remove the "+ratio_camera" above
				else:
					forward = False
					# timeline.rewind_one_frame()
			if write_flow:
				if my_recorder_flow._enable_record:
					simulation_context.render()
					my_recorder_flow._counter += 1
					time.sleep(1.5)  # this seems necessary
					my_recorder_flow._update()

			# you have two ways to proceed here. the sleeping performs just the rendering and then you manually toggle the recorder below
			# otherwise use pub_and_write_images which automatically calls it if necessary. In the latter case, remember to increase the counter
			sleeping(simulation_context, viewport_window_list, is_rtx)
			# if write:
			# 	if my_recorder._enable_record:
			# 		my_recorder._counter += 1
			# pub_and_write_images(my_recorder, simulation_context, viewport_window_list, True, ros_camera_list, is_rtx)

			if write:
				if my_recorder._enable_record:
					my_recorder._counter += 1
					my_recorder._update()
			if write_normals:
				if my_recorder_normals._enable_record:
					my_recorder_normals._counter += 1
					my_recorder_normals._update()

		# new sensor here -- imagine 30 fps -- in that case I need to publish
		# if you need sensors in the middle you need to interpolate
		# using IMU and TF readings
		# you can access those from the rosbags
		# note you might need to work with the timeline times if the rate that you want is different
		# as we show above in the comments
		# once the animation detection by lidar is solved
		# if simulation_step % ratio_camera == 0:
		# 	print("Publish LiDAR")
		# 	if simulation_step == 0:
		# 		pub twice - first one enables
				# omni.kit.commands.execute("RosBridgeTickComponent", path=lidar_prim)
			# simulation_context.render() # this IS necessary
			# omni.kit.commands.execute("RosBridgeTickComponent", path=lidar_prim)

		if simulation_step % ratio_camera == 0 and simulation_step / ratio_camera == experiment_length:
			print("End of experiment!!!")
			simulation_context.pause()
			break
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
