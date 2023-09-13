import utils.misc_utils
from omni.isaac.core.utils.prims import set_targets
from scipy.spatial.transform import Rotation
from utils.misc_utils import *
from omni.isaac.core.utils.render_product import create_hydra_texture

def create_odom_message(_dc, robot_body_ptr, handle, meters_per_unit):
	"""
	Create odometry message for the robot_body_ptr.
	Converts the readings from the IsaacSim unit to the mps when necessary
	Gets the current rostime

	header frame us "WORLD" and the child frame is from the "handle"
	"""
	lin_vel = _dc.get_rigid_body_local_linear_velocity(robot_body_ptr)
	ang_vel = _dc.get_rigid_body_angular_velocity(robot_body_ptr)
	pose = _dc.get_rigid_body_pose(robot_body_ptr)
	odom_msg = Odometry()
	odom_msg.header.frame_id = "world"
	odom_msg.header.stamp = rospy.Time.now()
	odom_msg.child_frame_id = handle[1:] if handle.startswith("/") else handle
	odom_msg.pose.pose.position.x = pose.p.x * meters_per_unit
	odom_msg.pose.pose.position.y = pose.p.y * meters_per_unit
	odom_msg.pose.pose.position.z = pose.p.z * meters_per_unit
	odom_msg.pose.pose.orientation.x = pose.r.x
	odom_msg.pose.pose.orientation.y = pose.r.y
	odom_msg.pose.pose.orientation.z = pose.r.z
	odom_msg.pose.pose.orientation.w = pose.r.w
	odom_msg.twist.twist.linear.x = lin_vel.x * meters_per_unit
	odom_msg.twist.twist.linear.y = lin_vel.y * meters_per_unit
	odom_msg.twist.twist.linear.z = lin_vel.z * meters_per_unit
	odom_msg.twist.twist.angular.x = ang_vel.x
	odom_msg.twist.twist.angular.y = ang_vel.y
	odom_msg.twist.twist.angular.z = ang_vel.z
	p_cov = np.array([0.0] * 36).reshape(6, 6)
	p_cov[0:2, 0:2] = 0.00
	p_cov[5, 5] = 0.00
	odom_msg.pose.covariance = tuple(p_cov.ravel().tolist())
	odom_msg.twist.covariance = tuple(p_cov.ravel().tolist())
	return odom_msg


def create_diff_odom_message(_dc, robot_body_ptr, handle, meters_per_unit, base_body_ptr, base_handle):
	"""
	Create odometry message for the robot_body_ptr.
	Converts the readings from the IsaacSim unit to the mps when necessary
	Gets the current rostime

	header frame us "WORLD" and the child frame is from the "handle"
	"""
	lin_vel = _dc.get_rigid_body_local_linear_velocity(robot_body_ptr)
	ang_vel = _dc.get_rigid_body_angular_velocity(robot_body_ptr)
	pose = _dc.get_rigid_body_pose(robot_body_ptr)
	base_lin_vel = _dc.get_rigid_body_local_linear_velocity(base_body_ptr)
	base_ang_vel = _dc.get_rigid_body_angular_velocity(base_body_ptr)
	base_pose = _dc.get_rigid_body_pose(base_body_ptr)
	odom_msg = Odometry()
	odom_msg.header.frame_id = base_handle
	odom_msg.header.stamp = rospy.Time.now()
	odom_msg.child_frame_id = handle[1:] if handle.startswith("/") else handle
	odom_msg.pose.pose.position.x = (pose.p.x - base_pose.p.x) * meters_per_unit
	odom_msg.pose.pose.position.y = (pose.p.y - base_pose.p.y) * meters_per_unit
	odom_msg.pose.pose.position.z = (pose.p.z - base_pose.p.z) * meters_per_unit
	q1 = Quaternion(base_pose.r.w, base_pose.r.x, base_pose.r.y, base_pose.r.z)
	q2 = Quaternion(pose.r.w, pose.r.x, pose.r.y, pose.r.z)
	q = q1.conjugate * q2
	odom_msg.pose.pose.orientation.x = q.x
	odom_msg.pose.pose.orientation.y = q.y
	odom_msg.pose.pose.orientation.z = q.z
	odom_msg.pose.pose.orientation.w = q.w
	odom_msg.twist.twist.linear.x = (lin_vel.x - base_lin_vel.x) * meters_per_unit
	odom_msg.twist.twist.linear.y = (lin_vel.y - base_lin_vel.y) * meters_per_unit
	odom_msg.twist.twist.linear.z = (lin_vel.z - base_lin_vel.z) * meters_per_unit
	odom_msg.twist.twist.angular.x = (ang_vel.x - base_ang_vel.x)
	odom_msg.twist.twist.angular.y = (ang_vel.y - base_ang_vel.y)
	odom_msg.twist.twist.angular.z = (ang_vel.z - base_ang_vel.z)
	p_cov = np.array([0.0] * 36).reshape(6, 6)
	p_cov[0:2, 0:2] = 0.00
	p_cov[5, 5] = 0.00
	odom_msg.pose.covariance = tuple(p_cov.ravel().tolist())
	odom_msg.twist.covariance = tuple(p_cov.ravel().tolist())
	return odom_msg


def create_camera_pose_message(_dc, camera_body_ptr, handle, meters_per_unit):
	"""
	Similar to the odom, but it's just for a pose message, in this case for the camera
	"""
	pose = _dc.get_rigid_body_pose(camera_body_ptr)
	camera_pose = PoseStamped()
	camera_pose.header.frame_id = "world"
	camera_pose.header.stamp = rospy.Time.now()
	camera_pose.pose.position.x = pose.p.x * meters_per_unit
	camera_pose.pose.position.y = pose.p.y * meters_per_unit
	camera_pose.pose.position.z = pose.p.z * meters_per_unit
	camera_pose.pose.orientation.x = pose.r.x
	camera_pose.pose.orientation.y = pose.r.y
	camera_pose.pose.orientation.z = pose.r.z
	camera_pose.pose.orientation.w = pose.r.w
	return camera_pose


def add_pose_tree(path: str, irotate: bool=False):
	"""
	Add the tf publisher to the desired path.
	This path should be the robot itself.
	Each robot has a pose tree.
	"""
	if path.startswith("/"):
		path = path[1:]
	og.Controller.edit(
		{"graph_path": f"/{path}/TFActionGraph", "evaluator_name": "execution"},
		{
			og.Controller.Keys.CREATE_NODES: [
				("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
				("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
				("PublishTF", "omni.isaac.ros_bridge.ROS1PublishTransformTree"),
			],
			og.Controller.Keys.CONNECT: [
				("OnImpulseEvent.outputs:execOut", "PublishTF.inputs:execIn"),
				("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
			],
			og.Controller.Keys.SET_VALUES: [
				("PublishTF.inputs:nodeNamespace", f"/{path}"),

			]
		},
	)
	# fixme
	if irotate:
		omni.kit.commands.execute('ChangeProperty',
		                          prop_path=Sdf.Path('/my_robot_0/ROS_PoseTree.poseTreePubTopic'),
		                          value='/tf2',
		                          prev='/tf')
	set_target_prims(primPath=f"/{path}/TFActionGraph/PublishTF", inputName="inputs:targetPrims",
	                 targetPrimPaths=[f"/{path}"])
	return f"/{path}/TFActionGraph"


def add_camera_and_viewport(path: str, resolution: list, old_h_ape, old_v_ape, sc, index=0,
                            robot_index=0, cam_per_robot=1, camera_path="Camera"):
	"""
	The function create first the ROSBridge Camera and then the corresponding viewport.
	index is the number of the camera for the given robot.
	headless is a boolean that indicates if the simulation is headless or not (i.e. create a visual viewport or not).
	robot_index correspond to the n-th robot in the scene.
	"""
	resolution = tuple(resolution)

	camera_path = path + f"/{camera_path}"
	index = robot_index * cam_per_robot + index

	stage = omni.usd.get_context().get_stage()
	camera = stage.GetPrimAtPath(camera_path)
	old_h_ape.append(camera.GetAttribute("horizontalAperture").Get())
	old_v_ape.append(camera.GetAttribute("verticalAperture").Get())

	viewport_name = "Viewport" + (f" {index + 1}" if str(index + 1) != "0" and str(index + 1) != "1" else "")
	sc.step()

	keys = og.Controller.Keys
	(camera_graph, _, _, _) = og.Controller.edit(
		{
			"graph_path": f"{path}/ROSCamera_{index}_Graph",
			"evaluator_name": "push",
			"pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
		},
		{
			keys.CREATE_NODES: [
				("OnTick", "omni.graph.action.OnTick"),
				("createViewport", "omni.isaac.core_nodes.IsaacCreateViewport"),
				("setViewportResolution", "omni.isaac.core_nodes.IsaacSetViewportResolution"),
				("getRenderProduct", "omni.isaac.core_nodes.IsaacGetViewportRenderProduct"),
				("setCamera", "omni.isaac.core_nodes.IsaacSetCameraOnRenderProduct"),
				("cameraHelperRgb", "omni.isaac.ros_bridge.ROS1CameraHelper"),
				("cameraHelperInfo", "omni.isaac.ros_bridge.ROS1CameraHelper"),
				("cameraHelperDepth", "omni.isaac.ros_bridge.ROS1CameraHelper"),
			],
			keys.CONNECT: [
				("OnTick.outputs:tick", "createViewport.inputs:execIn"),
				("createViewport.outputs:execOut", "getRenderProduct.inputs:execIn"),
				("createViewport.outputs:viewport", "getRenderProduct.inputs:viewport"),

				("createViewport.outputs:execOut", "setViewportResolution.inputs:execIn"),
				("createViewport.outputs:viewport", "setViewportResolution.inputs:viewport"),

				("getRenderProduct.outputs:execOut", "setCamera.inputs:execIn"),
				("getRenderProduct.outputs:renderProductPath", "setCamera.inputs:renderProductPath"),
				("setCamera.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
				("setCamera.outputs:execOut", "cameraHelperInfo.inputs:execIn"),
				("setCamera.outputs:execOut", "cameraHelperDepth.inputs:execIn"),
				("getRenderProduct.outputs:renderProductPath", "cameraHelperRgb.inputs:renderProductPath"),
				("getRenderProduct.outputs:renderProductPath", "cameraHelperInfo.inputs:renderProductPath"),
				("getRenderProduct.outputs:renderProductPath", "cameraHelperDepth.inputs:renderProductPath"),
			],
			og.Controller.Keys.SET_VALUES: [
				("createViewport.inputs:viewportId", index),
				("setViewportResolution.inputs:height", int(resolution[1])),
				("setViewportResolution.inputs:width", int(resolution[0])),

				("cameraHelperRgb.inputs:frameId", path[1:]),
				("cameraHelperRgb.inputs:topicName", path + f"/{index}/rgb/image_raw"),
				("cameraHelperRgb.inputs:type", "rgb"),

				("cameraHelperDepth.inputs:frameId", path[1:]),
				("cameraHelperDepth.inputs:topicName", path + f"/{index}/depth/image_raw"),
				("cameraHelperDepth.inputs:type", "depth"),

				("cameraHelperInfo.inputs:frameId", path[1:]),
				("cameraHelperInfo.inputs:topicName", path + f"/{index}/camera_info"),
				("cameraHelperInfo.inputs:type", "camera_info"),
			],
		},
	)

	set_targets(
		prim=omni.usd.get_context().get_stage().GetPrimAtPath(f"{path}/ROSCamera_{index}_Graph/setCamera"),
		attribute="inputs:cameraPrim",
		target_prim_paths=[camera_path],
	)
	og.Controller.evaluate_sync(camera_graph)
	for _ in range(5):
		sc.step()
		omni.kit.app.get_app().update()
	viewport_handle = [x for x in omni.kit.viewport.window.get_viewport_window_instances()][-1].viewport_api
	viewport_handle.set_texture_resolution((resolution[0], resolution[1]))
	for _ in range(5):
		sc.step()
		omni.kit.app.get_app().update()
	return camera_graph.get_path_to_graph(), viewport_handle


def add_joint_state(path: str):
	if path.startswith("/"):
		path = path[1:]
	og.Controller.edit(
		{"graph_path": f"/{path}/JointActionGraph", "evaluator_name": "execution"},
		{
			og.Controller.Keys.CREATE_NODES: [
				("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
				("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
				("PublishJointState", "omni.isaac.ros_bridge.ROS1PublishJointState"),
				("SubscribeJointState", "omni.isaac.ros_bridge.ROS1SubscribeJointState"),
				("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
			],
			og.Controller.Keys.CONNECT: [
				("OnImpulseEvent.outputs:execOut", "PublishJointState.inputs:execIn"),
				("OnImpulseEvent.outputs:execOut", "SubscribeJointState.inputs:execIn"),
				("OnImpulseEvent.outputs:execOut", "ArticulationController.inputs:execIn"),

				("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),

				("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
				("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
				("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
				("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
			],
			og.Controller.Keys.SET_VALUES: [
				# Providing path to Articulation Controller node
				# Providing the robot path is equivalent to setting the targetPrim in Articulation Controller node
				("ArticulationController.inputs:usePath", True),
				("ArticulationController.inputs:robotPath", "/" + path),
				# Assigning topic names to clock publishers
				("PublishJointState.inputs:topicName", "/" + path + "/joint_states"),
				("SubscribeJointState.inputs:topicName", "/" + path + "/joint_commands"),
			],
		},
	)
	# set_target_prims(primPath=f"/{path}/JointActionGraph/SubscribeJointState", targetPrimPaths=[f"/{path}"])
	set_target_prims(primPath=f"/{path}/JointActionGraph/PublishJointState", targetPrimPaths=[f"/{path}"])
	return f"/{path}/JointActionGraph"


def add_clock():
	(_clock_graph, _, _, _) = og.Controller.edit(
		{"graph_path": "/ClockActionGraph", "evaluator_name": "push",
		 "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND, },
		{
			og.Controller.Keys.CREATE_NODES: [
				("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
				("OnTick", "omni.graph.action.OnTick"),
				("PublishManualClock", "omni.isaac.ros_bridge.ROS1PublishClock"),
			],
			og.Controller.Keys.CONNECT: [
				# Connecting execution of OnImpulseEvent node to PublishManualClock so it will only publish when an impulse event is triggered
				("OnTick.outputs:tick", "PublishManualClock.inputs:execIn"),
				# Connecting simulationTime data of ReadSimTime to the clock publisher nodes
				("ReadSimTime.outputs:simulationTime", "PublishManualClock.inputs:timeStamp"),
			],
			og.Controller.Keys.SET_VALUES: [
				# Assigning topic names to clock publishers
				("PublishManualClock.inputs:topicName", "/clock"),
			],
		},
	)
	return _clock_graph


def get_robot_yaw(x, y, z, env_mesh, shifts):
	"""
	Checks the best robot yaw angle for the given position.
	Cast rays from the robot position to the environment mesh and returns the angle
	It considers 36 rays.
	For each ray we compute the distance to the nearest point on the mesh.
	If the distance is infinite, it gets interpolated.
	We compute a rolling window sum (with a window size of 4 rays) of the distances.
	Return the best yaw angle in RADIANS.
	"""
	checking_steps = 36
	angles = [[np.cos(np.pi * 2.0 / checking_steps * c_step), np.sin(np.pi * 2.0 / checking_steps * c_step), 0] for
	          c_step in range(checking_steps)]
	positions = [[x + shifts[0], y + shifts[1], z + shifts[2]] for _ in range(checking_steps)]
	checking_rays = trimesh.proximity.longest_ray(env_mesh, positions, angles)
	checking_rays[checking_rays < 0] = 0
	nans, x = inf_helper(checking_rays)
	checking_rays[nans] = np.interp(x(nans), x(~nans), checking_rays[~nans])
	checking_rays[checking_rays > 8] = 8
	rolling_rays = int(40 / (360 / checking_steps))
	checking_rays = np.append(checking_rays, checking_rays[:rolling_rays - 1])
	checking_rays = np.convolve(checking_rays, np.ones(rolling_rays, dtype=int), 'valid') / rolling_rays
	return (np.argmax(checking_rays) + rolling_rays / 2) * 2 * np.pi / checking_steps


def get_vp_list():
	from omni.kit.viewport.window import get_viewport_window_instances
	return [x for x in get_viewport_window_instances()]


def create_viewport(camera_path, is_headless, index, resolution, old_h_ape, old_v_ape, sc):
	"""
	The function create the viewport for the given camera.
	Creates an handle, a viewport and the window position/size if the system is not headless.
	"""
	stage = omni.usd.get_context().get_stage()
	camera = stage.GetPrimAtPath(camera_path)
	old_h_ape.append(camera.GetAttribute("horizontalAperture").Get())
	old_v_ape.append(camera.GetAttribute("verticalAperture").Get())
	index += 1  # omniverse starts from 1
	viewport_name = "Viewport" + (f" {index}" if str(index) != "0" and str(index) != "1" else "")
	viewport = omni.kit.viewport.utility.get_active_viewport_window(window_name=viewport_name)
	viewport_handle = omni.kit.viewport.utility.get_viewport_from_window_name(viewport_name)
	if not viewport_handle:
		viewport = omni.kit.viewport.utility.create_viewport_window(name=viewport_name)
		viewport_handle = omni.kit.viewport.utility.get_viewport_from_window_name(viewport.name)
	if not is_headless:
		viewport.setPosition(1000, 400)
		viewport.height, viewport.width = 300, 300

	viewport_handle.set_active_camera(camera_path)
	for _ in range(10):
		sc.step()
	viewport_handle.set_texture_resolution((resolution[0], resolution[1]))
	sc.step()
	return viewport_handle, viewport.name


def ros_launchers_setup(roslaunch, env_limits_shifted, config):
	"""
	Setup the ros launchers for the simulation.
	We need an exploration manager for every robot, and a collision checking service to place the objects.
	"""
	roslaunch_files = []
	roslaunch_args = []
	launch_files = []
	print("launching ros nodes...")
	if not config["only_placement"].get():
		for i in range(config["num_robots"].get()):
			# TODO hack to be compatible with the old version
			if type(config["is_iRotate"].get()) == list:
				is_irotate = config["is_iRotate"].get()[i]
			else:
				is_irotate = config["is_iRotate"].get()
			if not is_irotate:
				cli_args1 = ["exploration_manager", "my_exploration.launch",
				             # cli_args1 = ["/home/ebonetto/catkin_ws/src/FUEL/fuel_planner/exploration_manager/launch/my_exploration.launch",
				             "box_min_x:={:.4f}".format(env_limits_shifted[0] - 0.2),
				             "box_min_y:={:.4f}".format(env_limits_shifted[1] - 0.2),
				             "box_min_z:={:.4f}".format(env_limits_shifted[2]),
				             "box_max_x:={:.4f}".format(env_limits_shifted[3] + 0.2),
				             "box_max_y:={:.4f}".format(env_limits_shifted[4] + 0.2),
				             "box_max_z:={:.4f}".format(min(3, env_limits_shifted[5] - 0.1)),
				             f"mav_name:={config['robot_base_prim_path'].get()}{i}"]
				roslaunch_files.append(roslaunch.rlutil.resolve_launch_arguments(cli_args1)[0])
				roslaunch_args.append(cli_args1[2:])
				launch_files.append((roslaunch_files[-1], roslaunch_args[-1]))
			else:
				cli_args1 = ["custom_joint_controller_ros_irotate", "publish_joint_commands_node.launch",
				             "position_limit_x:={:.4f}".format(env_limits_shifted[3] + 0.2),
				             "position_limit_y:={:.4f}".format(env_limits_shifted[4] + 0.2),
				             "position_limit_z:={:.4f}".format(3),
				             "robot_id:=1", "frame_id:='base'"]
				roslaunch_files.append(roslaunch.rlutil.resolve_launch_arguments(cli_args1)[0])
				roslaunch_args.append(cli_args1[2:])
				launch_files.append((roslaunch_files[-1], roslaunch_args[-1]))
	# TODO hack because we pre-cache the robot mesh
	if type(config["robot_mesh_path"].get()) == list:
		mesh_path = config["robot_mesh_path"].get()[0]
	else:
		mesh_path = config["robot_mesh_path"].get()
	cli_args2 = ["collision_check", "collision_check.launch",
	             "robot_mesh_path:={}".format(mesh_path)]
	roslaunch_file2 = roslaunch.rlutil.resolve_launch_arguments(cli_args2)[0]
	roslaunch_args2 = cli_args2[2:]
	launch_files.append((roslaunch_file2, roslaunch_args2))
	return launch_files


def create_imu_message(frame, last_reading, meters_per_unit):
	"""
	Create the IMU message from the last reading.
	"""
	imu_msg = Imu()
	imu_msg.header.frame_id = frame[1:] if frame.startswith("/") else frame
	imu_msg.header.stamp = rospy.Time.now()
	imu_msg.angular_velocity.x = last_reading.ang_vel_x
	imu_msg.angular_velocity.y = last_reading.ang_vel_y
	imu_msg.angular_velocity.z = last_reading.ang_vel_z
	imu_msg.linear_acceleration.x = last_reading.lin_acc_x * meters_per_unit * meters_per_unit
	imu_msg.linear_acceleration.y = last_reading.lin_acc_y * meters_per_unit * meters_per_unit
	imu_msg.linear_acceleration.z = last_reading.lin_acc_z * meters_per_unit * meters_per_unit
	imu_msg.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
	imu_msg.linear_acceleration_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
	return imu_msg


def setup_imu_sensor(_is, config, imu_sensor_path):
	"""
	Setup the IMU sensor config.
	Keep in mind that this is relative to the parent body, so any transform the parent has is already reflected.
	"""
	add_imu_sensor, sensor = omni.kit.commands.execute(
		"IsaacSensorCreateImuSensor",
		path="/imu_sensor",
		parent=imu_sensor_path,
		sensor_period=1 / config["physics_hz"].get(),
		orientation=Gf.Quatd(1, 0, 0, 0),
		visualize=False,
	)
	if not add_imu_sensor:
		raise Exception("Failed to add IMU sensor")
	return sensor


def pub_imu(_is, imu_pubs, robot_imu_frames, meters_per_unit):
	"""
	Simple message publisher
	"""
	for index, handle in enumerate(robot_imu_frames):
		last_reading = _is.get_sensor_sim_reading(handle + "/imu_sensor")
		imu_pubs[index].publish(create_imu_message(handle, last_reading, meters_per_unit))


def pub_cam_pose(camera_pose_frames, cam_pose_pubs, _dc, meters_per_unit):
	"""
	Simple message publisher
	"""
	for index, handle in enumerate(camera_pose_frames):
		camera_body_ptr = _dc.get_rigid_body(handle)
		cam_pose_pubs[index].publish(create_camera_pose_message(_dc, camera_body_ptr, handle, meters_per_unit))


def pub_odom(robot_odom_frames, odom_pubs, _dc, meters_per_unit, diff_odom_frames=[]):
	"""
	Simple message publisher
	"""
	odoms = []
	angles = []
	if len(diff_odom_frames) == 0:
		for index, handle in enumerate(robot_odom_frames):
			robot_body_ptr = _dc.get_rigid_body(handle)
			odom = create_odom_message(_dc, robot_body_ptr, handle, meters_per_unit)
			odoms.append([odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z])
			angles.append(Rotation.from_quat(
				[odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z,
				 odom.pose.pose.orientation.w]).as_euler("XYZ"))
			odom_pubs[index].publish(odom)
	else:
		for index, handle in enumerate(robot_odom_frames):
			robot_body_ptr = _dc.get_rigid_body(handle)
			diff_body_ptr = _dc.get_rigid_body(diff_odom_frames[index])
			diff_handle = diff_odom_frames[index][1:] if diff_odom_frames[index].startswith("/") else diff_odom_frames[
				index]
			odom = create_diff_odom_message(_dc, robot_body_ptr, handle, meters_per_unit, diff_body_ptr, diff_handle)
			odoms.append([odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z])
			angles.append(Rotation.from_quat(
				[odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z,
				 odom.pose.pose.orientation.w]).as_euler("XYZ"))
			odom_pubs[index].publish(odom)

	return odoms, angles


def import_robot(robot_base_prim_path, n, usd_robot_path, local_file_prefix=''):
	"""
	Add the robot to the stage.
	Add semantics.
	"""
	stage = omni.usd.get_context().get_stage()
	res, _ = omni.kit.commands.execute("CreateReferenceCommand",
	                                   usd_context=omni.usd.get_context(),
	                                   path_to=f"{robot_base_prim_path}{n}",
	                                   asset_path=local_file_prefix + usd_robot_path,
	                                   instanceable=False)
	if res:
		clear_properties(f"{robot_base_prim_path}{n}")
		add_semantics(stage.GetPrimAtPath(f"{robot_base_prim_path}{n}"), "robot")
	else:
		raise Exception("Failed to import robot")


def get_valid_robot_location(environment, first):
	"""
	Query the service to place the robot in a free space AND compute an initial good yaw.
	"""
	x, y, z, _ = position_object(environment, type=0, reset=first)
	# robot is nearly circular so I do not have to worry about collisionsif environment.env_mesh != None:
	if environment.env_mesh != None:
		yaw = get_robot_yaw(x[0], y[0], z[0], environment.env_mesh, environment.shifts)
	print(f"Initial yaw: {yaw}")
	return x[0], y[0], z[0], yaw


def control_camera(viewport, sc):
	sc.step()

	if viewport is not None:
		import omni.syntheticdata._syntheticdata as sd

		stage = omni.usd.get_context().get_stage()
		# Required for editing the SDGPipeline graph which exists in the Session Layer
		with Usd.EditContext(stage, stage.GetSessionLayer()):
			# Get name of rendervar for RGB sensor type
			rv_rgb = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)

			# Get path to IsaacSimulationGate node in RGB pipeline
			rgb_camera_gate_path = omni.syntheticdata.SyntheticData._get_node_path(
				rv_rgb + "IsaacSimulationGate", viewport.get_render_product_path()
			)

			# Get name of rendervar for DistanceToImagePlane sensor type
			rv_depth = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
				sd.SensorType.DistanceToImagePlane.name)

			# Get path to IsaacSimulationGate node in Depth pipeline
			depth_camera_gate_path = omni.syntheticdata.SyntheticData._get_node_path(
				rv_depth + "IsaacSimulationGate", viewport.get_render_product_path()
			)

			# Get path to IsaacSimulationGate node in CameraInfo pipeline
			camera_info_gate_path = omni.syntheticdata.SyntheticData._get_node_path(
				"PostProcessDispatch" + "IsaacSimulationGate", viewport.get_render_product_path()
			)

			return rgb_camera_gate_path, depth_camera_gate_path, camera_info_gate_path


def add_ros_components(robot_base_prim_path, n, ros_transform_components, ros_camera_list, viewport_window_list,
                       camera_pose_frames, cam_pose_pubs, imu_pubs, robot_imu_frames,
                       robot_odom_frames, odom_pubs, lidars,
                       dynamic_prims, config, old_h_ape, old_v_ape, _is, simulation_context, _clock, irotate=False):
	"""
	Add the ROS components to the robot.
	This is done because we need different topics for each robot.
	Components added:
	- joint_states (publisher and subscriber)
	- tf broadcaster
	- camera
	- camera pose
	- imu
	- odom
	When necessary we create also the corresponding publisher (whenever the RosBridge component is not available).
	Publishers created:
	- imu
	- odom
	- camera pose
	"""
	ros_transform_components.append(add_joint_state(f"{robot_base_prim_path}{n}"))
	ros_transform_components.append(add_pose_tree(f"{robot_base_prim_path}{n}", irotate))

	# create camera
	component, viewport = add_camera_and_viewport(f"{robot_base_prim_path}{n}/camera_link",
	                                              config["robot_sensor_size"].get(),
	                                              old_h_ape, old_v_ape, simulation_context,
	                                              0, n, cam_per_robot=1)  # cam index is useful if you want multiple cameras
	cam_outputs = control_camera(viewport, simulation_context)
	ros_camera_list.append([n + 0, component, cam_outputs])
	viewport_window_list.append(viewport)

	# component, viewport = add_camera_and_viewport(f"{robot_base_prim_path}{n}/camera_link",
	#                                               config["robot_sensor_size"].get(),
	#                                               old_h_ape, old_v_ape, simulation_context,
	#                                               1, n, cam_per_robot=2)  # cam index is useful if you want multiple cameras
	# cam_outputs = control_camera(viewport, simulation_context)
	# ros_camera_list.append([n + 1, component, cam_outputs])
	# viewport_window_list.append(viewport)

	omni.kit.app.get_app().update()

	# append camera pose frame (we need only one) and pubs
	camera_pose_frames.append(f"{robot_base_prim_path}{n}/camera_link")
	cam_pose_pubs.append(rospy.Publisher(f"{robot_base_prim_path}{n}/camera/pose", PoseStamped, queue_size=10))

	for _ in range(10):
		og.Controller.set(og.Controller.attribute(f"{ros_transform_components[-1]}/OnImpulseEvent.state:enableImpulse"),
		                  True)
		og.Controller.set(og.Controller.attribute(f"{ros_transform_components[-2]}/OnImpulseEvent.state:enableImpulse"),
		                  True)
		og.Controller.evaluate_sync(_clock)
		simulation_context.step()

	# attach IMU sensor to the robot

	if irotate:
		setup_imu_sensor(_is, config, f"{robot_base_prim_path}{n}/imu_link")
		imu_pubs.append(rospy.Publisher(f"{robot_base_prim_path}{n}/imu_cam", Imu, queue_size=10))
		robot_imu_frames.append(f"{robot_base_prim_path}{n}/imu_link")

		setup_imu_sensor(_is, config, f"{robot_base_prim_path}{n}/base_link")
		imu_pubs.append(rospy.Publisher(f"{robot_base_prim_path}{n}/imu_body", Imu, queue_size=10))
		robot_imu_frames.append(f"{robot_base_prim_path}{n}/base_link")
		robot_odom_frames.append(f"{robot_base_prim_path}{n}/base_link")
	else:
		setup_imu_sensor(_is, config, f"{robot_base_prim_path}{n}/imu_link")
		imu_pubs.append(rospy.Publisher(f"{robot_base_prim_path}{n}/imu_body", Imu, queue_size=10))
		robot_imu_frames.append(f"{robot_base_prim_path}{n}/imu_link")

		setup_imu_sensor(_is, config, f"{robot_base_prim_path}{n}/camera_link")
		imu_pubs.append(rospy.Publisher(f"{robot_base_prim_path}{n}/imu_camera", Imu, queue_size=10))
		robot_imu_frames.append(f"{robot_base_prim_path}{n}/camera_link")

		robot_odom_frames.append(f"{robot_base_prim_path}{n}/yaw_link")

	odom_pubs.append(rospy.Publisher(f"{robot_base_prim_path}{n}/odom", Odometry, queue_size=10))

	stage = omni.usd.get_context().get_stage()
	dynamic_prims.append(stage.GetPrimAtPath(f"{robot_base_prim_path}{n}"))
	if lidars:
		stage = omni.usd.get_context().get_stage()
		dynamic_prims.append(stage.GetPrimAtPath(f"{robot_base_prim_path}{n}"))
		sensor = add_lidar(f"{robot_base_prim_path}{n}/yaw_link", [0, 0, -.1], [0, 0, 0], is_3d=True, is_2d=True)
		lidars.append(sensor)

def get_robot_joint_init_loc(name):
	"""
	It gets the initial location of the robot's joints

	:param name: The name of the robot
	:return: The initial location of the robot's joints.
	"""
	stage = omni.usd.get_context().get_stage()
	x = UsdPhysics.Joint.Get(stage, name + '/base_link/x_joint').GetLocalPos0Attr().Get()[0]
	y = UsdPhysics.Joint.Get(stage, name + '/x_link/y_joint').GetLocalPos0Attr().Get()[1]
	z = UsdPhysics.Joint.Get(stage, name + '/y_link/z_joint').GetLocalPos0Attr().Get()[2]

	roll = UsdPhysics.RevoluteJoint.Get(stage, name + '/z_link/roll_joint').GetLocalRot0Attr().Get()
	roll = Rotation.from_quat([roll.imaginary[0], roll.imaginary[1], roll.imaginary[2], roll.real]).as_euler('XYZ')[0]
	pitch = UsdPhysics.RevoluteJoint.Get(stage, name + '/roll_link/pitch_joint').GetLocalRot0Attr().Get()
	pitch = Rotation.from_quat([pitch.imaginary[0], pitch.imaginary[1], pitch.imaginary[2], pitch.real]).as_euler('XYZ')[
		1]
	yaw = UsdPhysics.RevoluteJoint.Get(stage, name + '/pitch_link/yaw_joint').GetLocalRot0Attr().Get()
	yaw = Rotation.from_quat([yaw.imaginary[0], yaw.imaginary[1], yaw.imaginary[2], yaw.real]).as_euler('XYZ')[2]

	return x, y, z, roll, pitch, yaw



def set_drone_joints_init_loc(name: str, pos: [], orientation: [], upper_zlim: float=100, lower_zlim: float=0, irotate=False):
	"""
	Move the drone to the specified location by acting on the JOINTS.

	PLEASE NOTE: the intial joint position published by joint_states will be 0,0,0 strangely. #IsaacBug
	The joints should be named as follows:
	- base_link/x_joint
	- x_link/y_joint
	- y_link/z_joint
	- z_link/roll_joint
	- roll_link/pitch_joint
	- pitch_link/yaw_joint

	name: the name of the robot (e.g. "my_robot_0", the prim path)
	pos: the position of the robot (x,y,z)
	orientation: the orientation of the robot (roll,pitch,yaw), in rad
	upper_zlim: the z limit of the robot (z)
	irotate: if True, the joints considered are the iRotate ones
	"""
	x, y, z = pos
	upper_zlim = max(upper_zlim, z)
	roll, pitch, yaw = orientation
	stage = omni.usd.get_context().get_stage()
	if irotate:
		UsdPhysics.Joint.Get(stage, name + '/x_link/x_joint').GetLocalPos0Attr().Set(Gf.Vec3f(x, 0, 0))
		UsdPhysics.Joint.Get(stage, name + '/y_link/y_joint').GetLocalPos0Attr().Set(Gf.Vec3f(0, y, 0))

		yaw = np.rad2deg(yaw)
		quat = (
				Gf.Rotation(Gf.Vec3d.XAxis(), 0)
				* Gf.Rotation(Gf.Vec3d.YAxis(), 0)
				* Gf.Rotation(Gf.Vec3d.ZAxis(), yaw)
		)
		UsdPhysics.RevoluteJoint.Get(stage, name + '/yaw_link/yaw_joint').GetLocalRot1Attr().Set(Gf.Quatf(quat.GetQuat()))
	else:
		UsdPhysics.Joint.Get(stage, name + '/base_link/x_joint').GetLocalPos0Attr().Set(Gf.Vec3f(x, 0, 0))
		UsdPhysics.Joint.Get(stage, name + '/x_link/y_joint').GetLocalPos0Attr().Set(Gf.Vec3f(0, y, 0))
		UsdPhysics.Joint.Get(stage, name + '/y_link/z_joint').GetLocalPos0Attr().Set(Gf.Vec3f(0, 0, z))
		stage.GetPrimAtPath(name + '/y_link/z_joint').GetAttribute('physics:lowerLimit').Set(-z + lower_zlim)
		stage.GetPrimAtPath(name + '/y_link/z_joint').GetAttribute('physics:upperLimit').Set(upper_zlim - z)

		roll = np.rad2deg(roll)
		quat = (
				Gf.Rotation(Gf.Vec3d.XAxis(), roll)
				* Gf.Rotation(Gf.Vec3d.YAxis(), 0)
				* Gf.Rotation(Gf.Vec3d.ZAxis(), 0)
		)
		UsdPhysics.RevoluteJoint.Get(stage, name + '/z_link/roll_joint').GetLocalRot0Attr().Set(Gf.Quatf(quat.GetQuat()))

		pitch = np.rad2deg(pitch)
		quat = (
				Gf.Rotation(Gf.Vec3d.XAxis(), pitch)
				* Gf.Rotation(Gf.Vec3d.YAxis(), 0)
				* Gf.Rotation(Gf.Vec3d.ZAxis(), 90)
		)
		UsdPhysics.RevoluteJoint.Get(stage, name + '/roll_link/pitch_joint').GetLocalRot0Attr().Set(
			Gf.Quatf(quat.GetQuat()))

		yaw = np.rad2deg(yaw)
		quat = (
				Gf.Rotation(Gf.Vec3d.XAxis(), 0)
				* Gf.Rotation(Gf.Vec3d.YAxis(), 0)
				* Gf.Rotation(Gf.Vec3d.ZAxis(), yaw)
		)
		UsdPhysics.RevoluteJoint.Get(stage, name + '/pitch_link/yaw_joint').GetLocalRot0Attr().Set(Gf.Quatf(quat.GetQuat()))


def add_robot_traj(path: str, config, meters_per_unit, time_codes_per_second):
	"""
	It adds a translation and rotation animation to the given path,
	using the given configuration, meters per unit, and time codes per second

	:param path: The path to the USD stage
	:type path: str
	:param config: The configuration file that contains the robot trajectory
	:param meters_per_unit: The scale of the scene
	:param time_codes_per_second: This is the number of time codes per second. This is the same as the frame rate of the
	animation
	"""
	clear_properties(path)
	for entry in config["robot_traj"].get():
		add_translate_anim(path, Gf.Vec3d(entry["pose"]["x"] / meters_per_unit, entry["pose"]["y"] / meters_per_unit,
		                                  entry["pose"]["z"] / meters_per_unit),
		                   entry["time"] * time_codes_per_second)
		add_rotation_anim(path, Gf.Vec3d(entry["pose"]["roll"], entry["pose"]["pitch"], entry["pose"]["yaw"]),
		                  entry["time"] * time_codes_per_second, use_double=True)


def diff_angle(alpha, beta):
	dist = (alpha - beta + np.pi + 2 * np.pi) % (2 * np.pi) - np.pi
	return dist


# assume position control
def check_pose_and_goals(init_loc, init_angle, c_pose, c_angle, path, goal_list, meters_per_unit, first):
	"""
	It sets the target position of the joints to the next goal in the list

	:param init_loc: the initial location of the robot
	:param init_angle: the initial orientation of the robot
	:param c_pose: current pose of the robot
	:param c_angle: current angle of the robot
	:param path: the path to the robot in the simulation
	:param goal_list: a list of goals, each goal is a list of 6 elements: x, y, z, roll, pitch, yaw
	:param meters_per_unit: This is the scale of the robot
	:param first: whether this is the first time the function is called
	:return: The goal list is being returned.
	"""
	dist_roll = abs(diff_angle(np.deg2rad(goal_list[0][3]), diff_angle(c_angle[0], init_angle[0])))
	dist_pitch = abs(diff_angle(np.deg2rad(goal_list[0][4]), diff_angle(c_angle[1], init_angle[1])))
	dist_yaw = abs(diff_angle(np.deg2rad(goal_list[0][5]), diff_angle(c_angle[2], init_angle[2])))
	sum_dist = dist_roll + dist_pitch + dist_yaw
	if not first and \
			(np.linalg.norm(np.array([goal_list[0][0], goal_list[0][1], goal_list[0][2]]) - np.array(c_pose) + np.array(
				init_loc[0:3])) > 0.8 \
			 or sum_dist > 0.6):
		return goal_list

	if not first:
		goal_list.pop(0)
	if len(goal_list) == 0:
		return []

	omni.kit.commands.execute('ChangeProperty',
	                          prop_path=Sdf.Path(f'{path}/base_link/x_joint.drive:linear:physics:stiffness'),
	                          value=1200.0,
	                          prev=0.0)
	omni.kit.commands.execute('ChangeProperty',
	                          prop_path=Sdf.Path(f'{path}/base_link/x_joint.drive:linear:physics:damping'),
	                          value=1000.0,
	                          prev=0.0)
	omni.kit.commands.execute('ChangeProperty',
	                          prop_path=Sdf.Path(f'{path}/base_link/x_joint.drive:linear:physics:maxForce'),
	                          value=500.0,
	                          prev=0.0)
	omni.kit.commands.execute('ChangeProperty',
	                          prop_path=Sdf.Path(f'{path}/base_link/x_joint.physxJoint:maxJointVelocity'),
	                          value=200.0,  # cm/s
	                          prev=0.0)
	omni.kit.commands.execute('ChangeProperty',
	                          prop_path=Sdf.Path(f'{path}/base_link/x_joint.drive:linear:physics:targetPosition'),
	                          value=(goal_list[0][0]) / meters_per_unit,
	                          prev=0.0)

	omni.kit.commands.execute('ChangeProperty',
	                          prop_path=Sdf.Path(f'{path}/x_link/y_joint.drive:linear:physics:stiffness'),
	                          value=1200.0,
	                          prev=0.0)
	omni.kit.commands.execute('ChangeProperty',
	                          prop_path=Sdf.Path(f'{path}/x_link/y_joint.drive:linear:physics:damping'),
	                          value=1000.0,
	                          prev=0.0)
	omni.kit.commands.execute('ChangeProperty',
	                          prop_path=Sdf.Path(f'{path}/x_link/y_joint.drive:linear:physics:maxForce'),
	                          value=500.0,
	                          prev=0.0)
	omni.kit.commands.execute('ChangeProperty',
	                          prop_path=Sdf.Path(f'{path}/x_link/y_joint.physxJoint:maxJointVelocity'),
	                          value=200.0,
	                          prev=0.0)
	omni.kit.commands.execute('ChangeProperty',
	                          prop_path=Sdf.Path(f'{path}/x_link/y_joint.drive:linear:physics:targetPosition'),
	                          value=(goal_list[0][1]) / meters_per_unit,
	                          prev=0.0)

	omni.kit.commands.execute('ChangeProperty',
	                          prop_path=Sdf.Path(f'{path}/y_link/z_joint.drive:linear:physics:stiffness'),
	                          value=1200.0,
	                          prev=0.0)
	omni.kit.commands.execute('ChangeProperty',
	                          prop_path=Sdf.Path(f'{path}/y_link/z_joint.drive:linear:physics:damping'),
	                          value=1000.0,
	                          prev=0.0)
	omni.kit.commands.execute('ChangeProperty',
	                          prop_path=Sdf.Path(f'{path}/y_link/z_joint.drive:linear:physics:maxForce'),
	                          value=500.0,
	                          prev=0.0)
	omni.kit.commands.execute('ChangeProperty',
	                          prop_path=Sdf.Path(f'{path}/y_link/z_joint.physxJoint:maxJointVelocity'),
	                          value=200.0,
	                          prev=0.0)
	omni.kit.commands.execute('ChangeProperty',
	                          prop_path=Sdf.Path(f'{path}/y_link/z_joint.drive:linear:physics:targetPosition'),
	                          value=(goal_list[0][2]) / meters_per_unit,
	                          prev=0.0)

	omni.kit.commands.execute('ChangeProperty',
	                          prop_path=Sdf.Path(f'{path}/z_link/roll_joint.drive:angular:physics:stiffness'),
	                          value=1200.0,
	                          prev=0.0)
	omni.kit.commands.execute('ChangeProperty',
	                          prop_path=Sdf.Path(f'{path}/z_link/roll_joint.drive:angular:physics:damping'),
	                          value=1000.0,
	                          prev=0.0)
	omni.kit.commands.execute('ChangeProperty',
	                          prop_path=Sdf.Path(f'{path}/z_link/roll_joint.drive:angular:physics:maxForce'),
	                          value=300.0,
	                          prev=0.0)
	omni.kit.commands.execute('ChangeProperty',
	                          prop_path=Sdf.Path(f'{path}/z_link/roll_joint.physxJoint:maxJointVelocity'),
	                          value=0.2,
	                          prev=0.0)
	omni.kit.commands.execute('ChangeProperty',
	                          prop_path=Sdf.Path(f'{path}/z_link/roll_joint.drive:angular:physics:targetPosition'),
	                          value=(goal_list[0][3]),
	                          prev=0.0)

	omni.kit.commands.execute('ChangeProperty',
	                          prop_path=Sdf.Path(f'{path}/roll_link/pitch_joint.drive:angular:physics:stiffness'),
	                          value=1200.0,
	                          prev=0.0)
	omni.kit.commands.execute('ChangeProperty',
	                          prop_path=Sdf.Path(f'{path}/roll_link/pitch_joint.drive:angular:physics:damping'),
	                          value=1000.0,
	                          prev=0.0)
	omni.kit.commands.execute('ChangeProperty',
	                          prop_path=Sdf.Path(f'{path}/roll_link/pitch_joint.drive:angular:physics:maxForce'),
	                          value=300.0,
	                          prev=0.0)
	omni.kit.commands.execute('ChangeProperty',
	                          prop_path=Sdf.Path(f'{path}/roll_link/pitch_joint.physxJoint:maxJointVelocity'),
	                          value=0.2,
	                          prev=0.0)
	omni.kit.commands.execute('ChangeProperty',
	                          prop_path=Sdf.Path(f'{path}/roll_link/pitch_joint.drive:angular:physics:targetPosition'),
	                          value=(goal_list[0][4]),
	                          prev=0.0)

	omni.kit.commands.execute('ChangeProperty',
	                          prop_path=Sdf.Path(f'{path}/pitch_link/yaw_joint.drive:angular:physics:stiffness'),
	                          value=1200.0,
	                          prev=0.0)
	omni.kit.commands.execute('ChangeProperty',
	                          prop_path=Sdf.Path(f'{path}/pitch_link/yaw_joint.drive:angular:physics:damping'),
	                          value=1000.0,
	                          prev=0.0)
	omni.kit.commands.execute('ChangeProperty',
	                          prop_path=Sdf.Path(f'{path}/pitch_link/yaw_joint.drive:angular:physics:maxForce'),
	                          value=300.0,
	                          prev=0.0)
	omni.kit.commands.execute('ChangeProperty',
	                          prop_path=Sdf.Path(f'{path}/pitch_link/yaw_joint.physxJoint:maxJointVelocity'),
	                          value=1.3,
	                          prev=0.0)
	omni.kit.commands.execute('ChangeProperty',
	                          prop_path=Sdf.Path(f'{path}/pitch_link/yaw_joint.drive:angular:physics:targetPosition'),
	                          value=(goal_list[0][5]),
	                          prev=0.0)
	return goal_list


def add_irotate_ros_components(camera_odom_frames, camera_odom_pubs, lidar_components, robot_base_prim_path, n):
	"""
	Add the irotate-specific ros-components to the robot.
	"""
	camera_odom_frames.append(f"{robot_base_prim_path}{n}/cameraholder_link")
	camera_odom_pubs.append(rospy.Publisher(f"{robot_base_prim_path}{n}/camera_odom", Odometry, queue_size=10))

	lidar_components.append(add_lidar(f"{robot_base_prim_path}{n}/lasersensor_link"), is_2d = True, is_3d=False)


def add_lidar(path, translation=[0, 0, 0], orientation=[0, 0, 0], is_2d=True, is_3d=False, degrees=True):

	# drive sim applies 0.5,-0.5,-0.5,w(-0.5), we have to apply the reverse
	base_or = tf.Rotation.from_quat([0.5, -0.5, -0.5, -0.5])
	orientation = tf.Rotation.from_euler('XYZ', orientation, degrees=degrees)
	orientation = (base_or * orientation).as_quat()

	success, sensor = omni.kit.commands.execute(
		"IsaacSensorCreateRtxLidar",
		path="/RTX_Lidar",
		parent=path,
		config="Example_Rotary",
		translation=(translation[0], translation[1], translation[2]),
		orientation=Gf.Quatd(orientation[3], orientation[0], orientation[1], orientation[2]),  # Gf.Quatd is w,i,j,k
	)

	omni.kit.app.get_app().update()
	omni.kit.app.get_app().update()
	omni.kit.app.get_app().update()
	render_product_path = rep.create.render_product(sensor.GetPath().pathString, resolution=(1, 1))
	# _, render_product_path = create_hydra_texture([1, 1], sensor.GetPath().pathString)
	omni.kit.app.get_app().update()
	omni.kit.app.get_app().update()
	# add the lidar to the graph
	# config is isaac_sim-2022.2.1/exts/omni.sensors.nv.lidar/data/Example_Rotary.json
	if is_3d:
		writer = rep.writers.get("RtxLidar" + "ROS1PublishPointCloud")
		writer.initialize(topicName=f"{path}/lidar/point_cloud", frameId=path[1:])
		writer.attach([render_product_path])
	if is_2d:
		writer = rep.writers.get("RtxLidar" + "ROS1PublishLaserScan")
		writer.initialize(topicName=f"{path}/lidar/laser_scan", frameId=path[1:], rotationRate=100,
		                  horizontalFov=360, depthRange=[0.1,10000], horizontalResolution=0.1)
		writer.attach([render_product_path])



	# todo for lidar  one can change directly /Render/PostProcess/SDGPipeline/RenderProduct_Isaac_RtxSensorCpuIsaacComputeRTXLidarFlatScan
	# but NOT for the 3d lidar
	# todo theoretically I can avoid returning anything making just sure that I render at each loop

	return omni.syntheticdata.SyntheticData._get_node_path(
				"PostProcessDispatch" + "IsaacSimulationGate", render_product_path
			)
def add_npy_viewport(viewport_window_list, robot_base_prim_path, n, old_h_ape, old_v_ape, config, sc,
                     tot_num_ros_cam=1):
	viewport_npy, _ = create_viewport(f"{robot_base_prim_path}{n}/camera_link/Camera_npy", config["headless"].get(),
	                                  tot_num_ros_cam + 1 * n, config["npy_sensor_size"].get(), old_h_ape, old_v_ape, sc)
	viewport_window_list.append(viewport_npy)


def change_joint_limit(joint: str, limit):
	omni.kit.commands.execute('ChangeProperty',
	                          prop_path=Sdf.Path(f'{joint}'),
	                          value=(limit),
	                          prev=0.0)
