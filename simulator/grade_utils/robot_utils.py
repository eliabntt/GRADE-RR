import grade_utils.misc_utils
from grade_utils.misc_utils import *

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

def import_robot(robot_base_prim_path: str, n: int, usd_robot_path: str, local_file_prefix: str = ""):
	"""
	Import robot USD as a reference under `${robot_base_prim_path}{n}`.
	"""
	asset_path = usd_robot_path
	if local_file_prefix and isinstance(asset_path, str) and asset_path.startswith("/"):
		asset_path = local_file_prefix + asset_path
	path_to = f"{robot_base_prim_path}{n}"
	omni.kit.commands.execute(
		"CreateReferenceCommand",
		usd_context=omni.usd.get_context(),
		path_to=path_to,
		asset_path=asset_path,
		instanceable=False,
	)
	clear_properties(path_to)
	return path_to


def set_drone_joints_init_loc(name: str, pos: list, orientation: list, upper_zlim: float = 100, lower_zlim: float = 0,
							  irotate: bool = False):
	"""
	Compatibility shim for legacy drone joint initialization.
	"""
	stage = omni.usd.get_context().get_stage()
	prim = stage.GetPrimAtPath(name)
	if prim and prim.IsValid():
		set_translate(prim, pos)
		set_rotate(prim, orientation)


class _ViewportShim:
	def __init__(self, camera_path: str):
		self._camera_path = camera_path

	def get_active_camera(self):
		return self._camera_path

	def set_active_camera(self, camera_path: str):
		self._camera_path = camera_path


def add_npy_viewport(viewport_window_list: list, robot_base_prim_path: str, n: int, old_h_ap: list, old_v_ap: list,
					 config, simulation_context, tot_num_ros_cam: int = 0):
	"""
	Create a lightweight viewport/camera shim for numpy recorder workflows.
	"""
	stage = omni.usd.get_context().get_stage()
	camera_path = f"{robot_base_prim_path}{n}/camera"
	cam_prim = stage.GetPrimAtPath(camera_path)
	if cam_prim is None or not cam_prim.IsValid() or cam_prim.GetTypeName().lower() != "camera":
		camera_path = f"/World/RecorderCamera_{n}"
		cam_prim = UsdGeom.Camera.Define(stage, camera_path).GetPrim()

	cam_prim.GetAttribute("horizontalAperture").Set(
		cam_prim.GetAttribute("horizontalAperture").Get() or 20.955
	)
	cam_prim.GetAttribute("verticalAperture").Set(
		cam_prim.GetAttribute("verticalAperture").Get() or 15.2908
	)

	old_h_ap.append(cam_prim.GetAttribute("horizontalAperture").Get())
	old_v_ap.append(cam_prim.GetAttribute("verticalAperture").Get())
	viewport_window_list.append(_ViewportShim(camera_path))

