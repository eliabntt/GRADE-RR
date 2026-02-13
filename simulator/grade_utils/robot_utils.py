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

# ...rest of the file unchanged...
