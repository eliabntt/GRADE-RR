import argparse
import time
import os
import numpy as np

# base_env_path and other settings are in the config file
out_dir = "" # set this to a temporary empty dir

from omni.isaac.kit import SimulationApp

def boolean_string(s):
	if s.lower() not in {'false', 'true'}:
		raise ValueError('Not a valid boolean string')
	return s.lower() == 'true'
parser = argparse.ArgumentParser(description="Your second IsaacSim run")
parser.add_argument("--headless", type=boolean_string, default=True, help="Wheter to run it in headless mode or not")
parser.add_argument("--rtx_mode", type=boolean_string, default=False, help="Use rtx when True, use path tracing when False")
parser.add_argument("--config_file", type=str, default="config.yaml")
parser.add_argument("--fix_env", type=str, default="",
	                    help="leave it empty to have a random env, fix it to use a fixed one. Useful for loop processing")

args, unknown = parser.parse_known_args()
config = confuse.Configuration("world_and_robot", __name__)
config.set_file(args.config_file) 
config.set_args(args)

CONFIG = {"display_options": 3286, "width": 1280, "height": 720, "headless": config["headless"].get()}
kit = SimulationApp(launch_config=CONFIG, experience=f"{os.environ['EXP_PATH']}/omni.isaac.sim.python.kit")

omni.usd.get_context().open_stage(config["base_env_path"].get(), None)
kit.update()
kit.update()

print("Loading stage...")
while is_stage_loading():
    kit.update()
print("Loading Complete")

context = omni.usd.get_context()
stage = context.get_stage() 

meters_per_unit = config["meters_per_unit"].get()
simulation_context = SimulationContext(physics_dt=1.0 / config["physics_hz"].get(), rendering_dt=1.0 / config["render_hz"].get(), stage_units_in_meters=meters_per_unit, backend='torch')
simulation_context.initialize_physics()
physx_interface = omni.physx.acquire_physx_interface()
physx_interface.start_simulation()

print("Adding ROS clock, you can check with rostopic echo /clock")
_clock_graph = add_clock()  # add ROS clock
simulation_context.play()
for _ in range(10):
	simulation_context.step() # remember that this step also the physics
	og.Controller.evaluate_sync(_clock_graph)
simulation_context.stop()

import utils.misc_utils
from utils.misc_utils import *
from utils.robot_utils import *
from utils.simulation_utils import *

simulation_environment_setup(need_ros = True) 
if base_world_path != "":
	from utils.environment_utils import *
	print("Loading environment...")
	environment = environment(config, meters_per_unit=meters_per_unit) 
	env_prim_path = environment.load_and_center(config["env_prim_path"].get())
	process_semantics(config["env_prim_path"].get())

	print("Visualization...")
	for _ in range(1000):
		simulation_context.render()
	    simulation_context.step(render=False)
	print("Environment loading done...")
	add_colliders(env_prim_path)
	print("Colliders added..")
	simulation_context.play()
	x, y, z = 0, 0, 0
	if out_dir != "":
		environment.generate_map(out_dir, origin=[x,y,z])
	print("Map generated..")

simulation_context.stop()

# prepare some containers
joint_states = [] 
tf_trees = []
camera_list = []
viewport_list = []
camera_pose, camera_pose_pub = [], []
imus,imu_pubs = [], []
lidars = []
odoms, odom_pubs = [], []

# get the interface to add imu sensors
from omni.isaac.sensor import _sensor
_is = _sensor.acquire_imu_sensor_interface()

# these are kept because the aperture is resetted based on the h aperture by IsaacSim. 
# In v2021 this could have been reverted. In v2022 not.
old_h_ape, old_v_ape = [], [] 


# get the interface to access dynamics of the assets
_dc = dynamic_control_interface()

print("Loading robots..")
robot_base_prim_path = config["robot_base_prim_path"].get()
usd_robot_path = str(config["usd_robot_path"].get())
for n in range(config["num_robots"].get()):
	import_robot(robot_base_prim_path, n, usd_robot_path)
	x, y, z, yaw = np.random.randint(-100,100,4)
    set_drone_joints_init_loc(f"{robot_base_prim_path}{n}",
				[x / meters_per_unit, y / meters_per_unit, z / meters_per_unit],
               	[0, 0, np.deg2rad(yaw)],
				upper_zlim = z * 2,
				lower_zlim = -z * 2
				)

	print("Adding ROS components")
	joint_states.append(add_joint_state(f"{robot_base_prim_path}{n}"))
	tf_trees.append(add_pose_tree(f"{robot_base_prim_path}{n}"))

	# create the viewport, the camera component
	component, viewport = add_camera_and_viewport(f"{robot_base_prim_path}{n}/camera_link",
	                                              config["robot_sensor_size"].get(),
	                                              old_h_ape, old_v_ape, simulation_context,
	                                              0, n, cam_per_robot=1)  # cam index is useful if you want multiple cameras
	cam_outputs = control_camera(viewport, simulation_context)
	camera_list.append([n + 0, component, cam_outputs])
	viewport_list.append(viewport)
	omni.kit.app.get_app().update()

	camera_pose.append(f"{robot_base_prim_path}{n}/camera_link")
	camera_pose_pub.append(rospy.Publisher(f"{robot_base_prim_path}{n}/camera/pose", PoseStamped, queue_size=10))

	setup_imu_sensor(_is, config, f"{robot_base_prim_path}{n}/imu_link")
	imu_pubs.append(rospy.Publisher(f"{robot_base_prim_path}{n}/imu_body", Imu, queue_size=10))
	imus.append(f"{robot_base_prim_path}{n}/imu_link")

	odoms.append(f"{robot_base_prim_path}{n}/yaw_link")
	odom_pubs.append(rospy.Publisher(f"{robot_base_prim_path}{n}/odom", Odometry, queue_size=10))
	
	sensor = add_lidar(f"{robot_base_prim_path}{n}/yaw_link", [0, 0, -.1], [0, 0, 0], is_3d=True, is_2d=True)
	lidars.append(sensor)

	# alternatively 
	# add_ros_components(robot_base_prim_path, n, ros_transform_components, camera_list, viewport_list,
    #                    camera_pose, camera_pose_pub, imu_pubs, imus,
    #                    odoms, odom_pubs, lidars,
    #                    [], config, old_h_ape, old_v_ape, _is, simulation_context, _clock, irotate=False):

print("Loading robots done")

# set some settings for the rendering
if (config["rtx_mode"].get()):
	set_raytracing_settings(config["physics_hz"].get())
else:
	set_pathtracing_settings(config["physics_hz"].get())

print("Note that the rendering is now blocking until finished")
for i in range(100):
	print(f"Iteration {i}/100", end="\r")
	sleeping(simulation_context, viewport_list, raytracing=config["rtx_mode"].get())

# deselect all objects
omni.usd.get_context().get_selection().clear_selected_prim_paths()
omni.usd.get_context().get_selection().set_selected_prim_paths([], False)

simulation_context.play()
for i in range(2000):
	simulation_context.step(render=False)
	og.Controller.evaluate_sync(_clock)
	time.sleep(0.2)
	simulation_context.render()
	
	# publish IMU
	print("Publishing IMU...")
	pub_imu(_is, imu_pubs, robot_imu_frames, meters_per_unit)

	if i % ratio_joints == 0:
		for js in joint_states:
			og.Controller.set(og.Controller.attribute(f"{js}/OnImpulseEvent.state:enableImpulse"), True)
	if i % ratio_tf:
		for tf in tf_trees:
			og.Controller.set(og.Controller.attribute(f"{tf}/OnImpulseEvent.state:enableImpulse"), True)
	if simulation_step % ratio_odom == 0:
		c_pose, _ = pub_odom(odoms, odom_pubs, _dc, meters_per_unit)
		pub_cam_pose(camera_pose, camera_pose_pub, _dc, meters_per_unit)

	if simulation_step % ratio_camera == 0:
		# The RTX LiDAR is still a fuzzy component. The "normal" LiDAR is more stable, but won't see non-colliding objects
		for lidar in lidars:
			og.Controller.attribute(lidar+".inputs:step").set(1)
		ctime = timeline.get_current_time()
		simulation_context.render()
		timeline.set_current_time(ctime)
		for lidar in lidars:
			og.Controller.attribute(lidar+".inputs:step").set(0)

		pub_and_write_images(simulation_context, viewport_list, ros_camera_list, raytracing) # clearly not writing anything here
simulation_context.stop()

try:
	kit.close()
except:
	pass
