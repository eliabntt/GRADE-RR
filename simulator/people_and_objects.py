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
_clock_graph = add_clock()
simulation_context.play()
for _ in range(10):
	simulation_context.step()
	og.Controller.evaluate_sync(_clock_graph)
simulation_context.stop()

import utils.misc_utils
from utils.misc_utils import *
from utils.robot_utils import *
from utils.simulation_utils import *
from utils.objects_utils import *
from utils.human_utils import *

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

ros_transform_components = []
camera_list = []
viewport_list = []
camera_pose, camera_pose_pub = [], []
imus,imu_pubs = [], []
lidars = []
odoms, odom_pubs = [], []

from omni.isaac.sensor import _sensor
_is = _sensor.acquire_imu_sensor_interface()

old_h_ape, old_v_ape = [], [] 

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
	add_ros_components(robot_base_prim_path, n, ros_transform_components, camera_list, viewport_list,
                        camera_pose, camera_pose_pub, imu_pubs, imus,
                        odoms, odom_pubs, lidars,
                        [], config, old_h_ape, old_v_ape, _is, simulation_context, _clock, irotate=False)
	kit.update()

timeline = setup_timeline(config) # setup the timeline before adding anything animated

print("Loading people")
n = 0
human_base_prim_path = config["human_base_prim_path"].get()
while n < config["num_humans"].get():
	folder = rng.choice(human_folders)
	random_name = rng.choice(os.listdir(os.path.join(human_export_folder, folder)))
	asset_path = os.path.join(human_export_folder, folder, random_name, random_name + ".usd")

	print("Loading human {} from {}".format(random_name, folder))

	tmp_pkl = pkl.load(open(os.path.join(human_export_folder, folder, random_name, random_name + ".pkl"), 'rb'))
	used_ob_stl_paths.append(os.path.join(human_export_folder, folder, random_name, random_name + ".stl"))

	load_human(human_base_prim_path, n, asset_path)
	stl_path = os.path.join(human_export_folder, folder, random_name, random_name + ".stl")
	
	x = np.random.randint(environment.env_limits_shifted[0], environment.env_limits_shifted[3])
	y = np.random.randint(environment.env_limits_shifted[1], environment.env_limits_shifted[4])
	z = 0
	yaw = np.random.randint(0,360)
	# position the mesh
	set_translate(stage.GetPrimAtPath(f"{human_base_prim_path}{n}"),
			              [x / meters_per_unit, y / meters_per_unit, z / meters_per_unit])
	set_scale(stage.GetPrimAtPath(f"{human_base_prim_path}{n}"), 1 / meters_per_unit)
	set_rotate(stage.GetPrimAtPath(f"{human_base_prim_path}{n}"), [0, 0, np.deg2rad(yaw)])
	
	n += 1
	
print("Load objects")
google_ob_used, shapenet_ob_used = load_objects(config, environment, np.random.default_rng(), [], 1/meters_per_unit)

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


timeline.set_current_time(0)
timeline.set_auto_update(False) # this no longer works as expected.
# Theoretically, once this is set and the timeline plays, rendering will not advance the timeline
# this is no longer the case. Thus, keep track of the ctime (as we do within sleeping function)
# the simulation context can be kept stopped, but that will prevent physics and time to advance.
# https://forums.developer.nvidia.com/t/the-timeline-set-auto-update-false-no-longer-works/253504/10
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
		timeline.forward_one_frame() # advancing the timeline
simulation_context.stop()

try:
	kit.close()
except:
	pass
