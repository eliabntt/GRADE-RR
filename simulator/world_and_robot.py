import argparse

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

# new options
parser.add_argument("--config_file", type=str, default="config.yaml")
parser.add_argument("--fix_env", type=str, default="",
	                    help="leave it empty to have a random env, fix it to use a fixed one. Useful for loop processing")

args, unknown = parser.parse_known_args()
config = confuse.Configuration("world_and_robot", __name__)

# load the config file specified 
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

for _ in range(100):
    simulation_context.render()
    simulation_context.step(render=False)

import utils.misc_utils
from utils.misc_utils import *
from utils.robot_utils import *
from utils.simulation_utils import *

simulation_environment_setup(need_ros = False) # enable some extensions, check if ros is running automatically

if base_world_path != "":
	from utils.environment_utils import *
	print("Loading environment...")
	environment = environment(config, meters_per_unit=meters_per_unit) # setup the class
	env_prim_path = environment.load_and_center(config["env_prim_path"].get()) # actually load the env
	process_semantics(config["env_prim_path"].get()) # add semantic information based either on label you provide, or looking into fields of the objcets. This applies semantic to all childs

	print("Visualization...")
	for _ in range(1000):
		simulation_context.render()
	    simulation_context.step(render=False)
	print("Environment loading done...")
	print("Add colliders to the environment, if the environment is big this could take ages..")
	add_colliders(env_prim_path) # add colliders to the environment
	print("Colliders added..")
	print("For the next step please check out the code and set x, y, z manually to test them out..")
	print()
	ipdb.set_trace()
	simulation_context.play()
	x, y, z = 0, 0, 0
	if out_dir == "":
		print("Change out_dir")
	environment.generate_map(out_dir, origin=[x,y,z])
	print("Map generated..")

simulation_context.stop()

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

print("Loading robots done")
simulation_context.play()
for _ in range(2000):
	simulation_context.render()
	simulation_context.step(render=False)
simulation_context.stop()

try:
	kit.close()
except:
	pass
