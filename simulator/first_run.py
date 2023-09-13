import argparse

base_environment_path = "" # please edit this e.g. GRADE-RR/usds/env_base.usd

# necessary import
from omni.isaac.kit import SimulationApp

# simply use this to correctly parse booleans
def boolean_string(s):
	if s.lower() not in {'false', 'true'}:
		raise ValueError('Not a valid boolean string')
	return s.lower() == 'true'

parser = argparse.ArgumentParser(description="Your first IsaacSim run")
parser.add_argument("--headless", type=boolean_string, default=True, help="Wheter to run it in headless mode or not")
parser.add_argument("--rtx_mode", type=boolean_string, default=False, help="Use rtx when True, use path tracing when False")

args, unknown = parser.parse_known_args()
config = confuse.Configuration("first_run", __name__)
config.set_args(args)

# create a kit object which is your Simulation App
CONFIG = {"display_options": 3286, "width": 1280, "height": 720, "headless": config["headless"].get()}
kit = SimulationApp(launch_config=CONFIG, experience=f"{os.environ['EXP_PATH']}/omni.isaac.sim.python.kit")

# !!! you can ONLY load Isaac modules AFTER this point !!!
# after here you can do everything that you desire

# first step is usually opening a basic stage, perhaps with some assets already in as the sky
omni.usd.get_context().open_stage(base_environment_path, None)

# Wait two frames so that stage starts loading
kit.update()
kit.update()

print("Loading stage...")
while is_stage_loading():
    kit.update()
print("Loading Complete")

context = omni.usd.get_context()
stage = context.get_stage() # used to access the elements of the simulation

simulation_context = SimulationContext(physics_dt=1.0 / 60, rendering_dt=1.0 / 60, stage_units_in_meters=0.01, backend='torch')
simulation_context.initialize_physics()
physx_interface = omni.physx.acquire_physx_interface()
physx_interface.start_simulation()

for _ in range(100):
    simulation_context.render()
    simulation_context.step(render=False)

try:
	kit.close()
except:
	pass
