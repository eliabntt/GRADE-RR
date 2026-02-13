import time

import grade_utils.misc_utils
from grade_utils.misc_utils import *

GRAPH_PATH = "/Render/PostProcess/SDGPipeline"


from isaacsim.core.utils.extensions import enable_extension, disable_extension
import carb

def simulation_environment_setup(need_ros=True):
	"""
	Enable the necessary extensions that will be used within the simulation
	"""
	# Update extension names to new isaacsim.* equivalents where possible
	enable_extension("isaacsim.ros_bridge")
	enable_extension("isaacsim.physics_inspector")
	enable_extension("isaacsim.physics_utilities")
	enable_extension("omni.anim.skelJoint")
	enable_extension("omni.kit.window.sequencer")
	enable_extension("isaacsim.dynamic_control")
	enable_extension("isaacsim.shapenet")
	enable_extension("semantics.schema.editor")
	enable_extension("omni.hydra.iray")
	enable_extension("omni.iray.settings.core")
	enable_extension('isaacsim.occupancy_map')
	enable_extension('isaacsim.shapenet')
	enable_extension('isaacsim.range_sensor')
	disable_extension('isaacsim.sun_study')
	enable_extension('isaacsim.core_nodes')
	enable_extension('isaacsim.sensor')
	# Necessary ONLY if using NUCLEUS
	# Locate /Isaac folder on nucleus server to load sample
	# from isaacsim.core.utils.nucleus import get_assets_root_path
	# nucleus_server = get_assets_root_path()
	# if nucleus_server is None:
	#     carb.log_error("Could not find nucleus server with /Isaac folder, exiting")
	#     exit()
	# if need_ros:
	#     if not rosgraph.is_master_online():
	#         carb.log_error("Please run roscore before executing this script")
	#         exit()
