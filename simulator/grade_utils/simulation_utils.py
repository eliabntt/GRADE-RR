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
	try:
		try:
			enable_extension("omni.isaac.ros_bridge")
		except Exception as e:
			print("[INFO] omni.isaac.ros_bridge extension not available or failed to enable:", e)
	except Exception as e:
		print("[INFO] omni.isaac.ros_bridge extension not available or failed to enable:", e)
	enable_extension("omni.isaac.physics_inspector")
	enable_extension("omni.isaac.physics_utilities")
	enable_extension("omni.anim.skelJoint")
	enable_extension("omni.kit.window.sequencer")
	enable_extension("omni.isaac.dynamic_control")
	try:
		try:
			enable_extension("omni.isaac.shapenet")
		except Exception as e:
			print("[INFO] omni.isaac.shapenet extension not available or failed to enable:", e)
	except Exception as e:
		print("[INFO] omni.isaac.shapenet extension not available or failed to enable:", e)
	enable_extension("semantics.schema.editor")
	enable_extension("omni.hydra.iray")
	enable_extension("omni.iray.settings.core")
	enable_extension('omni.isaac.occupancy_map')
	try:
		try:
			enable_extension('omni.isaac.shapenet')
		except Exception as e:
			print("[INFO] omni.isaac.shapenet extension not available or failed to enable:", e)
	except Exception as e:
		print("[INFO] omni.isaac.shapenet extension not available or failed to enable:", e)
	enable_extension('omni.isaac.range_sensor')
	disable_extension('omni.isaac.sun_study')
	enable_extension('omni.isaac.core_nodes')
	enable_extension('omni.isaac.sensor')
	# Necessary ONLY if using NUCLEUS
	# Locate /Isaac folder on nucleus server to load sample
	# from omni.isaac.core.utils.nucleus import get_assets_root_path
	# nucleus_server = get_assets_root_path()
	# if nucleus_server is None:
	#     carb.log_error("Could not find nucleus server with /Isaac folder, exiting")
	#     exit()
	# if need_ros:
	#     if not rosgraph.is_master_online():
	#         carb.log_error("Please run roscore before executing this script")
	#         exit()
