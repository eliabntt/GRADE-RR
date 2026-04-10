import time

import grade_utils.misc_utils
from grade_utils.misc_utils import *

GRAPH_PATH = "/Render/PostProcess/SDGPipeline"



from isaacsim.core.utils.extensions import enable_extension, disable_extension
import carb


def _safe_enable_extension(name):
	try:
		enable_extension(name)
	except Exception as e:
		print(f"[INFO] {name} extension not available or failed to enable: {e}")


def _safe_disable_extension(name):
	try:
		disable_extension(name)
	except Exception:
		pass

def simulation_environment_setup(need_ros=True, need_sequencer=False, need_shapenet=True):
	"""
	Enable the necessary extensions that will be used within the simulation
	"""
	if need_ros:
		_safe_enable_extension("omni.isaac.ros_bridge")

	_safe_enable_extension("omni.isaac.physics_inspector")
	_safe_enable_extension("omni.isaac.physics_utilities")
	_safe_enable_extension("omni.anim.skelJoint")

	if need_sequencer:
		# Isaac Sim 5.1 exposes sequencer through core/usd extensions.
		_safe_enable_extension("omni.kit.sequencer.usd")
		_safe_enable_extension("omni.kit.sequencer.core")

	_safe_enable_extension("omni.isaac.dynamic_control")
	if need_shapenet:
		_safe_enable_extension("omni.isaac.shapenet")
	_safe_enable_extension("semantics.schema.editor")
	_safe_enable_extension("omni.hydra.iray")
	_safe_enable_extension("omni.iray.settings.core")
	_safe_enable_extension('omni.isaac.occupancy_map')
	_safe_enable_extension('omni.isaac.range_sensor')
	_safe_disable_extension('omni.isaac.sun_study')
	_safe_enable_extension('omni.isaac.core_nodes')
	_safe_enable_extension('omni.isaac.sensor')
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
