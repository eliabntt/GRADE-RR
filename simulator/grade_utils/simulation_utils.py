import time
from typing import Dict, Union

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


def set_common_stage_properties(rate):
	"""
	Set conservative renderer/runloop properties for deterministic stepping.
	"""
	settings = carb.settings.get_settings()
	_desired_render_settings: Dict[str, Union[bool, int, float]] = {
		"/app/asyncRendering": False,
		"/app/renderer/skipWhileMinimized": False,
		"/app/renderer/sleepMsOnFocus": 0,
		"/app/renderer/sleepMsOutOfFocus": 0,
		"/app/runLoops/main/rateLimitEnabled": True,
		"/app/runLoops/main/rateLimitFrequency": float(rate),
		"/persistent/simulation/minFrameRate": float(rate),
		"/app/runLoops/main/rateLimitUseBusyLoop": True,
	}
	for setting_name, setting_value in _desired_render_settings.items():
		try:
			settings.set(setting_name, setting_value)
		except Exception:
			pass


def set_raytracing_settings(physics_hz):
	"""
	Configure RTX real-time mode.
	"""
	settings = carb.settings.get_settings()
	set_common_stage_properties(physics_hz)
	for key, value in {
		"/rtx/rendermode": "RayTracedLighting",
		"/rtx/pathtracing/enabled": False,
		"/rtx/post/aa/op": 3,
	}.items():
		try:
			settings.set(key, value)
		except Exception:
			pass


def set_pathtracing_settings(physics_hz):
	"""
	Configure path tracing mode with stable accumulation defaults.
	"""
	settings = carb.settings.get_settings()
	set_common_stage_properties(physics_hz)
	for key, value in {
		"/rtx/rendermode": "PathTracing",
		"/rtx/pathtracing/enabled": True,
		"/rtx/pathtracing/spp": 1,
		"/rtx/pathtracing/totalSpp": 64,
	}.items():
		try:
			settings.set(key, value)
		except Exception:
			pass


class _TimelineCompat:
	def __init__(self, timeline_iface, time_codes_per_seconds: float):
		self._timeline = timeline_iface
		self._tps = float(time_codes_per_seconds)

	def set_current_time(self, value):
		if hasattr(self._timeline, "set_current_time"):
			return self._timeline.set_current_time(value)

	def set_auto_update(self, value):
		if hasattr(self._timeline, "set_auto_update"):
			return self._timeline.set_auto_update(value)

	def get_time_codes_per_seconds(self):
		if hasattr(self._timeline, "get_time_codes_per_seconds"):
			return self._timeline.get_time_codes_per_seconds()
		return self._tps

	def play(self):
		if hasattr(self._timeline, "play"):
			return self._timeline.play()

	def stop(self):
		if hasattr(self._timeline, "stop"):
			return self._timeline.stop()


def setup_timeline(config):
	"""
	Prepare timeline and stage time codes.
	"""
	import omni.timeline

	timeline = omni.timeline.get_timeline_interface()
	stage = omni.usd.get_context().get_stage()

	def _cfg(name, default):
		try:
			value = config[name].get()
			return default if value is None else value
		except Exception:
			return default

	time_codes_per_second = float(_cfg("physics_hz", 60.0))
	experiment_length = float(_cfg("experiment_length", 60.0))

	try:
		stage.SetStartTimeCode(0)
		stage.SetTimeCodesPerSecond(time_codes_per_second)
		stage.SetEndTimeCode(max(1.0, experiment_length * time_codes_per_second))
	except Exception:
		pass

	return _TimelineCompat(timeline, time_codes_per_second)


def compute_timeline_ratio(human_anim_len, reverse_strategy, experiment_length):
	if len(human_anim_len) == 0:
		return 1.0
	max_len = max(human_anim_len)
	if max_len <= 0:
		return 1.0
	if reverse_strategy:
		return float(experiment_length) / float(max_len)
	return 1.0


class _NullRecorder:
	def __init__(self, output_dir: str, enabled: bool, counter: int = 0):
		self._output_dir = output_dir
		self._enable_record = bool(enabled)
		self._counter = int(counter)

	def _update(self):
		return

	def record(self):
		return

	def finish(self):
		return


def recorder_setup(settings_cfg, out_dir_npy, enable_record, counter=0):
	"""
	Create recorder if available, otherwise return a no-op recorder.
	"""
	if extension_custom is None:
		return _NullRecorder(out_dir_npy, enable_record, counter)
	try:
		recorder = extension_custom.SyntheticRecorderExtensionCustom()
		if hasattr(recorder, "set_config"):
			recorder.set_config(settings_cfg)
		if hasattr(recorder, "set_output_dir"):
			recorder.set_output_dir(out_dir_npy)
		recorder._enable_record = bool(enable_record)
		recorder._counter = int(counter)
		return recorder
	except Exception:
		return _NullRecorder(out_dir_npy, enable_record, counter)


def pub_and_write_images(simulation_context, viewport_window_list, ros_camera_list, raytracing, my_recorder=None,
							 enable_recorder=True):
	"""
	Render/publish one frame and optionally write through recorder.
	"""
	sleeping(simulation_context, viewport_window_list, raytracing)
	if my_recorder is not None and enable_recorder and getattr(my_recorder, "_enable_record", True):
		if hasattr(my_recorder, "record"):
			my_recorder.record()


def sleeping(simulation_context, viewport_window_list, raytracing, totalSpp=64, spp=1):
	"""
	Block until a frame is sufficiently rendered/updated.
	"""
	if raytracing:
		simulation_context.step(render=False)
		simulation_context.render()
		simulation_context.step(render=False)
		simulation_context.render()
		return

	iterations = max(1, int(totalSpp / max(1, int(spp))))
	for _ in range(iterations):
		simulation_context.step(render=False)
		simulation_context.render()
