import time

import utils.misc_utils
from utils.misc_utils import *

GRAPH_PATH = "/Render/PostProcess/SDGPipeline"


def set_common_stage_properties(rate):
	"""
  Note: some properties as of now can only be set with the general environment USD file.
  """
	_desired_render_settings: Dict[str, Union[bool, int]] = {
		"/app/asyncRendering": False,
		"/app/renderer/skipWhileMinimized": False,
		"/app/renderer/sleepMsOnFocus": 0,
		"/app/renderer/sleepMsOutOfFocus": 0,
		"/app/runLoops/main/rateLimitEnabled": True,
		"/app/runLoops/main/rateLimitFrequency": rate,
		"/persistent/simulation/minFrameRate": rate,
		"/app/runLoops/main/rateLimitUseBusyLoop": True,
		"/app/runLoops/rendering_0/rateLimitEnabled": True,
		"/app/viewport/showSettingMenu": True,
		"/app/viewport/showCameraMenu": True,
		"/app/viewport/showRendererMenu": True,
		"/app/viewport/showHideMenu": True,
		"/app/viewport/showLayerMenu": True,
		"/app/viewport/grid/showOrigin": False,
		"/app/viewport/grid/enabled": False,  ## this does not work
		"/persistent/app/viewport/grid/lineWidth": 0,
		"/rtx/multiThreading/enabled": True,
		"/app/asyncRenderingLowLatency": False,
		# "/persistent/app/captureFrame/viewport": True,
	}
	for setting_key, desired_value in _desired_render_settings.items():
		set_carb_setting(carb.settings.get_settings(), setting_key, desired_value)


def simulation_environment_setup(need_ros = True):
	"""
    Enable the necessary extensions that will be used within the simulation
  """
	enable_extension("omni.isaac.ros_bridge")
	enable_extension("omni.isaac.physics_inspector")
	enable_extension("omni.isaac.physics_utilities")
	enable_extension("omni.anim.skelJoint")
	enable_extension("omni.kit.window.sequencer")
	enable_extension("omni.isaac.dynamic_control")
	enable_extension("omni.isaac.shapenet")
	enable_extension("semantics.schema.editor")
	enable_extension("omni.hydra.iray")
	enable_extension("omni.iray.settings.core")
	enable_extension('omni.isaac.occupancy_map')
	enable_extension('omni.isaac.shapenet')
	enable_extension('omni.isaac.range_sensor')
	disable_extension('omni.isaac.sun_study')
	enable_extension('omni.isaac.core_nodes')
	enable_extension('omni.isaac.sensor')
	# Necessary ONLY if using NUCLEUS
	# Locate /Isaac folder on nucleus server to load sample
	from omni.isaac.core.utils.nucleus import get_assets_root_path

	nucleus_server = get_assets_root_path()
	if nucleus_server is None:
		carb.log_error("Could not find nucleus server with /Isaac folder, exiting")
		exit()
	if need_ros:
		if not rosgraph.is_master_online():
			carb.log_error("Please run roscore before executing this script")
			exit()


def set_raytracing_settings(physics_hz):
	set_common_stage_properties(physics_hz)
	settings = carb.settings.get_settings()
	settings.set("/app/hydraEngine/waitIdle", True)
	settings.set_string("/rtx/rendermode", "RayTracing")
	settings.set_int('/rtx/post/aa/op', 2)


def set_pathtracing_settings(physics_hz):
	set_common_stage_properties(physics_hz)
	settings = carb.settings.get_settings()
	settings.set_string("/rtx/rendermode", "PathTracing")
	settings.set_int('/rtx/post/aa/op', 1)
	# settings.set_int('/rtx/multiThreading/enabled', True)
	# settings.set_bool('/rtx/multiThreading/enabled', True)
	settings.set_int('/rtx/post/histogram/filterType', 1)
	settings.set_int('/rtx/post/histogram/tau', 100)
	settings.set_float('/rtx/post/histogram/minEV', 2)
	settings.set_float('/rtx/post/histogram/maxEV', 50)
	settings.set_bool('/rtx/post/histogram/enabaled', True)
	settings.set_int('/rtx/post/tonemap/filmIso', 100)  # 400
	settings.set_int('/rtx/post/tonemap/cameraShutter', 30)
	settings.set_int('/rtx/post/tonemap/fStop', 4)
	settings.set_int("/rtx/pathtracing/maxBounces", 6)  # 6
	settings.set_int("/rtx/pathtracing/maxSpecularAndTransmissionBounces", 6)
	# settings.set_int("/rtx/pathtracing/maxDiffuseBounces", 10)
	settings.set_int("/rtx/pathtracing/spp", 1)
	settings.set_int("/rtx/pathtracing/totalSpp", 64)
	settings.set_int("/rtx/pathtracing/clampSpp", 64)
	settings.set_int("/rtx/pathtracing/cached/enabled", False)
	settings.set_bool("/rtx/pathtracing/cached/enabled", False)
	settings.set_int("/rtx/pathtracing/lightcache/cached/enabled", False)
	settings.set_bool("/rtx/pathtracing/lightcache/cached/enabled", False)
	settings.set("/app/hydraEngine/waitIdle", False)


def compute_timeline_ratio(human_anim_len, reverse_strategy, experiment_length):
	"""
  based on the reverse strategy compute how the system should roll back animations
  This might be counter-productive in some instances
  """
	if len(human_anim_len) == 0:
		return 1
	if reverse_strategy == "avg":
		return float(experiment_length) / (sum(human_anim_len) / len(human_anim_len))
	elif reverse_strategy == "min":
		return float(experiment_length) / min(human_anim_len)
	elif reverse_strategy == "max":
		return float(experiment_length) / max(human_anim_len)
	elif reverse_strategy == "half":
		return 2
	elif reverse_strategy == "none":
		return 1
	else:
		return 1


def pub_and_write_images(simulation_context, viewport_window_list, ros_camera_list, raytracing, my_recorder=None, enable_recorder=True):
	sleeping(simulation_context, viewport_window_list, raytracing)
	ctime = omni.timeline.get_timeline_interface().get_current_time()
	for i, cam, outs in ros_camera_list:
		print(f"Publishing camera {cam}...")
		for output in outs:
			og.Controller.attribute(output+ ".inputs:step").set(1)
	simulation_context.render()
	for i, cam, outs in ros_camera_list:
		for output in outs:
			og.Controller.attribute(output+ ".inputs:step").set(0)
	omni.timeline.get_timeline_interface().set_current_time(ctime)
	
	if my_recorder and my_recorder._enable_record and enable_recorder:
		my_recorder._update()
		print("Writing")


def sleeping(simulation_context, viewport_window_list, raytracing, totalSpp=64, spp=1):
"""
  Sleeps the simulation to be sure that the whole frame has been rendered and updated.
  First we render a couple of frames.
  In rtx mode we need to wait the fps of the viewport to be reached.
  In pathtracing mode we need to do "/rtx/pathtracing/spp" rendering steps.

  e.g.
  carb.settings.get_settings().get("/rtx/pathtracing/totalSpp")
  carb.settings.get_settings().get("/rtx/pathtracing/spp")
"""
	# todo is there a better way? I don"t think so, this is variable
	# fixme making sure timeline does not advance
	timeline = omni.timeline.get_timeline_interface()
	mytime = timeline.get_current_time()

	if raytracing:
		sleep_time = 0
		start = time.time()
		for _ in range(100):
			for vp in viewport_window_list:
				if vp.fps == 0: continue
				sleep_time = max(1 / vp.fps * 1.1, sleep_time)
			if sleep_time != 0 and time.time() - start > sleep_time * 2:  # overly cautious
				break
			simulation_context.render()
			timeline.set_current_time(mytime)
	else:
		cnt = totalSpp
		increase = spp
		while cnt >= 0:
			simulation_context.render()
			timeline.set_current_time(mytime)
			cnt -= increase
		simulation_context.render()
		timeline.set_current_time(mytime)
		simulation_context.render()
		timeline.set_current_time(mytime)
	time.sleep(0.2)


def recorder_setup(_recorder_settings, out_path, enabled, skip_cameras=1):
	my_recorder = extension_custom.MyRecorder()
	my_recorder.on_startup()
	my_recorder.set_single_settings(_recorder_settings)
	my_recorder._dir_name = os.path.join(out_path)
	my_recorder._enable_record = enabled
	my_recorder.skip_cameras = skip_cameras
	return my_recorder


def setup_timeline(config):
	"""
	It sets up the timeline to have a start time of 0.0, an end time of the experiment length * 2, and a time code per
	second of the fps

	:param config: a dictionary of parameters that are used to configure the experiment
	:return: timeline
	"""
	timeline = omni.timeline.get_timeline_interface()
	timeline.set_start_time(0.0)
	if "fps" not in config:
		fps = 30
	else:
		fps = config['fps'].get()

	if "experiment_length" in config:
		timeline.set_end_time(config["experiment_length"].get() * 2 / fps)  # *2 to have room
	else:
		print("No experiment length found, setting it to 3600")
		timeline.set_end_time(3600 / fps)
	timeline.set_time_codes_per_second(fps)
	return timeline
