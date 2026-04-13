import argparse
import os
from pathlib import Path
#conda activate env_isaaclab && python simulator/world_and_robot.py --config_file simulator/configs/world_and_robot.yaml --headless False
import confuse
from isaacsim import SimulationApp


def boolean_string(value):
	if value.lower() not in {"false", "true"}:
		raise ValueError("Not a valid boolean string")
	return value.lower() == "true"


def _resolve_local_defaults(base_env_path, usd_robot_path):
	repo_root = Path(__file__).resolve().parents[1]
	local_base = repo_root / "usds" / "empty.usd"
	local_drone = repo_root / "usds" / "drone_2022.usd"
	local_robotino = repo_root / "usds" / "robotino.usd"

	resolved_base = base_env_path if base_env_path else str(local_base)
	resolved_robot = usd_robot_path
	if not resolved_robot:
		if local_drone.exists():
			resolved_robot = str(local_drone)
		elif local_robotino.exists():
			resolved_robot = str(local_robotino)

	return resolved_base, resolved_robot


parser = argparse.ArgumentParser(description="World + robot local asset test")
parser.add_argument("--headless", type=boolean_string, default=True, help="Run headless")
parser.add_argument("--rtx_mode", type=boolean_string, default=False, help="Kept for compatibility")
parser.add_argument("--config_file", type=str, default="config.yaml")
parser.add_argument("--keep_open", type=boolean_string, default=True, help="Keep running until Ctrl+C")
args, _ = parser.parse_known_args()

config = confuse.Configuration("world_and_robot", __name__)
config.set_file(args.config_file)
config.set_args(args)


def cfg(name, default):
	try:
		value = config[name].get()
		if value is None:
			return default
		if isinstance(value, str) and value.strip() == "":
			return default
		return value
	except Exception:
		return default

launch_config = {
	"display_options": 3286,
	"width": 1280,
	"height": 720,
	"headless": bool(cfg("headless", True)),
}

kit = SimulationApp(launch_config=launch_config)

try:
	import omni
	from isaacsim.core.api import SimulationContext
	from isaacsim.core.utils.stage import is_stage_loading
	from pxr import Gf, Sdf, UsdGeom

	base_env_path, usd_robot_path = _resolve_local_defaults(
		cfg("base_env_path", ""),
		cfg("usd_robot_path", ""),
	)

	if not os.path.exists(base_env_path):
		raise FileNotFoundError(f"base_env_path not found: {base_env_path}")
	if not usd_robot_path or not os.path.exists(usd_robot_path):
		raise FileNotFoundError(f"usd_robot_path not found: {usd_robot_path}")

	num_robots = int(cfg("num_robots", 1))
	robot_base_prim_path = str(cfg("robot_base_prim_path", "/my_robot_"))
	meters_per_unit = float(cfg("meters_per_unit", 1.0))
	physics_hz = float(cfg("physics_hz", 60.0))
	render_hz = float(cfg("render_hz", physics_hz))

	print(f"[world_and_robot] opening stage: {base_env_path}")
	omni.usd.get_context().open_stage(base_env_path, None)
	kit.update()
	kit.update()

	while is_stage_loading():
		kit.update()

	stage = omni.usd.get_context().get_stage()
	if stage is None:
		raise RuntimeError("Failed to obtain USD stage after opening base environment")

	print(f"[world_and_robot] spawning {num_robots} robot(s) from: {usd_robot_path}")
	for idx in range(num_robots):
		prim_path = f"{robot_base_prim_path}{idx}"
		xform = UsdGeom.Xform.Define(stage, Sdf.Path(prim_path))
		xform.GetPrim().GetReferences().AddReference(usd_robot_path)
		UsdGeom.XformCommonAPI(xform).SetTranslate(
			Gf.Vec3d(float(idx) * (2.0 / max(meters_per_unit, 1e-9)), 0.0, 0.0)
		)

	simulation_context = SimulationContext(
		physics_dt=1.0 / physics_hz,
		rendering_dt=1.0 / render_hz,
		stage_units_in_meters=meters_per_unit,
	)
	simulation_context.initialize_physics()

	simulation_context.play()
	if bool(args.keep_open):
		print("[world_and_robot] running continuously (Ctrl+C to stop)")
		try:
			while True:
				simulation_context.step(render=True)
		except KeyboardInterrupt:
			print("[world_and_robot] stop requested by user")
	else:
		for _ in range(300):
			simulation_context.step(render=True)
	simulation_context.stop()

	print("[world_and_robot] completed successfully")

except Exception:
	import traceback

	traceback.print_exc()
	raise
finally:
	try:
		kit.close()
	except Exception:
		pass
