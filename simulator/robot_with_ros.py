import argparse
import os
from pathlib import Path

import confuse
from isaacsim import SimulationApp

#/home/jschwenkbeck/miniforge3/envs/env_isaaclab/bin/python /home/jschwenkbeck/Documents/GRADE/GRADE-RR/simulator/robot_with_ros.py --config_file /home/jschwenkbeck/Documents/GRADE/GRADE-RR/simulator/configs/robot_with_ros.yaml --headless False --steps 50000

def boolean_string(value: str) -> bool:
	if value.lower() not in {"false", "true"}:
		raise ValueError("Not a valid boolean string")
	return value.lower() == "true"


def _resolve_local_defaults(base_env_path: str, usd_robot_path: str) -> tuple[str, str]:
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


parser = argparse.ArgumentParser(description="Robot with ROS (Isaac Sim 5.1 local run)")
parser.add_argument("--headless", type=boolean_string, default=True, help="Run headless")
parser.add_argument("--config_file", type=str, default="config.yaml")
parser.add_argument("--ros", type=boolean_string, default=False, help="Enable ROS runtime check")
parser.add_argument("--steps", type=int, default=300, help="Number of simulation steps to run")
args, _ = parser.parse_known_args()

config = confuse.Configuration("robot_with_ros", __name__)
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

app = SimulationApp(launch_config)

try:
	import omni
	from isaacsim.core.api import SimulationContext
	from isaacsim.core.utils.stage import is_stage_loading
	from pxr import Gf, Sdf, UsdGeom, UsdPhysics

	if bool(cfg("ros", args.ros)):
		try:
			import rclpy  # noqa: F401
			print("[robot_with_ros] ROS runtime detected")
		except Exception as ros_exc:
			print(f"[robot_with_ros] ROS runtime not available: {ros_exc}")

	base_env_path, usd_robot_path = _resolve_local_defaults(
		str(cfg("base_env_path", "")),
		str(cfg("usd_robot_path", "")),
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

	print(f"[robot_with_ros] opening stage: {base_env_path}")
	omni.usd.get_context().open_stage(base_env_path, None)
	app.update()
	app.update()

	while is_stage_loading():
		app.update()

	stage = omni.usd.get_context().get_stage()
	if stage is None:
		raise RuntimeError("Failed to get stage")

	print(f"[robot_with_ros] spawning {num_robots} robot(s) from: {usd_robot_path}")
	spacing = 2.0 / max(meters_per_unit, 1e-9)
	for idx in range(num_robots):
		root_path = f"{robot_base_prim_path}{idx}_root"
		robot_path = f"{root_path}/robot"

		root_xform = UsdGeom.Xform.Define(stage, Sdf.Path(root_path))
		UsdGeom.XformCommonAPI(root_xform).SetTranslate(Gf.Vec3d(float(idx) * spacing, 0.0, 0.0))

		robot_xform = UsdGeom.Xform.Define(stage, Sdf.Path(robot_path))
		robot_xform.GetPrim().GetReferences().AddReference(usd_robot_path)

		# Let referenced prims resolve before applying collision overrides.
		app.update()
		app.update()

		visual_mesh_prim = stage.GetPrimAtPath(f"{robot_path}/yaw_link/visuals")
		if visual_mesh_prim and visual_mesh_prim.IsValid():
			mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(visual_mesh_prim)
			mesh_collision_api.CreateApproximationAttr().Set("convexHull")

	sim = SimulationContext(
		physics_dt=1.0 / physics_hz,
		rendering_dt=1.0 / render_hz,
		stage_units_in_meters=meters_per_unit,
	)
	sim.initialize_physics()

	steps = max(1, int(args.steps))
	sim.play()
	for _ in range(steps):
		sim.step(render=True)
	sim.stop()

	print("[robot_with_ros] completed successfully")

except Exception:
	import traceback

	traceback.print_exc()
	raise
finally:
	try:
		app.close()
	except Exception:
		pass
