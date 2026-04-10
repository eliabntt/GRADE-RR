import grade_utils.misc_utils
from grade_utils.misc_utils import *


def load_zebra(zebra_base_prim_path, n, asset_path):
	stage = omni.usd.get_context().get_stage()
	res, _ = omni.kit.commands.execute(
		"CreateReferenceCommand",
		usd_context=omni.usd.get_context(),
		path_to=f"{zebra_base_prim_path}{n}",
		asset_path=asset_path,
		instanceable=False,
	)
	clear_properties(f"{zebra_base_prim_path}{n}")
	return f"{zebra_base_prim_path}{n}"


def _sequencer_drop_fallback(sequence_prim_path: str, zebra_path: str, timeslot: float):
	"""
	Fallback sequencer insertion compatible with omni.kit.sequencer.core/usd when
	omni.kit.window.sequencer.scripts.sequencer_drop_controller is unavailable.
	"""
	track_name = Sdf.Path(zebra_path).name
	track_res, track = omni.kit.commands.execute(
		"SequencerTrackCreateCommand",
		sequence_path=Sdf.Path(sequence_prim_path),
		track_name=track_name,
		track_type="Asset",
	)
	if not track_res or track is None:
		raise RuntimeError(f"Failed to create sequencer track for {zebra_path}")

	track_path = track.GetPrim().GetPath().pathString
	clip_name = f"{track_name}_Clip"
	clip_end = float(timeslot) + 1.0
	clip_res, clip = omni.kit.commands.execute(
		"SequencerClipCreateCommand",
		track_path=track_path,
		clip_name=clip_name,
		prim_path=zebra_path,
		clip_start=float(timeslot),
		clip_end=clip_end,
	)
	if not clip_res or clip is None:
		raise RuntimeError(f"Failed to create sequencer clip for {zebra_path}")


def place_zebras(frame_info, rng, floor_points, meters_per_unit, hidden_position, config, max_anim_len, zebra_info):
	stage = omni.usd.get_context().get_stage()
	# create bool array as big as floor_points
	occupied = np.zeros((floor_points.shape[0] - 2, floor_points.shape[1] - 2), dtype=bool)

	deleted_zebras = []
	out_frame_info = {}
	min_number_zebras = config["min_number_zebras"].get()
	max_number_zebras = config["max_number_zebras"].get()
	selected_zebras = rng.choice(
		list(frame_info.keys()), size=int(rng.uniform(min_number_zebras, max_number_zebras)), replace=False
	)

	for zebra in selected_zebras:
		out_frame_info[zebra] = frame_info[zebra].copy()
		out_frame_info[zebra] = randomize_frame(out_frame_info[zebra], rng, max_anim_len, zebra_info)

		# process the box and extract xmin xmax ymin ymax
		box = np.array(out_frame_info[zebra]["box"])
		xmin = np.min(box[:, 0])
		xmax = np.max(box[:, 0])
		ymin = np.min(box[:, 1])
		ymax = np.max(box[:, 1])
		# box is the 2D box
		box = np.array([[xmin, ymin], [xmax, ymin], [xmax, ymax], [xmin, ymax]])

		# random yaw rotation of the box
		yaw = rng.uniform(0, 2 * np.pi)
		# create a rotation matrix
		rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
		# rotate the box
		box = np.matmul(box, rot)

		positioned = False
		newbox = []

		# get intermediate int points
		for i in range(4):
			p1 = np.round(box[i]).astype(int)
			p2 = np.round(box[(i + 1) % 4]).astype(int)
			# compute all int numbers between p1 and p2
			dx = p2[0] - p1[0]
			dy = p2[1] - p1[1]
			if dx == 0:
				x = p1[0]
				y = np.arange(min(p1[1], p2[1]), max(p1[1], p2[1]) + 1 if max(p1[1], p2[1]) >= 0 else -1)
				for j in range(len(y)):
					newbox.append([x, y[j]])
			elif dy == 0:
				x = np.arange(min(p1[0], p2[0]), max(p1[0], p2[0]) + 1 if max(p1[0], p2[0]) >= 0 else -1)
				y = p1[1]
				for j in range(len(x)):
					newbox.append([x[j], y])
			elif dx == 0 and dy == 0:
				newbox.append([p1[0], p1[1]])
			else:
				x = np.arange(min(p1[0], p2[0]), max(p1[0], p2[0]) + 1 if max(p1[0], p2[0]) >= 0 else -1)
				y = p1[1] + (x - p1[0]) * dy / dx
				for j in range(len(x)):
					newbox.append([x[j], y[j]])
		newbox = np.unique(np.array(newbox).astype(int), axis=0).astype(int)

		for _ in range(100):
			# get a random location in occupied -- this will be my center
			center = np.array([rng.integers(0, occupied.shape[1]), rng.integers(0, occupied.shape[0])])

			# check if all the cells covered by the box in occupied are free -- not only the boundaries
			collision = False

			for x_coor, y_coor in newbox:
				try:
					if occupied[center[0] - y_coor, center[1] + x_coor]:
						collision = True
						break
				except IndexError:
					collision = True
					break
				if collision:
					break
			if not collision:
				tmp_floor_points = []
				newcenter = np.array([center[0] + 1, center[1] + 1])
				# if there is no collision, set the cells covered by the box to occupied
				for x_coor, y_coor in newbox:
					occupied[center[0] - y_coor, center[1] + x_coor] = True
					# get the corresponding floor point given the center and x_coor and col
					# NOTE THAT Y IS OPPOSITE SIGN
					tmp_floor_points.append(floor_points[newcenter[0] - y_coor, newcenter[1] + x_coor])

				# set the position of the zebra to the center
				loc = np.mean(tmp_floor_points, axis=0) / meters_per_unit
				loc = np.array(floor_points[newcenter[0], newcenter[1]]) / meters_per_unit
				set_translate(stage.GetPrimAtPath(zebra), list(loc))
				out_frame_info[zebra]["position"] = loc * meters_per_unit
				out_frame_info[zebra]["rotation"] = [0, 0, yaw]
				out_frame_info[zebra]["center"] = newcenter
				out_frame_info[zebra]["box"] = box
				set_rotate(stage.GetPrimAtPath(zebra), [0, 0, yaw])  # todo refine this to account for terrain

				positioned = True
				break

		if not positioned:
			print("Could not position zebra", zebra)
			# delete the zebra
			deleted_zebras.append(zebra)
			set_translate(stage.GetPrimAtPath(zebra), list(hidden_position))

	for zebra in deleted_zebras:
		del out_frame_info[zebra]
	return out_frame_info


def randomize_frame(zebra, rng, max_anim_len, zebra_info):
	stage = omni.usd.get_context().get_stage()
	zebra_path = zebra["path"]

	scale = rng.integers(40, 100)
	set_scale(stage.GetPrimAtPath(zebra_path), scale)

	zebra_name = zebra["name"]
	prim = stage.GetPrimAtPath(f"/World/Sequence{zebra_path}{zebra_path}_Clip")
	anim_len = zebra_info[zebra_name]["length"]
	timeslot = max_anim_len - rng.integers(0, anim_len)

	prim.GetAttribute("startTime").Set(Sdf.TimeCode(timeslot * 1.0))
	prim.GetAttribute("endTime").Set(Sdf.TimeCode(float(max(timeslot + zebra_info[zebra_name]["length"], max_anim_len))))
	points_in_mesh = zebra_info[zebra_name]["points"][max_anim_len - timeslot] * scale / 100
	zebra = {
		"name": zebra_name,
		"time": timeslot,
		"used_frame": max_anim_len - timeslot + 1,
		"scale": scale,
		"box": trimesh.PointCloud(points_in_mesh).bounding_box.vertices,
		"path": zebra_path,
	}
	return zebra


def preload_all_zebras(
	config, rng, zebra_files, zebra_info, simulation_context, sequencer_drop_controller, max_anim_len, hidden_position
):
	stage = omni.usd.get_context().get_stage()

	# load a random number of zebras between min_number_zebra and max_number_zebra
	num_zebras = config["max_number_zebras"].get()
	frame_info = {}
	for n in range(num_zebras):
		# load a random zebra
		zebra_file = rng.choice(zebra_files)
		# load the zebra
		zebra_path = load_zebra("/zebra_", n, zebra_file)
		scale = rng.integers(40, 100)
		set_scale(stage.GetPrimAtPath(zebra_path), scale)

		zebra_name = zebra_file.split("/")[-1].split(".")[0]

		add_semantics(stage.GetPrimAtPath(zebra_path), "zebra")
		timeslot = max_anim_len - rng.integers(0, zebra_info[zebra_name]["length"])

		if sequencer_drop_controller is not None and hasattr(sequencer_drop_controller, "sequencer_drop"):
			sequencer_drop_controller.sequencer_drop(stage.GetPrimAtPath("/World/Sequence"), zebra_path, float(timeslot))
		else:
			_sequencer_drop_fallback("/World/Sequence", zebra_path, float(timeslot))

		prim = stage.GetPrimAtPath(f"/World/Sequence{zebra_path}{zebra_path}_Clip")
		prim.GetAttribute("startTime").Set(Sdf.TimeCode(timeslot * 1.0))
		prim.GetAttribute("endTime").Set(Sdf.TimeCode(float(max(timeslot + zebra_info[zebra_name]["length"], max_anim_len))))
		points_in_mesh = zebra_info[zebra_name]["points"][max_anim_len - timeslot] * scale / 100

		frame_info[zebra_path] = {
			"name": zebra_name,
			"time": timeslot,
			"used_frame": max_anim_len - timeslot + 1,
			"scale": scale,
			"box": trimesh.PointCloud(points_in_mesh).bounding_box.vertices,
			"path": zebra_path,
		}
		simulation_context.step(render=False)
		simulation_context.render()

		set_translate(stage.GetPrimAtPath(zebra_path), hidden_position)

	return frame_info
