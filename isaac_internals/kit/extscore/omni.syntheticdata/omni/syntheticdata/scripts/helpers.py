import math
from functools import lru_cache

import numpy.lib.recfunctions as rfn
import carb
import numpy as np
import omni.usd
from pxr import UsdGeom, UsdShade, Semantics

from .. import _syntheticdata

EPS = 1e-8


@lru_cache()
def _get_syntheticdata_iface():
	return _syntheticdata.acquire_syntheticdata_interface()


def _interpolate(p, a, b):
	p0 = 1.0 - p
	return [int(p0 * a[0] + p * b[0]), int(p0 * a[1] + p * b[1]), int(p0 * a[2] + p * b[2]), 255]


def get_bbox_3d_corners(extents):
	"""Return transformed points in the following order: [LDB, RDB, LUB, RUB, LDF, RDF, LUF, RUF]
    where R=Right, L=Left, D=Down, U=Up, B=Back, F=Front and LR: x-axis, UD: y-axis, FB: z-axis.

    Args:
        extents (numpy.ndarray): A structured numpy array containing the fields: [`x_min`, `y_min`,
            `x_max`, `y_max`, `transform`.

    Returns:
        (numpy.ndarray): Transformed corner coordinates with shape `(N, 8, 3)`.
    """
	rdb = [extents["x_max"], extents["y_min"], extents["z_min"]]
	ldb = [extents["x_min"], extents["y_min"], extents["z_min"]]
	lub = [extents["x_min"], extents["y_max"], extents["z_min"]]
	rub = [extents["x_max"], extents["y_max"], extents["z_min"]]
	ldf = [extents["x_min"], extents["y_min"], extents["z_max"]]
	rdf = [extents["x_max"], extents["y_min"], extents["z_max"]]
	luf = [extents["x_min"], extents["y_max"], extents["z_max"]]
	ruf = [extents["x_max"], extents["y_max"], extents["z_max"]]
	tfs = extents["transform"]

	corners = np.stack((ldb, rdb, lub, rub, ldf, rdf, luf, ruf), 0)
	corners_homo = np.pad(corners, ((0, 0), (0, 1), (0, 0)), constant_values=1.0)

	return np.einsum("jki,ikl->ijl", corners_homo, tfs)[..., :3]


def reduce_bboxes_2d(bboxes, instance_mappings):
	"""
    Reduce 2D bounding boxes of leaf nodes to prims with a semantic label.

    Args:
        bboxes (numpy.ndarray): A structured numpy array containing the fields: `[
            ("instanceId", "<u4"), ("semanticId", "<u4"), ("x_min", "<i4"),
            ("y_min", "<i4"), ("x_max", "<i4"), ("y_max", "<i4")]`
        instance_mappings (numpy.ndarray): A structured numpy array containing the fields:
            `[("uniqueId", np.int32), ("name", "O"), ("semanticId", "<u4"), ("semanticLabel", "O"),
              ("instanceIds", "O"), ("metadata", "O")]`

    Returns:
        (numpy.ndarray): A structured numpy array containing the fields: `[
            ("uniqueId", np.int32), ("name", "O"), ("semanticLabel", "O"), ("instanceIds", "O"),
            ("semanticId", "<u4"), ("metadata", "O"), ("x_min", "<i4"), ("y_min", "<i4"),
            ("x_max", "<i4"), ("y_max", "<i4")]`
    """
	bboxes = bboxes[bboxes["x_min"] < 2147483647]
	reduced_bboxes = []
	for im in instance_mappings:
		if im["instanceIds"]:  # if mapping has descendant instance ids
			mask = np.isin(bboxes["instanceId"], im["instanceIds"])
			bbox_masked = bboxes[mask]
			if len(bbox_masked) > 0:
				reduced_bboxes.append(
					(
						im["uniqueId"],
						im["name"],
						im["semanticLabel"],
						im["metadata"],
						im["instanceIds"],
						im["semanticId"],
						np.min(bbox_masked["x_min"]),
						np.min(bbox_masked["y_min"]),
						np.max(bbox_masked["x_max"]),
						np.max(bbox_masked["y_max"]),
					)
				)
	return np.array(
		reduced_bboxes,
		dtype=[("uniqueId", np.int32), ("name", "O"), ("semanticLabel", "O"), ("metadata", "O"), ("instanceIds", "O")]
		      + bboxes.dtype.descr[1:],
	)


def reduce_bboxes_3d(bboxes, instance_mappings):
	"""
    Reduce 3D bounding boxes of leaf nodes to prims with a semantic label.

    Args:
        bboxes (numpy.ndarray): A structured numpy array containing the fields: `[
            ("instanceId", "<u4"), ("semanticId", "<u4"), ("x_min", "<i4"),
            ("y_min", "<i4"), ("z_min", "<i4"), ("x_max", "<i4"), ("y_max", "<i4"),
            ("z_max", "<i4"), ("transform", "<f4", (4, 4))]`
        instance_mappings (numpy.ndarray): A structured numpy array containing the fields:
            `[("uniqueId", np.int32), ("name", "O"), ("semanticId", "<u4"), ("semanticLabel", "O"),
              ("instanceIds", "<u4"), ("metadata", "O")]`

    Returns:
        (numpy.ndarray): A structured numpy array containing the fields: `[("uniqueId", np.int32)
            ("name", "O"), ("semanticLabel", "O"), ("instanceIds", "O"), ("metadata", "O"),
            ("semanticId", "<u4"),("x_min", "<i4"), ("y_min", "<i4"), ("z_min", "<i4"),
            ("x_max", "<i4"), ("y_max", "<i4"), ("z_max", "<i4"), ("transform", "<f4", (4, 4))]`
            If `corners` field is supplied in `bboxes` argument, the field will be updated accordingly.
    """
	current_time = omni.timeline.get_timeline_interface().get_current_time()
	reduced_bboxes = []
	stage = omni.usd.get_context().get_stage()
	if "corners" in bboxes.dtype.names:
		corners = bboxes["corners"]
	else:
		# TODO if not corners, use extents
		corners = get_bbox_3d_corners(bboxes)
	max_instance_id = bboxes["instanceId"].max()
	idx_lut = np.zeros(max_instance_id + 1, dtype=np.uint32)
	for i, bb_id in enumerate(bboxes["instanceId"]):
		idx_lut[bb_id] = i
	for i, im in enumerate(instance_mappings):
		prim = stage.GetPrimAtPath(im["name"])
		tf = np.array(UsdGeom.Imageable(prim).ComputeLocalToWorldTransform(current_time))
		tf_inv = np.linalg.inv(tf)
		# filter instance ids that corresponding to invisible bounding boxes (not filtered in the instance mapping)
		instIds = [instId for instId in im["instanceIds"] if instId < len(idx_lut)]
		if len(instIds) == 0:
			continue
		idxs = idx_lut[instIds]
		children_corners = corners[idxs]
		children_corners_homo = np.pad(children_corners.reshape(-1, 3), ((0, 0), (0, 1)), constant_values=1.0)
		corners_local = np.einsum("bj,jk->bk", children_corners_homo, tf_inv)[:, :3]
		corners_local_min = corners_local[..., :3].reshape(-1, 3).min(0)
		corners_local_max = corners_local[..., :3].reshape(-1, 3).max(0)
		extents_local = np.stack([corners_local_min, corners_local_max])
		row = [
			im["uniqueId"],
			im["name"],
			im["semanticLabel"],
			im["metadata"],
			im["instanceIds"],
			im["semanticId"],
			*extents_local.reshape(-1),
			tf,
		]
		if "corners" in bboxes.dtype.names:
			world_corners = get_bbox_3d_corners(
				{
					"x_min": [extents_local[0, 0]],
					"x_max": [extents_local[1, 0]],
					"y_min": [extents_local[0, 1]],
					"y_max": [extents_local[1, 1]],
					"z_min": [extents_local[0, 2]],
					"z_max": [extents_local[1, 2]],
					"transform": [tf],
				}
			)
			row.append(world_corners)
		reduced_bboxes.append(tuple(row))
	return np.array(
		reduced_bboxes,
		dtype=[("uniqueId", np.int32), ("name", "O"), ("semanticLabel", "O"), ("metadata", "O"), ("instanceIds", "O")]
		      + bboxes.dtype.descr[1:],
	)


def merge_sensors(
		bounding_box_2d_tight=None, bounding_box_2d_loose=None, bounding_box_3d=None, occlusion_quadrants=None
):
	"""
    Merge sensor structured array outputs.

    Args:
        bounding_box_2d_tight (numpy.ndarray, optional):  A structured numpy array
            containing the fields: `[("uniqueId", "<i4"), ("name", "O"), ("semanticLabel", "O"),
            ("semanticId", "<u4"), ("metadata", "O"), ("instanceIds", "O"),
            ("x_min", "<i4"), ("y_min", "<i4"), ("x_max", "<i4"), ("y_max", "<i4")]`
        bounding_box_2d_loose (numpy.ndarray, optional):  A structured numpy array
            containing the fields: `[("uniqueId", "<i4"), ("name", "O"), ("semanticLabel", "O"),
            ("instanceId", "<u4"), ("semanticId", "<u4"), ("metadata", "O"), ("instanceIds", "O"),
            ("x_min", "<i4"), ("y_min", "<i4"), ("x_max", "<i4"), ("y_max", "<i4")]`
        bounding_box_3d (numpy.ndarray, optional): A structured numpy array containing the fields:
            `[("uniqueId", "<i4"), ("name", "O"), ("semanticLabel", "O"), ("semanticId", "<u4"),
            ("metadata", "O"), ("instanceIds", "O"), ("x_min", "<i4"), ("y_min", "<i4"), ("z_min", "<i4"),
            ("x_max", "<i4"), ("y_max", "<i4"), ("z_max", "<i4"), ("transform", "<f4", (4, 4))]`
        occlusion_quadrants (numpy.ndarray, optional): A structured numpy array containing the fields:
            [("uniqueId", "<i4"), ("name", "O"), ("semanticLabel", "O"),("semanticId", "<u4"),
            ("metadata", "O"), ("instanceIds", "O"), ("occlusion_quadrant", "O")]

    Returns:
        (numpy.ndarray): A structured array containing merged data from the arguments supplied.
    """
	arrays = []
	array_suffixes = []
	defaults = {"x_min": -1, "x_max": -1, "y_min": -1, "y_max": -1, "z_min": -1, "z_max": -1}

	# Add valid arrays to merge list and set suffixes
	if bounding_box_2d_tight is not None:
		arrays.append(bounding_box_2d_tight)
		array_suffixes.append("_bbox2d_tight")
	if bounding_box_2d_loose is not None:
		arrays.append(bounding_box_2d_loose)
		array_suffixes.append("_bbox2d_loose")
	if bounding_box_3d is not None:
		arrays.append(bounding_box_3d)
		array_suffixes.append("_bbox3d")
	if occlusion_quadrants is not None:
		arrays.append(occlusion_quadrants)
		array_suffixes.append("_occ")

	if not arrays:
		return None

	r0 = arrays.pop()
	r0_suf = array_suffixes.pop()
	while arrays:
		r1 = arrays.pop()
		r1_suf = array_suffixes.pop()

		# Add suffixes
		r0.dtype.names = [f"{n}{r0_suf}" if n in defaults.keys() else n for n in r0.dtype.names]
		r1.dtype.names = [f"{n}{r1_suf}" if n in defaults.keys() else n for n in r1.dtype.names]

		defaults_suf = {}
		defaults_suf.update({f"{k}{r0_suf}": v for k, v in defaults.items()})
		defaults_suf.update({f"{k}{r1_suf}": v for k, v in defaults.items()})
		r0 = rfn.join_by(
			["uniqueId", "name", "semanticId", "semanticLabel", "metadata", "instanceIds"],
			r0,
			r1,
			defaults=defaults_suf,
			r1postfix=r0_suf,
			r2postfix=r1_suf,
			jointype="outer",
			usemask=False,
		)
		r0_suf = ""
	return r0


def get_projection_matrix(fov, aspect_ratio, z_near, z_far):
	"""
    Calculate the camera projection matrix.

    Args:
        fov (float): Field of View (in radians)
        aspect_ratio (float): Image aspect ratio (Width / Height)
        z_near (float): distance to near clipping plane
        z_far (float): distance to far clipping plane

    Returns:
        (numpy.ndarray): View projection matrix with shape `(4, 4)`
    """
	a = -1.0 / math.tan(fov / 2)
	b = -a * aspect_ratio
	c = z_far / (z_far - z_near)
	d = z_near * z_far / (z_far - z_near)
	return np.array([[a, 0.0, 0.0, 0.0], [0.0, b, 0.0, 0.0], [0.0, 0.0, c, 1.0], [0.0, 0.0, d, 0.0]])


def get_view_proj_mat(view_params):
	"""
    Get View Projection Matrix.

    Args:
        view_params (dict): dictionary containing view parameters
    """
	z_near, z_far = view_params["clipping_range"]
	view_matrix = np.linalg.inv(view_params["view_to_world"])
	fov = 2 * math.atan(view_params["horizontal_aperture"] / (2 * view_params["focal_length"]))
	projection_mat = get_projection_matrix(fov, view_params["aspect_ratio"], z_near, z_far)
	return np.dot(view_matrix, projection_mat)


def project_pinhole(points, view_params):
	"""
    Project 3D points to 2D camera view using a pinhole camera model.

    Args:
        points (numpy.ndarray): Array of points in world frame of shape (num_points, 3).
        view_params:

    Returns:
        (numpy.ndarray): Image-space points of shape (num_points, 3)
    """
	view_proj_matrix = get_view_proj_mat(view_params)
	homo = np.pad(points, ((0, 0), (0, 1)), constant_values=1.0)
	tf_points = np.dot(homo, view_proj_matrix)
	tf_points = tf_points / (tf_points[..., -1:])
	tf_points[..., :2] = 0.5 * (tf_points[..., :2] + 1)
	return tf_points[..., :3]


def _parse_semantic_schemas(prim):
	""" Return the class name and (type, data) pairs of metadata """
	schemas = [s.split(":")[1] for s in prim.GetAppliedSchemas() if "SemanticsAPI" in s]
	metadata = []
	semantic_class = None
	for schema in schemas:
		sem = Semantics.SemanticsAPI.Get(prim, schema)
		sem_type = sem.GetSemanticTypeAttr().Get()
		sem_data = sem.GetSemanticDataAttr().Get()

		if sem_type == "class":
			semantic_class = sem_data
		else:
			metadata.append((sem_type, sem_data))
	return semantic_class, metadata


def _parse_instance_mappings(cur_prim):
	""" Recursively parse instance mappings. """
	# TODO @jlafleche currently not compatible with two labels per prim
	instance_mappings = []
	children = cur_prim.GetChildren()
	instance_ids = _get_syntheticdata_iface().get_instance_segmentation_id(cur_prim.GetPath().pathString).tolist()
	if children:
		for child in children:
			child_instance_ids, child_instance_mappings = _parse_instance_mappings(child)
			instance_ids += child_instance_ids
			instance_mappings += child_instance_mappings

	has_prim_semantics = cur_prim.HasAPI(Semantics.SemanticsAPI)
	material = UsdShade.MaterialBindingAPI(cur_prim).ComputeBoundMaterial()[0].GetPrim()
	has_material_semantics = material and material.HasAPI(Semantics.SemanticsAPI)

	if has_prim_semantics and instance_ids:
		semantic_class, metadata = _parse_semantic_schemas(cur_prim)
		if semantic_class:
			semantic_id = _get_syntheticdata_iface().get_semantic_segmentation_id_from_data("class", semantic_class)
			instance_mappings.append((str(cur_prim.GetPath()), semantic_id, semantic_class, instance_ids, metadata))
	elif has_material_semantics and instance_ids:
		semantic_class, metadata = _parse_semantic_schemas(material)
		if semantic_class:
			semantic_id = _get_syntheticdata_iface().get_semantic_segmentation_id_from_data("class", semantic_class)
			instance_mappings.append((str(cur_prim.GetPath()), semantic_id, semantic_class, instance_ids, metadata))
	return instance_ids, instance_mappings


def get_instance_mappings():
	"""
    Get instance mappings.
    Uses update number as frame ID for caching.
    """
	app = omni.kit.app.get_app_interface()
	frame_id = app.get_update_number()
	mappings = _get_instance_mappings(frame_id)
	return mappings


@lru_cache(maxsize=1)
def _get_instance_mappings(frame_id=None):
	"""
    Get instance mappings.
    Uses `frame_id` for caching.
    """
	stage = omni.usd.get_context().get_stage()
	""" Use the C++ API to retrieve the instance mapping """
	# _, instance_mappings = _parse_instance_mappings(stage.GetPseudoRoot())
	# mappings_raw = [(i + 1, *im) for i, im in enumerate(instance_mappings)]
	mappings_raw = _get_syntheticdata_iface().get_instance_mapping_list()
	mappings = np.array(
		mappings_raw,
		dtype=[
			("uniqueId", np.int32),
			("name", "O"),
			("semanticId", np.int32),
			("semanticLabel", "O"),
			("instanceIds", "O"),
			("metadata", "O"),
		],
	)
	return mappings


def reduce_occlusion(occlusion_data, instance_mappings):
	"""
    Reduce occlusion value of leaf nodes to prims with a semantic label.

    Args:
        sensor_data (numpy.ndarray): A structured numpy array with the fields: [("instanceId", "<u4"),
            ("semanticId", "<u4"), ("occlusionRatio", "<f4")], where occlusion ranges from 0
            (not occluded) to 1 (fully occluded).

    Returns:
        (numpy.ndarray): A structured numpy array with the fields: [("uniqueId", np.int32)
            ("name", "O"), ("semanticLabel", "O"), ("instanceIds", "O"), ("semanticId", "<u4"),
            ("metadata", "O"), ("occlusionRatio", "<f4")]
    """
	mapped_data = []
	occlusion_data = occlusion_data[~np.isnan(occlusion_data["occlusionRatio"])]
	for im in instance_mappings:
		if im["instanceIds"]:  # if mapping has descendant instance ids
			mask = np.isin(occlusion_data["instanceId"], im["instanceIds"])
			if mask.sum() > 1:
				carb.log_warn(
					f"[syntheticdata.viz] Mapping on {im['name']} contains multiple child meshes, occlusion value may be incorrect."
				)
			occ = occlusion_data[mask]
			if len(occ) > 0:
				mapped_data.append(
					(
						im["uniqueId"],
						im["name"],
						im["semanticLabel"],
						im["metadata"],
						im["instanceIds"],
						im["semanticId"],
						np.mean(occ["occlusionRatio"]),
					)
				)
	return np.array(
		mapped_data,
		dtype=[("uniqueId", np.int32), ("name", "O"), ("semanticLabel", "O"), ("metadata", "O"), ("instanceIds", "O")]
		      + occlusion_data.dtype.descr[1:],
	)


def _join_struct_arrays(arrays):
	"""
    Join N numpy structured arrays.
    """
	n = len(arrays[0])
	assert all([len(a) == n for a in arrays])
	dtypes = sum(([d for d in a.dtype.descr if d[0]] for a in arrays), [])
	joined = np.empty(n, dtype=dtypes)
	for a in arrays:
		joined[list(a.dtype.names)] = a
	return joined


def _fish_eye_map_to_sphere(screen, screen_norm, theta, max_fov):
	""" Utility function to map a sample from a disk on the image plane to a sphere. """
	direction = np.array([[0, 0, -1]] * screen.shape[0], dtype=np.float)
	extent = np.zeros(screen.shape[0], dtype=np.float)
	# A real fisheye have some maximum FOV after which the lens clips.
	valid_mask = theta <= max_fov
	# Map to a disk: screen / R normalizes the polar direction in screen space.
	xy = screen[valid_mask]
	screen_norm_mask = screen_norm[valid_mask] > 1e-5
	xy[screen_norm_mask] = xy[screen_norm_mask] / screen_norm[valid_mask, None]

	# Map disk to a sphere
	cos_theta = np.cos(theta[valid_mask])
	sin_theta = np.sqrt(1.0 - cos_theta ** 2)

	# Todo: is this right? Do we assume z is negative (RH coordinate system)?
	z = -cos_theta
	xy = xy * sin_theta[:, None]
	direction[valid_mask] = np.stack([xy[valid_mask, 0], xy[valid_mask, 1], z], axis=1)
	extent[valid_mask] = 1.0  # < far clip is not a plane, it's a sphere!

	return direction, extent


def project_fish_eye_map_to_sphere(direction):
	z = direction[:, 2:]
	cos_theta = -z
	theta = np.arccos(np.clip(cos_theta, 0.0, 1.0))
	theta = np.arccos(cos_theta)

	# TODO currently projecting outside of max FOV
	sin_theta = np.sqrt(1.0 - cos_theta * cos_theta + EPS)
	xy = direction[:, :2] / (sin_theta + EPS)
	return xy, theta


def fish_eye_polynomial(ndc, view_params):
	""" FTheta camera model based on DW src/rigconfiguration/CameraModelsNoEigen.hpp """

	# Convert NDC pixel position to screen space... well almost. It is screen space but the extent of x is [-0.5, 0.5]
	# and the extent of y is [-0.5/aspectRatio, 0.5/aspectRatio].
	screen = ndc - 0.5
	aspect_ratio = view_params["aspect_ratio"]
	screen[:, 1] /= -aspect_ratio

	# The FTheta polynomial works at a nominal resolution. So far we have done calculations in NDC to be
	# resolution independent. Here we scale by the nominal resolution in X.
	screen = (screen - view_params["ftheta"]["c_ndc"]) * view_params["ftheta"]["width"]

	# Compute the radial distance on the screen from its center point
	r = np.sqrt(screen[:, 0] ** 2 + screen[:, 1] ** 2)
	theta = ftheta_distortion(view_params["ftheta"], r)
	max_fov = math.radians(view_params["ftheta"]["max_fov"] / 2)
	return _fish_eye_map_to_sphere(screen, r, theta, max_fov)


def project_fish_eye_polynomial(points, view_params):
	""" Project F-Theta camera model.

    Args:
        points (numpy.ndarray): Array of points in world frame of shape (num_points, 3).
        view_params (dict): dictionary containing view parameters

    Returns:
        (numpy.ndarray): Image-space points of shape (num_points, 3)
    """
	points_h = np.pad(points, ((0, 0), (0, 1)), constant_values=1)
	points_cam_frame = np.einsum("jk,kl->jl", points_h, view_params["world_to_view"])[..., :3]
	directions = points_cam_frame / np.linalg.norm(points_cam_frame + EPS, axis=1)[:, None]
	xy, theta = project_fish_eye_map_to_sphere(directions)
	r = _ftheta_distortion_solver(view_params["ftheta"], theta)
	screen = xy * r
	screen = screen / view_params["ftheta"]["width"] + view_params["ftheta"]["c_ndc"]
	screen[:, 1] *= -view_params["aspect_ratio"]
	ndc = screen + 0.5
	ndc = np.pad(ndc, ((0, 0), (0, 1)), constant_values=0)
	return ndc


def get_view_params(viewport):
	""" Get view parameters.
    Args:
        viewport (opaque Viewport instance): Viewport from which to retrieve/create sensor.

    Returns:
        (dict): Dictionary containing view parameters.
    """
	stage = omni.usd.get_context().get_stage()
	camera = stage.GetPrimAtPath(viewport.camera_path)
	current_time = omni.timeline.get_timeline_interface().get_current_time()
	view_to_world = UsdGeom.Imageable(camera).ComputeLocalToWorldTransform(current_time)
	world_to_view = view_to_world.GetInverse()
	width, height = viewport.resolution

	projection_type = camera.GetAttribute("cameraProjectionType").Get(current_time)

	if projection_type == "fisheyePolynomial":
		ftheta = {
			"width": camera.GetAttribute("fthetaWidth").Get(),
			"height": camera.GetAttribute("fthetaHeight").Get(),
			"cx": camera.GetAttribute("fthetaCx").Get(),
			"cy": camera.GetAttribute("fthetaCy").Get(),
			"poly_a": camera.GetAttribute("fthetaPolyA").Get(),
			"poly_b": camera.GetAttribute("fthetaPolyB").Get(),
			"poly_c": camera.GetAttribute("fthetaPolyC").Get(),
			"poly_d": camera.GetAttribute("fthetaPolyD").Get(),
			"poly_e": camera.GetAttribute("fthetaPolyE").Get(),
			"max_fov": camera.GetAttribute("fthetaMaxFov").Get(),
		}
		ftheta["edge_fov"] = ftheta_distortion(ftheta, ftheta["width"] / 2)
		ftheta["c_ndc"] = np.array(
			[
				(ftheta["cx"] - ftheta["width"] / 2) / ftheta["width"],
				(ftheta["height"] / 2 - ftheta["cy"]) / ftheta["width"],
			]
		)
	else:
		ftheta = None

	view_params = {
		"view_to_world": np.array(view_to_world),
		"world_to_view": np.array(world_to_view),
		"projection_type": projection_type,
		"ftheta": ftheta,
		"width": width,
		"height": height,
		"aspect_ratio": width / height,
		"clipping_range": camera.GetAttribute("clippingRange").Get(current_time),
		"horizontal_aperture": camera.GetAttribute("horizontalAperture").Get(current_time),
		"focal_length": camera.GetAttribute("focalLength").Get(current_time),
	}
	return view_params


def image_to_world(image_coordinates, view_params):
	""" Map each image coordinate to a corresponding direction vector.
    Args:
        pixel (numpy.ndarray): Pixel coordinates of shape (num_pixels, 2)
        view_params (dict): dictionary containing view parameters
    Returns
        (numpy.ndarray): Direction vectors of shape (num_pixels, 3)
    """

	ndc = image_coordinates / np.array([view_params["width"], view_params["height"]])
	direction, extent = fish_eye_polynomial(ndc, view_params)
	view_to_world = view_params["view_to_world"]
	origin = np.matmul(np.array([0, 0, 0, 1]), view_to_world)[:3]
	direction = np.matmul(np.pad(direction, ((0, 0), (0, 1)), constant_values=0), view_to_world)[:, :3]
	direction /= np.linalg.norm(direction, axis=1, keepdims=True)
	return origin, direction


def world_to_image(points, viewport, view_params=None):
	""" Project world coordinates to image-space.
    Args:
        points (numpy.ndarray): Array of points in world frame of shape (num_points, 3).
        viewport (opaque Viewport instance): Viewport from which to retrieve/create sensor.
        view_params (dict, Optional): View parameters dictionary obtained from
            `omni.syntheticdata.helpers.get_view_params(viewport)`. Will be computed for current viewport state
            if not provided.

    Returns:
        (numpy.ndarray): Image-space points of shape (num_points, 3)
    """
	if view_params is None:
		view_params = get_view_params(viewport)
	if view_params["projection_type"] == "pinhole" or view_params["projection_type"] is None:
		points_image_space = project_pinhole(points, view_params)
	elif view_params["projection_type"] == "fisheyePolynomial":
		points_image_space = project_fish_eye_polynomial(points, view_params)
	else:
		raise ValueError(f"Projection type {view_params['projection_type']} is not currently supported.")
	return points_image_space


def ftheta_distortion(ftheta, x):
	""" F-Theta distortion. """
	return ftheta["poly_a"] + x * (
			ftheta["poly_b"] + x * (ftheta["poly_c"] + x * (ftheta["poly_d"] + x * ftheta["poly_e"]))
	)


def ftheta_distortion_prime(ftheta, x):
	""" Derivative to f_theta_distortion. """
	return ftheta["poly_b"] + x * (2 * ftheta["poly_c"] + x * (3 * ftheta["poly_d"] + x * 4 * ftheta["poly_e"]))


def _ftheta_distortion_solver(ftheta, y):
	# Guess by linear approximation. 2 loops provides sufficient precision in working range.
	ratio = ftheta["width"] / 2 / ftheta["edge_fov"]
	guess = y * ratio
	for i in range(2):
		guessed_y = ftheta_distortion(ftheta, guess)
		dy = y - guessed_y
		dx = ftheta_distortion_prime(ftheta, guess)
		mask = dx != 0
		guess[mask] += dy[mask] / dx[mask]
		guess[~mask] += dy[~mask] * ratio
	return guess
