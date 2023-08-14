import carb
import omni.usd
import omni.kit
from pxr import UsdGeom
import numpy as np
import asyncio

from .. import _syntheticdata
from . import helpers

from .SyntheticData import *


def get_synthetic_data():
	sdg = SyntheticData.Get()
	if not sdg:
		SyntheticData.Initialize()
		sdg = SyntheticData.Get()
	assert sdg
	return sdg


async def next_sensor_data_async(viewport=None, waitSimFrame: bool = False, inViewportId: int = None):
	"""Wait for frame complete event from Kit for specific viewport. """
	# next_sensor_data_async API previously passed inViewportId as ViewportHandle
	# This is actually incorrect and bad due to waiting on that handle, which can
	# change for a variety of reasons between the retrieval of the handle and
	# the wait on it below.
	if hasattr(viewport, "frame_info"):
		inViewportId = viewport.frame_info.get("viewport_handle")
	else:
		if inViewportId is None:
			if isinstance(viewport, int):
				inViewportId = viewport
			else:
				inViewportId = 0
		viewport = None
		carb.log_warn(
			f"Depreacted usage of next_sensor_data_async with inViewportId={inViewportId}, pass the Viewport instead")

	app = omni.kit.app.get_app()

	# wait for the next pre_update call
	pre_f = asyncio.Future()

	def on_pre_event(e: carb.events.IEvent):
		if not pre_f.done():
			swhFrameNumber = e.payload["SWHFrameNumber"]
			# drivesim legacy name
			if not swhFrameNumber:
				swhFrameNumber = e.payload["frameNumber"]
			pre_f.set_result(swhFrameNumber)

	sub_pre = app.get_pre_update_event_stream().create_subscription_to_pop(on_pre_event,
	                                                                       name="omni.kit.app._pre_update_async")

	# wait the next frame to be rendered
	render_f = asyncio.Future()

	def on_render_event(e: carb.events.IEvent):
		# Grab the ViewportHandle to match from the Viewport if we have it or the legacy inViewportId
		cur_viewport_handle = viewport.frame_info.get("viewport_handle") if viewport else inViewportId
		viewId = e.payload["viewport_handle"]
		frameNumber = e.payload["swh_frame_number"]
		if ((viewId == cur_viewport_handle) and (not waitSimFrame or (pre_f.done() and (frameNumber >= pre_f.result())))):
			if not render_f.done():
				render_f.set_result(frameNumber)

	sub_render = (
		omni.usd.get_context()
		.get_rendering_event_stream()
		.create_subscription_to_pop_by_type(
			int(omni.usd.StageRenderingEventType.NEW_FRAME),
			on_render_event,
			name="omni.syntheticdata.sensors._next_sensor_data_async",
			order=0,
		)
	)

	MAX_NUM_SKIPPED_UPDATE = 150
	num_skipped_update = 0
	while (num_skipped_update < MAX_NUM_SKIPPED_UPDATE) and (not render_f.done()):
		await app.next_update_async()
		num_skipped_update += 1
	if num_skipped_update >= MAX_NUM_SKIPPED_UPDATE:
		raise SyntheticDataException(f"waiting for next frame failed.")


def enable_sensors(viewport, sensor_types):
	""" activate the host buffer copy nodes for given sensor

        NB: This function is deprecated
    """
	for sensor_type in sensor_types:
		rendervar_name = SyntheticData.convert_sensor_type_to_rendervar(sensor_type.name)
		get_synthetic_data().activate_node_template(rendervar_name + "ExportRawArray", 0, [viewport.render_product_path])


def disable_sensors(viewport, sensor_types):
	""" deactivate the host buffer copy nodes for given sensor

        NB: This function is deprecated
    """
	for sensor_type in sensor_types:
		rendervar_name = SyntheticData.convert_sensor_type_to_rendervar(sensor_type.name)
		get_synthetic_data().deactivate_node_template(rendervar_name + "ExportRawArray", 0, [viewport.render_product_path])


def create_or_retrieve_sensor(viewport, sensor_type):
	""" Retrieve a sensor for the specified viewport and sensor type.
    If the sensor does not exist, it is created. Note that the sensor will be
    uninitialized until a frame is rendered after the sensor is created.

    NB: This function is deprecated and the asynchronous version below
    (create_or_retrieve_sensor_async) should be used instead to ensure sensors
    are properly initialized by the renderer after creation

    Args:
        viewport (opaque Viewport instance): Viewport from which to retrieve/create sensor.
        sensor_type (omni.syntheticdata._syntheticdata.SensorType): Type of sensor to retrieve/create.
    """
	enable_sensors(viewport, [sensor_type])
	return sensor_type


async def create_or_retrieve_sensor_async(viewport, sensor_type):
	""" Retrieve a sensor for the specified viewport and sensor type.
    If the sensor does not exist, it is created. Note that the sensor will be
    uninitialized until a frame is rendered after the sensor is created.

    Args:
        viewport (opaque Viewport instance): Viewport from which to retrieve/create sensor.
        sensor_type (omni.syntheticdata._syntheticdata.SensorType): Type of sensor to retrieve/create.
    """
	enable_sensors(viewport, [sensor_type])
	await next_sensor_data_async(viewport, True)
	return sensor_type


async def initialize_async(viewport, sensor_types):
	""" Initialize sensors in the list provided.

    Args:
        viewport (opaque Viewport instance): Viewport from which to retrieve/create sensor.
        sensor_types (list of omni.syntheticdata._syntheticdata.SensorType): List of sensor types to initialize.
    """
	await omni.kit.app.get_app_interface().next_update_async()
	enable_sensors(viewport, sensor_types)
	await next_sensor_data_async(viewport, True)


def get_sensor_array(viewport, sensor_type, elemType, elemCount, is2DArray):
	""" Retrieve the sensor array data from the last sensor node evaluation.

    Args:
        viewport (opaque Viewport instance): Viewport from which to retrieve/create sensor.
        sensor_type : Sensor type to retrieve the data from.
        is2DArray : True if the array to be retrieved is a 2d array
    """
	output_names = ["outputs:data"]
	if is2DArray:
		output_names.append("outputs:width")
		output_names.append("outputs:height")
	else:
		output_names.append("outputs:bufferSize")

	rendervar_name = SyntheticData.convert_sensor_type_to_rendervar(sensor_type.name)
	outputs = get_synthetic_data().get_node_attributes(rendervar_name + "ExportRawArray", output_names,
	                                                   viewport.render_product_path)

	data = outputs["outputs:data"] if outputs and ("outputs:data" in outputs) else None
	if is2DArray:
		height = outputs["outputs:height"] if outputs and ("outputs:height" in outputs) else 0
		width = outputs["outputs:width"] if outputs and ("outputs:width" in outputs) else 0
		bufferSize = height * width * elemCount * np.dtype(elemType).itemsize
	else:
		bufferSize = outputs["outputs:bufferSize"] if outputs and ("outputs:bufferSize" in outputs) else 0

	if (data is None) or (len(data) < np.dtype(elemType).itemsize):
		if is2DArray:
			shape = (0, 0, elemCount) if elemCount > 1 else (0, 0)
		else:
			shape = (0, elemCount) if elemCount > 1 else (0)
		return np.empty(shape, elemType)

	assert bufferSize == len(data)

	data = data.view(elemType)
	assert len(data) > 0

	if not is2DArray:
		return data.reshape(data.shape[0] // elemCount, elemCount) if elemCount > 1 else data

	return data.reshape(height, width, elemCount) if elemCount > 1 else data.reshape(height, width)


def get_rgb(viewport):
	""" Get RGB sensor output.

    Args:
        viewport (opaque Viewport instance): Viewport from which to retrieve/create sensor.

    Return:
        (numpy.ndarray): A uint8 array of shape (height, width, 4)
    """
	return get_sensor_array(viewport, _syntheticdata.SensorType.Rgb, np.uint8, 4, True)


def get_depth(viewport):
	""" Get Inverse Depth sensor output. *** DEPRECATED ***

    Args:
        viewport (opaque Viewport instance): Viewport from which to retrieve/create sensor.

    Return:
        (numpy.ndarray): A float32 array of shape (height, width, 1).
    """
	return get_sensor_array(viewport, _syntheticdata.SensorType.Depth, np.float32, 1, True)


def get_depth_linear(viewport):
	""" Get Linear Depth sensor output. *** DEPRECATED ***

    Args:
        viewport (opaque Viewport instance): Viewport from which to retrieve/create sensor.

    Return:
        (numpy.ndarray): A float32 array of shape (height, width, 1).
    """
	return get_sensor_array(viewport, _syntheticdata.SensorType.DepthLinear, np.float32, 1, True)


def get_distance_to_image_plane(viewport):
	""" Get distance to image plane sensor output.

    Args:
        viewport (opaque Viewport instance): Viewport from which to retrieve/create sensor.

    Return:
        (numpy.ndarray): A float32 array of shape (height, width, 1).
    """
	return get_sensor_array(viewport, _syntheticdata.SensorType.DistanceToImagePlane, np.float32, 1, True)


def get_distance_to_camera(viewport):
	""" Get distance to camera sensor output.

    Args:
        viewport (opaque Viewport instance): Viewport from which to retrieve/create sensor.

    Return:
        (numpy.ndarray): A float32 array of shape (height, width, 1).
    """
	return get_sensor_array(viewport, _syntheticdata.SensorType.DistanceToCamera, np.float32, 1, True)


def get_camera_3d_position(viewport):
	""" Get camera space 3d position sensor output.

    Args:
        viewport (opaque Viewport instance): Viewport from which to retrieve/create sensor.

    Return:
        (numpy.ndarray): A float32 array of shape (height, width, 4).
    """
	return get_sensor_array(viewport, _syntheticdata.SensorType.Camera3dPosition, np.float32, 4, True)


def get_bounding_box_3d(viewport, parsed=False, return_corners=False, camera_frame=False, instance_mappings=None):
	""" Get bounding box 3D sensor output.

    Args:
        viewport (opaque Viewport instance): Viewport from which to retrieve/create sensor.
        parsed (bool): If True, return a single bounding box for each prim with a semantic schema.
            Otherwise, a bounding box will be provided for each leaf prim.
        include_corners (bool): if True, calculate and return the 8 corners of each 3D bounding box.
            Corners are returned in the order: [LDB, RDB, LUB, RUB, LDF, RDF, LUF, RUF] where
            L=Left, R=Right, D=Down, U=Up, B=Back, F=Front and LR: x-axis, UD: y-axis, FB: z-axis.
        camera_frame (bool): If True, the transforms and corners will be returned in the camera's
            reference frame. Otherwise, coordinates are returned with respect to the world frame.
            Note: The coordinate system is right-handed.
        instance_mappings (numpy.ndarray, optional): A structured array returned by `helpers.get_instance_mappings`.
            If not provided (default), a new instance mappings will be computed.

    Return:
        (numpy.ndarray): A structured array with the fields: `[('instanceId', '<u4'), ('semanticId', '<u4'),
        ("metadata", "O"), ('x_min', '<f4'), ('y_min', '<f4'), ('z_min', '<f4'), ('x_max', '<f4'), ('y_max', '<f4'),
        ('z_max', '<f4'), ('transform', '<f4', (4, 4))]`. If `return_corners` is `True`, an additional
        field `('corners', '<f4', (8, 3)` is returned.
    """
	BoundingBox3DValuesType = np.dtype(
		[
			("instanceId", "<u4"),
			("semanticId", "<u4"),
			("x_min", "<f4"),
			("y_min", "<f4"),
			("z_min", "<f4"),
			("x_max", "<f4"),
			("y_max", "<f4"),
			("z_max", "<f4"),
			("transform", "<f4", (4, 4)),
		]
	)
	bboxes_3d_data = get_sensor_array(
		viewport, _syntheticdata.SensorType.BoundingBox3D, BoundingBox3DValuesType, 1, False
	)

	# Return immediately if empty
	if bboxes_3d_data.size == 0:
		return bboxes_3d_data

	if return_corners:
		corners = helpers.get_bbox_3d_corners(bboxes_3d_data)
		corners_struc = np.zeros(len(corners), dtype=[("corners", np.float32, (8, 3))])
		corners_struc["corners"] = corners
		bboxes_3d_data = helpers._join_struct_arrays([bboxes_3d_data, corners_struc])

	if parsed:
		if instance_mappings is None:
			instance_mappings = helpers.get_instance_mappings()
		bboxes_3d_data = helpers.reduce_bboxes_3d(bboxes_3d_data, instance_mappings)

	if camera_frame:
		stage = omni.usd.get_context().get_stage()
		camera = stage.GetPrimAtPath(viewport.camera_path)
		current_time = omni.timeline.get_timeline_interface().get_current_time()
		tf_mat = np.array(UsdGeom.Camera(camera).ComputeLocalToWorldTransform(current_time))
		view_matrix = np.linalg.inv(tf_mat)
		bboxes_3d_data["transform"] = np.einsum("ijk,kl->ijl", bboxes_3d_data["transform"], view_matrix)
		if return_corners:
			corners_homo = np.pad(bboxes_3d_data["corners"], ((0, 0), (0, 0), (0, 1)), constant_values=1.0)
			bboxes_3d_data["corners"] = np.einsum("ijk,kl->ijl", corners_homo, view_matrix)[..., :3]
	return bboxes_3d_data


def get_bounding_box_2d_tight(viewport, instance_mappings=None):
	""" Get Bounding Box 2D Tight sensor output.
    Tight bounding boxes only bound the visible or unoccluded portions of an object. If an object is
    completely occluded, it is omitted from the returned array. Bounds units are in pixels.

    Args:
        viewport (opaque Viewport instance): Viewport from which to retrieve/create sensor.
        instance_mappings (numpy.ndarray, optional): A structured array returned by `helpers.get_instance_mappings`.
            If not provided (default), a new instance mappings will be computed.

    Return:
        (np.ndarray): A structured numpy array with the fields: `[('name', 'O'), ('semanticLabel', 'O'),
            ('instanceId', '<u4'), ('semanticId', '<u4'), ("metadata", "O"), ('x_min', '<i4'), ('y_min', '<i4'),
            ('x_max', '<i4'), ('y_max', '<i4')]`
    """
	BoundingBox2DValuesType = np.dtype(
		[
			("instanceId", "<u4"),
			("semanticId", "<u4"),
			("x_min", "<i4"),
			("y_min", "<i4"),
			("x_max", "<i4"),
			("y_max", "<i4"),
		]
	)
	bboxes_2d_data = get_sensor_array(
		viewport, _syntheticdata.SensorType.BoundingBox2DTight, BoundingBox2DValuesType, 1, is2DArray=False
	)
	if instance_mappings is None:
		instance_mappings = helpers.get_instance_mappings()
	bboxes_2d_data = helpers.reduce_bboxes_2d(bboxes_2d_data, instance_mappings)
	return bboxes_2d_data


def get_bounding_box_2d_loose(viewport, instance_mappings=None):
	""" Get Bounding Box 2D Loose sensor output.
    Loose bounding boxes bound the full extents of an object, even if totally occluded. Bounds units are
    in pixels.

    Args:
        viewport (opaque Viewport instance): Viewport from which to retrieve/create sensor.
        instance_mappings (numpy.ndarray, optional): A structured array returned by `helpers.get_instance_mappings`.
            If not provided (default), a new instance mappings will be computed.
    Return:
        (np.ndarray): A structured numpy array with the fields: `[('name', 'O'), ('semanticLabel', 'O'),
            ('instanceId', '<u4'), ('semanticId', '<u4'), ("metadata", "O"), ('x_min', '<i4'), ('y_min', '<i4'),
            ('x_max', '<i4'), ('y_max', '<i4')]`
    """
	BoundingBox2DValuesType = np.dtype(
		[
			("instanceId", "<u4"),
			("semanticId", "<u4"),
			("x_min", "<i4"),
			("y_min", "<i4"),
			("x_max", "<i4"),
			("y_max", "<i4"),
		]
	)
	bboxes_2d_data = get_sensor_array(
		viewport, _syntheticdata.SensorType.BoundingBox2DLoose, BoundingBox2DValuesType, 1, is2DArray=False
	)
	if instance_mappings is None:
		instance_mappings = helpers.get_instance_mappings()
	bboxes_2d_data = helpers.reduce_bboxes_2d(bboxes_2d_data, instance_mappings)
	return bboxes_2d_data


def get_semantic_segmentation(viewport):
	""" Get semantic segmentation sensor output.

    Args:
        viewport (opaque Viewport instance): Viewport from which to retrieve/create sensor.

    Return:
        output (np.ndarray): A uint32 array of shape `(height, width)`.
    """
	return get_sensor_array(viewport, _syntheticdata.SensorType.SemanticSegmentation, np.uint32, 1, True)


def get_instance_segmentation(viewport, parsed=False, return_mapping=False, instance_mappings=None):
	""" Get instance segmentation sensor output.

    Args:
        viewport (opaque Viewport instance): Viewport from which to retrieve/create sensor.
        parsed (bool): If True, map each leaf prim to a parent with a semantic schema applied. Otherwise,
            each leaf prim is returned as a unique instance.
        return_mapping (bool): Whether to also return an array mapping instance IDs to their corresponding prims.
        instance_mappings (numpy.ndarray, optional): A structured array returned by `helpers.get_instance_mappings`.
            If not provided (default), a new instance mappings will be computed.

    Return:
        output (np.ndarray): A uint32 array of shape `(height, width)`.
        mapping (list): (optional) If `return_mapping` is True, there will be an additional array containing the
            mapping of instance IDs to their corresponding prims. Each row corresponds to a prim with a SemanticSchema
            of Type="class". The mapping is provided in the following format:
                (ID (int), path (str), semanticID (int), semanticLabel (str), descendentIDs (list of int))
    """
	instance_data = get_sensor_array(viewport, _syntheticdata.SensorType.InstanceSegmentation, np.uint32, 1, True)

	if parsed:
		if instance_mappings is None:
			instance_mappings = helpers.get_instance_mappings()
		if len(instance_mappings) == 0:
			return instance_data
		instances_list = [(im[0], im[4]) for im in instance_mappings][::-1]
		if len(instances_list) == 0:
			carb.log_warn("[omni.syntheticdata.visualize] No instances found.")
			return instance_data
		max_instance_id_list = max([max(il[1]) for il in instances_list])
		max_instance_id = instance_data.max()
		lut = np.zeros(max(max_instance_id, max_instance_id_list) + 1, dtype=np.uint32)
		# todo this avoids that overlapping instances are mapped to the same id, but it's not clear if this is the correct way
		for uid, il in instances_list:
			for j in il:
				if lut[j] == 0:
					lut[j] = uid
		instance_data = np.take(lut, instance_data)

	if return_mapping:
		if instance_mappings is None:
			instance_mappings = helpers.get_instance_mappings()
		return instance_data, instance_mappings
	else:
		return instance_data


def get_normals(viewport):
	""" Get Normals sensor output.

    Args:
        viewport (opaque Viewport instance): Viewport from which to retrieve/create sensor.

    Return:
        (numpy.ndarray): A float32 array of shape `(height, width, 3)` with values in the range of
            `(-1., 1.)`.
    """
	normals = get_sensor_array(viewport, _syntheticdata.SensorType.Normal, np.float32, 4, True)[..., :3]

	# Return (0, 0, 0) for background pixels
	# HACK: background is returned as (-0., -0., 1.), so use negative sign of 0 as background indicator
	bkg_mask = np.all(normals == np.array([0.0, 0.0, -1.0]), axis=-1) & np.all(
		np.copysign(np.ones_like(normals), normals) == -np.ones(3), axis=-1
	)
	normals[bkg_mask] = 0.0

	return normals


def get_motion_vector(viewport):
	""" Get Motion Vector sensor output.

    Args:
        viewport (opaque Viewport instance): Viewport from which to retrieve/create sensor.

    Return:
        TOCHECK : this does not describe what the legacy interface was returning
        (numpy.ndarray): A float32 array of shape `(height, width, 3)` with values in the range of
            `(-1., 1.)`.
    """
	return get_sensor_array(viewport, _syntheticdata.SensorType.MotionVector, np.float32, 4, True)


def get_cross_correspondence(viewport):
	""" Get Cross Correspondence sensor output.

    Args:
        viewport (opaque Viewport instance): Viewport from which to retrieve/create sensor.

    Return:
        (numpy.ndarray): A float32 array of shape `(height, width, 4)` with values in the range of
            `(-1., 1.)`.
    """
	return get_sensor_array(viewport, _syntheticdata.SensorType.CrossCorrespondence, np.float32, 4, True)


def get_occlusion(viewport, parsed=False, instance_mappings=None):
	"""Get Occlusion values.

    Returns occlusion of instances as a ratio from 0. to 1. Note that this sensor is only applied to leaf prims.
    For example, if an asset is composed of multiple sub-meshes, an occlusion value will be calculated for each
    sub-mesh.

    Args:
        viewport (opaque Viewport instance): Viewport from which to retrieve/create sensor.
        parsed (bool): If True, map occlusion values to prims with a semantic class. If the mapped prim has more than
            one child with an occlusion value, a naive average will be performed. Note that this value will likely not
            be accurate.
        instance_mappings (numpy.ndarray, optional): A structured array returned by `helpers.get_instance_mappings`.
            If not provided (default), a new instance mappings will be computed.

    Returns:
        (numpy.ndarray): A structured numpy array with the fields: [('instanceId', '<u4'), ('semanticId', '<u4'),
            ('occlusionRatio', '<f4')], where occlusion ranges from 0 (not occluded) to 1 (fully occluded).
            If `parsed` is True, the additional fields [('name', 'O'), ('semanticLabel', 'O'), ("metadata", "O")]
            are returned.
    """
	OcclusionType = np.dtype([("instanceId", "<u4"), ("semanticId", "<u4"), ("occlusionRatio", "<f4")])
	data = get_sensor_array(viewport, _syntheticdata.SensorType.Occlusion, OcclusionType, 1, is2DArray=False)

	if parsed:
		if instance_mappings is None:
			instance_mappings = helpers.get_instance_mappings()
		return helpers.reduce_occlusion(data, instance_mappings)
	return data


def get_semantic_data(instance_mappings=None):
	""" Get Semantic Data.

    Args:
        instance_mappings (numpy.ndarray, optional): A structured array returned by `helpers.get_instance_mappings`.
            If not provided (default), a new instance mappings will be computed.

    Returns:
        (numpy.ndarray): A structured numpy array with the fields: [('uniqueId', '<i4'),
            ('name', 'O'), ('semanticLabel', 'O'), ('metadata', 'O')]
    """
	if instance_mappings is None:
		instance_mappings = helpers.get_instance_mappings()
	output = []
	for row in instance_mappings:
		output.append((row[0], row[1], row[3], row[5]))
	output = np.array(output, dtype=[("uniqueId", np.int32), ("name", "O"), ("semanticLabel", "O"), ("metadata", "O")])
	return output


def get_occlusion_quadrant(viewport, return_bounding_boxes=False):
	""" Get Occlusion Quadrant.

    Uses loose and tight bounding boxes to return the occluded quadrant of all prims with semantic class.
    Note that the label "fully-visible" specifies that the full height and width of the prim's bounds
    can be determined, and the prim may still be partially occluded.

    Args:
        viewport (opaque Viewport instance): Viewport from which to retrieve/create sensor.

    Returns:
        (numpy.ndarray): A structured numpy array with the fields: [('name', 'O'), ('semanticLabel', 'O'),
            ('instanceId', '<u4'), ('semanticId', '<u4'), ('occlusion_quadrant', 'O')], where
            occlusion_quadrant is a string from ['bottom', 'top', 'right', 'left',
            'bottom-right', 'bottom-left', 'top-right', 'top-left', 'fully-visible', 'fully-occluded'].
            If `return_bounding_boxes` is True, the fields `x_min`, `y_min`, `x_max`, `y_max` for
            with suffixes `_bbox2d_tight` and `_bbox2d_loose` will be returned as well.
    """
	tight_data = get_bounding_box_2d_tight(viewport)
	loose_data = get_bounding_box_2d_loose(viewport)

	merged_data = helpers.merge_sensors(bounding_box_2d_tight=tight_data, bounding_box_2d_loose=loose_data)

	is_fully_occluded = merged_data["x_min_bbox2d_tight"] == -1
	is_occluded_left = (merged_data["x_min_bbox2d_tight"] > merged_data["x_min_bbox2d_loose"]) & ~is_fully_occluded
	is_occluded_right = (merged_data["x_max_bbox2d_tight"] < merged_data["x_max_bbox2d_loose"]) & ~is_fully_occluded
	is_occluded_top = (merged_data["y_min_bbox2d_tight"] > merged_data["y_min_bbox2d_loose"]) & ~is_fully_occluded
	is_occluded_bottom = (merged_data["y_max_bbox2d_tight"] < merged_data["y_max_bbox2d_loose"]) & ~is_fully_occluded

	is_occluded_bottom_left = is_occluded_bottom & is_occluded_left
	is_occluded_bottom_right = is_occluded_bottom & is_occluded_right
	is_occluded_top_right = is_occluded_top & is_occluded_right
	is_occluded_top_left = is_occluded_top & is_occluded_left

	label = np.array(["fully-visible"] * len(merged_data), dtype=[("occlusion_quadrant", "O")])
	label[is_occluded_top] = "top"
	label[is_occluded_bottom] = "bottom"
	label[is_occluded_right] = "right"
	label[is_occluded_left] = "left"
	label[is_occluded_bottom_left] = "bottom-left"
	label[is_occluded_bottom_right] = "bottom-right"
	label[is_occluded_top_left] = "top-left"
	label[is_occluded_top_right] = "top-right"
	label[is_fully_occluded] = "fully-occluded"

	if return_bounding_boxes:
		output = helpers._join_struct_arrays([merged_data, label])
	else:
		output = helpers._join_struct_arrays(
			[merged_data[["uniqueId", "name", "semanticLabel", "metadata", "instanceIds"]], label]
		)

	return output
