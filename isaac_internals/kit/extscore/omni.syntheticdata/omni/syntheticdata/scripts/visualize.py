import random
import colorsys

import numpy as np
import carb
from PIL import Image, ImageDraw

from . import helpers
from . import sensors


# Colorize Helpers
def colorize_distance(image_data):
    height, width = image_data.shape[:2]
    colorized_image = np.zeros((height, width, 4))
    image_data[image_data == 0.0] = 1e-5
    image_data = np.clip(image_data, 0, 255)
    image_data -= np.min(image_data)
    image_data /= np.max(image_data) + 1e-8
    colorized_image[:, :, 0] = image_data
    colorized_image[:, :, 1] = image_data
    colorized_image[:, :, 2] = image_data
    colorized_image[:, :, 3] = 1
    colorized_image = (colorized_image * 255).astype(int)
    return colorized_image


def colorize_segmentation(segmentation_image):
    segmentation_ids = np.unique(segmentation_image)
    num_colours = len(segmentation_ids)

    # This is to avoid generating lots of colours for semantic classes not in frame
    lut = np.array([segmentation_ids, list(range(num_colours))])
    new_segmentation_image = lut[1, np.searchsorted(lut[0, :], segmentation_image)]

    colours = np.array([[0.0] * 4] + random_colours(num_colours))
    segmentation_image_rgba = (colours[new_segmentation_image] * 255).astype(int)
    return segmentation_image_rgba


def colorize_semantic_from_instance(instance_image, instance_mappings):
    if len(instance_mappings) == 0:
        blank = np.zeros_like(instance_image)
        return colorize_segmentation(blank)
    semantic_instances = {}
    for im in instance_mappings[::-1]:
        semantic_instances.setdefault(im["semanticId"], []).extend(im["instanceIds"])
    max_semantic_instance_id = np.max([max(il) for _, il in semantic_instances.items()])
    max_instance_id = instance_image.max()
    lut = np.zeros(max(max_semantic_instance_id, max_instance_id) + 1, dtype=np.uint32)
    for i, (_, il) in enumerate(semantic_instances.items()):
        lut[np.array(il)] = i + 1  # +1 to differentiate from background
    re_instanced = np.take(lut, instance_image)
    colours = np.array([[0.0] * 4] + random_colours(len(semantic_instances)))
    instance_rgba = (colours[re_instanced] * 255).astype(np.uint8)
    return instance_rgba


def colorize_bboxes(bboxes_2d_data, bboxes_2d_rgb):
    semantic_id_list = []
    bbox_2d_list = []
    for bbox_2d in bboxes_2d_data:
        if bbox_2d["semanticId"] > 0:
            semantic_id_list.append(bbox_2d["semanticId"])
            bbox_2d_list.append(bbox_2d)
    semantic_id_list_np = np.unique(np.array(semantic_id_list))
    color_list = random_colours(len(semantic_id_list_np.tolist()))
    img = Image.fromarray(bboxes_2d_rgb)
    draw = ImageDraw.Draw(img)
    for bbox_2d in bbox_2d_list:
        index = np.where(semantic_id_list_np == bbox_2d["semanticId"])[0][0]
        bbox_color = color_list[index]
        draw.rectangle(
            xy=[(bbox_2d["x_min"], bbox_2d["y_min"]), (bbox_2d["x_max"], bbox_2d["y_max"])],
            outline=(
                int(255 * bbox_color[0]),
                int(255 * bbox_color[1]),
                int(255 * bbox_color[2]),
                int(255 * bbox_color[3]),
            ),
            width=4,
        )
    return np.asarray(img)


def colorize_bboxes_3d(bboxes_3d_corners, rgb):
    """bboxes_3d_corners: in the local camera frame"""
    height, width = rgb.shape[:2]

    # FILTER BOXES
    mask_uv = ~np.any(np.all(bboxes_3d_corners < 0, axis=1), axis=1) & ~np.any(
        np.all(bboxes_3d_corners > 1, axis=1), axis=1
    )
    mask_z = np.all(np.all(bboxes_3d_corners[..., 2:] >= 0, axis=1), axis=1) & np.all(
        np.all(bboxes_3d_corners[..., 2:] <= 1, axis=1), axis=1
    )
    bboxes_3d_corners = bboxes_3d_corners[mask_uv & mask_z]

    bboxes_3d_corners = bboxes_3d_corners[..., :2].reshape(-1, 8, 2) * np.array([[width, height]])
    face_idx_list = [[0, 1, 3, 2], [4, 5, 7, 6], [2, 3, 7, 6], [0, 1, 5, 4], [0, 2, 6, 4], [1, 3, 7, 5]]
    colours = random_colours(len(face_idx_list))

    master_overlay = np.zeros_like(rgb)
    master_overlay_img = Image.fromarray(master_overlay)
    for face_idxs, colour in zip(face_idx_list, colours):
        overlay = Image.new("RGBA", (width, height))
        draw = ImageDraw.Draw(overlay)
        colour = [int(c * 255) for c in colour]
        for p in bboxes_3d_corners:
            draw.polygon([tuple(xy) for xy in p[face_idxs]], fill=tuple([*colour[:3], 120]))
            draw.line([tuple(xy) for xy in p[face_idxs]], width=3, fill=tuple(colour))

        master_overlay_img = Image.alpha_composite(master_overlay_img, overlay)

    rgb_img = Image.fromarray(rgb)
    rgb_img = Image.alpha_composite(rgb_img, master_overlay_img)
    return np.asarray(rgb_img)


def random_colours(N):
    """
    Generate random colors.
    Generate visually distinct colours by linearly spacing the hue
    channel in HSV space and then convert to RGB space.
    """
    colour_rand = random.Random(2018)  # Produces consistent random colours
    start = colour_rand.random()
    hues = [(start + i / N) % 1.0 for i in range(N)]
    colours = [list(colorsys.hsv_to_rgb(h, 0.9, 1.0)) + [1.0] for i, h in enumerate(hues)]
    colour_rand.shuffle(colours)
    return colours


def get_bbox2d_tight(viewport):
    rgb_data = sensors.get_rgb(viewport)
    bboxes_2d_data = sensors.get_bounding_box_2d_tight(viewport)
    bboxes_2d_rgb = colorize_bboxes(bboxes_2d_data, rgb_data)
    return bboxes_2d_rgb


def get_bbox2d_loose(viewport):
    rgb_data = sensors.get_rgb(viewport)
    bboxes_2d_data = sensors.get_bounding_box_2d_loose(viewport)
    bboxes_2d_rgb = colorize_bboxes(bboxes_2d_data, rgb_data)
    return bboxes_2d_rgb


def get_normals(viewport):
    normals = sensors.get_normals(viewport)
    background_mask = np.sum(normals, axis=-1) == 0.0
    # normalize from [-1, 1] to [0, 255]
    normals = (normals + 1.0) / 2 * 255
    # Set background alpha to 0.
    normals = np.pad(normals, ((0, 0), (0, 0), (0, 1)), constant_values=255)
    normals[background_mask, 3] = 0.0
    return normals.astype(np.uint8)


def get_motion_vector(viewport):
    motion_vector = sensors.get_motion_vector(viewport)
    _min, _max = motion_vector.min(), motion_vector.max()
    motion_vector = (motion_vector - _min) / (_max - _min) * 255.0
    return motion_vector.astype(np.uint8)


def get_cross_correspondence(viewport):
    cross_correspondence = sensors.get_cross_correspondence(viewport)
    # normalize from [-1, 1] to [0, 255]
    # invalid values of -1 convert to 0
    cross_correspondence = ((cross_correspondence + 1.0) / 2) * 255
    return cross_correspondence.astype(np.uint8)


def get_instance_segmentation(viewport, mode=None):
    if not mode:
        carb.log_info('[omni.syntheticdata.visualize] No semantic mode provided, defaulting to "parsed"')
        mode = "parsed"
    if mode == "raw":
        instance_data = sensors.get_instance_segmentation(viewport, parsed=False)
    elif mode == "parsed":
        instance_data = sensors.get_instance_segmentation(viewport, parsed=True)
    else:
        raise NotImplementedError
    instance_image = colorize_segmentation(instance_data)
    return instance_image


def get_semantic_segmentation(viewport, mode=""):
    if not mode:
        carb.log_info('[omni.syntheticdata.visualize] No semantic mode provided, defaulting to "parsed"')
        mode = "instance_map"
    # s = time.time()
    if mode == "raw":
        semantic_data = sensors.get_semantic_segmentation(viewport)
        semantic_image = colorize_segmentation(semantic_data)
    elif mode == "parsed":
        instance_data = sensors.get_instance_segmentation(viewport)
        instance_mappings = helpers.get_instance_mappings()
        semantic_image = colorize_semantic_from_instance(instance_data, instance_mappings)
    else:
        raise NotImplementedError
    return semantic_image


def get_bbox3d(viewport, mode="parsed"):
    rgb_data = sensors.get_rgb(viewport)

    bbox_3d_data = sensors.get_bounding_box_3d(viewport, parsed=(mode == "parsed"), return_corners=True)
    if bbox_3d_data.size == 0:
        carb.log_info("[omni.syntheticdata.visualize] No 3D bounding boxes found.")
        return rgb_data
    bbox_3d_corners = bbox_3d_data["corners"]
    projected_corners = helpers.world_to_image(bbox_3d_corners.reshape(-1, 3), viewport).reshape(-1, 8, 3)

    bboxes_3d_rgb = colorize_bboxes_3d(projected_corners, rgb_data)
    return bboxes_3d_rgb


# *** DEPRECATED ***
def get_depth(viewport, mode="linear"):
    if mode == "linear":
        depth_data = sensors.get_depth_linear(viewport)
        depth_data[depth_data == depth_data.max()] = 0.0
    elif mode == "inverse_depth":
        depth_data = sensors.get_depth(viewport)
    else:
        raise ValueError(f"Mode {mode} is invalid. Choose between " "['linear', 'inverse_depth'].")
    return colorize_distance(depth_data.squeeze())


def get_distance(viewport, mode="image_plane"):
    if mode == "image_plane":
        distance_data = sensors.get_distance_to_image_plane(viewport)
        distance_data[distance_data == distance_data.max()] = 0.0
    elif mode == "camera":
        distance_data = sensors.get_distance_to_camera(viewport)
        distance_data[distance_data == distance_data.max()] = 0.0
    else:
        raise ValueError(f"Mode {mode} is invalid. Choose between " "['image_plane', 'camera'].")
    return colorize_distance(distance_data.squeeze())
