# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import struct
import random
import colorsys
import numpy as np
from PIL import Image, ImageDraw


def random_colours(N, enable_random=True, num_channels=3):
    """
    Generate random colors.
    Generate visually distinct colours by linearly spacing the hue 
    channel in HSV space and then convert to RGB space.
    """
    start = 0
    if enable_random:
        random.seed(10)
        start = random.random()
    hues = [(start + i / N) % 1.0 for i in range(N)]
    colours = [list(colorsys.hsv_to_rgb(h, 0.9, 1.0)) for i, h in enumerate(hues)]
    if num_channels == 4:
        for color in colours:
            color.append(1.0)
    if enable_random:
        random.shuffle(colours)
    return colours


def plot_boxes(ax, bboxes, labels=None, colours=None, label_size=10):
    import matplotlib.pyplot as plt

    if colours is None:
        colours = random_colours(len(bboxes))
    if labels is None:
        labels = [""] * len(bboxes)
    for bb, label, colour in zip(bboxes, labels, colours):
        maxint = 2 ** (struct.Struct("i").size * 8 - 1) - 1
        # if a bbox is not visible, do not draw
        if bb[0] != maxint and bb[1] != maxint:
            x = bb[0]
            y = bb[1]
            w = bb[2] - x
            h = bb[3] - y
            box = plt.Rectangle((x, y), w, h, fill=False, edgecolor=colour)
            ax.add_patch(box)
            if label:
                font = {"family": "sans-serif", "color": colour, "size": label_size}
                ax.text(bb[0], bb[1], label, fontdict=font)


def colorize_depth(depth_image, width, height, num_channels=3):
    """ Colorizes depth data for visualization.


        Args:
            depth_image (numpy.ndarray): Depth data from the sensor.
            width (int): Width of the viewport.
            height (int): Height of the viewport.
            num_channels (int): Specify number of channels i.e. 3 or 4.
    """
    colorized_image = np.zeros((height, width, num_channels))
    depth_image[depth_image == 0.0] = 1e-5
    depth_image = np.clip(depth_image, 0, 255)
    depth_image -= np.min(depth_image)
    depth_image /= np.max(depth_image) - np.min(depth_image)
    colorized_image[:, :, 0] = depth_image
    colorized_image[:, :, 1] = depth_image
    colorized_image[:, :, 2] = depth_image
    if num_channels == 4:
        colorized_image[:, :, 3] = 1
    colorized_image = (colorized_image * 255).astype(int)
    return colorized_image


def colorize_segmentation(segmentation_image, width, height, num_channels=3, num_colors=None):
    """ Colorizes segmentation data for visualization.


        Args:
            segmentation_image (numpy.ndarray): Segmentation data from the sensor.
            width (int): Width of the viewport.
            height (int): Height of the viewport.
            num_channels (int): Specify number of channels i.e. 3 or 4.
            num_colors (int): Specify number of colors for consistency across frames.
    """
    segmentation_mappings = segmentation_image[:, :, 0]
    segmentation_list = np.unique(segmentation_mappings)
    if num_colors is None:
        num_colors = np.max(segmentation_list) + 1
    color_pixels = random_colours(num_colors, True, num_channels)
    color_pixels = [[color_pixel[i] * 255 for i in range(num_channels)] for color_pixel in color_pixels]
    segmentation_masks = np.zeros((len(segmentation_list), *segmentation_mappings.shape), dtype=np.bool)
    index_list = []
    for index, segmentation_id in enumerate(segmentation_list):
        segmentation_masks[index] = segmentation_mappings == segmentation_id
        index_list.append(segmentation_id)
    color_image = np.zeros((height, width, num_channels), dtype=np.uint8)
    for index, mask, colour in zip(index_list, segmentation_masks, color_pixels):
        color_image[mask] = color_pixels[index] if index > 0 else 0
    return color_image


def colorize_bboxes(bboxes_2d_data, bboxes_2d_rgb, num_channels=3):
    """ Colorizes 2D bounding box data for visualization.


        Args:
            bboxes_2d_data (numpy.ndarray): 2D bounding box data from the sensor.
            bboxes_2d_rgb (numpy.ndarray): RGB data from the sensor to embed bounding box.
            num_channels (int): Specify number of channels i.e. 3 or 4.
    """
    semantic_id_list = []
    bbox_2d_list = []
    rgb_img = Image.fromarray(bboxes_2d_rgb)
    rgb_img_draw = ImageDraw.Draw(rgb_img)
    for bbox_2d in bboxes_2d_data:
        if bbox_2d[5] > 0:
            semantic_id_list.append(bbox_2d[1])
            bbox_2d_list.append(bbox_2d)
    semantic_id_list_np = np.unique(np.array(semantic_id_list))
    color_list = random_colours(len(semantic_id_list_np.tolist()), True, num_channels)
    for bbox_2d in bbox_2d_list:
        index = np.where(semantic_id_list_np == bbox_2d[1])[0][0]
        bbox_color = color_list[index]
        outline = (int(255 * bbox_color[0]), int(255 * bbox_color[1]), int(255 * bbox_color[2]))
        if num_channels == 4:
            outline = (
                int(255 * bbox_color[0]),
                int(255 * bbox_color[1]),
                int(255 * bbox_color[2]),
                int(255 * bbox_color[3]),
            )
        rgb_img_draw.rectangle([(bbox_2d[6], bbox_2d[7]), (bbox_2d[8], bbox_2d[9])], outline=outline, width=2)
    bboxes_2d_rgb = np.array(rgb_img)
    return bboxes_2d_rgb
