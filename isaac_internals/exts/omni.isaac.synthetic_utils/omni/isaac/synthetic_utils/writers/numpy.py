# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.


"""Helper class for writing groundtruth data offline in numpy format.
"""

import copy
import omni
import os
import numpy as np
from PIL import Image
from .base import BaseWriter
from omni.isaac.core.utils.viewports import get_viewport_names


class NumpyWriter(BaseWriter):
    def __init__(self, data_dir, num_worker_threads, max_queue_size=500, sensor_settings=None):
        BaseWriter.__init__(self, data_dir, num_worker_threads, max_queue_size)
        from omni.isaac.synthetic_utils import visualization

        self.visualization = visualization
        self.create_output_folders(sensor_settings)

    def worker(self):
        """Processes task from queue. Each tasks contains groundtruth data and metadata which is used to transform the output and write it to disk."""
        while True:
            groundtruth = self.q.get()
            if groundtruth is None:
                break
            filename = groundtruth["METADATA"]["image_id"]
            viewport_name = groundtruth["METADATA"]["viewport_name"]
            for gt_type, data in groundtruth["DATA"].items():
                if gt_type == "RGB":
                    self.save_image(viewport_name, gt_type, data, filename)
                elif gt_type == "DEPTH":
                    if groundtruth["METADATA"]["DEPTH"]["NPY"]:
                        self.depth_folder = self.data_dir + "/" + str(viewport_name) + "/depth/"
                        np.save(self.depth_folder + filename + ".npy", data)
                    if groundtruth["METADATA"]["DEPTH"]["COLORIZE"]:
                        self.save_image(viewport_name, gt_type, data, filename)
                elif gt_type ==  "DEPTHLINEAR":
                    if groundtruth["METADATA"]["DEPTHLINEAR"]["NPY"]:
                        self.depthLinear_folder = self.data_dir + "/" + str(viewport_name) + "/depthLinear/"
                        np.save(self.depthLinear_folder + filename + ".npy", data)
                    if groundtruth["METADATA"]["DEPTHLINEAR"]["COLORIZE"]:
                        self.save_image(viewport_name, gt_type, data, filename)
                elif gt_type == "INSTANCE":
                    self.save_segmentation(
                        viewport_name,
                        gt_type,
                        data,
                        filename,
                        groundtruth["METADATA"]["INSTANCE"]["WIDTH"],
                        groundtruth["METADATA"]["INSTANCE"]["HEIGHT"],
                        groundtruth["METADATA"]["INSTANCE"]["COLORIZE"],
						            groundtruth["METADATA"]["INSTANCE"]["MAPPINGS"],
                        groundtruth["METADATA"]["INSTANCE"]["NPY"],
                    )
                elif gt_type == "SEMANTIC":
                    self.save_segmentation(
                        viewport_name,
                        gt_type,
                        data,
                        filename,
                        groundtruth["METADATA"]["SEMANTIC"]["WIDTH"],
                        groundtruth["METADATA"]["SEMANTIC"]["HEIGHT"],
                        groundtruth["METADATA"]["SEMANTIC"]["COLORIZE"],
						            groundtruth["METADATA"]["SEMANTIC"]["MAPPINGS"],
                        groundtruth["METADATA"]["SEMANTIC"]["NPY"],
                    )
                elif gt_type in ["BBOX2DTIGHT", "BBOX2DLOOSE"]:
                    self.save_bbox(
                        viewport_name,
                        gt_type,
                        data,
                        filename,
                        groundtruth["METADATA"][gt_type]["COLORIZE"],
                        groundtruth["DATA"]["RGB"],
                        groundtruth["METADATA"][gt_type]["NPY"],
                    )
                elif gt_type  in ["BBOX3D"]:
                  self.save_bbox(
                    viewport_name,
                    gt_type,
                    data,
                    filename,
                    groundtruth["METADATA"][gt_type]["COLORIZE"],
                    groundtruth["METADATA"]["BBOX3D_IMAGE"],
                    groundtruth["METADATA"][gt_type]["NPY"],
                  )
                elif gt_type in ["MOTIONVECTOR"]:
                  self.save_motion(
                    viewport_name,
                    gt_type,
                    data,
                    filename,
                    groundtruth["METADATA"][gt_type]["COLORIZE"],
                    groundtruth["DATA"]["RGB"],
                    groundtruth["METADATA"][gt_type]["NPY"],
                  )
                elif gt_type == "CAMERA":
                    self.camera_folder = self.data_dir + "/" + str(viewport_name) + "/camera/"
                    np.save(self.camera_folder + filename + ".npy", data)
                elif gt_type == "POSES":
                    self.poses_folder = self.data_dir + "/" + str(viewport_name) + "/poses/"
                    np.save(self.poses_folder + filename + ".npy", data)
                elif gt_type == "NORMALS":
                    self.normals_folder = self.data_dir + "/" + str(viewport_name) + "/normals/"
                    np.save(self.normals_folder + filename + ".npy", data)
                else:
                    raise NotImplementedError
            self.q.task_done()

    def save_motion(
        self, viewport_name, data_type, data, filename, width=1280, height=720, display_rgb=True, save_npy=True
    ):
        self.motion_folder = self.data_dir + "/" + str(viewport_name) + "/motion-vector/"
        if save_npy:
            np.save(self.motion_folder + filename + ".npy", data)

    def save_segmentation(
        self, viewport_name, data_type, data, filename, width=1280, height=720, display_rgb=True, mappings=True,
			save_npy=True):
        self.instance_folder = self.data_dir + "/" + str(viewport_name) + "/instance/"
        self.semantic_folder = self.data_dir + "/" + str(viewport_name) + "/semantic/"
        # Save ground truth data locally as npy
        if not mappings:
            data = data[0]
        if data_type == "INSTANCE" and save_npy:
            np.save(self.instance_folder + filename + ".npy", data)
        if data_type == "SEMANTIC" and save_npy:
            np.save(self.semantic_folder + filename + ".npy", data)
        if mappings:
            data = data[0]
        if display_rgb:
            image_data = np.frombuffer(data, dtype=np.uint8).reshape(*data.shape, -1)
            num_colors = 50 if data_type == "SEMANTIC" else None
            color_image = self.visualization.colorize_segmentation(image_data, width, height, 3, num_colors)
            # color_image = visualize.colorize_instance(image_data)
            color_image_rgb = Image.fromarray(color_image, "RGB")
            if data_type == "INSTANCE":
                color_image_rgb.save(f"{self.instance_folder}/{filename}.png")
            if data_type == "SEMANTIC":
                color_image_rgb.save(f"{self.semantic_folder}/{filename}.png")

    def save_image(self, viewport_name, img_type, image_data, filename):
        self.rgb_folder = self.data_dir + "/" + str(viewport_name) + "/rgb/"
        self.depth_folder = self.data_dir + "/" + str(viewport_name) + "/depth/"
        self.depthLinear_folder = self.data_dir + "/" + str(viewport_name) + "/depthLinear/"
        if img_type == "RGB":
            # Save ground truth data locally as png
            rgb_img = Image.fromarray(image_data, "RGBA")
            rgb_img.save(f"{self.rgb_folder}/{filename}.png")
        elif img_type == "DEPTH" or img_type == "DEPTHLINEAR":
            # Convert linear depth to inverse depth for better visualization
            image_data = image_data * 100
            # Save ground truth data locally as png
            image_data[image_data == 0.0] = 1e-5
            image_data = np.clip(image_data, 0, 255)
            image_data -= np.min(image_data)
            if np.max(image_data) > 0:
                image_data /= np.max(image_data)
            depth_img = Image.fromarray((image_data * 255.0).astype(np.uint8))
            if img_type == "DEPTH":
                depth_img.save(f"{self.depth_folder}/{filename}.png")
            if img_type == "DEPTHLINEAR":
                depth_img.save(f"{self.depthLinear_folder}/{filename}.png")
    def save_bbox(self, viewport_name, data_type, data, filename, display_rgb=True, rgb_data=None, save_npy=True):
        self.bbox_2d_tight_folder = self.data_dir + "/" + str(viewport_name) + "/bbox_2d_tight/"
        self.bbox_2d_loose_folder = self.data_dir + "/" + str(viewport_name) + "/bbox_2d_loose/"
        self.bbox_3d_folder = self.data_dir + "/" + str(viewport_name) + "/bbox_3d/"
        # Save ground truth data locally as npy
        if data_type == "BBOX2DTIGHT" and save_npy:
            np.save(self.bbox_2d_tight_folder + filename + ".npy", data)
        if data_type == "BBOX2DLOOSE" and save_npy:
            np.save(self.bbox_2d_loose_folder + filename + ".npy", data)
        if data_type == "BBOX3D" and save_npy:
          np.save(self.bbox_3d_folder + filename + ".npy", data)
        if display_rgb and rgb_data is not None:
            if "2D" in data_type:
                color_image = self.visualization.colorize_bboxes(data, rgb_data)
                color_image_rgb = Image.fromarray(color_image, "RGBA")
                if data_type == "BBOX2DTIGHT":
                    color_image_rgb.save(f"{self.bbox_2d_tight_folder}/{filename}.png")
                if data_type == "BBOX2DLOOSE":
                    color_image_rgb.save(f"{self.bbox_2d_loose_folder}/{filename}.png")
            if "3D" in data_type:
                rgb_img = Image.fromarray(rgb_data, "RGBA")
                rgb_img.save(f"{self.bbox_3d_folder}/{filename}.png")

    def create_output_folders(self, sensor_settings=None):
        """Checks if the sensor output folder corresponding to each viewport is created. If not, it creates them."""
        if not os.path.exists(self.data_dir):
            os.mkdir(self.data_dir)
        if sensor_settings is None:
            sensor_settings = dict()
            viewport_names = get_viewport_names()
            sensor_settings_viewport = {
                "rgb": {"enabled": True},
                "depth": {"enabled": True, "colorize": True, "npy": True},
                "depthLinear": {"enabled": True, "colorize": True, "npy": True},
                "instance": {"enabled": True, "colorize": True, "npy": True},
                "semantic": {"enabled": True, "colorize": True, "npy": True},
                "bbox_2d_tight": {"enabled": True, "colorize": True, "npy": True},
                "bbox_2d_loose": {"enabled": True, "colorize": True, "npy": True},
                "camera": {"enabled": True, "npy": True},
                "poses": {"enabled": True, "npy": True},
                "motion-vector": {"enabled": True, "npy": True, "colorize": True},
                "bbox_3d": {"enabled": True, "npy": True, "colorize": True},
                "normals": {"enabled": True, "npy": True, "colorize": True},
            }
            for name in viewport_names:
                sensor_settings[name] = copy.deepcopy(sensor_settings_viewport)

        for viewport_name in sensor_settings:
            viewport_folder = self.data_dir + "/" + str(viewport_name)
            if not os.path.exists(viewport_folder):
                os.mkdir(viewport_folder)
            for sensor_name in sensor_settings[viewport_name]:
                if sensor_settings[viewport_name][sensor_name]["enabled"]:
                    sensor_folder = self.data_dir + "/" + str(viewport_name) + "/" + str(sensor_name)
                    if not os.path.exists(sensor_folder):
                        os.mkdir(sensor_folder)
