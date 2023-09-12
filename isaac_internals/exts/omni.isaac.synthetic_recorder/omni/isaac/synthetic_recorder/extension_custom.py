# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import carb
import omni
import omni.syntheticdata._syntheticdata as gt
import omni.ui as ui
from omni.kit.menu.utils import add_menu_items, remove_menu_items, MenuItemDescription

import asyncio
import atexit
import colorsys
import copy
import queue
import random
import os
import threading
import numpy as np
import weakref

from carb.settings import get_settings
from PIL import Image, ImageDraw
from omni.isaac.synthetic_utils import visualization as vis
from omni.isaac.synthetic_utils import SyntheticDataHelper, NumpyWriter
from omni.syntheticdata import sensors, visualize

EXTENSION_NAME = "Synthetic Data Recorder 2"


class MyRecorder():
    def on_startup(self):
        """Called to load the extension"""
        self._timeline = omni.timeline.get_timeline_interface()
        self._display_paths = []
        self._interface = gt.acquire_syntheticdata_interface()
        self._enable_record = False
        self._enable_timeline_record = False
        self._counter = 0

        # self.sub_update = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self._update)
        self._update_fps = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
            self._fps_info_update, name="omni.kit.debug.frame_info update"
        )
        self._current_fps = ""

        self._settings = get_settings()
        self._viewport = omni.kit.viewport.window
        self._viewport_names = []
        self._num_viewports = 0
        self.data_writer = None
        self.sd_helper = SyntheticDataHelper()
        self.sensor_settings_default = {
            "rgb": {"enabled": False},
            "depth": {"enabled": False, "colorize": False, "npy": False},
            "depthLinear": {"enabled": False, "colorize": False, "npy": False},
            "instance": {"enabled": False, "colorize": False, "npy": False},
            "semantic": {"enabled": False, "colorize": False, "npy": False},
            "bbox_2d_tight": {"enabled": False, "colorize": False, "npy": False},
            "bbox_2d_loose": {"enabled": False, "colorize": False, "npy": False},
            "normals": {"enabled": False, "colorize": False, "npy": False},
            "motion-vector": {"enabled": False, "colorize": False, "npy": False},
            "bbox_3d": {"enabled": False, "colorize": False, "npy": False},
            "camera": {"enabled": False, "colorize": False, "npy": False},
            "poses": {"enabled": False, "colorize": False, "npy": False},
        }
        self._sensor_settings = {}
        self._sensor_settings_single = {}
        self.set_settings(self.sensor_settings_default)
        self._viewport_names = []
        self._dir_name = ''
        self._num_threads = 10
        self._max_queue_size = 500
        self.verify = {}
        self.skip_cameras = 0

    def get_default_settings(self):
        return self.sensor_settings_default

    def set_single_settings(self, settings):
        self._sensor_settings_single = copy.deepcopy(settings)

    def set_settings(self, settings):
        for index, viewport_name in enumerate(self._viewport_names):
            if index >= self.skip_cameras:
                viewport_name = viewport_name.split(" ")[0] + str(int(index - self.skip_cameras))
                self._sensor_settings[viewport_name] = copy.deepcopy(settings)
            else:
                continue

    # def _update(self, e: carb.events.IEvent):
    def _update(self):
        tmp = [x for x in self._viewport.get_viewport_window_instances()]

        if len(tmp) != self._num_viewports:
            self._num_viewports = len(tmp)
            self._viewport_names = [x.name for x in tmp]
            self._is_first_run = [True] * self._num_viewports
            self.set_settings(self._sensor_settings_single)
            self.verify[self._viewport_names[-1]] = True

        if not self._timeline.is_playing():
            print("Cannot Generate Data! Editor is not playing.")
            self._enable_record = False
            return

        if self._dir_name == "":
            print("Cannot generate data, empty dir")
            return

        data_dir = str(self._dir_name)
        if self.data_writer is None:
            self.data_writer = NumpyWriter(
                data_dir,
                self._num_threads,
                self._max_queue_size,
                self._sensor_settings,
            )
            self.data_writer.start_threads()
        self._render_mode = str(self._settings.get("/rtx/rendermode"))
        for index, viewport_name in enumerate(self._viewport_names):
            if index < self.skip_cameras:
                continue
            real_viewport_name = viewport_name
            viewport_name = viewport_name.split(" ")[0] + str(int(index - self.skip_cameras))

            groundtruth = {
                "METADATA": {
                    "image_id": str(self._counter),
                    "viewport_name": viewport_name,
                    "DEPTH": {},
                    "DEPTHLINEAR": {},
                    "INSTANCE": {},
                    "SEMANTIC": {},
                    "BBOX2DTIGHT": {},
                    "BBOX2DLOOSE": {},
                    "NORMALS": {},
                    "MOTIONVECTOR": {},
                    "BBOX3D": {},
                    "CAMERA": {},
                    "POSES": {},
                },
                "DATA": {},
            }
            gt_list = []
            if self._sensor_settings[viewport_name]["rgb"]["enabled"]:
                gt_list.append("rgb")
            if self._sensor_settings[viewport_name]["depthLinear"]["enabled"]:
                gt_list.append("depthLinear")
            if self._sensor_settings[viewport_name]["depth"]["enabled"]:
                gt_list.append("depth")
            if self._sensor_settings[viewport_name]["bbox_2d_tight"]["enabled"]:
                gt_list.append("boundingBox2DTight")
            if self._sensor_settings[viewport_name]["bbox_2d_loose"]["enabled"]:
                gt_list.append("boundingBox2DLoose")
            if self._sensor_settings[viewport_name]["instance"]["enabled"]:
                gt_list.append("instanceSegmentation")
            if self._sensor_settings[viewport_name]["semantic"]["enabled"]:
                gt_list.append("semanticSegmentation")
            if self._sensor_settings[viewport_name]["motion-vector"]["enabled"]:
                gt_list.append("motion-vector")
            if self._sensor_settings[viewport_name]["normals"]["enabled"]:
                gt_list.append("normals")
            if self._sensor_settings[viewport_name]["bbox_3d"]["enabled"]:
                gt_list.append("boundingBox3D")
            if self._sensor_settings[viewport_name]["poses"]["enabled"]:
                gt_list.append("pose")
            if self._sensor_settings[viewport_name]["camera"]["enabled"]:
                gt_list.append("camera")

            for j in tmp:
                if j.name == real_viewport_name:
                    viewport = j
                    break
            try:
                gt = self.sd_helper.get_groundtruth(gt_list, viewport.viewport_api,
                                                    verify_sensor_init=self.verify[real_viewport_name])
                self.verify[real_viewport_name] = False
            except:
                self.verify[real_viewport_name] = True
                gt = self.sd_helper.get_groundtruth(gt_list, viewport.viewport_api,
                                                    verify_sensor_init=self.verify[real_viewport_name])
                self.verify[real_viewport_name] = False

            if self._enable_record == False:
                continue
            mappings = []
            # RGB
            if self._sensor_settings[viewport_name]["rgb"]["enabled"] and gt["state"]["rgb"]:
                groundtruth["DATA"]["RGB"] = gt["rgb"]

            # Depth
            if self._sensor_settings[viewport_name]["depth"]["enabled"] and gt["state"]["depth"]:
                groundtruth["DATA"]["DEPTH"] = gt["depth"].squeeze()
                groundtruth["METADATA"]["DEPTH"]["COLORIZE"] = self._sensor_settings[viewport_name]["depth"]["colorize"]
                groundtruth["METADATA"]["DEPTH"]["NPY"] = self._sensor_settings[viewport_name]["depth"]["npy"]

            # DepthLinear
            if self._sensor_settings[viewport_name]["depthLinear"]["enabled"] and gt["state"]["depthLinear"]:
                groundtruth["DATA"]["DEPTHLINEAR"] = gt["depthLinear"].squeeze()
                groundtruth["METADATA"]["DEPTHLINEAR"]["COLORIZE"] = self._sensor_settings[viewport_name]["depthLinear"]["colorize"]
                groundtruth["METADATA"]["DEPTHLINEAR"]["NPY"] = self._sensor_settings[viewport_name]["depthLinear"]["npy"]

            # Instance Segmentation
            if self._sensor_settings[viewport_name]["instance"]["enabled"] and gt["state"]["instanceSegmentation"]:
                import ipdb;
                ipdb.set_trace()

                instance_data = gt["instanceSegmentation"]
                groundtruth["DATA"]["INSTANCE"] = instance_data
                try:
                    groundtruth["METADATA"]["INSTANCE"]["WIDTH"] = instance_data[0].shape[1]
                    groundtruth["METADATA"]["INSTANCE"]["HEIGHT"] = instance_data[0].shape[0]
                    mappings = instance_data[1]
                except:
                    groundtruth["METADATA"]["INSTANCE"]["WIDTH"] = instance_data.shape[1]
                    groundtruth["METADATA"]["INSTANCE"]["HEIGHT"] = instance_data.shape[0]
                    mappings = []
                groundtruth["METADATA"]["INSTANCE"]["MAPPINGS"] = self._sensor_settings[viewport_name]["instance"][
                    "mappings"
                ]
                groundtruth["METADATA"]["INSTANCE"]["COLORIZE"] = self._sensor_settings[viewport_name]["instance"][
                    "colorize"
                ]
                groundtruth["METADATA"]["INSTANCE"]["NPY"] = self._sensor_settings[viewport_name]["instance"]["npy"]

            # Semantic Segmentation
            if self._sensor_settings[viewport_name]["semantic"]["enabled"] and gt["state"]["semanticSegmentation"]:
                semantic_data = gt["semanticSegmentation"]
                semantic_data[semantic_data == 65535] = 0  # deals with invalid semantic id
                groundtruth["DATA"]["SEMANTIC"] = (semantic_data, mappings)
                groundtruth["METADATA"]["SEMANTIC"]["WIDTH"] = semantic_data.shape[1]
                groundtruth["METADATA"]["SEMANTIC"]["HEIGHT"] = semantic_data.shape[0]
                groundtruth["METADATA"]["SEMANTIC"]["MAPPINGS"] = self._sensor_settings[viewport_name]["instance"][
                    "mappings"
                ]
                groundtruth["METADATA"]["SEMANTIC"]["COLORIZE"] = self._sensor_settings[viewport_name]["semantic"][
                    "colorize"
                ]
                groundtruth["METADATA"]["SEMANTIC"]["NPY"] = self._sensor_settings[viewport_name]["semantic"]["npy"]

            # 2D Tight BBox
            if self._sensor_settings[viewport_name]["bbox_2d_tight"]["enabled"] and gt["state"]["boundingBox2DTight"]:
                groundtruth["DATA"]["BBOX2DTIGHT"] = gt["boundingBox2DTight"]
                groundtruth["METADATA"]["BBOX2DTIGHT"]["COLORIZE"] = self._sensor_settings[viewport_name][
                    "bbox_2d_tight"
                ]["colorize"]
                groundtruth["METADATA"]["BBOX2DTIGHT"]["NPY"] = self._sensor_settings[viewport_name]["bbox_2d_tight"][
                    "npy"
                ]

            # 2D Loose BBox
            if self._sensor_settings[viewport_name]["bbox_2d_loose"]["enabled"] and gt["state"]["boundingBox2DLoose"]:
                groundtruth["DATA"]["BBOX2DLOOSE"] = gt["boundingBox2DLoose"]
                groundtruth["METADATA"]["BBOX2DLOOSE"]["COLORIZE"] = self._sensor_settings[viewport_name][
                    "bbox_2d_loose"
                ]["colorize"]
                groundtruth["METADATA"]["BBOX2DLOOSE"]["NPY"] = self._sensor_settings[viewport_name]["bbox_2d_loose"][
                    "npy"
                ]

            # 3D Bounding Box
            if self._sensor_settings[viewport_name]["bbox_3d"]["enabled"] and gt["state"]["boundingBox3D"]:
                groundtruth["DATA"]["BBOX3D"] = gt["boundingBox3D"].squeeze()
                groundtruth["METADATA"]["BBOX3D"]["COLORIZE"] = self._sensor_settings[viewport_name]["bbox_3d"][
                    "colorize"]
                groundtruth["METADATA"]["BBOX3D"]["NPY"] = self._sensor_settings[viewport_name]["bbox_3d"]["npy"]
                groundtruth["METADATA"]["BBOX3D_IMAGE"] =visualize.get_bbox3d(viewport.viewport_api)

            # Motion vector
            if self._sensor_settings[viewport_name]["motion-vector"]["enabled"] and gt["state"]["motion-vector"]:
                groundtruth["DATA"]["MOTIONVECTOR"] = gt["motion-vector"].squeeze()
                groundtruth["METADATA"]["MOTIONVECTOR"]["COLORIZE"] = \
                    self._sensor_settings[viewport_name]["motion-vector"][
                        "colorize"]
                groundtruth["METADATA"]["MOTIONVECTOR"]["NPY"] = self._sensor_settings[viewport_name]["motion-vector"][
                    "npy"]

            # Poses
            if self._sensor_settings[viewport_name]["poses"]["enabled"] and gt["pose"]:
                groundtruth["DATA"]["POSES"] = gt["pose"]
                groundtruth["METADATA"]["POSES"]["NPY"] = self._sensor_settings[viewport_name]["poses"]["npy"]

            # Camera
            if self._sensor_settings[viewport_name]["camera"]["enabled"] and gt["state"]["camera"]:
                groundtruth["DATA"]["CAMERA"] = gt["camera"]
                groundtruth["METADATA"]["CAMERA"]["NPY"] = self._sensor_settings[viewport_name]["camera"]["npy"]

            # Normals
            if self._sensor_settings[viewport_name]["normals"]["enabled"] and gt["state"]["normals"]:
                groundtruth["DATA"]["NORMALS"] = gt["normals"].squeeze()
                groundtruth["METADATA"]["NORMALS"]["NPY"] = self._sensor_settings[viewport_name]["normals"]["npy"]
            self.data_writer.q.put(copy.deepcopy(groundtruth))

    # self._counter = self._counter + 1

    def _fps_info_update(self, event):
        dt = event.payload["dt"] * 1000.0
        self._current_fps = "%1.2fms [%1.2f]" % (dt, 1000.0 / dt)
