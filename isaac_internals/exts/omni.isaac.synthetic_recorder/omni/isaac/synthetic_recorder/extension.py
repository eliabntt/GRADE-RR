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

EXTENSION_NAME = "Synthetic Data Recorder"


class Extension(omni.ext.IExt):
    def on_startup(self):
        """Caled to load the extension"""
        self._timeline = omni.timeline.get_timeline_interface()
        self._display_paths = []
        self._interface = gt.acquire_syntheticdata_interface()
        self._enable_record = False
        self._enable_timeline_record = False
        self._counter = 0
        self._window = ui.Window(EXTENSION_NAME, width=600, height=400, visible=True)
        self._window.deferred_dock_in("Console", omni.ui.DockPolicy.DO_NOTHING)
        self._window.visible = False
        self._menu_items = [
            MenuItemDescription(name=EXTENSION_NAME, onclick_fn=lambda a=weakref.proxy(self): a._menu_callback())
        ]
        add_menu_items(self._menu_items, "Synthetic Data")
        self.sub_update = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self._update)
        self._settings = get_settings()
        self._viewport = omni.kit.viewport.get_viewport_interface()
        self._viewport_names = []
        self._num_viewports = 0
        self._sensor_settings = {}
        self._sensor_settings_ui = {}
        self._build_window_ui()
        self._accumulated_time = 0
        self.data_writer = None
        self.sd_helper = SyntheticDataHelper()
        # self._menu_callback()

    def on_shutdown(self):
        """Called when the extesion us unloaded"""
        if self.data_writer is not None:
            self.data_writer.stop_threads()
        remove_menu_items(self._menu_items, "Synthetic Data")
        self._window = None

    def _menu_callback(self):
        self._window.visible = not self._window.visible

    def _build_window_ui(self):
        sensor_settings_default = {
            "rgb": {"enabled": False},
            "depth": {"enabled": False, "colorize": False, "npy": False},
            "instance": {"enabled": False, "colorize": False, "npy": False},
            "semantic": {"enabled": False, "colorize": False, "npy": False},
            "bbox_2d_tight": {"enabled": False, "colorize": False, "npy": False},
            "bbox_2d_loose": {"enabled": False, "colorize": False, "npy": False},
        }
        sensor_settings_ui_default = {
            "rgb": {"checkbox": None},
            "depth": {"checkbox": None, "colorize": None, "npy": None},
            "instance": {"checkbox": None, "colorize": None, "npy": None},
            "semantic": {"checkbox": None, "colorize": None, "npy": None},
            "bbox_2d_tight": {"checkbox": None, "colorize": None, "npy": None},
            "bbox_2d_loose": {"checkbox": None, "colorize": None, "npy": None},
        }
        viewports = self._viewport.get_instance_list()
        self._viewport_names = [self._viewport.get_viewport_window_name(vp) for vp in viewports]
        self._num_viewports = len(self._viewport_names)
        for viewport_name in self._viewport_names:
            self._sensor_settings[viewport_name] = copy.deepcopy(sensor_settings_default)
            self._sensor_settings_ui[viewport_name] = copy.deepcopy(sensor_settings_ui_default)

        with self._window.frame:
            with ui.VStack(spacing=5):
                for viewport_name in self._viewport_names:
                    viewport = self._viewport.get_viewport_window(self._viewport.get_instance(viewport_name))
                    with ui.CollapsableFrame(viewport_name + ": Sensor Settings", height=0):
                        with ui.VStack(spacing=5):

                            def toggle_rgb_sensor(self, viewport_name, value):
                                self._sensor_settings[viewport_name]["rgb"]["enabled"] = value
                                if value == False:
                                    self._sensor_settings_ui[viewport_name]["bbox_2d_tight"]["colorize"].enabled = value
                                    self._sensor_settings_ui[viewport_name]["bbox_2d_loose"]["colorize"].enabled = value
                                else:
                                    asyncio.ensure_future(
                                        sensors.initialize_async(viewport, [self.sd_helper.sd.SensorType.Rgb])
                                    )
                                    if self._sensor_settings[viewport_name]["bbox_2d_tight"]["enabled"]:
                                        self._sensor_settings_ui[viewport_name]["bbox_2d_tight"][
                                            "colorize"
                                        ].enabled = value
                                    if self._sensor_settings[viewport_name]["bbox_2d_loose"]["enabled"]:
                                        self._sensor_settings_ui[viewport_name]["bbox_2d_loose"][
                                            "colorize"
                                        ].enabled = value

                            def toggle_depth_sensor(self, viewport_name, value):
                                self._sensor_settings[viewport_name]["depth"]["enabled"] = value
                                self._sensor_settings_ui[viewport_name]["depth"]["colorize"].enabled = value
                                self._sensor_settings_ui[viewport_name]["depth"]["npy"].enabled = value
                                if value:
                                    asyncio.ensure_future(
                                        sensors.initialize_async(viewport, [self.sd_helper.sd.SensorType.DepthLinear])
                                    )

                            def toggle_depth_colorize(self, viewport_name, value):
                                self._sensor_settings[viewport_name]["depth"]["colorize"] = value

                            def toggle_depth_npy(self, viewport_name, value):
                                self._sensor_settings[viewport_name]["depth"]["npy"] = value

                            def toggle_instance_segmentation_sensor(self, viewport_name, value):
                                self._sensor_settings[viewport_name]["instance"]["enabled"] = value
                                self._sensor_settings_ui[viewport_name]["instance"]["colorize"].enabled = value
                                self._sensor_settings_ui[viewport_name]["instance"]["npy"].enabled = value
                                if value:
                                    asyncio.ensure_future(
                                        sensors.initialize_async(
                                            viewport, [self.sd_helper.sd.SensorType.InstanceSegmentation]
                                        )
                                    )

                            def toggle_instance_colorize(self, viewport_name, value):
                                self._sensor_settings[viewport_name]["instance"]["colorize"] = value

                            def toggle_instance_npy(self, viewport_name, value):
                                self._sensor_settings[viewport_name]["instance"]["npy"] = value

                            def toggle_semantic_segmentation_sensor(self, viewport_name, value):
                                self._sensor_settings[viewport_name]["semantic"]["enabled"] = value
                                self._sensor_settings_ui[viewport_name]["semantic"]["colorize"].enabled = value
                                self._sensor_settings_ui[viewport_name]["semantic"]["npy"].enabled = value
                                if value:
                                    asyncio.ensure_future(
                                        sensors.initialize_async(
                                            viewport, [self.sd_helper.sd.SensorType.SemanticSegmentation]
                                        )
                                    )

                            def toggle_semantic_colorize(self, viewport_name, value):
                                self._sensor_settings[viewport_name]["semantic"]["colorize"] = value

                            def toggle_semantic_npy(self, viewport_name, value):
                                self._sensor_settings[viewport_name]["semantic"]["npy"] = value

                            def toggle_bbox_2d_tight_sensor(self, viewport_name, value):
                                self._sensor_settings[viewport_name]["bbox_2d_tight"]["enabled"] = value
                                self._sensor_settings_ui[viewport_name]["bbox_2d_tight"]["colorize"].enabled = value
                                self._sensor_settings_ui[viewport_name]["bbox_2d_tight"]["npy"].enabled = value
                                if value:
                                    self._sensor_settings_ui[viewport_name]["bbox_2d_tight"][
                                        "colorize"
                                    ].enabled = self._sensor_settings[viewport_name]["rgb"]["enabled"]
                                    asyncio.ensure_future(
                                        sensors.initialize_async(
                                            viewport, [self.sd_helper.sd.SensorType.BoundingBox2DTight]
                                        )
                                    )

                            def toggle_bbox_2d_tight_colorize(self, viewport_name, value):
                                self._sensor_settings[viewport_name]["bbox_2d_tight"]["colorize"] = value

                            def toggle_bbox_2d_tight_npy(self, viewport_name, value):
                                self._sensor_settings[viewport_name]["bbox_2d_tight"]["npy"] = value

                            def toggle_bbox_2d_loose_sensor(self, viewport_name, value):
                                self._sensor_settings[viewport_name]["bbox_2d_loose"]["enabled"] = value
                                self._sensor_settings_ui[viewport_name]["bbox_2d_loose"]["colorize"].enabled = value
                                self._sensor_settings_ui[viewport_name]["bbox_2d_loose"]["npy"].enabled = value
                                if value:
                                    self._sensor_settings_ui[viewport_name]["bbox_2d_loose"][
                                        "colorize"
                                    ].enabled = self._sensor_settings[viewport_name]["rgb"]["enabled"]
                                    asyncio.ensure_future(
                                        sensors.initialize_async(
                                            viewport, [self.sd_helper.sd.SensorType.BoundingBox2DLoose]
                                        )
                                    )

                            def toggle_bbox_2d_loose_colorize(self, viewport_name, value):
                                self._sensor_settings[viewport_name]["bbox_2d_loose"]["colorize"] = value

                            def toggle_bbox_2d_loose_npy(self, viewport_name, value):
                                self._sensor_settings[viewport_name]["bbox_2d_loose"]["npy"] = value

                            def toggle_record_anim(self, value):
                                self._enable_timeline_record = value

                            with ui.HStack(height=30):
                                ui.Spacer(width=10)
                                ui.Label("Sensor Name", height=0, width=150)
                                ui.Label("Status", height=0, width=75)
                                ui.Label("Colorize", height=0, width=75)
                                ui.Label("Save array", height=0, width=75)

                            with ui.HStack(height=30):
                                ui.Spacer(width=10)
                                ui.Label("RGB", height=0, width=150)
                                self._sensor_settings_ui[viewport_name]["rgb"]["checkbox"] = ui.CheckBox()
                                self._sensor_settings_ui[viewport_name]["rgb"]["checkbox"].model.add_value_changed_fn(
                                    lambda a, v=viewport_name, this=self: toggle_rgb_sensor(
                                        self, v, a.get_value_as_bool()
                                    )
                                )
                            with ui.HStack(height=30):
                                ui.Spacer(width=10)
                                ui.Label("Depth", height=0, width=150)
                                self._sensor_settings_ui[viewport_name]["depth"]["checkbox"] = ui.CheckBox(width=75)
                                self._sensor_settings_ui[viewport_name]["depth"]["checkbox"].model.add_value_changed_fn(
                                    lambda a, v=viewport_name, this=self: toggle_depth_sensor(
                                        self, v, a.get_value_as_bool()
                                    )
                                )
                                self._sensor_settings_ui[viewport_name]["depth"]["colorize"] = ui.CheckBox(
                                    enabled=False, width=75
                                )
                                self._sensor_settings_ui[viewport_name]["depth"]["colorize"].model.add_value_changed_fn(
                                    lambda a, v=viewport_name, this=self: toggle_depth_colorize(
                                        self, v, a.get_value_as_bool()
                                    )
                                )
                                self._sensor_settings_ui[viewport_name]["depth"]["npy"] = ui.CheckBox(
                                    enabled=False, width=75
                                )
                                self._sensor_settings_ui[viewport_name]["depth"]["npy"].model.add_value_changed_fn(
                                    lambda a, v=viewport_name, this=self: toggle_depth_npy(
                                        self, v, a.get_value_as_bool()
                                    )
                                )
                            with ui.HStack(height=30):
                                ui.Spacer(width=10)
                                ui.Label("Semantic Segmentation", height=0, width=150)
                                self._sensor_settings_ui[viewport_name]["semantic"]["checkbox"] = ui.CheckBox(width=75)
                                self._sensor_settings_ui[viewport_name]["semantic"][
                                    "checkbox"
                                ].model.add_value_changed_fn(
                                    lambda a, v=viewport_name, this=self: toggle_semantic_segmentation_sensor(
                                        self, v, a.get_value_as_bool()
                                    )
                                )
                                self._sensor_settings_ui[viewport_name]["semantic"]["colorize"] = ui.CheckBox(
                                    enabled=False, width=75
                                )
                                self._sensor_settings_ui[viewport_name]["semantic"][
                                    "colorize"
                                ].model.add_value_changed_fn(
                                    lambda a, v=viewport_name, this=self: toggle_semantic_colorize(
                                        self, v, a.get_value_as_bool()
                                    )
                                )
                                self._sensor_settings_ui[viewport_name]["semantic"]["npy"] = ui.CheckBox(
                                    enabled=False, width=75
                                )
                                self._sensor_settings_ui[viewport_name]["semantic"]["npy"].model.add_value_changed_fn(
                                    lambda a, v=viewport_name, this=self: toggle_semantic_npy(
                                        self, v, a.get_value_as_bool()
                                    )
                                )
                            with ui.HStack(height=30):
                                ui.Spacer(width=10)
                                ui.Label("Instance Segmentation", height=0, width=150)
                                self._sensor_settings_ui[viewport_name]["instance"]["checkbox"] = ui.CheckBox(width=75)
                                self._sensor_settings_ui[viewport_name]["instance"][
                                    "checkbox"
                                ].model.add_value_changed_fn(
                                    lambda a, v=viewport_name, this=self: toggle_instance_segmentation_sensor(
                                        self, v, a.get_value_as_bool()
                                    )
                                )
                                self._sensor_settings_ui[viewport_name]["instance"]["colorize"] = ui.CheckBox(
                                    enabled=False, width=75
                                )
                                self._sensor_settings_ui[viewport_name]["instance"][
                                    "colorize"
                                ].model.add_value_changed_fn(
                                    lambda a, v=viewport_name, this=self: toggle_instance_colorize(
                                        self, v, a.get_value_as_bool()
                                    )
                                )
                                self._sensor_settings_ui[viewport_name]["instance"]["npy"] = ui.CheckBox(
                                    enabled=False, width=75
                                )
                                self._sensor_settings_ui[viewport_name]["instance"]["npy"].model.add_value_changed_fn(
                                    lambda a, v=viewport_name, this=self: toggle_instance_npy(
                                        self, v, a.get_value_as_bool()
                                    )
                                )
                            with ui.HStack(height=30):
                                ui.Spacer(width=10)
                                bbox_2d_tight_label = ui.Label("2D Tight Bounding Box", height=0, width=150)
                                bbox_2d_tight_label.set_tooltip("To colorize sensor output, enable RGB")
                                self._sensor_settings_ui[viewport_name]["bbox_2d_tight"]["checkbox"] = ui.CheckBox(
                                    width=75
                                )
                                self._sensor_settings_ui[viewport_name]["bbox_2d_tight"][
                                    "checkbox"
                                ].model.add_value_changed_fn(
                                    lambda a, v=viewport_name, this=self: toggle_bbox_2d_tight_sensor(
                                        self, v, a.get_value_as_bool()
                                    )
                                )
                                self._sensor_settings_ui[viewport_name]["bbox_2d_tight"]["colorize"] = ui.CheckBox(
                                    enabled=False, width=75
                                )
                                self._sensor_settings_ui[viewport_name]["bbox_2d_tight"][
                                    "colorize"
                                ].model.add_value_changed_fn(
                                    lambda a, v=viewport_name, this=self: toggle_bbox_2d_tight_colorize(
                                        self, v, a.get_value_as_bool()
                                    )
                                )
                                self._sensor_settings_ui[viewport_name]["bbox_2d_tight"]["npy"] = ui.CheckBox(
                                    enabled=False, width=75
                                )
                                self._sensor_settings_ui[viewport_name]["bbox_2d_tight"][
                                    "npy"
                                ].model.add_value_changed_fn(
                                    lambda a, v=viewport_name, this=self: toggle_bbox_2d_tight_npy(
                                        self, v, a.get_value_as_bool()
                                    )
                                )
                            with ui.HStack(height=30):
                                ui.Spacer(width=10)
                                bbox_2d_loose_label = ui.Label("2D Loose Bounding Box", height=0, width=150)
                                bbox_2d_loose_label.set_tooltip("To colorize sensor output, enable RGB")
                                self._sensor_settings_ui[viewport_name]["bbox_2d_loose"]["checkbox"] = ui.CheckBox(
                                    width=75
                                )
                                self._sensor_settings_ui[viewport_name]["bbox_2d_loose"][
                                    "checkbox"
                                ].model.add_value_changed_fn(
                                    lambda a, v=viewport_name, this=self: toggle_bbox_2d_loose_sensor(
                                        self, v, a.get_value_as_bool()
                                    )
                                )
                                self._sensor_settings_ui[viewport_name]["bbox_2d_loose"]["colorize"] = ui.CheckBox(
                                    enabled=False, width=75
                                )
                                self._sensor_settings_ui[viewport_name]["bbox_2d_loose"][
                                    "colorize"
                                ].model.add_value_changed_fn(
                                    lambda a, v=viewport_name, this=self: toggle_bbox_2d_loose_colorize(
                                        self, v, a.get_value_as_bool()
                                    )
                                )
                                self._sensor_settings_ui[viewport_name]["bbox_2d_loose"]["npy"] = ui.CheckBox(
                                    enabled=False, width=75
                                )
                                self._sensor_settings_ui[viewport_name]["bbox_2d_loose"][
                                    "npy"
                                ].model.add_value_changed_fn(
                                    lambda a, v=viewport_name, this=self: toggle_bbox_2d_loose_npy(
                                        self, v, a.get_value_as_bool()
                                    )
                                )
                with ui.CollapsableFrame("Recorder Settings", height=0):
                    with ui.VStack(spacing=5):
                        with ui.HStack():
                            ui.Spacer(width=10)
                            self._ui_dir_label = ui.Label("Output Directory:", width=100)
                            default_dir = os.path.join(os.getcwd(), "output")
                            self._ui_dir_name = ui.StringField(width=300)
                            self._ui_dir_name.model.set_value(default_dir)
                        with ui.HStack():
                            ui.Spacer(width=10)
                            self._ui_render_mode_label = ui.Label("Render Mode: ", width=100)
                            self._ui_render_mode = ui.ComboBox(0, "Use Current", "RayTracing", "PathTracing", width=300)
                        with ui.HStack():
                            ui.Spacer(width=10)
                            self._ui_spp_label = ui.Label("Samples per pixel: ", width=100)
                            self._spp_value = ui.FloatField(width=300)
                            self._spp_value.model.set_value(1.0)
                        with ui.HStack():
                            ui.Spacer(width=10)
                            self._ui_dir_label = ui.Label("Capture period in seconds:", width=150)
                            self._capture_period = ui.FloatField(width=250)
                            # 0 means capture every frame
                            self._capture_period.model.set_value(0.0)
                        with ui.HStack():
                            ui.Spacer(width=10)
                            self._ui_thread_label = ui.Label("Number of worker threads:", width=150)
                            self._num_threads = ui.IntField(width=250)
                            self._num_threads.model.set_value(4)
                        with ui.HStack():
                            ui.Spacer(width=10)
                            self._ui_queue_label = ui.Label("Size of queue:", width=150)
                            self._max_queue_size = ui.IntField(width=250)
                            self._max_queue_size.model.set_value(500)
                        with ui.HStack():
                            ui.Spacer(width=10)
                            self._ui_anim_label = ui.Label("Record Animation", width=150)
                            self.record_anim_checkbox = ui.CheckBox(width=75)
                            self.record_anim_checkbox.model.add_value_changed_fn(
                                lambda a, this=self: toggle_record_anim(self, a.get_value_as_bool())
                            )
                        with ui.HStack():
                            ui.Spacer(width=5)
                            self._capture_btn = ui.Button("Start Recording", width=100)
                            self._capture_btn.set_clicked_fn(self.generate_data_fn)
                            self._reset_btn = ui.Button("Reset", width=50)
                            self._reset_btn.set_clicked_fn(self.reset_counter_fn)

    def rename_button(self):
        if self._enable_record:
            print("Generating Data!")
            self.data_writer = None
            self._capture_btn.text = "Stop Recording"
            if self._enable_timeline_record:
                # rewind
                self._timeline.set_current_time(0)
                # disable automatic time update in timeline
                self._timeline.set_auto_update(False)
                # set usd time code second to target frame rate
                self._saved_timecodes_per_second = self._timeline.get_time_codes_per_seconds()

        else:
            self._capture_btn.text = "Start Recording"

    def generate_data_fn(self):
        current_render_index = self._ui_render_mode.model.get_item_value_model().as_int
        self._enable_record = not self._enable_record
        if current_render_index == 1:
            self._settings.set_string("/rtx/rendermode", "RayTracing")
            carb.log_warn("Switching to RayTracing Mode")
        elif current_render_index == 2:
            self._settings.set_string("/rtx/rendermode", "PathTracing")
            self._settings.set_float("/rtx/pathtracing/spp", self._spp_value.model.get_value_as_float())
            self._settings.set_float("/rtx/pathtracing/totalSpp", self._spp_value.model.get_value_as_float())
            carb.log_warn("Switching to PathTracing Mode")
        else:
            carb.log_warn("Keeping current Render Mode")
        self.rename_button()

    def reset_counter_fn(self):
        self._counter = 0

    def _update(self, e: carb.events.IEvent):
        if len(self._viewport.get_instance_list()) != self._num_viewports:
            self._num_viewports = len(self._viewport.get_instance_list())
            self._window.frame.clear()
            self._build_window_ui()

        if self._enable_record == False:
            return

        if not self._timeline.is_playing():
            print("Cannot Generate Data! Editor is not playing.")
            self._enable_record = False
            self.rename_button()
            return

        if self._enable_timeline_record:
            self._timeline.set_prerolling(False)
            self._timeline.set_current_time(self._counter / self._saved_timecodes_per_second)

        dt = e.payload["dt"]
        if self._accumulated_time + dt < self._capture_period.model.get_value_as_float():
            self._accumulated_time += dt
            return

        # reset _accumulated_time
        self._accumulated_time = 0

        data_dir = str(self._ui_dir_name.model.get_value_as_string())

        if self.data_writer is None:
            self.data_writer = NumpyWriter(
                data_dir,
                self._num_threads.model.get_value_as_int(),
                self._max_queue_size.model.get_value_as_int(),
                self._sensor_settings,
            )
            self.data_writer.start_threads()

        self._render_mode = str(self._settings.get("/rtx/rendermode"))

        for viewport_name in self._viewport_names:
            groundtruth = {
                "METADATA": {
                    "image_id": str(self._counter),
                    "viewport_name": viewport_name,
                    "DEPTH": {},
                    "INSTANCE": {},
                    "SEMANTIC": {},
                    "BBOX2DTIGHT": {},
                    "BBOX2DLOOSE": {},
                },
                "DATA": {},
            }

            gt_list = []
            if self._sensor_settings[viewport_name]["rgb"]["enabled"]:
                gt_list.append("rgb")
            if self._sensor_settings[viewport_name]["depth"]["enabled"]:
                gt_list.append("depthLinear")
            if self._sensor_settings[viewport_name]["bbox_2d_tight"]["enabled"]:
                gt_list.append("boundingBox2DTight")
            if self._sensor_settings[viewport_name]["bbox_2d_loose"]["enabled"]:
                gt_list.append("boundingBox2DLoose")
            if self._sensor_settings[viewport_name]["instance"]["enabled"]:
                gt_list.append("instanceSegmentation")
            if self._sensor_settings[viewport_name]["semantic"]["enabled"]:
                gt_list.append("semanticSegmentation")
            # print(viewport_name, " : ", gt_list)

            # viewport = omni.kit.viewport.get_default_viewport_window()
            viewport = self._viewport.get_viewport_window(self._viewport.get_instance(viewport_name))
            gt = self.sd_helper.get_groundtruth(gt_list, viewport, verify_sensor_init=False)
            # RGB
            if self._sensor_settings[viewport_name]["rgb"]["enabled"] and gt["state"]["rgb"]:
                groundtruth["DATA"]["RGB"] = gt["rgb"]

            # Depth
            if self._sensor_settings[viewport_name]["depth"]["enabled"] and gt["state"]["depthLinear"]:
                groundtruth["DATA"]["DEPTH"] = gt["depthLinear"].squeeze()
                groundtruth["METADATA"]["DEPTH"]["COLORIZE"] = self._sensor_settings[viewport_name]["depth"]["colorize"]
                groundtruth["METADATA"]["DEPTH"]["NPY"] = self._sensor_settings[viewport_name]["depth"]["npy"]

            # Instance Segmentation
            if self._sensor_settings[viewport_name]["instance"]["enabled"] and gt["state"]["instanceSegmentation"]:
                instance_data = gt["instanceSegmentation"][0]
                groundtruth["DATA"]["INSTANCE"] = instance_data
                groundtruth["METADATA"]["INSTANCE"]["WIDTH"] = instance_data.shape[1]
                groundtruth["METADATA"]["INSTANCE"]["HEIGHT"] = instance_data.shape[0]
                groundtruth["METADATA"]["INSTANCE"]["COLORIZE"] = self._sensor_settings[viewport_name]["instance"][
                    "colorize"
                ]
                groundtruth["METADATA"]["INSTANCE"]["NPY"] = self._sensor_settings[viewport_name]["instance"]["npy"]

            # Semantic Segmentation
            if self._sensor_settings[viewport_name]["semantic"]["enabled"] and gt["state"]["semanticSegmentation"]:
                semantic_data = gt["semanticSegmentation"]
                semantic_data[semantic_data == 65535] = 0  # deals with invalid semantic id
                groundtruth["DATA"]["SEMANTIC"] = semantic_data
                groundtruth["METADATA"]["SEMANTIC"]["WIDTH"] = semantic_data.shape[1]
                groundtruth["METADATA"]["SEMANTIC"]["HEIGHT"] = semantic_data.shape[0]
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

            self.data_writer.q.put(copy.deepcopy(groundtruth))

        self._counter = self._counter + 1
