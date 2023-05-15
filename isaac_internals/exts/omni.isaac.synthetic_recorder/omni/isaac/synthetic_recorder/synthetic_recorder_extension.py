# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import os
import gc
import time
import asyncio
import json
import carb
import carb.events
import omni.kit.ui
import omni.ui as ui
import omni.replicator.core as rep
import omni.timeline
from omni.kit.viewport.utility import get_active_viewport
from omni.kit.window.extensions.utils import open_file_using_os_default
from omni.replicator.core import orchestrator
from pxr import Semantics

from functools import lru_cache
from enum import Enum

PARAM_TOOLTIPS = {
    "rgb": "Produces an array of type np.uint8 with shape (width, height, 4), where the four channels correspond to R,G,B,A.",
    "bounding_box_2d_tight": "Outputs tight 2d bounding box of each entity with semantics in the camera's viewport.\nTight bounding boxes bound only the visible pixels of entities.\nCompletely occluded entities are ommited.\nBounds only visible pixels.",
    "bounding_box_2d_loose": "Outputs loose 2d bounding box of each entity with semantics in the camera's field of view.\nLoose bounding boxes bound the entire entity regardless of occlusions.\nWill produce the loose 2d bounding box of any prim in the viewport, no matter if is partially occluded or fully occluded.",
    "semantic_segmentation": "Outputs semantic segmentation of each entity in the camera's viewport that has semantic labels.\nIf colorize is set to True (mapping from color to semantic labels), the image will be a 2d array of types np.uint8 with 4 channels.\nIf colorize is set to False (mapping from semantic id to semantic labels), the image will be a 2d array of types np.uint32 with 1 channel, which is the semantic id of the entities.",
    "colorize_semantic_segmentation": "If True, semantic segmentation is converted to an image where semantic ids are mapped to colors and saved as a uint8 4 channel PNG image.\nIf False, the output is saved as a uint32 PNG image.",
    "instance_id_segmentation": "Outputs instance id segmentation of each entity in the camera's viewport.\nThe instance id is unique for each prim in the scene with different paths.\nIf colorize is set to True (mapping from color to usd prim path of that entity), the image will be a 2d array of types np.uint8 with 4 channels.\nIf colorize is set to False (mapping from instance id to usd prim path of that entity), the image will be a 2d array of types np.uint32 with 1 channel, which is the instance id of the entities.",
    "colorize_instance_id_segmentation": "If True, instance id segmentation is converted to an image where instance ids are mapped to colors and saved as a uint8 4 channel PNG image.\nIf False, the output is saved as a uint32 PNG image.",
    "instance_segmentation": "Outputs instance segmentation of each entity in the camera' viewport.\nThe main difference between instance id segmentation and instance segmentation are that instance segmentation annotator goes down the hierarchy to the lowest level prim which has semantic labels,\n whereas instance id segmentation always goes down to the leaf prim.\nIf colorize is set to True (mapping from color to usd prim path of that semantic entity), the image will be a 2d array of types np.uint8 with 4 channels.\nIf colorize is set to False (mapping from instance id to usd prim path of that semantic entity), the image will be a 2d array of types np.uint32 with 1 channel, which is the instance id of the semantic entities.",
    "colorize_instance_segmentation": "If True, instance segmentation is converted to an image where instance are mapped to colors and saved as a uint8 4 channel PNG image.\nIf False, the output is saved as a uint32 PNG image.",
    "distance_to_camera": "Outputs a depth map from objects to camera positions.\nProduces a 2d array of types np.float32 with 1 channel.",
    "distance_to_image_plane": "Outputs a depth map from objects to image plane of the camera.\nProduces a 2d array of types np.float32 with 1 channel.",
    "bounding_box_3d": "Outputs 3D bounding box of each entity with semantics in the camera's viewport, generated regardless of occlusion.",
    "occlusion": "Outputs the occlusion of each entity in the camera's viewport.\nContains the instanceId, semanticId and the occlusionRatio.",
    "normals": "Produces an array of type np.float32 with shape (height, width, 4).\nThe first three channels correspond to (x, y, z).\nThe fourth channel is unused.",
    "motion_vectors": "Outputs a 2D array of motion vectors representing the relative motion of a pixel in the camera's viewport between frames.\nProduces a 2darray of types np.float32 with 4 channels.\nEach value is a normalized direction in 3D space.\nThe values represent motion relative to camera space.",
    "camera_params": "Outputs the camera model (pinhole or fisheye models), view matrix, projection matrix, fisheye nominal width/height, fisheye optical centre, fisheye maximum field of view, fisheye polynomial, near/far clipping range.",
    "pointcloud": "Outputs a 2D array of shape (N, 3) representing the points sampled on the surface of the prims in the viewport, where N is the number of point.\nPoint positions are in the world space.\nSample resolution is determined by the resolution of the render product.\nTo get the mapping from semantic id to semantic labels, pointcloud annotator is better used with semantic segmentation annotator, and users can extract the idToLabels data from the semantic segmentation annotator.",
    "pointcloud_include_unlabelled": "If True, pointcloud annotator will capture any prim in the camera's perspective, not matter if it has semantics or not.\nIf False, only prims with semantics will be captured.",
    "skeleton_data": "Retrieves skeleton data given skeleton prims and camera parameters",
    "s3_bucket": "The S3 Bucket name to write to. If not provided, disk backend will be used instead.\nThis backend requires that AWS credentials are set up in ~/.aws/credentials.",
    "s3_region": "If provided, this is the region the S3 bucket will be set to. Default: us-east-1",
    "s3_endpoint": "Gateway endpoint for Amazon S3",
}

MAX_RESOLUTION = (7680, 4320)  # 8K

WINDOW_NAME = "Synthetic Data Recorder"
MENU_PATH = f"Replicator/{WINDOW_NAME}"


class OutWriteType(Enum):
    OVERWRITE = 0
    INCREMENT = 1
    TIMESTAMP = 2


@lru_cache()
def _ui_get_delete_glyph():
    return omni.ui.get_custom_glyph_code("${glyphs}/menu_delete.svg")


@lru_cache()
def _ui_get_open_folder_glyph():
    return omni.ui.get_custom_glyph_code("${glyphs}/folder_open.svg")


@lru_cache()
def _ui_get_reset_glyph():
    return omni.ui.get_custom_glyph_code("${glyphs}/menu_refresh.svg")


class SyntheticRecorderExtension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        """Caled to load the extension"""

        self._ext_id = ext_id
        self._window = ui.Window(WINDOW_NAME, dockPreference=ui.DockPreference.RIGHT_BOTTOM, visible=True)
        self._window.deferred_dock_in("Property", omni.ui.DockPolicy.DO_NOTHING)

        editor_menu = omni.kit.ui.get_editor_menu()
        if editor_menu:
            self._menu = editor_menu.add_item(MENU_PATH, self._menu_callback, toggle=True, value=True)
        self._window.set_visibility_changed_fn(self._visibility_changed_fn)

        self._writer_name = "BasicWriter"
        self._custom_writer_name = "MyCustomWriter"
        self._writer = None
        self._num_frames = 0
        self._rt_subframes = 0
        self._control_timeline = False

        self._orchestrator_status = rep.orchestrator.get_status()
        self._in_running_state = False

        # Orchestrator status update callback
        self._orchestrator_status_cb = rep.orchestrator.register_status_callback(self._on_orchestrator_status_changed)

        # Stage event callback
        self._sub_stage_event = (
            omni.usd.get_context()
            .get_stage_event_stream()
            .create_subscription_to_pop_by_type(int(omni.usd.StageEventType.CLOSING), self._on_stage_closing_event)
        )

        # Editor quit callback
        self._sub_shutdown = (
            omni.kit.app.get_app()
            .get_shutdown_event_stream()
            .create_subscription_to_pop_by_type(
                omni.kit.app.POST_QUIT_EVENT_TYPE,
                self._on_editor_quit_event,
                name="omni.isaac.synthetic_recorder::shutdown_callback",
                order=0,
            )
        )

        self._config_dir = os.path.abspath(
            os.path.join(omni.kit.app.get_app().get_extension_manager().get_extension_path(self._ext_id), "data", "")
        )
        self._last_config_path = os.path.join(self._config_dir, "last_config.json")
        self._custom_params_path = ""
        self._config_file = "custom_config.json"
        self._out_working_dir = os.getcwd() + "/"
        self._out_dir = "_out_sdrec"
        self._out_write_type = OutWriteType.OVERWRITE
        self._use_s3 = False
        self._s3_params = {"s3_bucket": "", "s3_region": "", "s3_endpoint": ""}

        self._basic_writer_params = {
            "rgb": True,
            "bounding_box_2d_tight": False,
            "bounding_box_2d_loose": False,
            "semantic_segmentation": False,
            "colorize_semantic_segmentation": False,
            "instance_id_segmentation": False,
            "colorize_instance_id_segmentation": False,
            "instance_segmentation": False,
            "colorize_instance_segmentation": False,
            "distance_to_camera": False,
            "distance_to_image_plane": False,
            "bounding_box_3d": False,
            "occlusion": False,
            "normals": False,
            "motion_vectors": False,
            "camera_params": False,
            "pointcloud": False,
            "pointcloud_include_unlabelled": False,
            "skeleton_data": False,
        }

        self._render_products = []
        self._rp_data = [["/OmniverseKit_Persp", 512, 512]]

        # UI - frames collapsed state
        self._writer_frame_collapsed = False
        self._writer_params_frame_collapsed = True
        self._rp_frame_collapsed = False
        self._output_frame_collapsed = False
        self._s3_params_frame_collapsed = True
        self._config_frame_collapsed = True
        self._control_frame_collapsed = False
        self._control_params_frame_collapsed = False

        # UI - Buttons
        self._start_stop_button = None
        self._pause_resume_button = None

        # Load latest or default config values
        if os.path.isfile(self._last_config_path):
            self.load_config(self._last_config_path)
        else:
            self.load_config(os.path.join(self._config_dir, "default_config.json"))

        # Build the window ui
        self._build_window_ui()

    def _menu_callback(self, menu, value):
        self._window.visible = not self._window.visible

    def _visibility_changed_fn(self, visible):
        omni.kit.ui.get_editor_menu().set_value(MENU_PATH, visible)

    def _on_orchestrator_status_changed(self, status):
        new_status = status is not self._orchestrator_status
        if new_status:
            self._orchestrator_status = status
            # Check if the recorder was running and it stopped because it reached the number of requested frames
            has_finished_recording = self._in_running_state and status is rep.orchestrator.Status.STOPPED
            if has_finished_recording:
                asyncio.ensure_future(self._on_orchestrator_finish_async())

    def _on_stage_closing_event(self, e: carb.events.IEvent):
        self._disable_all_buttons()
        if self._orchestrator_status is not orchestrator.Status.STOPPED:
            rep.orchestrator.stop()
        self._clear_recorder()
        self._enable_buttons(case="reset")

    def _on_editor_quit_event(self, e: carb.events.IEvent):
        # Fast shutdown of the extension, stop recorder save config files
        if self._orchestrator_status is not orchestrator.Status.STOPPED:
            rep.orchestrator.stop()
            self._clear_recorder()
        self.save_config(self._last_config_path)

    def on_shutdown(self):
        # Clean shutdown of the extension, called when the extension is unloaded (not called when the editor is closed)
        if self._orchestrator_status is not orchestrator.Status.STOPPED:
            rep.orchestrator.stop()
            self._clear_recorder()
        self._orchestrator_status_cb.unregister()
        self.save_config(self._last_config_path)
        editor_menu = omni.kit.ui.get_editor_menu()
        if editor_menu:
            self._menu = editor_menu.remove_item(MENU_PATH)
        self._menu = None
        self._window = None
        gc.collect()

    def _open_dir(self, path):
        if not os.path.isdir(path):
            carb.log_warn(f"Could not open directory {path}.")
            return
        open_file_using_os_default(path)

    def load_config(self, path):
        if not os.path.isfile(path):
            carb.log_warn(f"Could not find config file {path}.")
            return
        with open(path, "r") as f:
            config = json.load(f)
            if "writer_name" in config and config["writer_name"]:
                self._writer_name = config["writer_name"]
            if "custom_writer_name" in config and config["custom_writer_name"]:
                self._custom_writer_name = config["custom_writer_name"]
            if "num_frames" in config:
                self._num_frames = config["num_frames"]
            if "rt_subframes" in config:
                self._rt_subframes = config["rt_subframes"]
            if "control_timeline" in config:
                self._control_timeline = config["control_timeline"]
            if "config_file" in config and config["config_file"]:
                self._config_file = config["config_file"]
            if "custom_params_path" in config and config["custom_params_path"]:
                self._custom_params_path = config["custom_params_path"]
            if "out_working_dir" in config and config["out_working_dir"]:
                self._out_working_dir = config["out_working_dir"]
            if "out_dir" in config and config["out_dir"]:
                self._out_dir = config["out_dir"]
            if "out_write_type" in config:
                self._out_write_type = OutWriteType[config["out_write_type"]]
            if "use_s3" in config:
                self._use_s3 = config["use_s3"]
            if "s3_params" in config:
                self._s3_params = config["s3_params"]
            if "basic_writer_params" in config and isinstance(config["basic_writer_params"], dict):
                for key, value in config["basic_writer_params"].items():
                    if key in self._basic_writer_params:
                        self._basic_writer_params[key] = value
            if "rp_data" in config:
                self._rp_data = config["rp_data"]

    def _load_config_and_refresh_ui(self, directory, filename):
        self.load_config(os.path.join(directory, filename))
        self._build_window_ui()

    def save_config(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        if os.path.isfile(path):
            carb.log_info(f"Overwriting config file {path}.")
        with open(path, "w") as json_file:
            config = {
                "num_frames": self._num_frames,
                "rt_subframes": self._rt_subframes,
                "control_timeline": self._control_timeline,
                "out_write_type": self._out_write_type.name,
                "use_s3": self._use_s3,
                "s3_params": self._s3_params,
                "basic_writer_params": self._basic_writer_params,
                "rp_data": self._rp_data,
            }
            # Only save string values if they are not empty
            if self._writer_name:
                config["writer_name"] = self._writer_name
            if self._custom_writer_name:
                config["custom_writer_name"] = self._custom_writer_name
            if self._config_file:
                config["config_file"] = self._config_file
            if self._custom_params_path:
                config["custom_params_path"] = self._custom_params_path
            if self._out_working_dir:
                config["out_working_dir"] = self._out_working_dir
            if self._out_dir:
                config["out_dir"] = self._out_dir

            json.dump(config, json_file, indent=4)

    def _get_custom_params(self, path):
        custom_params = {}
        if not os.path.isfile(path):
            carb.log_warn(f"Could not find params file {path}.")
            return custom_params
        with open(path, "r") as f:
            params = json.load(f)
            for key in params:
                custom_params[key] = params[key]
            return custom_params

    def _reset_config_dir(self):
        self._config_dir = os.path.abspath(
            os.path.join(omni.kit.app.get_app().get_extension_manager().get_extension_path(self._ext_id), "data", "")
        )
        self._build_window_ui()

    def _reset_out_working_dir(self):
        self._out_working_dir = os.getcwd() + "/"
        self._build_window_ui()

    def _get_dir_next_numerical_suffix(self, path, dir_name):
        nums = [-1]
        for file in os.listdir(path):
            if file.startswith(dir_name) and os.path.isdir(os.path.join(path, file)):
                file = file[len(dir_name) :]
                file = file.replace("_", "")
                if file.isdecimal():
                    nums.append(int(file))
        suffix = "_" + str(max(nums) + 1)
        return suffix

    def _get_output_dir(self):
        out_dir = self._out_dir
        if self._out_write_type is OutWriteType.INCREMENT:
            out_dir = out_dir + self._get_dir_next_numerical_suffix(self._out_working_dir, out_dir)
        elif self._out_write_type is OutWriteType.TIMESTAMP:
            out_dir = out_dir + time.strftime("_%Y-%m-%d-%H-%M-%S")
        return os.path.join(self._out_working_dir, out_dir, "")

    def _check_if_valid_camera(self, path):
        context = omni.usd.get_context()
        stage = context.get_stage()
        prim = stage.GetPrimAtPath(path)

        if not prim.IsValid():
            carb.log_warn(f"{path} is not a valid prim path.")
            return False

        if prim.GetTypeName() == "Camera":
            return True
        else:
            carb.log_warn(f"{prim} is not a 'Camera' type.")
            return False

    def _check_if_valid_resolution(self, width, height):
        if 0 <= width <= MAX_RESOLUTION[0] and 0 <= height <= MAX_RESOLUTION[1]:
            return True
        else:
            carb.log_warn(
                f"Invalid resolution: {width}x{height}. Must be between 1x1 and {MAX_RESOLUTION[0]}x{MAX_RESOLUTION[1]}."
            )
        return False

    def _check_if_valid_rp_entry(self, entry):
        if (
            len(entry) == 3
            and self._check_if_valid_camera(entry[0])
            and self._check_if_valid_resolution(entry[1], entry[2])
        ):
            return True
        else:
            return False

    def _check_if_stage_is_semantically_labeled(self):
        stage = omni.usd.get_context().get_stage()
        for prim in stage.Traverse():
            if prim.HasAPI(Semantics.SemanticsAPI):
                return True
        carb.log_warn("Stage is not semantically labeled, semantics related annotators will not work.")
        return False

    def _check_if_stage_has_skeleton_prims(self):
        stage = omni.usd.get_context().get_stage()
        for prim in stage.Traverse():
            if prim.GetTypeName() == "Skeleton":
                return True
        carb.log_warn("Stage does not have any skeleton prims, skeleton annotator will not work.")
        return False

    def _remove_semantics_annotators(self, writer_params):
        disabled_annotators = []
        semantics_annotators = [
            "bounding_box_2d_tight",
            "bounding_box_2d_loose",
            "semantic_segmentation",
            "instance_id_segmentation",
            "instance_segmentation",
            "bounding_box_3d",
            "occlusion",
        ]

        for annotator in semantics_annotators:
            if annotator in writer_params and writer_params[annotator]:
                writer_params[annotator] = False
                disabled_annotators.append(annotator)
        if disabled_annotators:
            carb.log_warn(f"Disabled the following semantics related annotators: {disabled_annotators}.")

    def _update_rp_entry(self, idx, field, value):
        self._rp_data[idx][field] = value

    def _remove_rp_entry(self, idx):
        del self._rp_data[idx]
        self._build_window_ui()

    def _add_new_rp_field(self):
        # If cameras are selected in the stage viewer use them default values
        context = omni.usd.get_context()
        stage = context.get_stage()
        selected_prims = context.get_selection().get_selected_prim_paths()
        selected_cameras = [path for path in selected_prims if stage.GetPrimAtPath(path).GetTypeName() == "Camera"]

        if selected_cameras:
            for path in selected_cameras:
                self._rp_data.append([path, 512, 512])
        else:
            # Use selected viewport camera as default value
            active_vp = get_active_viewport()
            active_cam = active_vp.get_active_camera()
            self._rp_data.append([str(active_cam), 512, 512])

        self._build_window_ui()

    def _clear_recorder(self):
        if self._writer:
            self._writer.detach()
            self._writer = None
        self._render_products.clear()

    def _init_recorder(self) -> bool:
        if self._writer is None:
            try:
                self._writer = rep.WriterRegistry.get(self._writer_name)
            except Exception as e:
                carb.log_warn(f"Could not create writer {self._writer_name}: {e}")
                return False

        # Set the number of subframes
        if self._rt_subframes != carb.settings.get_settings().get("/omni/replicator/RTSubframes"):
            rep.settings.carb_settings("/omni/replicator/RTSubframes", self._rt_subframes)
            carb.log_info(f"Setting 'RTSubframes' to {self._rt_subframes}.")

        # Init the default or custom writer with its parameters
        writer_params = {}
        if self._writer_name == "BasicWriter":
            # In case S3 is selected, make sure the s3 parameters are valid
            if self._use_s3:
                # s3_bucket is a required parameter, if it is not set, do not initialize the writer
                if not self._s3_params["s3_bucket"]:
                    carb.log_warn("Could not initialize writer, s3_bucket parameters is missing.")
                    return False

                # Other S3 parameters are optional, set them to None in case of empty strings
                for key, value in self._s3_params.items():
                    if value == "":
                        self._s3_params[key] = None
                writer_params = {**self._basic_writer_params, **self._s3_params}
            else:
                writer_params = {**self._basic_writer_params}

            # Disable semantics related annotators if the stage is not semantically labeled
            stage_is_labeled = self._check_if_stage_is_semantically_labeled()
            if not stage_is_labeled:
                self._remove_semantics_annotators(writer_params)

            # Disable skeleton annotator if the stage does not have any skeleton prims
            if writer_params["skeleton_data"] and not self._check_if_stage_has_skeleton_prims():
                carb.log_warn("Stage does not have any skeleton prims, disabling 'skeleton_data' annotator.")
                writer_params["skeleton_data"] = False
        else:
            # Custom writers will not get any sanity cheks, it is up to the user to make sure the parameters are valid
            custom_params = self._get_custom_params(self._custom_params_path)
            writer_params = {**custom_params}

        # Output path can suffixed with an increment or a timestamp if the user has enabled the option
        output_dir = self._get_output_dir()
        try:
            self._writer.initialize(output_dir=output_dir, **writer_params)
        except Exception as e:
            carb.log_warn(f"Could not initialize writer {self._writer_name}: {e}")
            return False

        # Create the render products
        for rp_entry in self._rp_data:
            if self._check_if_valid_rp_entry(rp_entry):
                rp = rep.create.render_product(rp_entry[0], (rp_entry[1], rp_entry[2]))
                self._render_products.append(rp)
            else:
                carb.log_warn(f"Invalid render product entry {rp_entry}.")

        if not self._render_products:
            carb.log_warn("No valid render products found to initialize the writer.")
            return False

        try:
            self._writer.attach(self._render_products)
        except Exception as e:
            carb.log_warn(f"Could not attach render products to writer: {e}")
            return False
        return True

    async def _on_orchestrator_finish_async(self):
        if self._control_timeline:
            await self._set_timeline_state_async(case="reset")
        await rep.orchestrator.wait_until_complete_async()
        self._clear_recorder()
        self._disable_all_buttons()
        self._enable_buttons(case="stop")
        self._in_running_state = False

    async def _set_timeline_state_async(self, case="reset"):
        timeline = omni.timeline.get_timeline_interface()
        if case == "reset":
            if timeline.is_playing():
                timeline.stop()
            timeline.set_current_time(0)
            await omni.kit.app.get_app().next_update_async()
        elif case == "pause":
            if timeline.is_playing():
                timeline.pause()
        elif case == "resume":
            if not timeline.is_playing():
                timeline.play()

    async def _start_stop_recorder_async(self):
        if self._orchestrator_status is orchestrator.Status.STOPPED:
            self._disable_all_buttons()
            if self._init_recorder():
                num_frames = None if self._num_frames <= 0 else self._num_frames
                await rep.orchestrator.run_async(num_frames=num_frames, start_timeline=self._control_timeline)
                self._in_running_state = True
                self._enable_buttons(case="start")
            else:
                self._clear_recorder()
                self._enable_buttons(case="reset")
        elif self._orchestrator_status in [orchestrator.Status.STARTED, orchestrator.Status.PAUSED]:
            self._disable_all_buttons()
            await rep.orchestrator.stop_async()
            if self._control_timeline:
                await self._set_timeline_state_async(case="reset")
            self._clear_recorder()
            self._in_running_state = False
            self._enable_buttons(case="stop")
        else:
            carb.log_warn(
                f"Replicator's current state({self._orchestrator_status.name}) is different state than STOPPED, STARTED or PAUSED. Try again in a bit."
            )

    def _pause_resume_recorder(self):
        self._pause_resume_button.enabled = False
        if self._orchestrator_status is orchestrator.Status.STARTED:
            rep.orchestrator.pause()
            if self._control_timeline:
                asyncio.ensure_future(self._set_timeline_state_async(case="pause"))
            self._pause_resume_button.text = "Resume"
        elif self._orchestrator_status is orchestrator.Status.PAUSED:
            rep.orchestrator.resume()
            if self._control_timeline:
                asyncio.ensure_future(self._set_timeline_state_async(case="resume"))
            self._pause_resume_button.text = "Pause"
        else:
            carb.log_warn(
                f"Replicator's current state ({self._orchestrator_status.name}) is different state than STARTED or PAUSED. Try again in a bit."
            )
        self._pause_resume_button.enabled = True

    def _disable_all_buttons(self):
        self._start_stop_button.enabled = False
        self._pause_resume_button.enabled = False

    def _enable_buttons(self, case="reset"):
        if case == "reset":
            self._start_stop_button.text = "Start"
            self._pause_resume_button.text = "Pause"
            self._start_stop_button.enabled = True
        elif case == "start":
            self._start_stop_button.text = "Stop"
            self._start_stop_button.enabled = True
            self._pause_resume_button.enabled = True
        elif case == "stop":
            self._start_stop_button.text = "Start"
            self._pause_resume_button.text = "Pause"
            self._start_stop_button.enabled = True

    def _build_config_ui(self):
        with ui.VStack(spacing=5):
            with ui.HStack():
                ui.Spacer(width=10)
                ui.Label("Config Directory", tooltip="Config files directory path")
            with ui.HStack():
                ui.Spacer(width=10)
                config_dir_model = ui.StringField(read_only=False).model
                config_dir_model.set_value(self._config_dir)

                def config_dir_changed(model):
                    self._config_dir = model.as_string

                config_dir_model.add_value_changed_fn(config_dir_changed)

                ui.Spacer(width=5)
                ui.Button(
                    f"{_ui_get_open_folder_glyph()}",
                    width=20,
                    clicked_fn=lambda: self._open_dir(self._config_dir),
                    tooltip="Open config directory",
                )

                ui.Button(
                    f"{_ui_get_reset_glyph()}",
                    width=20,
                    clicked_fn=lambda: self._reset_config_dir(),
                    tooltip="Reset config directory to default",
                )

            with ui.HStack(spacing=5):
                ui.Spacer(width=5)
                config_file_model = ui.StringField(tooltip="Config file name").model
                config_file_model.set_value(self._config_file)

                def config_file_changed(model):
                    self._config_file = model.as_string

                config_file_model.add_value_changed_fn(config_file_changed)

                ui.Button(
                    "Load",
                    clicked_fn=lambda: self._load_config_and_refresh_ui(self._config_dir, self._config_file),
                    tooltip="Load and apply selected config file",
                )
                ui.Button(
                    "Save",
                    clicked_fn=lambda: self.save_config(os.path.join(self._config_dir, self._config_file)),
                    tooltip="Save current config to selected file",
                )

    def _build_s3_ui(self):
        with ui.VStack(spacing=5):
            with ui.HStack():
                ui.Spacer(width=10)
                ui.Label("Use S3", alignment=ui.Alignment.LEFT, tooltip="Write data to S3 buckets")
                s3_model = ui.CheckBox().model
                s3_model.set_value(self._use_s3)

                def value_changed(m):
                    self._use_s3 = m.as_bool
                    if self._use_s3 and self._out_write_type is OutWriteType.INCREMENT:
                        print(f"Incremental output is not supported for S3. Switching to Timestamp.")
                        self._out_write_type = OutWriteType.TIMESTAMP
                        # Rebuild ui to update radio buttons state to TIMESTAMP
                        self._build_window_ui()

                s3_model.add_value_changed_fn(value_changed)

            for key, val in self._s3_params.items():
                with ui.HStack():
                    ui.Spacer(width=10)
                    ui.Label(key, alignment=ui.Alignment.LEFT, tooltip=PARAM_TOOLTIPS[key])
                    model = ui.StringField().model
                    if val:
                        model.set_value(val)
                    else:
                        model.set_value("")

                    def value_changed(m, k=key):
                        self._s3_params[k] = m.as_string

                    model.add_value_changed_fn(value_changed)

    def _build_output_ui(self):
        with ui.VStack(spacing=5):
            with ui.HStack():
                ui.Spacer(width=10)
                ui.Label("Working Directory")
            with ui.HStack():
                ui.Spacer(width=10)
                out_working_dir_model = ui.StringField().model
                out_working_dir_model.set_value(self._out_working_dir)

                def out_working_dir_changed(model):
                    self._out_working_dir = model.as_string

                out_working_dir_model.add_value_changed_fn(out_working_dir_changed)

                ui.Spacer(width=5)
                ui.Button(
                    f"{_ui_get_open_folder_glyph()}",
                    width=20,
                    clicked_fn=lambda: self._open_dir(self._out_working_dir),
                    tooltip="Open working directory",
                )

                ui.Button(
                    f"{_ui_get_reset_glyph()}",
                    width=20,
                    clicked_fn=lambda: self._reset_out_working_dir(),
                    tooltip="Reset directory to default",
                )

            with ui.HStack(spacing=5):
                ui.Spacer(width=5)
                out_dir_model = ui.StringField().model
                out_dir_model.set_value(self._out_dir)

                def out_dir_changed(model):
                    self._out_dir = model.as_string

                out_dir_model.add_value_changed_fn(out_dir_changed)

                write_collection = ui.RadioCollection()
                write_collection.model.set_value(self._out_write_type.value)

                def write_collection_changed(model):
                    out_write_type = OutWriteType(model.as_int)
                    if self._use_s3 and out_write_type is OutWriteType.INCREMENT:
                        print(f"Incremental output is not supported for S3. Switching to Timestamp.")
                        self._out_write_type = OutWriteType.TIMESTAMP
                        # Rebuild ui to update radio buttons state to TIMESTAMP
                        self._build_window_ui()
                    else:
                        self._out_write_type = out_write_type

                write_collection.model.add_value_changed_fn(write_collection_changed)

                ui.RadioButton(
                    text="Overwrite",
                    radio_collection=write_collection,
                    tooltip="Overwrite data if output folder already exists",
                )
                ui.RadioButton(
                    text="Increment",
                    radio_collection=write_collection,
                    tooltip="Append numerical increments to output folder (e.g., _01, _02). NOTE: does not work with S3",
                )
                ui.RadioButton(
                    text="Timestamp",
                    radio_collection=write_collection,
                    tooltip="Append timestamp to output folder (e.g., _YYYY-mm-dd-HH-MM-SS)",
                )

            s3_frame = ui.CollapsableFrame("S3 Bucket", height=0, collapsed=self._s3_params_frame_collapsed)
            with s3_frame:

                def on_collapsed_changed(collapsed):
                    self._s3_params_frame_collapsed = collapsed

                s3_frame.set_collapsed_changed_fn(on_collapsed_changed)
                self._build_s3_ui()

    def _build_rp_ui(self):
        with ui.VStack(spacing=5):
            with ui.HStack(spacing=5):
                ui.Spacer(width=15)
                ui.Label("Camera Path", width=200, tooltip="Camera prim to be used as a render product")
                ui.Spacer(width=15)
                ui.Label("X", tooltip="X resolution of the render product")
                ui.Spacer(width=15)
                ui.Label("Y", tooltip="Y resolution of the render product")
            for i, entry in enumerate(self._rp_data):
                with ui.HStack(spacing=5):
                    ui.Spacer(width=10)
                    path_field_model = ui.StringField(width=200).model
                    path_field_model.set_value(entry[0])
                    path_field_model.add_value_changed_fn(lambda m, idx=i: self._update_rp_entry(idx, 0, m.as_string))
                    ui.Spacer(width=10)
                    x_field = ui.IntField()
                    x_field.model.set_value(entry[1])
                    x_field.model.add_value_changed_fn(lambda m, idx=i: self._update_rp_entry(idx, 1, m.as_int))
                    ui.Spacer(width=10)
                    y_field = ui.IntField()
                    y_field.model.set_value(entry[2])
                    y_field.model.add_value_changed_fn(lambda m, idx=i: self._update_rp_entry(idx, 2, m.as_int))
                    ui.Button(
                        f"{_ui_get_delete_glyph()}",
                        width=30,
                        clicked_fn=lambda idx=i: self._remove_rp_entry(idx),
                        tooltip="Remove entry",
                    )
            with ui.HStack(spacing=5):
                ui.Spacer(width=5)
                ui.Button(
                    "Add New Render Product Entry", clicked_fn=self._add_new_rp_field, tooltip="Create a new entry"
                )

    def _build_params_ui(self):
        with ui.VStack(spacing=5):
            with ui.HStack():
                ui.Spacer(width=10)
                ui.Label("Writer")

                writer_type_collection = ui.RadioCollection()
                if self._writer_name == "BasicWriter":
                    writer_type_collection.model.set_value(0)
                else:
                    writer_type_collection.model.set_value(1)

                def writer_type_collection_changed(model):
                    if model.as_int == 0:
                        self._custom_writer_name = self._writer_name
                        self._writer_name = "BasicWriter"
                    else:
                        self._writer_name = self._custom_writer_name
                    # self._writer_name = "BasicWriter" if model.as_int == 0 else "CustomWriter"
                    self._build_window_ui()

                writer_type_collection.model.add_value_changed_fn(writer_type_collection_changed)

                ui.RadioButton(
                    text="Default", radio_collection=writer_type_collection, tooltip="Uses the default BasicWriter"
                )
                ui.RadioButton(
                    text="Custom", radio_collection=writer_type_collection, tooltip="Loads a custom writer by name"
                )

            if self._writer_name == "BasicWriter":
                self._build_basic_writer_ui()
            else:
                self._build_custom_writer_ui()

    def _build_basic_writer_ui(self):
        for key, val in self._basic_writer_params.items():
            with ui.HStack(spacing=5):
                ui.Spacer(width=10)
                ui.Label(key, alignment=ui.Alignment.LEFT, tooltip=PARAM_TOOLTIPS[key])
                model = ui.CheckBox().model
                model.set_value(val)

                def value_changed(m, k=key):
                    self._basic_writer_params[k] = m.as_bool

                model.add_value_changed_fn(value_changed)

    def _build_custom_writer_ui(self):
        with ui.HStack(spacing=5):
            ui.Spacer(width=10)
            ui.Label("Name", tooltip="The name of the custom writer from the registry")
            writer_name_model = ui.StringField().model
            writer_name_model.set_value(self._writer_name)

            def writer_name_changed(m):
                self._writer_name = m.as_string

            writer_name_model.add_value_changed_fn(writer_name_changed)

        with ui.HStack(spacing=5):
            ui.Spacer(width=10)
            ui.Label("Parameters Path", tooltip="Path to the json file storing the custom writer parameters")
            path_model = ui.StringField().model
            path_model.set_value(self._custom_params_path)

            def path_changed(m):
                self._custom_params_path = m.as_string

            path_model.add_value_changed_fn(path_changed)

    def _build_writer_ui(self):
        with ui.VStack(spacing=5):
            rp_frame = ui.CollapsableFrame("Render Products", height=0, collapsed=self._rp_frame_collapsed)
            with rp_frame:

                def on_collapsed_changed(collapsed):
                    self._rp_frame_collapsed = collapsed

                rp_frame.set_collapsed_changed_fn(on_collapsed_changed)
                self._build_rp_ui()

            params_frame = ui.CollapsableFrame("Parameters", height=0, collapsed=self._writer_params_frame_collapsed)
            with params_frame:

                def on_collapsed_changed(collapsed):
                    self._writer_params_frame_collapsed = collapsed

                params_frame.set_collapsed_changed_fn(on_collapsed_changed)
                self._build_params_ui()

            output_frame = ui.CollapsableFrame("Output", height=0, collapsed=self._output_frame_collapsed)
            with output_frame:

                def on_collapsed_changed(collapsed):
                    self._output_frame_collapsed = collapsed

                output_frame.set_collapsed_changed_fn(on_collapsed_changed)
                self._build_output_ui()

            config_frame = ui.CollapsableFrame("Config", height=0, collapsed=self._config_frame_collapsed)
            with config_frame:

                def on_collapsed_changed(collapsed):
                    self._config_frame_collapsed = collapsed

                config_frame.set_collapsed_changed_fn(on_collapsed_changed)
                self._build_config_ui()

    def _build_control_params_ui(self):
        with ui.VStack(spacing=10):
            with ui.HStack(spacing=5):
                ui.Spacer(width=10)
                ui.Label("Number of frames", tooltip="If set to 0, data acquisition will run indefinitely")
                num_frames_model = ui.IntField().model
                num_frames_model.set_value(self._num_frames)

                def num_frames_changed(m):
                    self._num_frames = m.as_int

                num_frames_model.add_value_changed_fn(num_frames_changed)

                ui.Label("RTSubframes", tooltip="Render extra frames between captures to avoid rendering artifacts")
                rt_subframes_model = ui.IntField().model
                rt_subframes_model.set_value(self._rt_subframes)

                def num_rt_subframes_changed(m):
                    self._rt_subframes = m.as_int

                rt_subframes_model.add_value_changed_fn(num_rt_subframes_changed)

            with ui.HStack(spacing=5):
                ui.Spacer(width=10)
                ui.Label(
                    "Control Timeline",
                    alignment=ui.Alignment.LEFT,
                    tooltip="Start/Stop/Pause/Reset the timeline with the recorder",
                )
                control_timeline_model = ui.CheckBox().model
                control_timeline_model.set_value(self._control_timeline)

                def value_changed(m):
                    self._control_timeline = m.as_bool

                control_timeline_model.add_value_changed_fn(value_changed)

    def _build_control_ui(self):
        with ui.VStack(spacing=5):
            control_params_frame = ui.CollapsableFrame(
                "Parameters", height=0, collapsed=self._control_params_frame_collapsed
            )
            with control_params_frame:

                def on_collapsed_changed(collapsed):
                    self._control_params_frame_collapsed = collapsed

                control_params_frame.set_collapsed_changed_fn(on_collapsed_changed)
                self._build_control_params_ui()

            with ui.HStack(spacing=5):
                ui.Spacer(width=5)
                self._start_stop_button = ui.Button(
                    "Start",
                    clicked_fn=lambda: asyncio.ensure_future(self._start_stop_recorder_async()),
                    enabled=True,
                    tooltip="Start/stop the recording",
                )
                self._pause_resume_button = ui.Button(
                    "Pause", clicked_fn=self._pause_resume_recorder, enabled=False, tooltip="Pause/resume recording"
                )

    def _build_window_ui(self):
        with self._window.frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=5):
                    writer_frame = ui.CollapsableFrame("Writer", height=0, collapsed=self._writer_frame_collapsed)
                    with writer_frame:

                        def on_collapsed_changed(collapsed):
                            self._writer_frame_collapsed = collapsed

                        writer_frame.set_collapsed_changed_fn(on_collapsed_changed)
                        self._build_writer_ui()

                    control_frame = ui.CollapsableFrame("Control", height=0, collapsed=self._control_frame_collapsed)
                    with control_frame:

                        def on_collapsed_changed(collapsed):
                            self._control_frame_collapsed = collapsed

                        control_frame.set_collapsed_changed_fn(on_collapsed_changed)
                        self._build_control_ui()
