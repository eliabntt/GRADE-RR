# Copyright (c) 2021-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

__all__ = ["VisualizerWindow"]

import omni.ui as ui
from .SyntheticData import SyntheticData
from .model import RenderProductModel, RenderVarModel
import math

DEBUG_VIEW = False


class VisualizerWindow:

    def __init__(self, name, viewport_api):

        # create the window
        self._visualize_window = ui.Window(name + " Sensors Output ", width=800, height=600)
        self._visualize_window.set_width_changed_fn(lambda _: self._update_visualization_ui())
        self._visualize_window.set_height_changed_fn(lambda _: self._update_visualization_ui())
        self._visualize_window.visible = False
        self._render_product_combo_model = RenderProductModel(name, viewport_api) if DEBUG_VIEW else None
        self._render_var_combo_model = RenderVarModel(viewport_api)
        self._render_product_path = self._render_var_combo_model.get_render_product_path()

        # activated visualization contains the set of display node that have been activated through the UI
        self._visualization_activation = set()
        # visualisation_data contain the image provider for all currently activated display node
        self._activated_visualization_data = {}

        if hasattr(viewport_api, 'subscribe_to_frame_change'):
            self.__frame_changed_sub = viewport_api.subscribe_to_frame_change(self.__frame_changed)

    def __frame_changed(self, viewport_api):
        render_product = self._render_var_combo_model.get_render_product_path()
        self.update(render_product, viewport_api.stage)

    def close(self):
        self.__frame_changed_sub = None
        if self._visualize_window:
            self._visualize_window.visible = False
            self._visualize_window = None
        if self._render_product_combo_model:
            self._render_product_combo_model = None
        if self._render_var_combo_model:
            self._render_var_combo_model = None
        self._visualization_activation = set()
        self._activated_visualization_data = {}

    @property
    def render_product_combo_model(self):
        return self._render_product_combo_model

    @property
    def render_var_combo_model(self):
        return self._render_var_combo_model

    @property
    def visualization_activation(self):
        return self._visualization_activation

    # callback function for handling sensor selection
    def on_sensor_item_clicked(self, checked, sensor):
        if checked:
            self._visualization_activation.add(sensor)
        else:
            self._visualization_activation.remove(sensor)

    # visualization callback
    def toggle_enable_visualization(self):
        if self._visualize_window:
            self._visualize_window.visible = not self._visualize_window.visible

    def update(self, render_product_path: str, stage):
        sdg_iface = SyntheticData.Get()
    
        if render_product_path != self._render_product_path:
             for sensor in self._activated_visualization_data:
                sdg_iface.deactivate_node_template(sensor,0,[render_product_path])
             self._visualization_activation = set()
             self._activated_visualization_data = {}             
             self._render_product_path = render_product_path
             self._render_var_combo_model.set_default_item()

        # update the activated sensors
        visualization_activation = self._visualization_activation.copy()  # NB this is not threadsafe
        to_activate = visualization_activation.difference(set(self._activated_visualization_data.keys()))
        to_deactivate = set(self._activated_visualization_data.keys()).difference(visualization_activation)

        self._activated_visualization_data = {}
        for sensor in visualization_activation:
            self._activated_visualization_data[sensor] = None

        for sensor in to_activate:
            sdg_iface.activate_node_template(sensor, 0, [render_product_path], None, stage)

        for sensor in to_deactivate:
            sdg_iface.deactivate_node_template(sensor, 0, [render_product_path], stage)

        # update the visualization window
        if self._visualize_window.visible:
            for sensor in self._activated_visualization_data:
                # create image provider from the sensor texture data
                self._activated_visualization_data[sensor] = ui.ImageProvider()
                display_output_names = ["outputs:handlePtr", "outputs:width", "outputs:height", "outputs:format"]
                display_outputs = sdg_iface.get_node_attributes(sensor, display_output_names, render_product_path)
                if display_outputs and all(o in display_outputs for o in display_output_names):
                    self._activated_visualization_data[sensor].set_image_data(
                        display_outputs["outputs:handlePtr"],
                        display_outputs["outputs:width"],
                        display_outputs["outputs:height"],
                        ui.TextureFormat(display_outputs["outputs:format"])
                    )

            self._update_visualization_ui()

    def _update_visualization_ui(self):
        num_sensors = len(self._activated_visualization_data)
        if num_sensors == 0:
            rows, columns = 0, 0
        else:
            # Attempt a responsive layout to the number of enabled sensors
            columns = math.ceil(math.sqrt(num_sensors))
            rows = math.ceil(num_sensors / columns)
            if self._visualize_window.height > self._visualize_window.width:
                columns, rows = rows, columns

        enabled_sensors = list(self._activated_visualization_data.keys())
        with self._visualize_window.frame:
            with ui.VStack():
                idx = 0
                for _ in range(rows):
                    with ui.HStack():
                        for col in range(columns):
                            sensor = enabled_sensors[idx]
                            with ui.VStack():
                                ui.Label(sensor, alignment=ui.Alignment.CENTER, height=20)
                                ui.ImageWithProvider(
                                    self._activated_visualization_data[sensor], alignment=ui.Alignment.CENTER_TOP
                                )
                                ui.Spacer(height=20)
                                idx += 1
                            if col < columns - 1:
                                # Add a spacer if inner grid edge
                                ui.Spacer(width=3)
                            if idx >= len(enabled_sensors):
                                break
