# Copyright (c) 2021-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

__all__ = ["SynthDataMenuContainer"]


from omni.kit.viewport.menubar.core import (
    ComboBoxModel,
    ComboBoxItem,
    ComboBoxMenuDelegate,
    CheckboxMenuDelegate,
    IconMenuDelegate,
    SliderMenuDelegate,
    ViewportMenuContainer,
    ViewportMenuItem,
    ViewportMenuSeparator
)
from .SyntheticData import SyntheticData
from .visualizer_window import VisualizerWindow

import carb
import omni.ui as ui

from pathlib import Path
import weakref


ICON_PATH = Path(carb.tokens.get_tokens_interface().resolve("${omni.syntheticdata}")).joinpath("data")
UI_STYLE = {"Menu.Item.Icon::SyntheticData": {"image_url": str(ICON_PATH.joinpath("sensor_icon.svg"))}}


class SensorAngleModel(ui.AbstractValueModel):
    def __init__(self, getter, setter, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.__getter = getter
        self.__setter = setter

    def destroy(self):
        self.__getter = None
        self.__setter = None

    def get_value_as_float(self) -> float:
        return self.__getter()

    def get_value_as_int(self) -> int:
        return int(self.get_value_as_float())

    def set_value(self, value):
        value = float(value)
        if self.get_value_as_float() != value:
            self.__setter(value)
            self._value_changed()


class SensorVisualizationModel(ui.AbstractValueModel):
    def __init__(self, sensor: str, visualizer_window, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.__sensor = sensor
        self.__visualizer_window = visualizer_window

    def destroy(self):
        self.__visualizer_window = None

    def get_value_as_bool(self) -> bool:
        try:
            return bool(self.__sensor in self.__visualizer_window.visualization_activation)
        except:
            return False

    def get_value_as_int(self) -> int:
        return 1 if self.get_value_as_bool() else 0

    def set_value(self, enabled):
        enabled = bool(enabled)
        if self.get_value_as_bool() != enabled:
            self.__visualizer_window.on_sensor_item_clicked(enabled, self.__sensor)
            self._value_changed()


class SynthDataMenuContainer(ViewportMenuContainer):
    def __init__(self):
        super().__init__(name="SyntheticData",
                         delegate=IconMenuDelegate("SyntheticData"), # tooltip="Synthetic Data Sensors"),
                         order=-10, style=UI_STYLE)

        self.__hide_on_click = False
        self.__visualizer_window = None
        self.__sensor_models = set()

    def __del__(self):
        self.destroy()

    def destroy(self):
        self.__sensor_models = set()
        if self.__visualizer_window:
            self.__visualizer_window.close()
            self.__visualizer_window = None
        super().destroy()

    def build_fn(self, desc: dict):
        viewport_api = desc.get("viewport_api")
        if not viewport_api:
            raise RuntimeError("Need a viewport_api")

        if self.__visualizer_window:
            self.__visualizer_window.close()
            self.__visualizer_window = None

        name = f"{viewport_api.usd_context_name}"
        self.__visualizer_window = VisualizerWindow(name, viewport_api)

        with self:
            self.add_render_settings_items()

            ViewportMenuSeparator()
            self.add_angles_items()

            ViewportMenuSeparator()
            self.add_sensor_selection()

            ViewportMenuSeparator()
            ViewportMenuItem(name="Clear All", hide_on_click=self.__hide_on_click, onclick_fn=self.clear_all)
            ViewportMenuItem(name="Show Window", hide_on_click=self.__hide_on_click, onclick_fn=self.show_window)

        super().build_fn(desc)

    def add_render_settings_items(self):
        render_product_combo_model = self.__visualizer_window.render_product_combo_model
        if render_product_combo_model:
            ViewportMenuItem(
                "RenderProduct",
                delegate=ComboBoxMenuDelegate(model=render_product_combo_model),
                hide_on_click=self.__hide_on_click,
            )

        render_var_combo_model = self.__visualizer_window.render_var_combo_model
        if render_var_combo_model:
            ViewportMenuItem(
                "RenderVar",
                delegate=ComboBoxMenuDelegate(model=render_var_combo_model),
                hide_on_click=self.__hide_on_click,
            )

    def add_angles_items(self):
        render_var_combo_model = self.__visualizer_window.render_var_combo_model
        if render_var_combo_model:
            ViewportMenuItem(
                name="Angle",
                hide_on_click=self.__hide_on_click,
                delegate=SliderMenuDelegate(
                    model=SensorAngleModel(render_var_combo_model.get_combine_angle,
                                           render_var_combo_model.set_combine_angle),
                    min=-100.0,
                    max=100.0,
                    tooltip="Set Combine Angle",
                ),
            )

            ViewportMenuItem(
                name="X",
                hide_on_click=self.__hide_on_click,
                delegate=SliderMenuDelegate(
                    model=SensorAngleModel(render_var_combo_model.get_combine_divide_x,
                                           render_var_combo_model.set_combine_divide_x),
                    min=-100.0,
                    max=100.0,
                    tooltip="Set Combine Divide X",
                ),
            )

            ViewportMenuItem(
                name="Y",
                hide_on_click=self.__hide_on_click,
                delegate=SliderMenuDelegate(
                    model=SensorAngleModel(render_var_combo_model.get_combine_divide_y,
                                           render_var_combo_model.set_combine_divide_y),
                    min=-100.0,
                    max=100.0,
                    tooltip="Set Combine Divide Y",
                ),
            )

    def add_sensor_selection(self):
        for sensor_label, sensor in SyntheticData.get_registered_visualization_template_names_for_display():
            model = SensorVisualizationModel(sensor, weakref.proxy(self.__visualizer_window))
            self.__sensor_models.add(model)
            ViewportMenuItem(
                name=sensor_label,
                hide_on_click=self.__hide_on_click,
                delegate=CheckboxMenuDelegate(model=model, tooltip=f'Enable "{sensor}" visualization')
            )

    def clear_all(self, *args, **kwargs):
        for smodel in self.__sensor_models:
            smodel.set_value(False)
        # XXX: This isn't really neccessary
        if self.__visualizer_window:
            self.__visualizer_window.visualization_activation.clear()

    def show_window(self, *args, **kwargs):
        self.__visualizer_window.toggle_enable_visualization()
        SyntheticData.disable_async_rendering()
