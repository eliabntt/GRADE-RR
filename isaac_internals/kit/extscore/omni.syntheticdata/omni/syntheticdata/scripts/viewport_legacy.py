from pathlib import Path

from pxr import Sdf

import carb.settings

import omni.ui as ui
import omni.usd

from .SyntheticData import SyntheticData
from .visualizer_window import VisualizerWindow
import weakref


CURRENT_PATH = Path(__file__).parent.absolute()
ICON_PATH = CURRENT_PATH.parent.parent.parent.joinpath("data")
BUTTON_STYLE = {
    "height": 22,
    "width": 26,
    "style": {"Button": {"padding": 4, "background_color": 0x80303030}},
    "image_height": 14,
    "image_width": 26,
}
MENU_FLAGS = {"flags": ui.WINDOW_FLAGS_POPUP | ui.WINDOW_FLAGS_NO_TITLE_BAR, "auto_resize": True}


class ViewportLegacy:
    _g_visualizers = {}
    _g_iface = None

    @staticmethod
    def create_update_subscription():
        import omni.kit.viewport_legacy
        ViewportLegacy._g_iface = omni.kit.viewport_legacy.get_viewport_interface()
        if ViewportLegacy._g_iface is None:
            return

        import omni.kit.app
        event_stream = omni.kit.app.get_app().get_update_event_stream()
        return event_stream.create_subscription_to_pop(ViewportLegacy._on_update, name="omni.syntheticdata update")

    @staticmethod
    def close_viewports():
        visualizers, ViewportLegacy._g_visualizers = ViewportLegacy._g_visualizers, {}
        if visualizers:
            for visualizer, vp_delegate in visualizers.values():
                visualizer.close()
                vp_delegate.destroy()

    @staticmethod
    def _on_update(dt):
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return

        # retrieve the list of active viewports
        viewport_names = set([ViewportLegacy._g_iface.get_viewport_window_name(vp) for vp in ViewportLegacy._g_iface.get_instance_list()])
        visualizers = ViewportLegacy._g_visualizers

        # remove obsolete extension viewports data
        for vp_name in set(visualizers.keys()).difference(viewport_names):
            visualizer, vp_delegate = visualizers[vp_name]
            visualizer.close()
            vp_delegate.destroy()
            del visualizers[vp_name]

        # create missing extension viewports data
        for vp_name in viewport_names.difference(set(visualizers.keys())):
            vp_delegate = ViewportLegacy(vp_name)
            visualizer_window = VisualizerWindow(vp_name, vp_delegate)
            vp_delegate.set_visualizer_window(weakref.proxy(visualizer_window))
            visualizers[vp_name] = visualizer_window, vp_delegate

        # update all valid viewport
        for vp_name, vis_and_delegate in visualizers.items():
            legacy_vp = ViewportLegacy._g_iface.get_viewport_window(ViewportLegacy._g_iface.get_instance(vp_name))
            if legacy_vp:
                visualizer, vp_delegate = vis_and_delegate
                camera_path = legacy_vp.get_active_camera()
                vp_delegate._update_legacy_buttons(Sdf.Path(camera_path).name, legacy_vp.is_visible())
                visualizer.update(legacy_vp.get_render_product_path(), stage)

    def __init__(self, name: str):
        self.__window_name = name
        self.__visualizer_window = None

        # initialize ui
        self.__menus = None
        self.__btns = {"window": ui.Window(name, detachable=False)}
        with self.__btns["window"].frame:
            with ui.VStack():
                ui.Spacer(height=4)
                with ui.HStack(height=0, width=0):
                    self.__btns["spacer"] = ui.Spacer(width=300)
                    self.__btns["icon"] = ui.Button(
                        tooltip="Synthetic Data Sensors", image_url=f"{ICON_PATH}/sensor_icon.svg", **BUTTON_STYLE
                    )
                    self.__btns["icon"].set_mouse_pressed_fn(lambda x, y, *_: self._show_legacy_ui_menu(x, y))

    def __del__(self):
        self.destroy()

    def destroy(self):
        self.__btns = None
        self.__menus = None
        self.__window_name = None
        self.__visualizer_window = None

    def set_visualizer_window(self, visualizer_window):
        self.__visualizer_window = visualizer_window

    def _update_legacy_buttons(self, cam_name: str, is_visible: bool):
        # update the buttons in a legacy viewport (dependent on camera name length)
        render_mode = carb.settings.get_settings().get("/rtx/rendermode")
        render_spacing = 15
        if render_mode == "RaytracedLighting":
            render_spacing = 12
        elif render_mode == "PathTracing":
            render_spacing = 31
        spacing = 5 + (len(cam_name) + render_spacing) * 15
        self.__btns["spacer"].width = ui.Length(max(300, spacing))
        self.__btns["window"].visible = is_visible

    def _build_legacy_ui_menu(self):
        self.__menus = ui.Window(f"{self.__window_name}-sensor-menu", **MENU_FLAGS)
        with self.__menus.frame:
            with ui.VStack(width=200, spacing=5):
                render_product_combo_model = self.__visualizer_window.render_product_combo_model
                if render_product_combo_model:
                    with ui.HStack(height=40):
                        ui.Label("RenderProduct", width=150)
                        ui.ComboBox(render_product_combo_model)

                render_var_combo_model = self.__visualizer_window.render_var_combo_model
                if render_var_combo_model:
                    with ui.HStack(height=40):
                        ui.Label("RenderVar", width=150)
                        ui.ComboBox(render_var_combo_model)
                    with ui.HStack(height=20):
                        model = ui.FloatSlider(name="angle", min=-100.0, max=100.0).model
                        model.add_value_changed_fn(
                            lambda m: render_var_combo_model.set_combine_angle(m.get_value_as_float())
                        )
                        model = ui.FloatSlider(name="x", min=-100.0, max=100.0).model
                        model.add_value_changed_fn(
                            lambda m: render_var_combo_model.set_combine_divide_x(m.get_value_as_float())
                        )
                        model = ui.FloatSlider(name="y", min=-100.0, max=100.0).model
                        model.add_value_changed_fn(
                            lambda m: render_var_combo_model.set_combine_divide_y(m.get_value_as_float())
                        )

                with ui.HStack(height=40):
                    ui.Label("Synthetic Data Sensors", width=150)
                    btn = ui.Button("Clear All")

                selection_stack = ui.VStack(spacing=5)
                btn.set_clicked_fn(lambda ss=selection_stack: self._clear_all(ss))
                selection_stack.clear()
                with selection_stack:
                    self._build_ui_sensor_selection()

        self.__menus.visible = False

    # callback to reset the sensor selection
    def _clear_all(self, selection_stack):
        self.__visualizer_window.visualization_activation.clear()
        selection_stack.clear()
        with selection_stack:
            self._build_ui_sensor_selection()

    def _show_window(self):
        self.__visualizer_window.toggle_enable_visualization()
        SyntheticData.disable_async_rendering()

    def _build_ui_sensor_selection(self):
        for sensor_label, sensor in SyntheticData.get_registered_visualization_template_names_for_display():
            with ui.HStack():
                ui.Label(sensor_label, width=300)
                cb = ui.CheckBox(
                    width=0, style={"font_size": 24, "margin": 3}, style_type_name_override="Options.CheckBox"
                )
                cb.model.set_value(sensor in self.__visualizer_window.visualization_activation)
                cb.model.add_value_changed_fn(lambda c, s=sensor: self.__visualizer_window.on_sensor_item_clicked(c.as_bool, s))

        ui.Button("Show", height=40, clicked_fn=lambda: self._show_window())

    def _show_legacy_ui_menu(self, x, y):
        self.__menus = None
        self._build_legacy_ui_menu()
        self.__menus.position_x = x
        self.__menus.position_y = y
        self.__menus.visible = True

    @property
    def render_product_path(self):
        legacy_vp = ViewportLegacy._g_iface.get_viewport_window(ViewportLegacy._g_iface.get_instance(self.__window_name))
        return legacy_vp.get_render_product_path() if legacy_vp else None

    @render_product_path.setter
    def render_product_path(self, prim_path: str):
        legacy_vp = ViewportLegacy._g_iface.get_viewport_window(ViewportLegacy._g_iface.get_instance(self.__window_name))
        if legacy_vp:
            legacy_vp.set_render_product_path(prim_path)

    @property
    def usd_context(self):
        legacy_vp = ViewportLegacy._g_iface.get_viewport_window(ViewportLegacy._g_iface.get_instance(self.__window_name))
        usd_context_name = legacy_vp.get_usd_context_name() if hasattr(legacy_vp, 'get_usd_context_name') else ''
        return omni.usd.get_context(usd_context_name)
