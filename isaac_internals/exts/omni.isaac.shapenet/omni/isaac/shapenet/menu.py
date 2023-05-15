# Copyright (c) 2018-2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.ui as ui
from omni.kit.menu.utils import add_menu_items, remove_menu_items, MenuItemDescription
from omni.isaac.ui.menu import make_menu_item_description
import weakref
from .settings import ShapenetSettings
from .globals import *

EXTENSION_NAME = "ShapeNet Loader"

HELP_TEXT = (
    "    This omni.isaac.shapenet plugin allows you to add ShapeNetCore.V2 models from shapenet.org to your stage in Omniverse Kit.\n\n"
    "    You can use the ShapeNet menu to add shapes.\n\n"
    "    If should already have ShapeNetCore V2 installed locally, this plugin will use the local files.  Use the env var SHAPENET_LOCAL_DIR to set that location (IMPORTANT NOTE: Make sure there are no periods, ., in the path name), otherwise, omni.isaac.shapenet will use the default ${data}/shapenet folder.  By using local folders, you can edit shapenet models before their conversion to usd.  If you want to keep the original file, just save the modified file as "
    ' "models/modified/model.obj" in that shape\'s /models folder.\n\n'
    "    If the shape is already on the omniverse server at g_omni_shape_loc (defaults to /Projects/shapenet), then that model will be used instead of the locally saved or modified shapenet obj file.\n\n"
)


class ShapenetMenu:
    def __init__(self, ext_id: str):
        self._window = None
        self._settings_ui = None
        self._models = {}

        self._menu_items = [
            make_menu_item_description(ext_id, EXTENSION_NAME, lambda a=weakref.proxy(self): a._create_window())
        ]
        add_menu_items(self._menu_items, "Isaac Utils")

        # self._create_window() # comment this out to prevent window from being visible until menu is clicked

    def _create_window(self):
        if self._window == None:
            """ build ShapeNet window"""
            self._window = ui.Window(
                title="ShapeNet Loader",
                width=400,
                height=150,
                visible=True,
                dockPreference=ui.DockPreference.LEFT_BOTTOM,
            )
            self._window.deferred_dock_in("Console", ui.DockPolicy.DO_NOTHING)
            with self._window.frame:
                with ui.VStack():
                    with ui.HStack(height=20):
                        self._models["add_button"] = ui.Button(
                            "Add A Model", width=0, clicked_fn=lambda b=None: self._settings_ui._on_add_model_fn()
                        )
                        ui.Spacer()
                        self._models["help_button"] = ui.Button(
                            "Help", width=0, clicked_fn=lambda b=None: self._on_help_menu_click()
                        )
                    with ui.HStack(height=20):
                        self._create_settings_ui()
            # TODO need this? self._models["add_button"].visible = True

        self._window.visible = True

    def _create_settings_ui(self):
        self._settings_ui = ShapenetSettings()

    def _on_help_menu_click(self):
        help_message = HELP_TEXT

        flags = ui.WINDOW_FLAGS_NO_RESIZE | ui.WINDOW_FLAGS_MODAL
        flags |= ui.WINDOW_FLAGS_NO_SCROLLBAR
        self._help_window = ui.Window("Shapenet Help", width=500, height=0, flags=flags)
        with self._help_window.frame:
            with ui.VStack(name="root", style={"VStack::root": {"margin": 10}}, height=0, spacing=20):
                ui.Label(help_message, alignment=ui.Alignment.LEFT, word_wrap=True)

    def shutdown(self):
        remove_menu_items(self._menu_items, "Isaac Utils")
        self._menu_items = None
        self._settings_ui = None
