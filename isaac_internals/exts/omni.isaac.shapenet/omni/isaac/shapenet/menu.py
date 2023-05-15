# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.ui as ui
from omni.kit.menu.utils import add_menu_items, remove_menu_items, rebuild_menus, MenuItemDescription
import weakref
from .login import ShapenetLogin
from .settings import ShapenetSettings
from .globals import *
from pathlib import Path
import omni

EXTENSION_NAME = "ShapeNet Loader"

ADD_DB_TEXT = (
    "    Please register an account at https://www.shapenet.org/ so you can make the "
    "database of ShapeNetCore.V1 csv files necessary to run this extension.  Once you "
    "have a valid shapenet.org login, use the menu to create the database.  You "
    "should only have to do this once.\n\n"
)

HELP_TEXT = (
    "    This omni.isaac.shapenet plugin allows you to add ShapeNetCore.V2 models from shapenet.org to your stage in Omniverse Kit.\n\n"
    "    You can use the ShapeNet menu to add shapes.\n\n"
    "    You can also use an external python session to send json formatted commands via http and load shapes with comm_kit.py.\n\n"
    "    See comm_kit.test_comm() or run:\n"
    "\t>  jupyter notebook ShapeNet Python Example.ipynb\n"
    "for examples.\n\n"
    "    If you already have ShapeNetCore V2 installed locally, this plugin can use the local files.  Use the env var SHAPENET_LOCAL_DIR to set that location (IMPORTANT NOTE: Make sure there are no periods, ., in the path name), otherwise, omni.isaac.shapenet will use the default ${data}/shapenet folder.  By using local folders, you can edit shapenet models before their conversion to usd.  If you want to keep the original file, just save the modified file as "
    ' "models/modified/model.obj" in that shape\'s /models folder.\n\n'
    "    If the shape is already on the omniverse server at g_omni_shape_loc (defaults to /Projects/shapenet), then that model will be used instead of the downloaded original or locally saved or modified shapenet obj file.\n\n"
)


class ShapenetMenu:
    def __init__(self):
        self._window = None
        self._settings_ui = None
        self._login_ui = None
        self._models = {}

        self._menu_items = [
            MenuItemDescription(name=EXTENSION_NAME, onclick_fn=lambda a=weakref.proxy(self): a._create_window())
        ]
        add_menu_items(self._menu_items, "Isaac Utils")

        self._create_window()

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
            self._window.deferred_dock_in("Console", omni.ui.DockPolicy.DO_NOTHING)
            with self._window.frame:
                with ui.VStack():
                    with ui.HStack(height=20):
                        self._models["add_button"] = ui.Button(
                            "Add A Model", width=0, clicked_fn=lambda b=None: self._settings_ui._on_add_model_fn()
                        )
                        self._models["login_button"] = ui.Button(
                            "Create Shapenet Database Index File",
                            width=0,
                            clicked_fn=lambda b=None: self._hide_login_show_add(),
                        )
                        ui.Spacer()
                        self._models["help_button"] = ui.Button(
                            "Help", width=0, clicked_fn=lambda b=None: self._on_help_menu_click()
                        )
                    with ui.HStack(height=20):
                        self._create_settings_ui()
            if pickle_file_exists():
                self._models["login_button"].visible = False
                self._models["add_button"].visible = True
            else:
                self._models["login_button"].visible = True
                self._models["add_button"].visible = False

        self._window.visible = True

    def _create_settings_ui(self):
        self._settings_ui = ShapenetSettings()

    def _hide_login_show_add(self):
        if pickle_file_exists():
            self._models["login_button"].visible = False
            self._models["add_button"].visible = True
        else:
            self._login_ui = ShapenetLogin(self)
            self._models["login_button"].visible = True
            self._models["add_button"].visible = False

    def _on_help_menu_click(self):
        help_message = ""
        if not pickle_file_exists():
            help_message = ADD_DB_TEXT
        help_message = help_message + HELP_TEXT

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
        self._login_ui = None
