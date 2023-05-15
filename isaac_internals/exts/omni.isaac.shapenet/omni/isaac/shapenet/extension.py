# Copyright (c) 2018-2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

""" This plugin is used to load shapenet objects into Kit.

        If the shape already exists as a USD on a connected omniverse server, then
    it will use that version, unless there is an override.
        If not on omniverse, the plugin will convert the obj from a folder on the
    machine and upoad it to omniverse if there is a connection.
"""
import omni.ext
import omni.kit

from .globals import DEBUG_PRINT_ON
from .menu import ShapenetMenu

EXTENSION_NAME = "ShapeNet Loader"


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        if DEBUG_PRINT_ON:
            print("\nI STARTED I STARTED!\n")
        self._menu = ShapenetMenu(ext_id)

        if DEBUG_PRINT_ON:
            print("\nafter ShapenetMenu\n")

    def on_shutdown(self):

        self._menu.shutdown()
        self._menu = None

    def get_name(self):
        """Return the name of the extension"""
        return EXTENSION_NAME
