# Copyright (c) 2018-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import carb
from omni.kit.widget.settings import create_setting_widget, SettingType
import omni.ui as ui
from pxr import Gf
from .globals import *
from .shape import addShapePrim
from .globals import g_default_omni_server


class ShapenetSettings:
    def __init__(self):
        self._settings = carb.settings.get_settings()

        # note: Not sure this is the best place to set default values.
        self._settings.set_default_string("/isaac/shapenet/omniverseServer", g_default_omni_server)
        self._settings.set_default_string("/isaac/shapenet/synsetId", "random")
        self._settings.set_default_string("/isaac/shapenet/modelId", "random")
        self._settings.set_float_array("/isaac/shapenet/pos", [0.0, 0.0, 0.0])
        self._settings.set_float_array("/isaac/shapenet/rotaxis", [0.0, 0.0, 0.0])
        self._settings.set_default_float("/isaac/shapenet/rotangle", 0.0)
        self._settings.set_default_float("/isaac/shapenet/scale", 1.0)
        self._settings.set_default_bool("/isaac/shapenet/auto_add_physics", False)
        self._settings.set_default_bool("/isaac/shapenet/use_convex_decomp", False)
        self._build_ui()

    def _build_ui(self):
        """ Add Shape Settings """
        with ui.CollapsableFrame(title="Add Model Parameters"):
            with ui.VStack(spacing=2):
                with ui.HStack(height=20):
                    ui.Label("Omniverse Server", word_wrap=True, width=ui.Percent(35))
                    create_setting_widget("/isaac/shapenet/omniverseServer", SettingType.STRING)
                with ui.HStack(height=20):
                    ui.Label("synsetId and modelId", word_wrap=True, width=ui.Percent(35))
                    create_setting_widget("/isaac/shapenet/synsetId", SettingType.STRING)
                    ui.Spacer()
                    create_setting_widget("/isaac/shapenet/modelId", SettingType.STRING)
                with ui.HStack(height=20):
                    ui.Label("X Y Z Position", word_wrap=True, width=ui.Percent(35))
                    create_setting_widget("/isaac/shapenet/pos", SettingType.DOUBLE3)
                with ui.HStack(height=20):
                    ui.Label("X Y Z Axis Angle", word_wrap=True, width=ui.Percent(35))
                    create_setting_widget("/isaac/shapenet/rotaxis", SettingType.DOUBLE3)
                    ui.Spacer()
                    create_setting_widget("/isaac/shapenet/rotangle", SettingType.FLOAT)
                with ui.HStack(height=20):
                    ui.Label("Scale of add", word_wrap=True, width=ui.Percent(35))
                    create_setting_widget("/isaac/shapenet/scale", SettingType.FLOAT)
                with ui.HStack(height=20):
                    ui.Label("Automatically add physics", word_wrap=True, width=ui.Percent(35))
                    create_setting_widget("/isaac/shapenet/auto_add_physics", SettingType.BOOL)
                with ui.HStack(height=20):
                    ui.Label("Use convex decomponsition", word_wrap=True, width=ui.Percent(35))
                    create_setting_widget("/isaac/shapenet/use_convex_decomp", SettingType.BOOL)

    def _on_add_model_fn(self):
        pos = self.getPos()
        rot = self.getRot()
        scale = self.getScale()

        global g_shapenet_db
        g_shapenet_db = get_database()
        if g_shapenet_db == None:
            print(
                "Please create an Shapenet ID by logging into shapenet.org with the UI, or by downloading it manually and setting the SHAPENET_LOCAL_DIR environment variable."
            )
            return

        synsetId = self.getSynsetId()

        modelId = self.getModelId()

        return addShapePrim(
            self._settings.get("/isaac/shapenet/omniverseServer"),
            synsetId,
            modelId,
            pos,
            rot,
            scale,
            self._settings.get("/isaac/shapenet/auto_add_physics"),
            self._settings.get("/isaac/shapenet/use_convex_decomp"),
        )

    def getPos(self):
        pos = self._settings.get("/isaac/shapenet/pos")
        return Gf.Vec3d(pos[0], pos[1], pos[2])

    def getRot(self):
        axis = self._settings.get("/isaac/shapenet/rotaxis")
        a = self._settings.get("/isaac/shapenet/rotangle")
        return Gf.Rotation(Gf.Vec3d(axis[0], axis[1], axis[2]), a)

    def getScale(self):
        s = self._settings.get("/isaac/shapenet/scale")
        return s

    def getSynsetId(self):
        s = self._settings.get("/isaac/shapenet/synsetId")
        return s

    def getModelId(self):
        s = self._settings.get("/isaac/shapenet/modelId")
        return s
