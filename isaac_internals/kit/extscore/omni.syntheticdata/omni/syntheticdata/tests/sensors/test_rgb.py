# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import os
import math
import asyncio
from time import time
from pathlib import Path
import unittest
from PIL import Image

import carb
import numpy as np
from numpy.lib.arraysetops import unique
import omni.kit.test
from omni.kit.viewport.utility import get_active_viewport
from pxr import Gf, UsdGeom, Sdf, UsdLux


# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.syntheticdata as syn

from ..utils import add_semantics


FILE_DIR = os.path.dirname(os.path.realpath(__file__))
TIMEOUT = 200


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestRGB(omni.kit.test.AsyncTestCase):
    def __init__(self, methodName: str) -> None:
        super().__init__(methodName=methodName)
        self.golden_image_path = Path(os.path.dirname(os.path.abspath(__file__))) / ".." / "data" / "golden"

    # Before running each test
    async def setUp(self):
        np.random.seed(1234)

        settings = carb.settings.get_settings()
        settings.set_string("/rtx/rendermode", "PathTracing")
        settings.set_int("/rtx/pathtracing/spp", 32)
        settings.set_int("/persistent/app/viewport/displayOptions", 0)

        # Setup viewport
        self.viewport = get_active_viewport()

        # Initialize Sensor
        await omni.usd.get_context().new_stage_async()
        stage = omni.usd.get_context().get_stage()
        await omni.kit.app.get_app().next_update_async()
        await syn.sensors.create_or_retrieve_sensor_async(self.viewport, syn._syntheticdata.SensorType.Rgb)

    async def test_empty(self):
        """ Test RGB sensor on empty stage.
        """
        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_rgb(self.viewport)
        std_dev = np.sqrt(np.square(data - np.zeros_like(data)).astype(float).mean())
        assert std_dev < 2

    async def test_cube(self):
        """ Test RGB sensor on stage with cube.
        """
        stage = omni.usd.get_context().get_stage()
        cube = stage.DefinePrim("/Cube", "Cube")
        add_semantics(cube, "cube")
        cube.GetAttribute("size").Set(100)
        cube.GetAttribute("primvars:displayColor").Set([(0, 0, 1)])
        await omni.kit.app.get_app().next_update_async()

        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_rgb(self.viewport)
        golden_image = np.asarray(Image.open(str(self.golden_image_path / "rgb_cube.png")))
        std_dev = np.sqrt(np.square(data - golden_image).astype(float).mean())
        assert std_dev < 2

    async def test_dtype(self):
        stage = omni.usd.get_context().get_stage()
        cube = stage.DefinePrim("/Cube", "Cube")
        add_semantics(cube, "cube")
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_rgb(self.viewport)
        assert data.dtype == np.uint8

    @unittest.skip("OM-44741")
    async def test_cube_polynomial(self):
        """ Test RGB sensor on stage with cube.
        """
        settings = carb.settings.get_settings()
        settings.set_string("/rtx/rendermode", "PathTracing")
        settings.set_int("/rtx/pathtracing/spp", 32)
        settings.set_int("/persistent/app/viewport/displayOptions", 0)

        stage = omni.usd.get_context().get_stage()
        cube = stage.DefinePrim("/Cube", "Cube")
        add_semantics(cube, "cube")
        cube.GetAttribute("size").Set(100)
        cube.GetAttribute("primvars:displayColor").Set([(0, 0, 1)])
        await omni.kit.app.get_app().next_update_async()

        # TODO: Add a light
        camera = stage.DefinePrim("/Camera", "Camera")

        # Set the camera to be spherical camera
        camera.CreateAttribute("cameraProjectionType", Sdf.ValueTypeNames.Token).Set("fisheyePolynomial")
        
        UsdGeom.Xformable(camera).AddTranslateOp().Set((0, 0, 200))
        self.viewport.camera_path = camera.GetPath()

        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_rgb(self.viewport)

        # image = Image.fromarray(data)
        # image.save(str(self.golden_image_path / "rgb_cube_ftheta.png"))

        golden_image = np.asarray(Image.open(str(self.golden_image_path / "rgb_cube_ftheta.png")))
        std_dev = np.sqrt(np.square(data - golden_image).astype(float).mean())
        assert std_dev < 2

    @unittest.skip("OM-44741")
    async def test_cube_spherical(self):
        """ Test RGB sensor on stage with cube.
        """
        settings = carb.settings.get_settings()
        settings.set_string("/rtx/rendermode", "PathTracing")
        settings.set_int("/rtx/pathtracing/spp", 32)
        settings.set_int("/persistent/app/viewport/displayOptions", 0)

        stage = omni.usd.get_context().get_stage()
        cube = stage.DefinePrim("/Cube", "Cube")
        add_semantics(cube, "cube")
        cube.GetAttribute("size").Set(100)
        cube.GetAttribute("primvars:displayColor").Set([(0, 0, 1)])
        await omni.kit.app.get_app().next_update_async()

        # TODO: Add a light
        camera = stage.DefinePrim("/Camera", "Camera")

        # Set the camera to be spherical camera
        camera.CreateAttribute("cameraProjectionType", Sdf.ValueTypeNames.Token).Set("fisheyeSpherical")
        
        UsdGeom.Xformable(camera).AddTranslateOp().Set((0, 0, 200))
        self.viewport.camera_path = camera.GetPath()

        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_rgb(self.viewport)

        # image = Image.fromarray(data)
        # image.save(str(self.golden_image_path / "rgb_cube_spherical.png"))

        golden_image = np.asarray(Image.open(str(self.golden_image_path / "rgb_cube_spherical.png")))
        std_dev = np.sqrt(np.square(data - golden_image).astype(float).mean())
        assert std_dev < 2

    # After running each test
    async def tearDown(self):
        pass
