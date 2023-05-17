# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import os
import math
import asyncio
from PIL import Image
from time import time
from pathlib import Path


import carb
import numpy as np
from numpy.lib.arraysetops import unique
import unittest
import omni.kit.test
from omni.kit.viewport.utility import get_active_viewport
from pxr import Gf, UsdGeom

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.syntheticdata as syn

from ..utils import add_semantics


FILE_DIR = os.path.dirname(os.path.realpath(__file__))
TIMEOUT = 200


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestMotionVector(omni.kit.test.AsyncTestCase):
    def __init__(self, methodName: str) -> None:
        super().__init__(methodName=methodName)
        self.golden_image_path = Path(os.path.dirname(os.path.abspath(__file__))) / ".." / "data" / "golden"
        self.output_image_path = Path(os.path.dirname(os.path.abspath(__file__))) / ".." / "data" / "output"

    def writeDataToImage(self, data, name):
        if not os.path.isdir(self.output_image_path):
            os.mkdir(self.output_image_path)
        data = ((data + 1.0) / 2) * 255
        outputPath = str(self.output_image_path) + "/" + name + ".png"
        print("Writing data to " + outputPath)
        Image.fromarray(data.astype(np.uint8), "RGBA").save(outputPath)

    # Before running each test
    async def setUp(self):
        np.random.seed(1234)
        # Setup viewport
        self.viewport = get_active_viewport()

        # Initialize Sensor
        await omni.usd.get_context().new_stage_async()
        stage = omni.usd.get_context().get_stage()
        await omni.kit.app.get_app().next_update_async()
        await syn.sensors.create_or_retrieve_sensor_async(self.viewport, syn._syntheticdata.SensorType.MotionVector)

    async def test_empty(self):
        """ Test motion vector sensor on empty stage.
        """
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_motion_vector(self.viewport)
        allChannelsAreZero = np.allclose(data, 0, atol=0.001)
        if not allChannelsAreZero:
            self.writeDataToImage(data, "test_empty")

        assert allChannelsAreZero

    async def test_dtype(self):
        stage = omni.usd.get_context().get_stage()
        cube = stage.DefinePrim("/Cube", "Cube")
        add_semantics(cube, "cube")
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_motion_vector(self.viewport)
        assert data.dtype == np.float32

    async def test_unmoving_cube(self):
        stage = omni.usd.get_context().get_stage()
        cube = stage.DefinePrim("/Cube", "Cube")
        add_semantics(cube, "cube")
        cube.GetAttribute("size").Set(100)
        cube.GetAttribute("primvars:displayColor").Set([(0, 0, 1)])
        UsdGeom.Xformable(cube).AddTranslateOp()
        cube.GetAttribute("xformOp:translate").Set((350, 365, 350), time=0)
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_motion_vector(self.viewport)
        # 4th channel will wary based on geo hit, so we ignore checking it here
        rgbChannelsAreZero = np.allclose(data[:, [0, 1, 2]], 0, atol=0.001)
        if not rgbChannelsAreZero:
            self.writeDataToImage(data, "test_unmoving_cube")

        assert rgbChannelsAreZero

    @unittest.skip("OM-44310")
    async def test_partially_disoccluding_cube(self):
        # disabling temporarly the test for OMNI-GRAPH support : OM-44310
        
        stage = omni.usd.get_context().get_stage()
        stage.SetStartTimeCode(0)
        stage.SetEndTimeCode(100)
        cube = stage.DefinePrim("/Cube", "Cube")
        add_semantics(cube, "cube")
        cube.GetAttribute("size").Set(10)
        cube.GetAttribute("primvars:displayColor").Set([(0, 0, 1)])

        # add translation down to create disocclusion due to fetching from out of screen bounds
        UsdGeom.Xformable(cube).AddTranslateOp()
        cube.GetAttribute("xformOp:translate").Set((480, 487, 480), time=0)
        cube.GetAttribute("xformOp:translate").Set((480, 480, 480), time=0.001)

        # add rotation around up vector to create disocclusion due to fetching from an incompatible surface
        UsdGeom.Xformable(cube).AddRotateYOp()
        cube.GetAttribute("xformOp:rotateY").Set(40, time=0)
        cube.GetAttribute("xformOp:rotateY").Set(70, time=0.001)
        await omni.kit.app.get_app().next_update_async()
        # Render one frame
        itl = omni.timeline.get_timeline_interface()
        itl.play()
        await syn.sensors.next_sensor_data_async(self.viewport, True)
        data = syn.sensors.get_motion_vector(self.viewport)

        golden_image = np.load(self.golden_image_path / "motion_partially_disoccluding_cube.npz")["array"]

        # normalize xy (mvec) to zw channels' value range
        # x100 seems like a good number to bring mvecs to ~1
        data[:, [0, 1]] *= 100
        golden_image[:, [0, 1]] *= 100

        std_dev = np.sqrt(np.square(data - golden_image).astype(float).mean())

        # OM-41605 - using higher std dev here to make linux run succeed
        std_dev_tolerance = 0.12
        print("Calculated std.dev: " + str(std_dev), " Std dev tolerance: " + str(std_dev_tolerance))

        if std_dev >= std_dev_tolerance:
            self.writeDataToImage(golden_image, "test_partially_disoccluding_cube_golden")
            self.writeDataToImage(data, "test_partially_disoccluding_cube")
            np.savez_compressed(self.output_image_path / "motion_partially_disoccluding_cube.npz", array=data)
        assert std_dev < std_dev_tolerance

    # After running each test
    async def tearDown(self):
        pass
