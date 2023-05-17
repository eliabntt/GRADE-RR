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
import omni.kit.test
from pxr import Gf, UsdGeom
from omni.kit.viewport.utility import get_active_viewport, next_viewport_frame_async, create_viewport_window

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.syntheticdata as syn

from ..utils import add_semantics


FILE_DIR = os.path.dirname(os.path.realpath(__file__))
TIMEOUT = 200

cameras = ["/World/Cameras/CameraFisheyeLeft", "/World/Cameras/CameraPinhole", "/World/Cameras/CameraFisheyeRight"]

# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
# This test has to run last and thus it's prefixed as such to force that:
#  - This is because it has to create additional viewports which makes the test
#    get stuck if it's not the last one in the OV process session
class ZZHasToRunLast_TestCrossCorrespondence(omni.kit.test.AsyncTestCase):
    def __init__(self, methodName: str) -> None:
        super().__init__(methodName=methodName)
        self.golden_image_path = Path(os.path.dirname(os.path.abspath(__file__))) / ".." / "data" / "golden"
        self.output_image_path = Path(os.path.dirname(os.path.abspath(__file__))) / ".." / "data" / "output"
        self.StdDevTolerance = 0.1
        self.sensorViewport = None

    # Before running each test
    async def setUp(self):
        global cameras
        np.random.seed(1234)

        # Load the scene
        scenePath = os.path.join(FILE_DIR, "../data/scenes/cross_correspondence.usda")
        await omni.usd.get_context().open_stage_async(scenePath)
        await omni.kit.app.get_app().next_update_async()

        # Get the main-viewport as the sensor-viewport
        self.sensorViewport = get_active_viewport()
        await next_viewport_frame_async(self.sensorViewport)

        # Setup viewports
        resolution = self.sensorViewport.resolution
        viewport_windows = [None] * 2
        x_pos, y_pos = 12, 75
        for i in range(len(viewport_windows)):
            viewport_windows[i] = create_viewport_window(width=resolution[0], height=resolution[1], position_x=x_pos, position_y=y_pos)
            viewport_windows[i].width = 500
            viewport_windows[i].height = 500
            x_pos += 500

        # Setup cameras
        self.sensorViewport.camera_path = cameras[0]
        for i in range(len(viewport_windows)):
            viewport_windows[i].viewport_api.camera_path = cameras[i + 1]

        # Use default viewport for sensor target as otherwise sensor enablement doesn't work
        # also the test will get stuck

        # Initialize Sensor
        await syn.sensors.create_or_retrieve_sensor_async(
            self.sensorViewport, syn._syntheticdata.SensorType.CrossCorrespondence
        )

    async def test_golden_image(self):
        # Render one frame
        await syn.sensors.next_sensor_data_async(self.sensorViewport,True)
        data = syn.sensors.get_cross_correspondence(self.sensorViewport)
        golden_image = np.load(self.golden_image_path / "cross_correspondence.npz")["array"]

        # normalize xy (uv offset) to zw channels' value range
        # x100 seems like a good number to bring uv offset to ~1
        data[:, [0, 1]] *= 100
        golden_image[:, [0, 1]] *= 100

        std_dev = np.sqrt(np.square(data - golden_image).astype(float).mean())
        
        if std_dev >= self.StdDevTolerance:
            if not os.path.isdir(self.output_image_path):
                os.mkdir(self.output_image_path)
            np.savez_compressed(self.output_image_path / "cross_correspondence.npz", array=data)
            golden_image = ((golden_image + 1.0) / 2) * 255
            data = ((data + 1.0) / 2) * 255
            Image.fromarray(golden_image.astype(np.uint8), "RGBA").save(
                self.output_image_path / "cross_correspondence_golden.png"
            )
            Image.fromarray(data.astype(np.uint8), "RGBA").save(self.output_image_path / "cross_correspondence.png")

        self.assertTrue(std_dev < self.StdDevTolerance)

    # After running each test
    async def tearDown(self):
        pass
