# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import os
import math
import asyncio
from time import time


import carb
import numpy as np
import omni.kit.test
from omni.kit.viewport.utility import get_active_viewport
from pxr import Gf, UsdGeom, Sdf

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.syntheticdata as syn

from ..utils import add_semantics


FILE_DIR = os.path.dirname(os.path.realpath(__file__))
TIMEOUT = 200


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestDepth(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        np.random.seed(1234)
        # Setup viewport
        self.viewport = get_active_viewport()

        # Initialize Sensor
        await omni.usd.get_context().new_stage_async()
        stage = omni.usd.get_context().get_stage()
        await omni.kit.app.get_app().next_update_async()
        await syn.sensors.create_or_retrieve_sensor_async(self.viewport, syn._syntheticdata.SensorType.Depth)

    async def test_parsed_empty(self):
        """ Test depth sensor on empty stage.
        """
        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_depth(self.viewport)
        assert data.sum() == 0

    async def test_parsed_dtype(self):
        stage = omni.usd.get_context().get_stage()
        cube = stage.DefinePrim("/Cube", "Cube")
        add_semantics(cube, "cube")
        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_depth(self.viewport)
        assert data.dtype == np.float32

    async def test_distances(self):
        stage = omni.usd.get_context().get_stage()

        camera = stage.DefinePrim("/Camera", "Camera")
        UsdGeom.Xformable(camera).AddTranslateOp().Set((0, 0, 0))
        self.viewport.camera_path = camera.GetPath()
        await omni.kit.app.get_app().next_update_async()

        for n in range(10, 100, 10):
            cube = stage.DefinePrim("/Cube", "Cube")
            add_semantics(cube, "cube")
            # n = 5
            UsdGeom.XformCommonAPI(cube).SetTranslate((0, 0, -n))
            
            # Render one frame
            await syn.sensors.next_sensor_data_async(self.viewport,True)
            data = syn.sensors.get_depth(self.viewport)
            assert np.isclose(data.min(), 0, atol=1e-5)
            # The front of the cube is 1 ahead of its center position
            assert np.isclose(data.max(), 1 / (n - 1), atol=1e-5)

    async def test_distances_pathtracing(self):
        """ Basic funtionality test of the sensor, but in path tracing mode.
        """

        # Set the rendering mode to be pathtracing
        settings = carb.settings.get_settings()
        settings.set_string("/rtx/rendermode", "PathTracing")
        settings.set_int("/rtx/pathtracing/spp", 32)
        settings.set_int("/persistent/app/viewport/displayOptions", 0)
        stage = omni.usd.get_context().get_stage()

        camera = stage.DefinePrim("/Camera", "Camera")
        UsdGeom.Xformable(camera).AddTranslateOp().Set((0, 0, 0))
        self.viewport.camera_path = camera.GetPath()
        await omni.kit.app.get_app().next_update_async()

        for n in range(10, 100, 10):
            cube = stage.DefinePrim("/Cube", "Cube")
            add_semantics(cube, "cube")
            # n = 5
            UsdGeom.XformCommonAPI(cube).SetTranslate((0, 0, -n))
            
            # Render one frame
            await syn.sensors.next_sensor_data_async(self.viewport,True)
            data = syn.sensors.get_depth(self.viewport)
            assert np.isclose(data.min(), 0, atol=1e-5)
            # The front of the cube is 1 ahead of its center position
            assert np.isclose(data.max(), 1 / (n - 1), atol=1e-5)
    
    async def test_distances_ray_traced_lighting(self):
        """ Basic funtionality test of the sensor, but in ray traced lighting.
        """

        # Set the rendering mode to be pathtracing
        settings_interface = carb.settings.get_settings()
        settings_interface.set_string("/rtx/rendermode", "RayTracedLighting")
        stage = omni.usd.get_context().get_stage()

        camera = stage.DefinePrim("/Camera", "Camera")
        UsdGeom.Xformable(camera).AddTranslateOp().Set((0, 0, 0))
        self.viewport.camera_path = camera.GetPath()
        await omni.kit.app.get_app().next_update_async()

        for n in range(10, 100, 10):
            cube = stage.DefinePrim("/Cube", "Cube")
            add_semantics(cube, "cube")
            # n = 5
            UsdGeom.XformCommonAPI(cube).SetTranslate((0, 0, -n))
            
            # Render one frame
            await syn.sensors.next_sensor_data_async(self.viewport,True)
            data = syn.sensors.get_depth(self.viewport)
            assert np.isclose(data.min(), 0, atol=1e-5)
            # The front of the cube is 1 ahead of its center position
            assert np.isclose(data.max(), 1 / (n - 1), atol=1e-5)
    
    async def test_ftheta_camera(self):
        """ Test the functionality of the sensor under f-theta camera.
        """
        settings = carb.settings.get_settings()
        settings.set_string("/rtx/rendermode", "PathTracing")
        settings.set_int("/rtx/pathtracing/spp", 32)
        settings.set_int("/persistent/app/viewport/displayOptions", 0)

        stage = omni.usd.get_context().get_stage()

        camera = stage.DefinePrim("/Camera", "Camera")

        # Set the camera to be polynomial fish eye camera.
        camera.CreateAttribute("cameraProjectionType", Sdf.ValueTypeNames.Token).Set("fisheyePolynomial")

        # Set the Camera's position
        UsdGeom.Xformable(camera).AddTranslateOp().Set((0, 0, 0))
        self.viewport.camera_path = camera.GetPath()
        await omni.kit.app.get_app().next_update_async()

        # Add a cube at the centre of the scene
        cube_prim = stage.DefinePrim("/Cube", "Cube")
        add_semantics(cube_prim, "cube")

        cube = UsdGeom.Cube(cube_prim)
        
        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_depth(self.viewport)
        await omni.kit.app.get_app().next_update_async()

        # Centre of the data should be half of the cube edge's length, adjusted to correct scale.
        edge_length = cube.GetSizeAttr().Get()

        assert np.isclose(1 / (edge_length - 1), data.max(), atol=1e-3)
        assert np.isclose(1 / (np.sqrt(((edge_length) ** 2)*2) - 1), data[data > 0].min(), atol=1e-1)

    # After running each test
    async def tearDown(self):
        pass
