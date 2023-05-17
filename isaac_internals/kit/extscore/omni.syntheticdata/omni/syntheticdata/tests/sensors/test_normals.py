# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import os
import math
import asyncio
from time import time
from pathlib import Path


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
class TestNormals(omni.kit.test.AsyncTestCase):
    def __init__(self, methodName: str) -> None:
        super().__init__(methodName=methodName)
        self.golden_image_path = Path(os.path.dirname(os.path.abspath(__file__))) / ".." / "data" / "golden"

    # Before running each test
    async def setUp(self):
        np.random.seed(1234)
        # Setup viewport
        self.viewport = get_active_viewport()

        # Initialize Sensor
        await omni.usd.get_context().new_stage_async()
        stage = omni.usd.get_context().get_stage()
        await omni.kit.app.get_app().next_update_async()
        await syn.sensors.create_or_retrieve_sensor_async(self.viewport, syn._syntheticdata.SensorType.Normal)

    async def test_parsed_empty(self):
        """ Test normals sensor on empty stage.
        """
        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_normals(self.viewport)
        assert np.allclose(data, 0, 1e-3)

    async def test_parsed_dtype(self):
        stage = omni.usd.get_context().get_stage()
        cube = stage.DefinePrim("/Cube", "Cube")
        add_semantics(cube, "cube")
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_normals(self.viewport)
        assert data.dtype == np.float32

    async def test_neg_z(self):
        """ Test that negative z faces are distinct from background
        """
        stage = omni.usd.get_context().get_stage()
        camera = stage.DefinePrim("/Camera", "Camera")
        UsdGeom.Xformable(camera).AddRotateYOp().Set(180)
        UsdGeom.Xformable(camera).AddTranslateOp().Set((0.0, 0.0, 20.0))
        self.viewport.camera_path = camera.GetPath()
        await omni.kit.app.get_app().next_update_async()
        cube = stage.DefinePrim("/Cube", "Cube")
        add_semantics(cube, "cube")
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_normals(self.viewport)
        assert len(np.unique(data)) == 2

    async def test_rotated_cube(self):
        stage = omni.usd.get_context().get_stage()

        cube = stage.DefinePrim("/Cube", "Cube")
        add_semantics(cube, "cube")
        cube.GetAttribute("size").Set(100)
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_normals(self.viewport)

        # np.savez_compressed(self.golden_image_path / 'normals_cube.npz', array=data)
        golden_image = np.load(self.golden_image_path / "normals_cube.npz")["array"]
        std_dev = np.sqrt(np.square(data - golden_image).astype(float).mean())
        assert std_dev < 2

    async def test_rotated_cube_pathtracing(self):
        """ Basic funtionality test of the sensor, but in path tracing mode.
        """
        settings = carb.settings.get_settings()
        settings.set_string("/rtx/rendermode", "PathTracing")
        settings.set_int("/rtx/pathtracing/spp", 32)
        settings.set_int("/persistent/app/viewport/displayOptions", 0)

        stage = omni.usd.get_context().get_stage()

        cube = stage.DefinePrim("/Cube", "Cube")
        add_semantics(cube, "cube")
        cube.GetAttribute("size").Set(100)
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_normals(self.viewport)

        # np.savez_compressed(self.golden_image_path / 'normals_cube.npz', array=data)
        golden_image = np.load(self.golden_image_path / "normals_cube.npz")["array"]
        std_dev = np.sqrt(np.square(data - golden_image).astype(float).mean())
        assert std_dev < 2

    async def test_rotated_cube_ray_traced_lighting(self):
        """ Basic funtionality test of the sensor, but in ray traced lighting.
        """
        # Set the rendering mode to be ray traced lighting.
        settings_interface = carb.settings.get_settings()
        settings_interface.set_string("/rtx/rendermode", "RayTracedLighting")

        stage = omni.usd.get_context().get_stage()

        cube = stage.DefinePrim("/Cube", "Cube")
        add_semantics(cube, "cube")
        cube.GetAttribute("size").Set(100)
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_normals(self.viewport)

        # np.savez_compressed(self.golden_image_path / 'normals_cube.npz', array=data)
        golden_image = np.load(self.golden_image_path / "normals_cube.npz")["array"]
        std_dev = np.sqrt(np.square(data - golden_image).astype(float).mean())
        assert std_dev < 2

    async def test_rotated_cube_ftheta(self):
        """ Basic funtionality test of the sensor in f theta camera.
        """
        # Set the mode to path traced for f theta camera.
        settings = carb.settings.get_settings()
        settings.set_string("/rtx/rendermode", "PathTracing")
        settings.set_int("/rtx/pathtracing/spp", 32)
        settings.set_int("/persistent/app/viewport/displayOptions", 0)

        stage = omni.usd.get_context().get_stage()

        cube = stage.DefinePrim("/Cube", "Cube")
        add_semantics(cube, "cube")
        cube.GetAttribute("size").Set(100)
        await omni.kit.app.get_app().next_update_async()

        # Setting up camera.
        camera = stage.DefinePrim("/Camera", "Camera")

        # Set the camera to be polynomial fish eye camera.
        camera.CreateAttribute("cameraProjectionType", Sdf.ValueTypeNames.Token).Set("fisheyePolynomial")

        # Set the Camera's position
        UsdGeom.Xformable(camera).AddTranslateOp().Set((200, 200, 200))
        self.viewport.camera_path = camera.GetPath()
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_normals(self.viewport)

        # np.savez_compressed(self.golden_image_path / 'normals_cube_ftheta.npz', array=data)

        golden_image = np.load(self.golden_image_path / "normals_cube_ftheta.npz")["array"]
        std_dev = np.sqrt(np.square(data - golden_image).astype(float).mean())
        assert std_dev < 2

    async def test_rotated_cube_spherical(self):
        """ Basic funtionality test of the sensor in fisheye spherical camera.
        """
        # Set the mode to path traced.
        settings = carb.settings.get_settings()
        settings.set_string("/rtx/rendermode", "PathTracing")
        settings.set_int("/rtx/pathtracing/spp", 32)
        settings.set_int("/persistent/app/viewport/displayOptions", 0)

        stage = omni.usd.get_context().get_stage()

        cube = stage.DefinePrim("/Cube", "Cube")
        add_semantics(cube, "cube")
        cube.GetAttribute("size").Set(100)

        # Setting up camera.
        camera = stage.DefinePrim("/Camera", "Camera")

        # Set the camera to be polynomial fish eye camera.
        camera.CreateAttribute("cameraProjectionType", Sdf.ValueTypeNames.Token).Set("fisheyeSpherical")

        # Set the Camera's position
        UsdGeom.Xformable(camera).AddTranslateOp().Set((200, 200, 200))
        self.viewport.camera_path = camera.GetPath()
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_normals(self.viewport)

        # np.savez_compressed(self.golden_image_path / 'normals_cube_spherical.npz', array=data)

        golden_image = np.load(self.golden_image_path / "normals_cube_spherical.npz")["array"]
        std_dev = np.sqrt(np.square(data - golden_image).astype(float).mean())
        assert std_dev < 2

    # After running each test
    async def tearDown(self):
        pass
