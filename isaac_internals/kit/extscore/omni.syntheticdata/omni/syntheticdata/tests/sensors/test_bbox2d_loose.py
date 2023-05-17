# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import os
import math
import asyncio
from time import time
import unittest


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
class TestBBox2DLoose(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        np.random.seed(1234)
        # Setup viewport
        self.viewport = get_active_viewport()

        # Initialize Sensor
        await omni.usd.get_context().new_stage_async()
        stage = omni.usd.get_context().get_stage()
        await omni.kit.app.get_app().next_update_async()
        await syn.sensors.create_or_retrieve_sensor_async(
            self.viewport, syn._syntheticdata.SensorType.BoundingBox2DLoose
        )

    async def test_parsed_empty(self):
        """ Test 2D bounding box on empty stage.
        """
        bbox2d_data = syn.sensors.get_bounding_box_2d_loose(self.viewport)
        assert not bool(bbox2d_data)

    async def test_bbox_2d_loose_fields_exist(self):
        stage = omni.usd.get_context().get_stage()
        cube = stage.DefinePrim("/Cube", "Cube")
        add_semantics(cube, "cube")
    
        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        bbox2d_data = syn.sensors.get_bounding_box_2d_loose(self.viewport)
    
        valid_dtype = [
            ("uniqueId", "<i4"),
            ("name", "O"),
            ("semanticLabel", "O"),
            ("metadata", "O"),
            ("instanceIds", "O"),
            ("semanticId", "<u4"),
            ("x_min", "<i4"),
            ("y_min", "<i4"),
            ("x_max", "<i4"),
            ("y_max", "<i4"),
        ]
        assert bbox2d_data.dtype == np.dtype(valid_dtype)

    async def test_bbox_2d_loose_cube(self):
        """ Basic test for the sensor.
        """
        stage = omni.usd.get_context().get_stage()

        camera = stage.DefinePrim("/Camera", "Camera")
        UsdGeom.Xformable(camera).AddTranslateOp().Set((0, 0, 0))
        self.viewport.camera_path = camera.GetPath()
        await omni.kit.app.get_app().next_update_async()

        cube = stage.DefinePrim("/Cube", "Cube")
        add_semantics(cube, "cube")

        UsdGeom.XformCommonAPI(cube).SetTranslate((0, 0, -10))
        
        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        bbox2d_data = syn.sensors.get_bounding_box_2d_loose(self.viewport)

        assert bbox2d_data['x_min'] == 301
        assert bbox2d_data['y_min'] == 21
        assert bbox2d_data['x_max'] == 978
        assert bbox2d_data['y_max'] == 698

    async def test_cube_pathtracing(self):
        """ Basic funtionality test of the sensor, but in path tracing mode.
        """
        settings = carb.settings.get_settings()
        settings.set_string("/rtx/rendermode", "PathTracing")
        settings.set_int("/rtx/pathtracing/spp", 32)
        settings.set_int("/persistent/app/viewport/displayOptions", 0)

        stage = omni.usd.get_context().get_stage()

        camera = stage.DefinePrim("/Camera", "Camera")
        UsdGeom.Xformable(camera).AddTranslateOp().Set((0, 0, 0))
        self.viewport.camera_path = camera.GetPath()
        await omni.kit.app.get_app().next_update_async()

        cube = stage.DefinePrim("/Cube", "Cube")
        add_semantics(cube, "cube")

        UsdGeom.XformCommonAPI(cube).SetTranslate((0, 0, -10))
        
        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        bbox2d_data = syn.sensors.get_bounding_box_2d_loose(self.viewport)

        assert bbox2d_data['x_min'] == 301
        assert bbox2d_data['y_min'] == 21
        assert bbox2d_data['x_max'] == 978
        assert bbox2d_data['y_max'] == 698
    
    async def test_cube_ray_traced_lighting(self):
        """ Basic test for the sensor, but in ray traced lighting mode.
        """
        # Set the rendering mode to be ray traced lighting.
        settings_interface = carb.settings.get_settings()
        settings_interface.set_string("/rtx/rendermode", "RayTracedLighting")

        stage = omni.usd.get_context().get_stage()

        camera = stage.DefinePrim("/Camera", "Camera")
        UsdGeom.Xformable(camera).AddTranslateOp().Set((0, 0, 0))
        self.viewport.camera_path = camera.GetPath()
        await omni.kit.app.get_app().next_update_async()

        cube = stage.DefinePrim("/Cube", "Cube")
        add_semantics(cube, "cube")

        UsdGeom.XformCommonAPI(cube).SetTranslate((0, 0, -10))
        
        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        bbox2d_data = syn.sensors.get_bounding_box_2d_loose(self.viewport)

        assert bbox2d_data['x_min'] == 301
        assert bbox2d_data['y_min'] == 21
        assert bbox2d_data['x_max'] == 978
        assert bbox2d_data['y_max'] == 698
    
    async def test_cube_ftheta(self):
        """ Basic funtionality test of the sensor in ftheta camera.
        """
        settings = carb.settings.get_settings()
        settings.set_string("/rtx/rendermode", "PathTracing")
        settings.set_int("/rtx/pathtracing/spp", 32)
        settings.set_int("/persistent/app/viewport/displayOptions", 0)

        stage = omni.usd.get_context().get_stage()

        camera = stage.DefinePrim("/Camera", "Camera")
        # Set the camera to be polynomial fish eye camera.
        camera.CreateAttribute("cameraProjectionType", Sdf.ValueTypeNames.Token).Set("fisheyePolynomial")

        UsdGeom.Xformable(camera).AddTranslateOp().Set((0, 0, 0))
        self.viewport.camera_path = camera.GetPath()
        await omni.kit.app.get_app().next_update_async()

        cube = stage.DefinePrim("/Cube", "Cube")
        add_semantics(cube, "cube")

        UsdGeom.XformCommonAPI(cube).SetTranslate((0, 0, -10))
        
        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        bbox2d_data = syn.sensors.get_bounding_box_2d_loose(self.viewport)

        assert bbox2d_data['x_min'] == 612
        assert bbox2d_data['y_min'] == 325
        assert bbox2d_data['x_max'] == 671
        assert bbox2d_data['y_max'] == 384

    async def test_cube_spherical(self):
        """ Basic funtionality test of the sensor in fisheye spherical camera.
        """
        settings = carb.settings.get_settings()
        settings.set_string("/rtx/rendermode", "PathTracing")
        settings.set_int("/rtx/pathtracing/spp", 32)
        settings.set_int("/persistent/app/viewport/displayOptions", 0)

        stage = omni.usd.get_context().get_stage()

        camera = stage.DefinePrim("/Camera", "Camera")
        # Set the camera to be polynomial fish eye camera.
        camera.CreateAttribute("cameraProjectionType", Sdf.ValueTypeNames.Token).Set("fisheyeSpherical")

        UsdGeom.Xformable(camera).AddTranslateOp().Set((0, 0, 0))
        self.viewport.camera_path = camera.GetPath()
        await omni.kit.app.get_app().next_update_async()

        cube = stage.DefinePrim("/Cube", "Cube")
        add_semantics(cube, "cube")

        UsdGeom.XformCommonAPI(cube).SetTranslate((0, 0, -10))
        
        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        bbox2d_data = syn.sensors.get_bounding_box_2d_loose(self.viewport)
        
        assert bbox2d_data['x_min'] == 617
        assert bbox2d_data['y_min'] == 335
        assert bbox2d_data['x_max'] == 662
        assert bbox2d_data['y_max'] == 384

    # After running each test
    async def tearDown(self):
        pass
