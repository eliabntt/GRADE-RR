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
from pxr import Gf, UsdGeom
from omni.kit.viewport.utility import get_active_viewport

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.syntheticdata as syn
from ..utils import add_semantics


FILE_DIR = os.path.dirname(os.path.realpath(__file__))
TIMEOUT = 200


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestBBox2DTight(omni.kit.test.AsyncTestCase):
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
            self.viewport, syn._syntheticdata.SensorType.BoundingBox2DTight
        )

    async def test_parsed_empty(self):
        """ Test 2D bounding box on empty stage.
        """
        bbox2d_data = syn.sensors.get_bounding_box_2d_tight(self.viewport)
        assert not bool(bbox2d_data)

    async def test_fields_exist(self):
        stage = omni.usd.get_context().get_stage()
        cube = stage.DefinePrim("/Cube", "Cube")
        add_semantics(cube, "cube")

        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        bbox2d_data = syn.sensors.get_bounding_box_2d_tight(self.viewport)

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
    
    async def test_cube(self):
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
        bbox2d_data = syn.sensors.get_bounding_box_2d_tight(self.viewport)

        assert bbox2d_data[0]

        x_min, y_min, x_max, y_max = bbox2d_data[0][6], bbox2d_data[0][7], bbox2d_data[0][8], bbox2d_data[0][9]
        
        assert x_min == 301
        assert y_min == 21
        assert x_max == 978
        assert y_max == 698
    
    @unittest.skip("OM-46398")
    async def test_bbox_2d_tight_scene_instance(self):
        """ Test sensor on scene instance.
        """
        settings = carb.settings.get_settings()

        if settings.get("/rtx/hydra/enableSemanticSchema"):
            path = os.path.join(FILE_DIR, "../data/scenes/scene_instance_test.usda")

            await omni.usd.get_context().open_stage_async(path)
            
            # Render one frame
            await syn.sensors.next_sensor_data_async(self.viewport,True)
            
            data = syn.sensors.get_bounding_box_2d_tight(self.viewport)
            # should be 3 prims in the scene.
            # TODO: Add more complicated test
            assert len(data) == 3

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
        bbox2d_data = syn.sensors.get_bounding_box_2d_tight(self.viewport)

        x_min, y_min, x_max, y_max = bbox2d_data[0][6], bbox2d_data[0][7], bbox2d_data[0][8], bbox2d_data[0][9]
        assert x_min == 301
        assert y_min == 21
        assert x_max == 978
        assert y_max == 698
    
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
        bbox2d_data = syn.sensors.get_bounding_box_2d_tight(self.viewport)

        x_min, y_min, x_max, y_max = bbox2d_data[0][6], bbox2d_data[0][7], bbox2d_data[0][8], bbox2d_data[0][9]
        assert x_min == 301
        assert y_min == 21
        assert x_max == 978
        assert y_max == 698

    # After running each test
    async def tearDown(self):
        pass
