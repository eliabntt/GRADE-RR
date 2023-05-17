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

import unittest


FILE_DIR = os.path.dirname(os.path.realpath(__file__))
TIMEOUT = 200


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestSemanticSeg(omni.kit.test.AsyncTestCase):
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
        await syn.sensors.initialize_async(
            self.viewport, 
            [
                syn._syntheticdata.SensorType.SemanticSegmentation,
                syn._syntheticdata.SensorType.InstanceSegmentation
            ]
        )
        
    async def test_empty(self):
        """ Test semantic segmentation on empty stage.
        """
        data = syn.sensors.get_semantic_segmentation(self.viewport)
        assert data.sum() == 0

    async def test_dtype(self):
        stage = omni.usd.get_context().get_stage()
        cube = stage.DefinePrim("/Cube", "Cube")
        add_semantics(cube, "cube")

        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_semantic_segmentation(self.viewport)
        assert data.dtype == np.uint32

    async def test_cube(self):
        stage = omni.usd.get_context().get_stage()
        cube = stage.DefinePrim("/Cube", "Cube")
        add_semantics(cube, "cube")
        cube.GetAttribute("size").Set(100)
        
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_semantic_segmentation(self.viewport)

        # np.savez_compressed(self.golden_image_path / 'semantic_seg_cube.npz', array=data)
        golden_image = np.load(self.golden_image_path / "semantic_seg_cube.npz")["array"]
        std_dev = np.sqrt(np.square(data - golden_image).astype(float).mean())
        assert std_dev < 0.1
    
    async def test_cube_sphere(self):
        stage = omni.usd.get_context().get_stage()
        cube = stage.DefinePrim("/Cube", "Cube")
        add_semantics(cube, "cube")
        cube.GetAttribute("size").Set(100)

        sphere_prim = stage.DefinePrim("/Sphere", "Sphere")
        UsdGeom.XformCommonAPI(sphere_prim).SetTranslate((300, 0, 0))

        add_semantics(sphere_prim, "sphere")
        sphere = UsdGeom.Sphere(sphere_prim)
        sphere.GetRadiusAttr().Set(100)
        
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_instance_segmentation(self.viewport)
        
        # np.savez_compressed(self.golden_image_path / 'instance_seg_cube.npz', array=data)
        assert len(data) != 0

    async def test_cube_pathtracing(self):
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
        data = syn.sensors.get_semantic_segmentation(self.viewport)
        golden_image = np.load(self.golden_image_path / "semantic_seg_cube.npz")["array"]
        std_dev = np.sqrt(np.square(data - golden_image).astype(float).mean())
        assert std_dev < 0.1

    async def test_cube_ray_traced_lighting(self):
        """ Basic funtionality test of the sensor, but in ray traced lighting.
        """
        settings = carb.settings.get_settings()
        settings.set_string("/rtx/rendermode", "RayTracedLighting")

        stage = omni.usd.get_context().get_stage()
        cube = stage.DefinePrim("/Cube", "Cube")
        add_semantics(cube, "cube")
        cube.GetAttribute("size").Set(100)
        
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_semantic_segmentation(self.viewport)
        golden_image = np.load(self.golden_image_path / "semantic_seg_cube.npz")["array"]
        std_dev = np.sqrt(np.square(data - golden_image).astype(float).mean())
        assert std_dev < 0.1

    async def test_cube_ftheta(self):
        """ Basic funtionality test of the sensor under f theta camera.
        """
        settings = carb.settings.get_settings()
        settings.set_string("/rtx/rendermode", "PathTracing")
        settings.set_int("/rtx/pathtracing/spp", 32)
        settings.set_int("/persistent/app/viewport/displayOptions", 0)

        stage = omni.usd.get_context().get_stage()
        cube = stage.DefinePrim("/Cube", "Cube")
        add_semantics(cube, "cube")
        cube.GetAttribute("size").Set(100)
        await omni.kit.app.get_app().next_update_async()

        camera = stage.DefinePrim("/Camera", "Camera")

        # Set the camera to be polynomial fish eye camera.
        camera.CreateAttribute("cameraProjectionType", Sdf.ValueTypeNames.Token).Set("fisheyePolynomial")

        # Set the Camera's position
        UsdGeom.Xformable(camera).AddTranslateOp().Set((100, 100, 100))
        self.viewport.camera_path = camera.GetPath()
        
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_semantic_segmentation(self.viewport)
        # np.savez_compressed(self.golden_image_path / 'semantic_seg_cube_ftheta.npz', array=data)
        golden_image = np.load(self.golden_image_path / "semantic_seg_cube_ftheta.npz")["array"]
        std_dev = np.sqrt(np.square(data - golden_image).astype(float).mean())
        assert std_dev < 0.1
    
    async def test_cube_spherical(self):
        """ Basic funtionality test of the sensor under spherical camera.
        """
        settings = carb.settings.get_settings()
        settings.set_string("/rtx/rendermode", "PathTracing")
        settings.set_int("/rtx/pathtracing/spp", 32)
        settings.set_int("/persistent/app/viewport/displayOptions", 0)

        stage = omni.usd.get_context().get_stage()
        cube = stage.DefinePrim("/Cube", "Cube")
        add_semantics(cube, "cube")
        cube.GetAttribute("size").Set(100)
        await omni.kit.app.get_app().next_update_async()

        camera = stage.DefinePrim("/Camera", "Camera")

        # Set the camera to be spherical fish eye camera.
        camera.CreateAttribute("cameraProjectionType", Sdf.ValueTypeNames.Token).Set("fisheyeSpherical")

        # Set the Camera's position
        UsdGeom.Xformable(camera).AddTranslateOp().Set((100, 100, 100))
        self.viewport.camera_path = camera.GetPath()

        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_semantic_segmentation(self.viewport)
        # np.savez_compressed(self.golden_image_path / 'semantic_seg_cube_spherical.npz', array=data)
        golden_image = np.load(self.golden_image_path / "semantic_seg_cube_spherical.npz")["array"]
        std_dev = np.sqrt(np.square(data - golden_image).astype(float).mean())
        assert std_dev < 0.1

    @unittest.skip("OM-46393")
    async def test_geom_subset(self):
        """ Test sensor on GeomSubset.
        """
        path = os.path.join(FILE_DIR, "../data/scenes/streetlamp_03_golden.usd")
        await omni.usd.get_context().open_stage_async(path)
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_semantic_segmentation(self.viewport)
        
        assert len(data) != 0

    @unittest.skip("OM-46394")
    async def test_sem_seg_scene_instance(self):
        """ Test sensor on scene instance.
        """
        path = os.path.join(FILE_DIR, "../data/scenes/scene_instance_test.usda")

        await omni.usd.get_context().open_stage_async(path)
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_semantic_segmentation(self.viewport)

        # TODO add more complicated test
        assert len(data) != 0

            
    # After running each test
    async def tearDown(self):
        pass
