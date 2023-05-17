# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import os
import math
import asyncio
from time import time
from pathlib import Path
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
class TestInstanceSeg(omni.kit.test.AsyncTestCase):
    def __init__(self, methodName: str) -> None:
        super().__init__(methodName=methodName)
        self.golden_image_path = Path(os.path.dirname(os.path.abspath(__file__))) / ".." / "data" / "golden"

    # Before running each test
    async def setUp(self):
        settings = carb.settings.get_settings()
        settings.set_bool("syntheticdata/sensors/perSubsetSegmentation", False)
        np.random.seed(1234)
        # Setup viewport
        self.viewport = get_active_viewport()

        # Initialize Sensor
        await omni.usd.get_context().new_stage_async()
        stage = omni.usd.get_context().get_stage()
        await omni.kit.app.get_app().next_update_async()
        await syn.sensors.create_or_retrieve_sensor_async(
            self.viewport, syn._syntheticdata.SensorType.InstanceSegmentation
        )

    #    TODO
    #    async def test_parsed_empty(self):
    #        """ Test instance segmentation on empty stage.
    #        """
    #        data = syn.sensors.get_instance_segmentation(self.viewport, parsed=True)
    #        assert data.sum() == 0

    async def test_parsed_dtype(self):
        stage = omni.usd.get_context().get_stage()
        cube = stage.DefinePrim("/Cube", "Cube")
        add_semantics(cube, "cube")
        await omni.kit.app.get_app().next_update_async()

        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_instance_segmentation(self.viewport, parsed=True)
        assert data.dtype == np.uint32

    async def test_cube(self):
        stage = omni.usd.get_context().get_stage()
        cube = stage.DefinePrim("/Cube", "Cube")
        add_semantics(cube, "cube")
        cube.GetAttribute("size").Set(100)
        
        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_instance_segmentation(self.viewport, return_mapping=False)
        # np.savez_compressed(self.golden_image_path / 'instance_seg_cube.npz', array=data)
        golden_image = np.load(self.golden_image_path / "instance_seg_cube.npz")["array"]
        std_dev = np.sqrt(np.square(data - golden_image).astype(float).mean())
        assert std_dev < 2

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
        
        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_instance_segmentation(self.viewport)
        # np.savez_compressed(self.golden_image_path / 'instance_seg_cube_sphere.npz', array=data)
        
        golden_image = np.load(self.golden_image_path / "instance_seg_cube_sphere.npz")["array"]
        std_dev = np.sqrt(np.square(data - golden_image).astype(float).mean())
        assert std_dev < 2

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
        
        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_instance_segmentation(self.viewport)
        # np.savez_compressed(self.golden_image_path / 'instance_seg_cube_pathtracing.npz', array=data)
        golden_image = np.load(self.golden_image_path / "instance_seg_cube_pathtracing.npz")["array"]
        std_dev = np.sqrt(np.square(data - golden_image).astype(float).mean())
        assert std_dev < 2
    
    async def test_cube_ray_traced_lighting(self):
        """ Basic funtionality test of the sensor, but in ray traced lighting.
        """
        # Set the rendering mode to be ray traced lighting.
        settings_interface = carb.settings.get_settings()
        settings_interface.set_string("/rtx/rendermode", "RayTracedLighting")

        stage = omni.usd.get_context().get_stage()
        cube = stage.DefinePrim("/Cube", "Cube")
        add_semantics(cube, "cube")
        cube.GetAttribute("size").Set(100)
        
        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_instance_segmentation(self.viewport)
        # np.savez_compressed(self.golden_image_path / 'instance_seg_cube_ray_traced_lighting.npz', array=data)
        golden_image = np.load(self.golden_image_path / "instance_seg_cube_ray_traced_lighting.npz")["array"]
        std_dev = np.sqrt(np.square(data - golden_image).astype(float).mean())
        assert std_dev < 2
    
    async def test_cube_ftheta(self):
        """ Basic funtionality test of the sensor under ftheta camera.
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
        
        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_instance_segmentation(self.viewport)
        # np.savez_compressed(self.golden_image_path / 'instance_seg_cube_ftheta.npz', array=data)
        golden_image = np.load(self.golden_image_path / "instance_seg_cube_ftheta.npz")["array"]
        std_dev = np.sqrt(np.square(data - golden_image).astype(float).mean())
        assert std_dev < 2
    
    async def test_cube_spherical(self):
        """ Basic funtionality test of the sensor under fisheye spherical camera.
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
        
        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_instance_segmentation(self.viewport)
        # np.savez_compressed(self.golden_image_path / 'instance_seg_cube_spherical.npz', array=data)
        golden_image = np.load(self.golden_image_path / "instance_seg_cube_spherical.npz")["array"]
        std_dev = np.sqrt(np.square(data - golden_image).astype(float).mean())
        assert std_dev < 2
    
    @unittest.skip("OM-46393")
    async def test_geom_subset(self):
        """ Test sensor on GeomSubset.
        """

        path = os.path.join(FILE_DIR, "../data/scenes/streetlamp_03_golden.usd")
        await omni.usd.get_context().open_stage_async(path)
        
        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_instance_segmentation(self.viewport)

        assert len(data) != 0
    
    async def test_instance_seg_scene_instance(self):
        """ Test sensor on scene instance.
        """
        settings = carb.settings.get_settings()

        path = os.path.join(FILE_DIR, "../data/scenes/scene_instance_test.usda")

        await omni.usd.get_context().open_stage_async(path)
        await omni.kit.app.get_app().next_update_async()
        await syn.sensors.create_or_retrieve_sensor_async(
            self.viewport, syn._syntheticdata.SensorType.InstanceSegmentation
        )
        
        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_instance_segmentation(self.viewport)

        assert len(data) != 0

    async def test_instance_seg_scene_instance_benchchair(self):
        """ Test sensor on scene instanced bench and chair data.
        """
        settings = carb.settings.get_settings()

        path = os.path.join(FILE_DIR, "../data/scenes/BenchChair_SceneInstance_Mini.usda")

        await omni.usd.get_context().open_stage_async(path)
        await omni.kit.app.get_app().next_update_async()
        await syn.sensors.create_or_retrieve_sensor_async(
            self.viewport, syn._syntheticdata.SensorType.InstanceSegmentation
        )

        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_instance_segmentation(self.viewport,parsed=True)

        assert len(data) != 0
        # should be 4 semantic objects in the scene.
        assert data.max() == 4

    async def test_instance_seg_point_instance_benchchair(self):
        """ Test sensor on point instanced bench and chair data.
        """
        settings = carb.settings.get_settings()

        path = os.path.join(FILE_DIR, "../data/scenes/BenchChair_Mini.usda")

        await omni.usd.get_context().open_stage_async(path)
        await omni.kit.app.get_app().next_update_async()
        await syn.sensors.create_or_retrieve_sensor_async(
            self.viewport, syn._syntheticdata.SensorType.InstanceSegmentation
        )

        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_instance_segmentation(self.viewport,parsed=True)

        assert len(data) != 0
        assert data.max() == 2

    async def test_instance_seg_point_instance_shapes(self):
        """ Test sensor on point instanced shapes that have semantics on the mesh.
        """
        settings = carb.settings.get_settings()

        path = os.path.join(FILE_DIR, "../data/scenes/point_instancer_semantic_shapes.usda")

        await omni.usd.get_context().open_stage_async(path)
        await omni.kit.app.get_app().next_update_async()
        await syn.sensors.create_or_retrieve_sensor_async(
            self.viewport, syn._syntheticdata.SensorType.InstanceSegmentation
        )

        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_instance_segmentation(self.viewport,parsed=True)

        assert len(data) != 0

    # After running each test
    async def tearDown(self):
        settings = carb.settings.get_settings()
        settings.set_bool("syntheticdata/sensors/perSubsetSegmentation", True)
        pass
