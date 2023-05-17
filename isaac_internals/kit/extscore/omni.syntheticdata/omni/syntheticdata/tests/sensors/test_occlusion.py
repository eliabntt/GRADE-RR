# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import os
import math
from time import time
from pathlib import Path


import carb
import numpy as np
import unittest
import omni.kit.test
from omni.kit.viewport.utility import get_active_viewport
from pxr import UsdGeom, Sdf

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.syntheticdata as syn

from ..utils import add_semantics


FILE_DIR = os.path.dirname(os.path.realpath(__file__))
TIMEOUT = 200


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestOcclusion(omni.kit.test.AsyncTestCase):
    def __init__(self, methodName: str) -> None:
        super().__init__(methodName=methodName)
        self.golden_image_path = Path(os.path.dirname(os.path.abspath(__file__))) / ".." / "data" / "golden"

    # Before running each test
    async def setUp(self):
        await omni.usd.get_context().new_stage_async()
        stage = omni.usd.get_context().get_stage()
        self.viewport = get_active_viewport()
        # Initialize Sensors
        syn.sensors.enable_sensors(
            self.viewport,
            [
                syn._syntheticdata.SensorType.BoundingBox2DLoose,
                syn._syntheticdata.SensorType.BoundingBox2DTight,
                syn._syntheticdata.SensorType.Occlusion,
            ],
        )
        await syn.sensors.next_sensor_data_async(self.viewport,True)

    async def test_fields_exist(self):
        stage = omni.usd.get_context().get_stage()
        cube = stage.DefinePrim("/Cube", "Cube")
        add_semantics(cube, "cube")
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_occlusion(self.viewport)

        valid_dtype = [("instanceId", "<u4"), ("semanticId", "<u4"), ("occlusionRatio", "<f4")]
        assert data.dtype == np.dtype(valid_dtype)

    async def test_fields_exist_parsed(self):
        stage = omni.usd.get_context().get_stage()
        cube = stage.DefinePrim("/Cube", "Cube")
        add_semantics(cube, "cube")
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        data = syn.sensors.get_occlusion(self.viewport, parsed=True)

        valid_dtype = [
            ("uniqueId", "<i4"),
            ("name", "O"),
            ("semanticLabel", "O"),
            ("metadata", "O"),
            ("instanceIds", "O"),
            ("semanticId", "<u4"),
            ("occlusionRatio", "<f4"),
        ]
        assert data.dtype == np.dtype(valid_dtype)

    async def test_occlusion(self):
        path = os.path.join(FILE_DIR, "../data/scenes/occlusion.usda")
        await omni.usd.get_context().open_stage_async(path)
        syn.sensors.enable_sensors(self.viewport, [syn._syntheticdata.SensorType.Occlusion])
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        occlusion_out = syn.sensors.get_occlusion(self.viewport, parsed=True)
        for row in occlusion_out:
            gt = float(row["semanticLabel"]) / 100.0
            assert math.isclose(gt, row["occlusionRatio"], abs_tol=0.015), f"Expected {gt}, got {row['occlusionRatio']}"

    async def test_self_occlusion(self):
        path = os.path.join(FILE_DIR, "../data/scenes/torus_sphere.usda")
        await omni.usd.get_context().open_stage_async(path)
        syn.sensors.enable_sensors(self.viewport, [syn._syntheticdata.SensorType.Occlusion])
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        occlusion_out = syn.sensors.get_occlusion(self.viewport)
        occlusion_out_ratios = np.sort(occlusion_out["occlusionRatio"])
        assert np.allclose(occlusion_out_ratios, [0.0, 0.6709], atol=0.05)

    async def test_full_occlusion(self):
        path = os.path.join(FILE_DIR, "../data/scenes/cube_full_occlusion.usda")
        await omni.usd.get_context().open_stage_async(path)
        syn.sensors.enable_sensors(self.viewport, [syn._syntheticdata.SensorType.Occlusion])
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        occlusion_out = syn.sensors.get_occlusion(self.viewport)
        occlusion_out_ratios = np.sort(occlusion_out["occlusionRatio"])
        assert np.allclose(occlusion_out_ratios, [0.0, 1.0], atol=0.05)

    async def test_occlusion_pathtracing(self):
        """ Basic funtionality test of the sensor, but in path tracing mode.
        """
        settings = carb.settings.get_settings()
        settings.set_string("/rtx/rendermode", "PathTracing")
        settings.set_int("/rtx/pathtracing/spp", 32)
        settings.set_int("/persistent/app/viewport/displayOptions", 0)

        path = os.path.join(FILE_DIR, "../data/scenes/occlusion.usda")
        await omni.usd.get_context().open_stage_async(path)
        syn.sensors.enable_sensors(self.viewport, [syn._syntheticdata.SensorType.Occlusion])
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        occlusion_out = syn.sensors.get_occlusion(self.viewport, parsed=True)
        for row in occlusion_out:
            gt = float(row["semanticLabel"]) / 100.0
            assert math.isclose(gt, row["occlusionRatio"], abs_tol=0.015), f"Expected {gt}, got {row['occlusionRatio']}"

    async def test_occlusion_ray_traced_lighting(self):
        """ Basic funtionality test of the sensor, but in ray traced lighting.
        """
        # Set the rendering mode to be ray traced lighting.
        settings_interface = carb.settings.get_settings()
        settings_interface.set_string("/rtx/rendermode", "RayTracedLighting")

        path = os.path.join(FILE_DIR, "../data/scenes/occlusion.usda")
        await omni.usd.get_context().open_stage_async(path)
        syn.sensors.enable_sensors(self.viewport, [syn._syntheticdata.SensorType.Occlusion])
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        occlusion_out = syn.sensors.get_occlusion(self.viewport, parsed=True)
        for row in occlusion_out:
            gt = float(row["semanticLabel"]) / 100.0
            assert math.isclose(gt, row["occlusionRatio"], abs_tol=0.015), f"Expected {gt}, got {row['occlusionRatio']}"

    async def test_occlusion_ftheta(self):
        """ Basic funtionality test of the sensor under ftheta camera.
        """
        settings = carb.settings.get_settings()
        settings.set_string("/rtx/rendermode", "PathTracing")
        settings.set_int("/rtx/pathtracing/spp", 32)
        settings.set_int("/persistent/app/viewport/displayOptions", 0)

        path = os.path.join(FILE_DIR, "../data/scenes/occlusion.usda")
        await omni.usd.get_context().open_stage_async(path)
        await omni.kit.app.get_app().next_update_async()

        stage = omni.usd.get_context().get_stage()

        camera = stage.DefinePrim("/Camera", "Camera")
        # Set the camera to be polynomial fish eye camera.
        camera.CreateAttribute("cameraProjectionType", Sdf.ValueTypeNames.Token).Set("fisheyePolynomial")

        # Set the Camera's position
        UsdGeom.Xformable(camera).AddTranslateOp().Set((100, 200, 300))
        self.viewport.camera_path = camera.GetPath()
        
        syn.sensors.enable_sensors(self.viewport, [syn._syntheticdata.SensorType.Occlusion])
        await syn.sensors.next_sensor_data_async(self.viewport,True)

        # Camera type should not affect occlusion.
        occlusion_out = syn.sensors.get_occlusion(self.viewport, parsed=True)
        data = np.array([row['occlusionRatio'] for row in occlusion_out])
        # np.savez_compressed(self.golden_image_path / 'occlusion_ftheta.npz', array=data)

        golden = np.load(self.golden_image_path / "occlusion_ftheta.npz")["array"]

        assert np.isclose(data, golden, atol=1e-3).all()

    async def test_occlusion_spherical(self):
        """ Basic funtionality test of the sensor under spherical camera.
        """
        settings = carb.settings.get_settings()
        settings.set_string("/rtx/rendermode", "PathTracing")
        settings.set_int("/rtx/pathtracing/spp", 32)
        settings.set_int("/persistent/app/viewport/displayOptions", 0)

        path = os.path.join(FILE_DIR, "../data/scenes/occlusion.usda")
        await omni.usd.get_context().open_stage_async(path)
        await omni.kit.app.get_app().next_update_async()

        stage = omni.usd.get_context().get_stage()

        camera = stage.DefinePrim("/Camera", "Camera")
        # Set the camera to be polynomial fish eye camera.
        camera.CreateAttribute("cameraProjectionType", Sdf.ValueTypeNames.Token).Set("fisheyeSpherical")

        # Set the Camera's position
        UsdGeom.Xformable(camera).AddTranslateOp().Set((100, 200, 300))
        self.viewport.camera_path = camera.GetPath()
        
        syn.sensors.enable_sensors(self.viewport, [syn._syntheticdata.SensorType.Occlusion])
        await syn.sensors.next_sensor_data_async(self.viewport,True)

        # Camera type should not affect occlusion.
        occlusion_out = syn.sensors.get_occlusion(self.viewport, parsed=True)
        data = np.array([row['occlusionRatio'] for row in occlusion_out])
        # np.savez_compressed(self.golden_image_path / 'occlusion_spherical.npz', array=data)

        golden = np.load(self.golden_image_path / "occlusion_spherical.npz")["array"]

        assert np.isclose(data, golden, atol=1e-1).all()

    @unittest.skip("OM-44310")
    async def test_occlusion_quadrant(self):
        # disabling temporarly the test for OMNI-GRAPH support : OM-44310
        # Test quadrant sensor. It takes loose and tight bounding boxes to
        # return the type of occlusion

        # Expected occlusion value for time=1, 2, 3...
        TESTS = [
            "fully-occluded",
            "left",
            "right",
            "bottom",
            "top",
            "fully-visible",  # corner occlusion
            "fully-visible",  # corner occlusion
            "bottom-right",
            "bottom-left",
            "top-right",
            "top-left",
            "fully-visible",
        ]

        path = os.path.join(FILE_DIR, "../data/scenes/occlusion_quadrant.usda")
        await omni.usd.get_context().open_stage_async(path)
        await omni.kit.app.get_app().next_update_async()
        syn.sensors.enable_sensors(
            self.viewport,
            [
                syn._syntheticdata.SensorType.BoundingBox2DLoose,
                syn._syntheticdata.SensorType.BoundingBox2DTight,
                syn._syntheticdata.SensorType.Occlusion,
            ],
        )
        await syn.sensors.next_sensor_data_async(self.viewport,True)

        timeline_iface = omni.timeline.acquire_timeline_interface()
        timeline_iface.set_time_codes_per_second(1)

        for time, gt in enumerate(TESTS):
            timeline_iface.set_current_time(time)
            await omni.kit.app.get_app().next_update_async()
            # Investigate these in OM-31155
            sensor_out = syn.sensors.get_occlusion_quadrant(self.viewport)
            result = sensor_out["occlusion_quadrant"][0]
            assert result == gt, f"Got {result}, expected {gt}"

    # After running each test
    async def tearDown(self):
        pass
