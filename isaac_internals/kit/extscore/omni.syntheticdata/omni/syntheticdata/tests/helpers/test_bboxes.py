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
from pxr import Sdf, UsdGeom, Vt

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.syntheticdata as syn

from ..utils import add_semantics


FILE_DIR = os.path.dirname(os.path.realpath(__file__))


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestBBoxes(omni.kit.test.AsyncTestCase):

    # Before running each test
    async def setUp(self):
        await omni.usd.get_context().new_stage_async()
        # Setup viewport
        self.viewport = get_active_viewport()

        await omni.usd.get_context().new_stage_async()
        self.stage = omni.usd.get_context().get_stage()
        prim = self.stage.DefinePrim("/World", "Xform")
        self.stage.SetDefaultPrim(prim)

        marked_cube = self.stage.DefinePrim("/World/MarkedCube0", "Cube")
        add_semantics(marked_cube, "cube")
        marked_cube.GetAttribute("size").Set(100)
        UsdGeom.XformCommonAPI(marked_cube).SetTranslate((3, 3, 0))

        unmarked_cube = self.stage.DefinePrim("/World/UnmarkedCube", "Cube")
        unmarked_cube.GetAttribute("size").Set(100)
        UsdGeom.XformCommonAPI(unmarked_cube).SetTranslate((3, 3, -100))
        await omni.kit.app.get_app().next_update_async()

        syn.sensors.enable_sensors(self.viewport, [syn._syntheticdata.SensorType.BoundingBox2DLoose])
        syn.sensors.enable_sensors(self.viewport, [syn._syntheticdata.SensorType.BoundingBox2DTight])
        syn.sensors.enable_sensors(self.viewport, [syn._syntheticdata.SensorType.BoundingBox3D])
        syn.sensors.enable_sensors(self.viewport, [syn._syntheticdata.SensorType.Occlusion])

    async def test_reduce_bboxes_3d(self):
        """Verify that reduce_bboxes_3d removes a cube without a semantic label"""
        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)

        # Get 3D bbox
        bbox = syn.sensors.get_bounding_box_3d(self.viewport, return_corners=True)
        assert np.allclose(bbox["z_min"], [-50, -50])
        # Transform of unmarked cube should be included in pre-reduced bbox but not included in reduced bbox
        UNMARKED_CUBE_GT = [[[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [3.0, 3.0, -100.0, 1.0]]]
        assert np.allclose(bbox["transform"][0], UNMARKED_CUBE_GT) or np.allclose(
            bbox["transform"][1], UNMARKED_CUBE_GT
        )

        instance_mappings = syn.helpers.get_instance_mappings()
        bbox_reduced = syn.helpers.reduce_bboxes_3d(bbox, instance_mappings)
        assert np.allclose(bbox_reduced["z_min"], [-50])
        assert np.allclose(
            bbox_reduced["transform"],
            [[[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [3.0, 3.0, 0.0, 1.0]]],
        )

    async def test_reduce_occlusion(self):
        """Verify that reduce_occlusion removes a cube without a semantic label"""

        # Add an extra cube
        cube = self.stage.DefinePrim("/World/MarkedCube1", "Cube")
        add_semantics(cube, "cube")
        cube.GetAttribute("size").Set(100)
        UsdGeom.XformCommonAPI(cube).SetTranslate((3, -10, 0))
        
        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport, True)
        # Get occlusion
        occlusion = syn.sensors.get_occlusion(self.viewport)
        occlusion_ratios = np.sort(occlusion["occlusionRatio"])
        assert np.allclose(occlusion_ratios, [0.0327, 0.38059998, 0.8886], atol=0.05)

        instance_mappings = syn.helpers.get_instance_mappings()
        reduced_occlusion = syn.helpers.reduce_occlusion(occlusion, instance_mappings)
        reduced_occlusion_ratios = np.sort(reduced_occlusion["occlusionRatio"])
        assert np.allclose(reduced_occlusion_ratios, [0.0327, 0.8886], atol=0.05)

    async def test_merge_sensors(self):
        """Verify that merge_sensors merges the data correctly"""

        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport, True)
        # Get bounding boxes and merge
        bounding_box_2d_tight = syn.sensors.get_bounding_box_2d_tight(self.viewport)
        bounding_box_2d_loose = syn.sensors.get_bounding_box_2d_loose(self.viewport)
        bounding_box_3d = syn.sensors.get_bounding_box_3d(self.viewport, parsed=True)
        merged_data = syn.helpers.merge_sensors(bounding_box_2d_tight, bounding_box_2d_loose, bounding_box_3d)

        for suffix, data_source in [
            ("_bbox2d_tight", bounding_box_2d_tight),
            ("_bbox2d_loose", bounding_box_2d_loose),
            ("_bbox3d", bounding_box_3d),
        ]:
            suffix_present = False
            for key in merged_data.dtype.fields:
                if key.endswith(suffix):
                    sub_key = key[: -len(suffix)]
                    assert merged_data[key] == data_source[key]
                    suffix_present = True
            assert suffix_present

    # After running each test
    async def tearDown(self):
        pass
