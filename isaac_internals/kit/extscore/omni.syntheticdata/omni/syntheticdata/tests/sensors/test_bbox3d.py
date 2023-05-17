# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import os
import unittest
import uuid
import math
import shutil
import asyncio
from time import time


import carb.tokens
import carb.settings
import numpy as np
import omni.kit.test
from omni.kit.viewport.utility import get_active_viewport
from pxr import Gf, UsdGeom, Usd, Sdf

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.syntheticdata as syn

from .. import utils


FILE_DIR = os.path.dirname(os.path.realpath(__file__))
TIMEOUT = 200
TMP = carb.tokens.get_tokens_interface().resolve("${temp}")


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestBBox3D(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        np.random.seed(1234)
        # Setup viewport
        self.viewport = get_active_viewport()

        # Initialize Sensor
        await omni.usd.get_context().new_stage_async()
        stage = omni.usd.get_context().get_stage()
        await omni.kit.app.get_app().next_update_async()
        await syn.sensors.create_or_retrieve_sensor_async(self.viewport, syn._syntheticdata.SensorType.BoundingBox3D)

    async def test_parsed_empty(self):
        """ Test 3D bounding box on empty stage.
        """
        bbox3d_data = syn.sensors.get_bounding_box_3d(self.viewport, parsed=True, return_corners=True)
        assert not bool(bbox3d_data)

    async def test_fields_exist(self):
        """ Test the correctness of the output dtype.
        """
        stage = omni.usd.get_context().get_stage()
        cube = stage.DefinePrim("/Cube", "Cube")
        utils.add_semantics(cube, "cube")
        
        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport, True)
        bbox3d_data_raw = syn.sensors.get_bounding_box_3d(self.viewport, parsed=False, return_corners=False)
        bbox3d_data_parsed = syn.sensors.get_bounding_box_3d(self.viewport, parsed=True, return_corners=True)

        raw_dtype = np.dtype(
            [
                ("instanceId", "<u4"),
                ("semanticId", "<u4"),
                ("x_min", "<f4"),
                ("y_min", "<f4"),
                ("z_min", "<f4"),
                ("x_max", "<f4"),
                ("y_max", "<f4"),
                ("z_max", "<f4"),
                ("transform", "<f4", (4, 4)),
            ]
        )
        parsed_dtype = np.dtype(
            [
                ("uniqueId", "<i4"),
                ("name", "O"),
                ("semanticLabel", "O"),
                ("metadata", "O"),
                ("instanceIds", "O"),
                ("semanticId", "<u4"),
                ("x_min", "<f4"),
                ("y_min", "<f4"),
                ("z_min", "<f4"),
                ("x_max", "<f4"),
                ("y_max", "<f4"),
                ("z_max", "<f4"),
                ("transform", "<f4", (4, 4)),
                ("corners", "<f4", (8, 3)),
            ]
        )
        assert bbox3d_data_raw.dtype == raw_dtype
        assert bbox3d_data_parsed.dtype == parsed_dtype

    async def test_parsed_nested_Y_pathtracing(self):
        """ Test 3D bounding box with nested semantics and transforms, Y-Up, in pathtracing mode.
        """
        settings = carb.settings.get_settings()
        settings.set_string("/rtx/rendermode", "PathTracing")
        settings.set_int("/rtx/pathtracing/spp", 32)
        settings.set_int("/persistent/app/viewport/displayOptions", 0)

        # Create 2 cubes (size=1) under a parent prim
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, "Y")
        parent = stage.DefinePrim("/World/Parent", "Xform")
        child1 = stage.DefinePrim("/World/Parent/Child1", "Cube")
        child2 = stage.DefinePrim("/World/Parent/Child2", "Cube")

        child1.GetAttribute("size").Set(1.0)
        child2.GetAttribute("size").Set(1.0)

        utils.add_semantics(parent, "parent")
        utils.add_semantics(child1, "child1")
        utils.add_semantics(child2, "child2")

        UsdGeom.Xformable(parent).ClearXformOpOrder()
        UsdGeom.Xformable(child1).ClearXformOpOrder()
        UsdGeom.Xformable(child2).ClearXformOpOrder()

        UsdGeom.Xformable(parent).AddRotateYOp().Set(45)
        UsdGeom.Xformable(child1).AddTranslateOp().Set((-0.5, 0.5, 0.0))
        UsdGeom.Xformable(child1).AddRotateYOp().Set(45)
        UsdGeom.Xformable(child2).AddTranslateOp().Set((0.5, -0.5, 0.0))
        await omni.kit.app.get_app().next_update_async()

        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        bbox3d_data = syn.sensors.get_bounding_box_3d(self.viewport, parsed=True, return_corners=True)

        parent_bbox = [row for row in bbox3d_data if row["name"] == parent.GetPath()][0]
        child1_bbox = [row for row in bbox3d_data if row["name"] == child1.GetPath()][0]
        child2_bbox = [row for row in bbox3d_data if row["name"] == child2.GetPath()][0]

        # Only takes into account child transforms
        a = math.cos(math.pi / 4)
        parent_bounds = [[-a - 0.5, -1.0, -a], [1.0, 1.0, a]]
        child1_bounds = [[-0.5, -0.5, -0.5], [0.5, 0.5, 0.5]]
        child2_bounds = [[-0.5, -0.5, -0.5], [0.5, 0.5, 0.5]]  # Doesn't take into account transforms

        for bbox, bounds in zip([parent_bbox, child1_bbox, child2_bbox], [parent_bounds, child1_bounds, child2_bounds]):
            self.assertAlmostEqual(bbox["x_min"], bounds[0][0], places=5)
            self.assertAlmostEqual(bbox["y_min"], bounds[0][1], places=5)
            self.assertAlmostEqual(bbox["z_min"], bounds[0][2], places=5)
            self.assertAlmostEqual(bbox["x_max"], bounds[1][0], places=5)
            self.assertAlmostEqual(bbox["y_max"], bounds[1][1], places=5)
            self.assertAlmostEqual(bbox["z_max"], bounds[1][2], places=5)

            prim = stage.GetPrimAtPath(bbox["name"])
            tf = np.array(UsdGeom.Imageable(prim).ComputeLocalToWorldTransform(0.0))
            gf_range = Gf.Range3f(*bounds)
            gf_corners = np.array([gf_range.GetCorner(i) for i in range(8)])
            gf_corners = np.pad(gf_corners, ((0, 0), (0, 1)), constant_values=1.0)
            gf_corners = np.dot(gf_corners, tf)[:, :3]
            assert np.allclose(bbox["corners"], gf_corners, atol=1e-5)

    async def test_parsed_nested_Y_ray_traced_lighting(self):
        """ Test 3D bounding box with nested semantics and transforms, Y-Up, in ray traced lighting mode.
        """
        # Set the rendering mode to be ray traced lighting.
        settings_interface = carb.settings.get_settings()
        settings_interface.set_string("/rtx/rendermode", "RayTracedLighting")

        # Create 2 cubes (size=1) under a parent prim
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, "Y")
        parent = stage.DefinePrim("/World/Parent", "Xform")
        child1 = stage.DefinePrim("/World/Parent/Child1", "Cube")
        child2 = stage.DefinePrim("/World/Parent/Child2", "Cube")

        child1.GetAttribute("size").Set(1.0)
        child2.GetAttribute("size").Set(1.0)

        utils.add_semantics(parent, "parent")
        utils.add_semantics(child1, "child1")
        utils.add_semantics(child2, "child2")

        UsdGeom.Xformable(parent).ClearXformOpOrder()
        UsdGeom.Xformable(child1).ClearXformOpOrder()
        UsdGeom.Xformable(child2).ClearXformOpOrder()

        UsdGeom.Xformable(parent).AddRotateYOp().Set(45)
        UsdGeom.Xformable(child1).AddTranslateOp().Set((-0.5, 0.5, 0.0))
        UsdGeom.Xformable(child1).AddRotateYOp().Set(45)
        UsdGeom.Xformable(child2).AddTranslateOp().Set((0.5, -0.5, 0.0))
        
        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        bbox3d_data = syn.sensors.get_bounding_box_3d(self.viewport, parsed=True, return_corners=True)

        parent_bbox = [row for row in bbox3d_data if row["name"] == parent.GetPath()][0]
        child1_bbox = [row for row in bbox3d_data if row["name"] == child1.GetPath()][0]
        child2_bbox = [row for row in bbox3d_data if row["name"] == child2.GetPath()][0]

        # Only takes into account child transforms
        a = math.cos(math.pi / 4)
        parent_bounds = [[-a - 0.5, -1.0, -a], [1.0, 1.0, a]]
        child1_bounds = [[-0.5, -0.5, -0.5], [0.5, 0.5, 0.5]]
        child2_bounds = [[-0.5, -0.5, -0.5], [0.5, 0.5, 0.5]]  # Doesn't take into account transforms

        for bbox, bounds in zip([parent_bbox, child1_bbox, child2_bbox], [parent_bounds, child1_bounds, child2_bounds]):
            self.assertAlmostEqual(bbox["x_min"], bounds[0][0], places=5)
            self.assertAlmostEqual(bbox["y_min"], bounds[0][1], places=5)
            self.assertAlmostEqual(bbox["z_min"], bounds[0][2], places=5)
            self.assertAlmostEqual(bbox["x_max"], bounds[1][0], places=5)
            self.assertAlmostEqual(bbox["y_max"], bounds[1][1], places=5)
            self.assertAlmostEqual(bbox["z_max"], bounds[1][2], places=5)

            prim = stage.GetPrimAtPath(bbox["name"])
            tf = np.array(UsdGeom.Imageable(prim).ComputeLocalToWorldTransform(0.0))
            gf_range = Gf.Range3f(*bounds)
            gf_corners = np.array([gf_range.GetCorner(i) for i in range(8)])
            gf_corners = np.pad(gf_corners, ((0, 0), (0, 1)), constant_values=1.0)
            gf_corners = np.dot(gf_corners, tf)[:, :3]
            assert np.allclose(bbox["corners"], gf_corners, atol=1e-5)

    async def test_parsed_nested_Y(self):
        """ Test 3D bounding box with nested semantics and transforms, Y-Up.
        """
        # Create 2 cubes (size=1) under a parent prim
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, "Y")
        parent = stage.DefinePrim("/World/Parent", "Xform")
        child1 = stage.DefinePrim("/World/Parent/Child1", "Cube")
        child2 = stage.DefinePrim("/World/Parent/Child2", "Cube")

        child1.GetAttribute("size").Set(1.0)
        child2.GetAttribute("size").Set(1.0)

        utils.add_semantics(parent, "parent")
        utils.add_semantics(child1, "child1")
        utils.add_semantics(child2, "child2")

        UsdGeom.Xformable(parent).ClearXformOpOrder()
        UsdGeom.Xformable(child1).ClearXformOpOrder()
        UsdGeom.Xformable(child2).ClearXformOpOrder()

        UsdGeom.Xformable(parent).AddRotateYOp().Set(45)
        UsdGeom.Xformable(child1).AddTranslateOp().Set((-0.5, 0.5, 0.0))
        UsdGeom.Xformable(child1).AddRotateYOp().Set(45)
        UsdGeom.Xformable(child2).AddTranslateOp().Set((0.5, -0.5, 0.0))
        
        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        bbox3d_data = syn.sensors.get_bounding_box_3d(self.viewport, parsed=True, return_corners=True)

        parent_bbox = [row for row in bbox3d_data if row["name"] == parent.GetPath()][0]
        child1_bbox = [row for row in bbox3d_data if row["name"] == child1.GetPath()][0]
        child2_bbox = [row for row in bbox3d_data if row["name"] == child2.GetPath()][0]

        # Only takes into account child transforms
        a = math.cos(math.pi / 4)
        parent_bounds = [[-a - 0.5, -1.0, -a], [1.0, 1.0, a]]
        child1_bounds = [[-0.5, -0.5, -0.5], [0.5, 0.5, 0.5]]
        child2_bounds = [[-0.5, -0.5, -0.5], [0.5, 0.5, 0.5]]  # Doesn't take into account transforms

        for bbox, bounds in zip([parent_bbox, child1_bbox, child2_bbox], [parent_bounds, child1_bounds, child2_bounds]):
            self.assertAlmostEqual(bbox["x_min"], bounds[0][0], places=5)
            self.assertAlmostEqual(bbox["y_min"], bounds[0][1], places=5)
            self.assertAlmostEqual(bbox["z_min"], bounds[0][2], places=5)
            self.assertAlmostEqual(bbox["x_max"], bounds[1][0], places=5)
            self.assertAlmostEqual(bbox["y_max"], bounds[1][1], places=5)
            self.assertAlmostEqual(bbox["z_max"], bounds[1][2], places=5)

            prim = stage.GetPrimAtPath(bbox["name"])
            tf = np.array(UsdGeom.Imageable(prim).ComputeLocalToWorldTransform(0.0))
            gf_range = Gf.Range3f(*bounds)
            gf_corners = np.array([gf_range.GetCorner(i) for i in range(8)])
            gf_corners = np.pad(gf_corners, ((0, 0), (0, 1)), constant_values=1.0)
            gf_corners = np.dot(gf_corners, tf)[:, :3]
            assert np.allclose(bbox["corners"], gf_corners, atol=1e-5)

    async def test_parsed_nested_Z(self):
        """ Test 3D bounding box with nested semantics and transforms, Z-Up.
        """
        # Create 2 cubes (size=1) under a parent prim
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, "Z")
        parent = stage.DefinePrim("/World/Parent", "Xform")
        child1 = stage.DefinePrim("/World/Parent/Child1", "Cube")
        child2 = stage.DefinePrim("/World/Parent/Child2", "Cube")

        child1.GetAttribute("size").Set(1.0)
        child2.GetAttribute("size").Set(1.0)

        utils.add_semantics(parent, "parent")
        utils.add_semantics(child1, "child1")
        utils.add_semantics(child2, "child2")

        UsdGeom.Xformable(parent).ClearXformOpOrder()
        UsdGeom.Xformable(child1).ClearXformOpOrder()
        UsdGeom.Xformable(child2).ClearXformOpOrder()

        UsdGeom.Xformable(parent).AddRotateYOp().Set(45)
        UsdGeom.Xformable(child1).AddTranslateOp().Set((-0.5, 0.5, 0.0))
        UsdGeom.Xformable(child1).AddRotateYOp().Set(45)
        UsdGeom.Xformable(child2).AddTranslateOp().Set((0.5, -0.5, 0.0))
        
        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        bbox3d_data = syn.sensors.get_bounding_box_3d(self.viewport, parsed=True, return_corners=True)

        parent_bbox = [row for row in bbox3d_data if row["name"] == parent.GetPath()][0]
        child1_bbox = [row for row in bbox3d_data if row["name"] == child1.GetPath()][0]
        child2_bbox = [row for row in bbox3d_data if row["name"] == child2.GetPath()][0]

        # Only takes into account child transforms
        a = math.cos(math.pi / 4)
        parent_bounds = [[-a - 0.5, -1.0, -a], [1.0, 1.0, a]]
        child1_bounds = [[-0.5, -0.5, -0.5], [0.5, 0.5, 0.5]]
        child2_bounds = [[-0.5, -0.5, -0.5], [0.5, 0.5, 0.5]]  # Doesn't take into account transforms

        for bbox, bounds in zip([parent_bbox, child1_bbox, child2_bbox], [parent_bounds, child1_bounds, child2_bounds]):
            self.assertAlmostEqual(bbox["x_min"], bounds[0][0], places=5)
            self.assertAlmostEqual(bbox["y_min"], bounds[0][1], places=5)
            self.assertAlmostEqual(bbox["z_min"], bounds[0][2], places=5)
            self.assertAlmostEqual(bbox["x_max"], bounds[1][0], places=5)
            self.assertAlmostEqual(bbox["y_max"], bounds[1][1], places=5)
            self.assertAlmostEqual(bbox["z_max"], bounds[1][2], places=5)

            prim = stage.GetPrimAtPath(bbox["name"])
            tf = np.array(UsdGeom.Imageable(prim).ComputeLocalToWorldTransform(0.0))
            gf_range = Gf.Range3f(*bounds)
            gf_corners = np.array([gf_range.GetCorner(i) for i in range(8)])
            gf_corners = np.pad(gf_corners, ((0, 0), (0, 1)), constant_values=1.0)
            gf_corners = np.dot(gf_corners, tf)[:, :3]
            assert np.allclose(bbox["corners"], gf_corners, atol=1e-5)

    @unittest.skip("OM-45008")
    async def test_camera_frame_simple_ftheta(self):
        """ Test 3D bounding box in a simple scene under ftheta camera.
        """
        settings = carb.settings.get_settings()
        settings.set_string("/rtx/rendermode", "PathTracing")
        settings.set_int("/rtx/pathtracing/spp", 32)
        settings.set_int("/persistent/app/viewport/displayOptions", 0)

        stage = omni.usd.get_context().get_stage()

        # TEST SIMPLE SCENE
        cube = stage.DefinePrim("/Cube", "Cube")
        cube.GetAttribute("size").Set(2.0)
        UsdGeom.Xformable(cube).AddTranslateOp().Set((10.0, 1.0, 2))
        utils.add_semantics(cube, "cube")

        camera = stage.DefinePrim("/Camera", "Camera")
        camera.CreateAttribute("cameraProjectionType", Sdf.ValueTypeNames.Token).Set("fisheyePolynomial")
        UsdGeom.Xformable(camera).AddTranslateOp().Set((10.0, 0.0, 0.0))
        self.viewport.camera_path = camera.GetPath()
        
        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        bbox3d_data = syn.sensors.get_bounding_box_3d(
            self.viewport, parsed=True, return_corners=True, camera_frame=True
        )

        # TODO: find the correct value of distorted result.
        # The f theta will distort the result.
        extents = Gf.Range3d([-1.0, 0, 1], [1.0, 2.0, 3])
        corners = np.array([[extents.GetCorner(i) for i in range(8)]])
        assert not np.allclose(bbox3d_data[0]["corners"], corners)

    @unittest.skip("OM-45008")
    async def test_camera_frame_simple_spherical(self):
        """ Test 3D bounding box in a simple scene under fisheye spherical camera.
        """
        settings = carb.settings.get_settings()
        settings.set_string("/rtx/rendermode", "PathTracing")
        settings.set_int("/rtx/pathtracing/spp", 32)
        settings.set_int("/persistent/app/viewport/displayOptions", 0)

        stage = omni.usd.get_context().get_stage()

        # TEST SIMPLE SCENE
        cube = stage.DefinePrim("/Cube", "Cube")
        cube.GetAttribute("size").Set(2.0)
        UsdGeom.Xformable(cube).AddTranslateOp().Set((10.0, 1.0, 2))
        utils.add_semantics(cube, "cube")

        camera = stage.DefinePrim("/Camera", "Camera")
        camera.CreateAttribute("cameraProjectionType", Sdf.ValueTypeNames.Token).Set("fisheyeSpherical")
        UsdGeom.Xformable(camera).AddTranslateOp().Set((10.0, 0.0, 0.0))
        self.viewport.camera_path = camera.GetPath()
        
        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        bbox3d_data = syn.sensors.get_bounding_box_3d(
            self.viewport, parsed=True, return_corners=True, camera_frame=True
        )

        # TODO: find the correct value of distorted result.
        # The spherical camera will distort the result.
        extents = Gf.Range3d([-1.0, 0, 1], [1.0, 2.0, 3])
        corners = np.array([[extents.GetCorner(i) for i in range(8)]])
        assert not np.allclose(bbox3d_data[0]["corners"], corners)

    async def test_camera_frame_simple(self):
        """ Test 3D bounding box in a simple scene.
        """
        stage = omni.usd.get_context().get_stage()

        # TEST SIMPLE SCENE
        cube = stage.DefinePrim("/Cube", "Cube")
        cube.GetAttribute("size").Set(2.0)
        UsdGeom.Xformable(cube).AddTranslateOp().Set((10.0, 0.0, 10.0))
        utils.add_semantics(cube, "cube")

        camera = stage.DefinePrim("/Camera", "Camera")
        UsdGeom.Xformable(camera).AddTranslateOp().Set((10.0, 0.0, 0.0))
        self.viewport.camera_path = camera.GetPath()
        
        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        bbox3d_data = syn.sensors.get_bounding_box_3d(
            self.viewport, parsed=True, return_corners=True, camera_frame=True
        )

        extents = Gf.Range3d([-1.0, -1.0, 9.0], [1.0, 1.0, 11.0])
        corners = np.array([[extents.GetCorner(i) for i in range(8)]])
        assert np.allclose(bbox3d_data[0]["corners"], corners)

        tf = np.eye(4)
        tf[3, 2] = 10.0
        assert np.allclose(bbox3d_data[0]["transform"], tf)

    async def test_camera_frame_reference(self):
        """ Test 3D bounding box in a simple scene.
        """
        ref_path = os.path.join(TMP, f"ref_stage{uuid.uuid1()}.usd")
        ref_stage = Usd.Stage.CreateNew(ref_path)
        world = ref_stage.DefinePrim("/World", "Xform")
        world_tf = utils.get_random_transform()
        UsdGeom.Xformable(world).AddTransformOp().Set(world_tf)

        cube = ref_stage.DefinePrim("/World/Cube", "Cube")
        cube.GetAttribute("size").Set(2.0)
        cube_tf = Gf.Matrix4d().SetTranslateOnly((10.0, 0.0, 10.0))
        UsdGeom.Xformable(cube).AddTransformOp().Set(cube_tf)
        utils.add_semantics(cube, "cube")

        camera = ref_stage.DefinePrim("/World/Camera", "Camera")
        camera_tf = cube_tf
        UsdGeom.Xformable(camera).AddTransformOp().Set(camera_tf)

        ref_stage.Save()

        # omni.usd.get_context().new_stage()
        # await omni.usd.get_context().new_stage_async()
        stage = omni.usd.get_context().get_stage()

        rig = stage.DefinePrim("/Rig", "Xform")
        rig_tf = utils.get_random_transform()
        UsdGeom.Xformable(rig).AddTransformOp().Set(rig_tf)
        ref = stage.DefinePrim("/Rig/Ref")
        ref.GetReferences().AddReference(ref_path, "/World")

        self.viewport.camera_path = "/Rig/Ref/Camera"
        
        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        bbox3d_data_world = syn.sensors.get_bounding_box_3d(
            self.viewport, parsed=True, return_corners=True, camera_frame=False
        )
        bbox3d_data_camera = syn.sensors.get_bounding_box_3d(
            self.viewport, parsed=True, return_corners=True, camera_frame=True
        )

        extents = Gf.Range3d([-1.0, -1.0, -1.0], [1.0, 1.0, 1.0])
        corners = np.array([[extents.GetCorner(i) for i in range(8)]])
        assert np.allclose(bbox3d_data_camera[0]["corners"], corners)

        combined_tf = np.matmul(cube_tf, np.matmul(world_tf, rig_tf))
        corners_tf = np.matmul(np.pad(corners.reshape(-1, 3), ((0, 0), (0, 1)), constant_values=1), combined_tf)
        corners_tf = corners_tf[:, :3].reshape(-1, 8, 3)
        assert np.allclose(bbox3d_data_world[0]["corners"], corners_tf)

        # tf = np.eye(4)
        # tf[3, 2] = 10.0
        assert np.allclose(bbox3d_data_world[0]["transform"], combined_tf)

        pt_camera_min = [bbox3d_data_camera[0][f"{a}_min"] for a in ["x", "y", "z"]]
        pt_camera_min = np.array([*pt_camera_min, 1.0])
        pt_camera_max = [bbox3d_data_camera[0][f"{a}_max"] for a in ["x", "y", "z"]]
        pt_camera_max = np.array([*pt_camera_max, 1.0])

        assert np.allclose(np.matmul(pt_camera_min, bbox3d_data_camera[0]["transform"])[:3], corners[0, 0])
        assert np.allclose(np.matmul(pt_camera_max, bbox3d_data_camera[0]["transform"])[:3], corners[0, 7])

    async def test_camera_frame_Y(self):
        # TEST NESTED TRANSFORMS, UP AXIS
        # Create 2 cubes (size=1) under a parent prim
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, "Y")
        parent = stage.DefinePrim("/World/Parent", "Xform")
        child1 = stage.DefinePrim("/World/Parent/Child1", "Cube")
        child2 = stage.DefinePrim("/World/Parent/Child2", "Cube")
        camera = stage.DefinePrim("/World/Camera", "Camera")

        child1.GetAttribute("size").Set(1.0)
        child2.GetAttribute("size").Set(1.0)

        utils.add_semantics(parent, "parent")
        utils.add_semantics(child1, "child1")
        utils.add_semantics(child2, "child2")

        UsdGeom.Xformable(parent).ClearXformOpOrder()
        UsdGeom.Xformable(child1).ClearXformOpOrder()
        UsdGeom.Xformable(child2).ClearXformOpOrder()
        UsdGeom.Xformable(camera).ClearXformOpOrder()

        UsdGeom.Xformable(parent).AddRotateYOp().Set(45)
        UsdGeom.Xformable(child1).AddTranslateOp().Set((-0.5, 0.5, 0.0))
        UsdGeom.Xformable(child1).AddRotateYOp().Set(45)
        UsdGeom.Xformable(child2).AddTranslateOp().Set((0.5, -0.5, 0.0))

        # Move camera with random transform
        camera_tf = utils.get_random_transform()
        UsdGeom.Xformable(camera).AddTransformOp().Set(Gf.Matrix4d(camera_tf))
        camera_tf_inv = np.linalg.inv(camera_tf)
        self.viewport.camera_path = camera.GetPath()
        
        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        bbox3d_data = syn.sensors.get_bounding_box_3d(
            self.viewport, parsed=True, return_corners=True, camera_frame=True
        )

        parent_bbox = [row for row in bbox3d_data if row["name"] == parent.GetPath()][0]
        child1_bbox = [row for row in bbox3d_data if row["name"] == child1.GetPath()][0]
        child2_bbox = [row for row in bbox3d_data if row["name"] == child2.GetPath()][0]

        # Only takes into account child transforms
        a = math.cos(math.pi / 4)
        parent_bounds = [[-a - 0.5, -1.0, -a], [1.0, 1.0, a]]
        child1_bounds = [[-0.5, -0.5, -0.5], [0.5, 0.5, 0.5]]
        child2_bounds = [[-0.5, -0.5, -0.5], [0.5, 0.5, 0.5]]  # Doesn't take into account transforms

        for bbox, bounds in zip([parent_bbox, child1_bbox, child2_bbox], [parent_bounds, child1_bounds, child2_bounds]):
            self.assertAlmostEqual(bbox["x_min"], bounds[0][0], places=5)
            self.assertAlmostEqual(bbox["y_min"], bounds[0][1], places=5)
            self.assertAlmostEqual(bbox["z_min"], bounds[0][2], places=5)
            self.assertAlmostEqual(bbox["x_max"], bounds[1][0], places=5)
            self.assertAlmostEqual(bbox["y_max"], bounds[1][1], places=5)
            self.assertAlmostEqual(bbox["z_max"], bounds[1][2], places=5)

            prim = stage.GetPrimAtPath(bbox["name"])
            tf = np.array(UsdGeom.Imageable(prim).ComputeLocalToWorldTransform(0.0))
            gf_range = Gf.Range3f(*bounds)
            gf_corners = np.array([gf_range.GetCorner(i) for i in range(8)])
            gf_corners = np.pad(gf_corners, ((0, 0), (0, 1)), constant_values=1.0)
            gf_corners = np.dot(gf_corners, tf)
            gf_corners = np.dot(gf_corners, camera_tf_inv)[:, :3]
            assert np.allclose(bbox["corners"], gf_corners, atol=1e-5)

    async def test_camera_frame_Z(self):
        # TEST NESTED TRANSFORMS, UP AXIS
        # Create 2 cubes (size=1) under a parent prim
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, "Z")
        parent = stage.DefinePrim("/World/Parent", "Xform")
        child1 = stage.DefinePrim("/World/Parent/Child1", "Cube")
        child2 = stage.DefinePrim("/World/Parent/Child2", "Cube")
        camera = stage.DefinePrim("/World/Camera", "Camera")

        child1.GetAttribute("size").Set(1.0)
        child2.GetAttribute("size").Set(1.0)

        utils.add_semantics(parent, "parent")
        utils.add_semantics(child1, "child1")
        utils.add_semantics(child2, "child2")

        UsdGeom.Xformable(parent).ClearXformOpOrder()
        UsdGeom.Xformable(child1).ClearXformOpOrder()
        UsdGeom.Xformable(child2).ClearXformOpOrder()
        UsdGeom.Xformable(camera).ClearXformOpOrder()

        UsdGeom.Xformable(parent).AddRotateYOp().Set(45)
        UsdGeom.Xformable(child1).AddTranslateOp().Set((-0.5, 0.5, 0.0))
        UsdGeom.Xformable(child1).AddRotateYOp().Set(45)
        UsdGeom.Xformable(child2).AddTranslateOp().Set((0.5, -0.5, 0.0))

        # Move camera with random transform
        camera_tf = np.eye(4)
        camera_tf[:3, :3] = Gf.Matrix3d(Gf.Rotation(np.random.rand(3).tolist(), np.random.rand(3).tolist()))
        camera_tf[3, :3] = np.random.rand(1, 3)
        UsdGeom.Xformable(camera).AddTransformOp().Set(Gf.Matrix4d(camera_tf))
        camera_tf_inv = np.linalg.inv(camera_tf)
        self.viewport.camera_path = camera.GetPath()
        
        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        bbox3d_data = syn.sensors.get_bounding_box_3d(
            self.viewport, parsed=True, return_corners=True, camera_frame=True
        )

        parent_bbox = [row for row in bbox3d_data if row["name"] == parent.GetPath()][0]
        child1_bbox = [row for row in bbox3d_data if row["name"] == child1.GetPath()][0]
        child2_bbox = [row for row in bbox3d_data if row["name"] == child2.GetPath()][0]

        # Only takes into account child transforms
        a = math.cos(math.pi / 4)
        parent_bounds = [[-a - 0.5, -1.0, -a], [1.0, 1.0, a]]
        child1_bounds = [[-0.5, -0.5, -0.5], [0.5, 0.5, 0.5]]
        child2_bounds = [[-0.5, -0.5, -0.5], [0.5, 0.5, 0.5]]  # Doesn't take into account transforms

        for bbox, bounds in zip([parent_bbox, child1_bbox, child2_bbox], [parent_bounds, child1_bounds, child2_bounds]):
            self.assertAlmostEqual(bbox["x_min"], bounds[0][0], places=5)
            self.assertAlmostEqual(bbox["y_min"], bounds[0][1], places=5)
            self.assertAlmostEqual(bbox["z_min"], bounds[0][2], places=5)
            self.assertAlmostEqual(bbox["x_max"], bounds[1][0], places=5)
            self.assertAlmostEqual(bbox["y_max"], bounds[1][1], places=5)
            self.assertAlmostEqual(bbox["z_max"], bounds[1][2], places=5)

            prim = stage.GetPrimAtPath(bbox["name"])
            tf = np.array(UsdGeom.Imageable(prim).ComputeLocalToWorldTransform(0.0))
            gf_range = Gf.Range3f(*bounds)
            gf_corners = np.array([gf_range.GetCorner(i) for i in range(8)])
            gf_corners = np.pad(gf_corners, ((0, 0), (0, 1)), constant_values=1.0)
            gf_corners = np.dot(gf_corners, tf)
            gf_corners = np.dot(gf_corners, camera_tf_inv)[:, :3]
            assert np.allclose(bbox["corners"], gf_corners, atol=1e-5)
    
    @unittest.skip("OM-46398")
    async def test_bbox_3d_scene_instance(self):
        """ Test sensor on scene instance.
        """
        path = os.path.join(FILE_DIR, "../data/scenes/scene_instance_test.usda")

        await omni.usd.get_context().open_stage_async(path)
        
        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        
        data = syn.sensors.get_bounding_box_3d_(self.viewport)
        # should be 3 prims in the scene
        # TODO: add more complicated test
        assert len(data) == 3

    # After running each test
    async def tearDown(self):
        pass
