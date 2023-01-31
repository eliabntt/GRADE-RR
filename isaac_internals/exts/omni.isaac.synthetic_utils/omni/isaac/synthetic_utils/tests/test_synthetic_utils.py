# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test

import omni.kit.commands
import carb
import carb.tokens
import copy
import os
import asyncio
import numpy as np
from pxr import Gf, UsdGeom, UsdPhysics
import random

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.syntheticdata as syn
from omni.isaac.synthetic_utils import SyntheticDataHelper
from omni.isaac.synthetic_utils.writers import NumpyWriter
from omni.isaac.synthetic_utils.writers import KittiWriter
from omni.syntheticdata.tests.utils import add_semantics
from omni.isaac.core.utils.physics import simulate_async
from omni.isaac.core.utils.nucleus import find_nucleus_server
from omni.isaac.core.utils.semantics import add_update_semantics
from omni.isaac.core.utils.extensions import get_extension_path_from_name
from omni.isaac.core.utils.stage import set_stage_up_axis
from omni.isaac.core import PhysicsContext
from omni.physx.scripts.physicsUtils import add_ground_plane

# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestSyntheticUtils(omni.kit.test.AsyncTestCaseFailOnLogError):
    # Before running each test
    async def setUp(self):
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self._physics_rate = 60
        set_stage_up_axis("z")
        PhysicsContext(physics_dt=1.0 / self._physics_rate)
        self._time_step = 1.0 / self._physics_rate
        carb.settings.get_settings().set_int("/app/runLoops/main/rateLimitFrequency", int(self._physics_rate))
        carb.settings.get_settings().set_bool("/app/runLoops/main/rateLimitEnabled", True)
        carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", int(self._physics_rate))
        carb.settings.get_settings().set("/app/asyncRendering", False)
        carb.settings.get_settings().set("/app/hydraEngine/waitIdle", True)
        carb.settings.get_settings().set("/rtx/hydra/enableSemanticSchema", True)
        await omni.kit.app.get_app().next_update_async()

        # Start Simulation and wait
        self._timeline = omni.timeline.get_timeline_interface()
        self._viewport_window = omni.kit.viewport.get_default_viewport_window()
        self._usd_context = omni.usd.get_context()
        self._sd_helper = SyntheticDataHelper()
        self._synthetic_utils_path = get_extension_path_from_name("omni.isaac.synthetic_utils")
        self._stage = self._usd_context.get_stage()
        self._camera_path = "/Camera"
        camera = self._stage.DefinePrim(self._camera_path, "Camera")
        self._viewport_window.set_active_camera(self._camera_path)

        pass

    # After running each test
    async def tearDown(self):
        await omni.kit.app.get_app().next_update_async()
        self._timeline.stop()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        pass

    async def initialize_sensors(self):
        # Initialize syntheticdata sensors
        await omni.kit.app.get_app().next_update_async()
        sensor_type = syn._syntheticdata.SensorType
        await syn.sensors.initialize_async(
            self._viewport_window,
            [
                sensor_type.Rgb,
                sensor_type.DepthLinear,
                sensor_type.InstanceSegmentation,
                sensor_type.SemanticSegmentation,
                sensor_type.BoundingBox2DLoose,
                sensor_type.BoundingBox2DTight,
                sensor_type.BoundingBox3D,
            ],
            timeout=10,
        )
        await omni.kit.app.get_app().next_update_async()

    # Acquire a copy of the ground truth.
    def get_groundtruth(self):
        gt = self._sd_helper.get_groundtruth(
            [
                "rgb",
                "depthLinear",
                "boundingBox2DTight",
                "boundingBox2DLoose",
                "instanceSegmentation",
                "semanticSegmentation",
                "boundingBox3D",
                "camera",
                "pose",
            ],
            self._viewport_window,
            verify_sensor_init=False,
        )
        return copy.deepcopy(gt)

    async def load_robot_scene(self):
        result, nucleus_server = find_nucleus_server()
        if result is False:
            carb.log_error("Could not find nucleus server with /Isaac folder")
            return
        robot_usd = nucleus_server + "/Isaac/Robots/Carter/carter_v1.usd"

        add_ground_plane(self._stage, "/physics/groundPlane", "Z", 1000.0, Gf.Vec3f(0.0, 0, -25), Gf.Vec3f(1.0))

        # setup high-level robot prim
        self.prim = self._stage.DefinePrim("/robot", "Xform")
        self.prim.GetReferences().AddReference(robot_usd)
        add_semantics(self.prim, "robot")
        rot_mat = Gf.Matrix3d(Gf.Rotation((0, 0, 1), 90))
        omni.kit.commands.execute(
            "TransformPrimCommand",
            path=self.prim.GetPath(),
            old_transform_matrix=None,
            new_transform_matrix=Gf.Matrix4d().SetRotate(rot_mat).SetTranslateOnly(Gf.Vec3d(0, -64, 0)),
        )

        # setup scene camera
        self._viewport_window.set_camera_position(self._camera_path, 300, 300, 300, True)
        self._viewport_window.set_camera_target(self._camera_path, 0, -64, 0, True)
        await self.initialize_sensors()

    # Unit test for sensor groundtruth
    async def test_groundtruth(self):
        await self.load_robot_scene()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1.0)
        await omni.kit.app.get_app().next_update_async()
        gt = self.get_groundtruth()
        # Validate Depth groundtruth
        gt_depth = gt["depthLinear"]
        self.assertAlmostEqual(np.min(gt_depth), 5.11157, delta=0.1)
        self.assertAlmostEqual(np.max(gt_depth), 7.4310575, delta=0.1)
        # Validate 2D BBox groundtruth
        gt_bbox2d = gt["boundingBox2DTight"]
        self.assertEqual(len(gt_bbox2d), 1)
        self.assertAlmostEqual(gt_bbox2d[0][6], 432, delta=2)
        self.assertAlmostEqual(gt_bbox2d[0][7], 138, delta=2)
        self.assertAlmostEqual(gt_bbox2d[0][8], 844, delta=2)
        self.assertAlmostEqual(gt_bbox2d[0][9], 542, delta=2)
        # Validate semantic segmentation groundtruth - 0 (unlabeled) and 1 (robot)
        gt_semantic = gt["semanticSegmentation"]
        self.assertEqual(len(np.unique(gt_semantic)), 2)
        user_semantic_label_map = {"robot": 4, "cylinder": 5, "cube": 6}
        mapped_data = self._sd_helper.get_mapped_semantic_data(gt_semantic, user_semantic_label_map)
        unique_data = np.unique(mapped_data)
        self.assertEqual(unique_data[0], 0)
        self.assertEqual(unique_data[1], 4)
        # Validate 3D BBox groundtruth
        gt_bbox3d = gt["boundingBox3D"]
        self.assertEqual(len(gt_bbox3d), 1)
        self.assertAlmostEqual(gt_bbox3d[0][6], -43.041847, delta=0.01)
        self.assertAlmostEqual(gt_bbox3d[0][7], -31.312422, delta=0.01)
        self.assertAlmostEqual(gt_bbox3d[0][8], -25.173292, delta=0.01)
        self.assertAlmostEqual(gt_bbox3d[0][9], 24.220554, delta=0.01)
        self.assertAlmostEqual(gt_bbox3d[0][10], 31.31649, delta=0.01)
        self.assertAlmostEqual(gt_bbox3d[0][11], 41.19104, delta=0.01)
        # Validate camera groundtruth - position, fov, focal length, aperature
        gt_camera = gt["camera"]
        gt_camera_trans = gt_camera["pose"][3, :3]
        self.assertAlmostEqual(gt_camera_trans[0], 300.0, delta=0.001)
        self.assertAlmostEqual(gt_camera_trans[1], 300.0, delta=0.001)
        self.assertAlmostEqual(gt_camera_trans[2], 300.0, delta=0.001)
        self.assertEqual(gt_camera["resolution"]["width"], 1280)
        self.assertEqual(gt_camera["resolution"]["height"], 720)
        self.assertAlmostEqual(gt_camera["fov"], 0.4131223226073451, 1e-5)
        self.assertAlmostEqual(gt_camera["focal_length"], 50.0, 1e-5)
        self.assertAlmostEqual(gt_camera["horizontal_aperture"], 20.954999923706055, 1e-2)
        # Validate pose groundtruth - prim path, semantic label, position
        gt_pose = gt["pose"]
        self.assertEqual(len(gt_pose), 1)
        self.assertEqual(gt_pose[0][0], "/robot")
        self.assertEqual(gt_pose[0][2], "robot")
        gt_pose_trans = (gt_pose[0])[3][3, :3]
        self.assertAlmostEqual(gt_pose_trans[0], 0.0, delta=0.001)
        self.assertAlmostEqual(gt_pose_trans[1], -64.0, delta=0.001)
        self.assertAlmostEqual(gt_pose_trans[2], 0.0, delta=0.001)
        pass

    # Unit test for data writer
    async def test_writer(self):
        await self.load_robot_scene()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1.0)
        await omni.kit.app.get_app().next_update_async()
        # Setting up config for writer
        sensor_settings = {}
        sensor_settings_viewport = {"rgb": {"enabled": True}}
        viewport_name = "Viewport"
        sensor_settings[viewport_name] = copy.deepcopy(sensor_settings_viewport)
        # Initialize data writer
        output_folder = os.getcwd() + "/output"
        data_writer = NumpyWriter(output_folder, 4, 100, sensor_settings)
        data_writer.start_threads()
        # Get rgb groundtruth
        gt = self._sd_helper.get_groundtruth(["rgb"], self._viewport_window, verify_sensor_init=False)
        # Write rgb groundtruth
        image_id = 1
        groundtruth = {"METADATA": {"image_id": str(image_id), "viewport_name": viewport_name}, "DATA": {}}
        groundtruth["DATA"]["RGB"] = gt["rgb"]
        data_writer.q.put(groundtruth)
        # Validate output file
        output_file_path = os.path.join(output_folder, viewport_name, "rgb", str(image_id) + ".png")
        data_writer.stop_threads()
        await asyncio.sleep(0.1)
        self.assertEqual(os.path.isfile(output_file_path), True)
        pass

        # Unit test for data writer

    async def test_kitti_writer(self):
        await self.load_robot_scene()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1.0)
        await omni.kit.app.get_app().next_update_async()
        # Setting up config for writer
        sensor_settings = {}
        sensor_settings_viewport = {"rgb": {"enabled": True}}
        viewport_name = "Viewport"
        sensor_settings[viewport_name] = copy.deepcopy(sensor_settings_viewport)
        # Initialize data writer
        output_folder_tight = os.getcwd() + "/kitti_tight"
        output_folder_loose = os.getcwd() + "/kitti_loose"
        data_writer_tight = KittiWriter(
            output_folder_tight, 4, 100, train_size=1, classes="robot", bbox_type="BBOX2DTIGHT"
        )
        data_writer_tight.start_threads()
        data_writer_loose = KittiWriter(
            output_folder_loose, 4, 100, train_size=1, classes="robot", bbox_type="BBOX2DLOOSE"
        )
        data_writer_loose.start_threads()
        # Get rgb groundtruth
        gt = self._sd_helper.get_groundtruth(
            ["rgb", "boundingBox2DTight", "boundingBox2DLoose"], self._viewport_window, verify_sensor_init=False
        )
        # Write rgb groundtruth
        image_id = 0
        groundtruth = {
            "METADATA": {
                "image_id": str(image_id),
                "viewport_name": viewport_name,
                "BBOX2DTIGHT": {},
                "BBOX2DLOOSE": {},
            },
            "DATA": {},
        }
        image = gt["rgb"]
        groundtruth["DATA"]["RGB"] = image
        groundtruth["DATA"]["BBOX2DTIGHT"] = gt["boundingBox2DTight"]
        groundtruth["METADATA"]["BBOX2DTIGHT"]["WIDTH"] = image.shape[1]
        groundtruth["METADATA"]["BBOX2DTIGHT"]["HEIGHT"] = image.shape[0]

        groundtruth["DATA"]["BBOX2DLOOSE"] = gt["boundingBox2DLoose"]
        groundtruth["METADATA"]["BBOX2DLOOSE"]["WIDTH"] = image.shape[1]
        groundtruth["METADATA"]["BBOX2DLOOSE"]["HEIGHT"] = image.shape[0]
        for f in range(2):
            groundtruth["METADATA"]["image_id"] = image_id
            data_writer_tight.q.put(copy.deepcopy(groundtruth))
            data_writer_loose.q.put(copy.deepcopy(groundtruth))
            image_id = image_id + 1

        # Validate output file
        data_writer_tight.stop_threads()
        data_writer_loose.stop_threads()
        await asyncio.sleep(0.1)

        for output_folder in [output_folder_tight, output_folder_loose]:
            self.assertEqual(os.path.isfile(os.path.join(output_folder + "/training/image_2", str(0) + ".png")), True)
            self.assertEqual(os.path.isfile(os.path.join(output_folder + "/training/label_2", str(0) + ".txt")), True)
            self.assertEqual(os.path.isfile(os.path.join(output_folder + "/testing/image_2", str(1) + ".png")), True)
        pass

    # create a cube.
    async def add_cube(self, path, size, offset):
        cubeGeom = UsdGeom.Cube.Define(self._stage, path)
        cubePrim = self._stage.GetPrimAtPath(path)

        # use add_semantics to set its class to Cube
        add_semantics(cubePrim, "cube")

        cubeGeom.CreateSizeAttr(size)

        cubeGeom.ClearXformOpOrder()
        cubeGeom.AddTranslateOp().Set(offset)

        await omni.kit.app.get_app().next_update_async()
        UsdPhysics.CollisionAPI.Apply(cubePrim)
        return cubePrim, cubeGeom

    # create a scene with a cube.
    async def load_cube_scene(self):

        # ensure we are done with all of scene setup.
        await omni.kit.app.get_app().next_update_async()

        # check units
        meters_per_unit = UsdGeom.GetStageMetersPerUnit(self._stage)
        add_ground_plane(self._stage, "/physics/groundPlane", "Z", 1000.0, Gf.Vec3f(0.0, 0, -25), Gf.Vec3f(1.0))

        # Add a cube at a "close" location
        self.cube_location = Gf.Vec3f(-300.0, 0.0, 50.0)
        self.cube, self.cube_geom = await self.add_cube("/World/Cube", 100.0, self.cube_location)

        # setup scene camera
        self._viewport_window.set_camera_position(self._camera_path, 1000, 1000, 1000, True)
        self._viewport_window.set_camera_target(self._camera_path, 0, 0, 0, True)
        await self.initialize_sensors()

    # Unit test for sensor groundtruth
    async def frame_lag_test(self, move):
        # start the scene

        # wait for update
        move(Gf.Vec3f(random.random() * 100, random.random() * 100, random.random() * 100))
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # grab ground truth
        gt1 = self.get_groundtruth()

        # move the cube
        move(Gf.Vec3f(random.random() * 100, random.random() * 100, random.random() * 100))

        # wait for update
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # grab ground truth
        gt2 = self.get_groundtruth()
        await omni.kit.app.get_app().next_update_async()
        gt3 = self.get_groundtruth()

        # ensure segmentation is identical
        gt_seg1 = gt1["semanticSegmentation"]
        gt_seg2 = gt2["semanticSegmentation"]
        self.assertEqual(len(np.unique(gt_seg1)), len(np.unique(gt_seg2)))

        # the cube 3d bboxes should be different after update
        gt_box3d1 = gt1["boundingBox3D"]
        gt_box3d2 = gt2["boundingBox3D"]
        gt_box3d3 = gt3["boundingBox3D"]

        # check the list size
        self.assertEqual(len(gt_box3d1), len(gt_box3d2))

        # check the corners, they should/must move to pass the test.
        self.assertNotEqual(gt_box3d1["corners"].tolist(), gt_box3d2["corners"].tolist())
        # Should be no change between these two frames
        self.assertEqual(gt_box3d2["corners"].tolist(), gt_box3d3["corners"].tolist())
        await omni.kit.app.get_app().next_update_async()
        # stop the scene

        pass

    # Test lag by executing a command
    async def test_oneframelag_kitcommand(self):
        await self.load_cube_scene()

        def set_prim_pose(location):
            omni.kit.commands.execute(
                "TransformPrimCommand",
                path=self.cube.GetPath(),
                old_transform_matrix=None,
                new_transform_matrix=Gf.Matrix4d()
                .SetRotate(Gf.Matrix3d(Gf.Rotation((0, 0, 1), 90)))
                .SetTranslateOnly(Gf.Vec3d(location)),
            )

        for frame in range(50):
            await self.frame_lag_test(set_prim_pose)
        pass

    # Test lag using a USD prim.
    async def test_oneframelag_usdprim(self):
        await self.load_cube_scene()

        def set_prim_pose(location):
            properties = self.cube.GetPropertyNames()
            if "xformOp:translate" in properties:
                translate_attr = self.cube.GetAttribute("xformOp:translate")
                translate_attr.Set(location)

        for frame in range(50):
            await self.frame_lag_test(set_prim_pose)
        pass

    async def test_remap_semantics(self):
        self._viewport_window.set_camera_position(self._camera_path, 1000, 1000, 1000, True)
        self._viewport_window.set_camera_target(self._camera_path, 0, 0, 0, True)
        usd_path = self._synthetic_utils_path + "/data/usd/tests/nested_semantics.usd"
        self.prim = self._stage.DefinePrim("/test_nested", "Xform")
        self.prim.GetReferences().AddReference(usd_path)
        await omni.kit.app.get_app().next_update_async()
        await self.initialize_sensors()
        gt = self.get_groundtruth()
        ids = self._sd_helper.get_semantic_ids(gt["semanticSegmentation"])
        labels = self._sd_helper.get_semantic_label_map(ids)
        # make sure remapping with remap_using_base_class True should work even if we don't have nested classes
        mapped_id_a = self._sd_helper.get_semantic_ids(
            self._sd_helper.get_mapped_semantic_data(
                gt["semanticSegmentation"], {"red": 1, "green": 10, "blue": 100}, remap_using_base_class=True
            )
        )
        mapped_id_b = self._sd_helper.get_semantic_ids(
            self._sd_helper.get_mapped_semantic_data(
                gt["semanticSegmentation"], {"red": 1, "green": 10, "blue": 100}, remap_using_base_class=False
            )
        )
        # if labels aren't nested, they should remain the same
        unique_data_a = np.unique(mapped_id_a).tolist()
        unique_data_b = np.unique(mapped_id_b).tolist()

        self.assertListEqual(unique_data_a, unique_data_b)
        self.assertEqual(unique_data_a[0], 0)
        self.assertEqual(unique_data_a[1], 1)
        self.assertEqual(unique_data_a[2], 10)
        self.assertEqual(unique_data_a[3], 100)

    async def test_nested_semantics(self):
        self._viewport_window.set_camera_position(self._camera_path, 1000, 1000, 1000, True)
        self._viewport_window.set_camera_target(self._camera_path, 0, 0, 0, True)
        usd_path = self._synthetic_utils_path + "/data/usd/tests/nested_semantics.usd"
        self.prim = self._stage.DefinePrim("/test_nested", "Xform")
        add_update_semantics(self.prim, "combined")
        self.prim.GetReferences().AddReference(usd_path)
        await omni.kit.app.get_app().next_update_async()
        await self.initialize_sensors()
        gt = self.get_groundtruth()
        ids = self._sd_helper.get_semantic_ids(gt["semanticSegmentation"])
        labels = self._sd_helper.get_semantic_label_map(ids)
        mapped_id_a = self._sd_helper.get_semantic_ids(
            self._sd_helper.get_mapped_semantic_data(
                gt["semanticSegmentation"], {"combined": 99}, remap_using_base_class=True
            )
        )
        mapped_id_b = self._sd_helper.get_semantic_ids(
            self._sd_helper.get_mapped_semantic_data(
                gt["semanticSegmentation"], {"combined": 99}, remap_using_base_class=False
            )
        )
        unique_data_a = np.unique(mapped_id_a).tolist()
        unique_data_b = np.unique(mapped_id_b).tolist()

        self.assertEqual(unique_data_a[0], 0)
        self.assertEqual(unique_data_a[1], 99)

        # remap_using_base_class false should result in the mapping not changing
        self.assertEqual(unique_data_b[0], 0)
        self.assertEqual(unique_data_b[1], 1)
        self.assertEqual(unique_data_b[2], 2)
        self.assertEqual(unique_data_b[3], 3)
