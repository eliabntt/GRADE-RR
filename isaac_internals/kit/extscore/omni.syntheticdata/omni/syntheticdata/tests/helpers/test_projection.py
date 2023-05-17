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
class TestProjection(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        await omni.usd.get_context().new_stage_async()
        # Setup viewport
        self.viewport = get_active_viewport()

        self.stage = omni.usd.get_context().get_stage()
        prim = self.stage.DefinePrim("/World", "Xform")
        self.stage.SetDefaultPrim(prim)
        cube = self.stage.DefinePrim("/World/Cube", "Cube")
        add_semantics(cube, "cube")

        usd_camera = UsdGeom.Camera.Define(self.stage, "/World/Camera")
        usd_camera.AddTranslateOp()
        self.camera = usd_camera.GetPrim()
        self.camera.CreateAttribute("cameraProjectionType", Sdf.ValueTypeNames.Token).Set(Vt.Token("pinhole"))
        self.camera.CreateAttribute("fthetaWidth", Sdf.ValueTypeNames.Float).Set(960)
        self.camera.CreateAttribute("fthetaHeight", Sdf.ValueTypeNames.Float).Set(604)
        self.camera.CreateAttribute("fthetaCx", Sdf.ValueTypeNames.Float).Set(460)
        self.camera.CreateAttribute("fthetaCy", Sdf.ValueTypeNames.Float).Set(340)
        self.camera.CreateAttribute("fthetaMaxFov", Sdf.ValueTypeNames.Float).Set(200.0)
        self.camera.CreateAttribute("fthetaPolyA", Sdf.ValueTypeNames.Float).Set(0.0)
        self.camera.CreateAttribute("fthetaPolyB", Sdf.ValueTypeNames.Float).Set(0.0059)
        self.camera.CreateAttribute("fthetaPolyC", Sdf.ValueTypeNames.Float).Set(0.0)
        self.camera.CreateAttribute("fthetaPolyD", Sdf.ValueTypeNames.Float).Set(0.0)
        self.camera.CreateAttribute("fthetaPolyE", Sdf.ValueTypeNames.Float).Set(0.0)

        self.viewport.camera_path = self.camera.GetPath()
        
        syn.sensors.enable_sensors(self.viewport, [syn._syntheticdata.SensorType.BoundingBox3D])
        await syn.sensors.next_sensor_data_async(self.viewport, True)

    async def test_pinhole(self):
        """ Test pinhole projection
        """
        self.camera.GetAttribute("xformOp:translate").Set((0.0, 0.0, 9.0))

        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        # Get 3D bbox
        bbox3d = syn.sensors.get_bounding_box_3d(self.viewport, return_corners=True, parsed=True)

        # Project corners
        corners = bbox3d["corners"]
        projected = syn.helpers.world_to_image(corners.reshape(-1, 3), self.viewport).reshape(-1, 8, 3)

        # GT
        # Confirmed visually to be correct
        GT = [
            [
                [0.26139346, 0.9241894, 0.9000009],
                [0.73860654, 0.9241894, 0.9000009],
                [0.26139346, 0.0758106, 0.9000009],
                [0.73860654, 0.0758106, 0.9000009],
                [0.20174183, 1.03023675, 0.87500088],
                [0.79825817, 1.03023675, 0.87500088],
                [0.20174183, -0.03023675, 0.87500088],
                [0.79825817, -0.03023675, 0.87500088],
            ]
        ]

        # Validate
        assert np.allclose(GT, projected)

    async def test_fisheye_polynomial(self):
        """ Test fisheye polynomial projection (F-Theta)
        """
        self.camera.GetAttribute("xformOp:translate").Set((0.0, 0.0, 3.0))
        self.camera.GetAttribute("cameraProjectionType").Set(Vt.Token("fisheyePolynomial"))
        
        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport, True)

        # Get 3D bbox
        bbox3d = syn.sensors.get_bounding_box_3d(self.viewport, return_corners=True, parsed=True)

        # Project corners
        corners = bbox3d["corners"]
        projected = syn.helpers.world_to_image(corners.reshape(-1, 3), self.viewport).reshape(-1, 8, 3)

        # GT
        # Confirmed visually to be correct
        GT = [
            [
                [0.43674065, 0.6457944, 0.0],
                [0.52159268, 0.6457944, 0.0],
                [0.43674065, 0.49494634, 0.0],
                [0.52159268, 0.49494634, 0.0],
                [0.40232877, 0.70697108, 0.0],
                [0.55600456, 0.70697108, 0.0],
                [0.40232877, 0.43376967, 0.0],
                [0.55600456, 0.43376967, 0.0],
            ]
        ]

        # Validate
        assert np.allclose(GT, projected)

        # Run the operation in reverse
        view_params = syn.helpers.get_view_params(self.viewport)
        proj_i2w = projected[0, :, :2]
        proj_i2w[..., 0] *= view_params["width"]
        proj_i2w[..., 1] *= view_params["height"]
        origin, directions = syn.helpers.image_to_world(proj_i2w, view_params)
        gt_corner_directions = corners[0] - origin
        gt_corner_directions /= np.linalg.norm(gt_corner_directions, axis=1, keepdims=True)
        assert np.allclose(gt_corner_directions, directions)

        # FOR VISUAL DEBUGGING
        self.camera.GetAttribute("clippingRange").Set((0.1, 1000000))
        for i, d in enumerate(directions):
            s = self.stage.DefinePrim(f"/World/pt{i}", "Sphere")
            UsdGeom.Xformable(s).AddTranslateOp().Set(tuple((d + origin).tolist()))
            s.GetAttribute("radius").Set(0.03)
        await syn.sensors.next_sensor_data_async(self.viewport,True)

    async def test_fisheye_polynomial_edge(self):
        """ Test fisheye polynomial projection (F-Theta) at edge of FOV
        """
        self.camera.GetAttribute("xformOp:translate").Set((4.0, 0.0, 0.5))
        self.camera.GetAttribute("cameraProjectionType").Set(Vt.Token("fisheyePolynomial"))
        
        # Render one frame
        await syn.sensors.next_sensor_data_async(self.viewport, True)
        # Get 3D bbox
        bbox3d = syn.sensors.get_bounding_box_3d(self.viewport, return_corners=True, parsed=True)

        # Project corners
        corners = bbox3d["corners"]
        projected = syn.helpers.world_to_image(corners.reshape(-1, 3), self.viewport).reshape(-1, 8, 3)

        # GT
        # Confirmed visually to be correct
        GT = [
            [
                [0.25675408, 0.6494504, 0.0],
                [0.2902532, 0.68231909, 0.0],
                [0.25675408, 0.49129034, 0.0],
                [0.2902532, 0.45842165, 0.0],
                [0.19030016, 0.67307846, 0.0],
                [0.18980286, 0.74184522, 0.0],
                [0.19030016, 0.46766228, 0.0],
                [0.18980286, 0.39889552, 0.0],
            ]
        ]

        # Validate
        assert np.allclose(GT, projected)

        # Run the operation in reverse
        view_params = syn.helpers.get_view_params(self.viewport)
        proj_i2w = projected[0, :, :2]
        proj_i2w[..., 0] *= view_params["width"]
        proj_i2w[..., 1] *= view_params["height"]
        origin, directions = syn.helpers.image_to_world(proj_i2w, view_params)
        gt_corner_directions = corners[0] - origin
        gt_corner_directions /= np.linalg.norm(gt_corner_directions, axis=1, keepdims=True)
        assert np.allclose(gt_corner_directions, directions)

        # FOR VISUAL DEBUGGING
        self.camera.GetAttribute("clippingRange").Set((0.1, 1000000))
        for i, d in enumerate(directions):
            s = self.stage.DefinePrim(f"/World/pt{i}", "Sphere")
            UsdGeom.Xformable(s).AddTranslateOp().Set(tuple((d + origin).tolist()))
        await syn.sensors.next_sensor_data_async(self.viewport,True)

    # After running each test
    async def tearDown(self):
        pass
