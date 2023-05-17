# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import os

import numpy as np
import omni.kit.test
from omni.kit.viewport.utility import get_active_viewport
from pxr import UsdGeom

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.syntheticdata as syn

from ..utils import add_semantics


FILE_DIR = os.path.dirname(os.path.realpath(__file__))
TIMEOUT = 50

# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestSemanticSegVis(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        np.random.seed(1234)
        # Setup viewport
        self.viewport = get_active_viewport()

        # Initialize Sensor
        await omni.usd.get_context().new_stage_async()
        stage = omni.usd.get_context().get_stage()
        await omni.kit.app.get_app().next_update_async()
        syn.sensors.enable_sensors(
            self.viewport,
            [syn._syntheticdata.SensorType.SemanticSegmentation, syn._syntheticdata.SensorType.InstanceSegmentation],
        )
        
    async def test_parsed_empty(self):
        """ Test semantic segmentation returns zero array with empty scene
        """
        await syn.sensors.next_sensor_data_async(self.viewport, True)
        data = syn.visualize.get_semantic_segmentation(self.viewport, mode="parsed")
        assert np.array_equal(data, np.zeros_like(data).astype(np.uint8))

    async def test_number_of_classes(self):
        """ Test that number of classes in output matches number of classes in scene
        """
        stage = omni.usd.get_context().get_stage()
        cube = stage.DefinePrim("/Cube1", "Cube")
        add_semantics(cube, "cube1")
        UsdGeom.Xformable(cube).AddTranslateOp().Set((0, 10, 0))
        cube = stage.DefinePrim("/Cube2", "Cube")
        add_semantics(cube, "cube2")
        UsdGeom.Xformable(cube).AddTranslateOp().Set((0, -10, 0))
        
        await syn.sensors.next_sensor_data_async(self.viewport, True)
        data = syn.visualize.get_semantic_segmentation(self.viewport, mode="parsed")
        data_non_bkg = data[data.sum(axis=-1) != 0]  # Remove background, encoded as (0, 0, 0, 0)

        assert len(np.unique(data_non_bkg, axis=0)) == 2

    # After running each test
    async def tearDown(self):
        pass
