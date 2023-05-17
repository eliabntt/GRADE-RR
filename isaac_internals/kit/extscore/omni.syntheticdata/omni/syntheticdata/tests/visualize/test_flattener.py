# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import os
import math
from time import time

import carb
import numpy as np
import omni.kit.test
from omni.kit.viewport.utility import get_active_viewport

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.syntheticdata as syn

FILE_DIR = os.path.dirname(os.path.realpath(__file__))
TIMEOUT = 50

BAKE_ACCURACY_THRESHOLD = 0.9

# segmentation mask testing against inputs of different resolutions
def test_against_golden(semantic_data, golden_semantic_data):
    input_dim = semantic_data.shape
    golden_dim = golden_semantic_data.shape
    correct_count = 0
    for y in range(0, input_dim[0]):
        for x in range(0, input_dim[1]):
            u = x / input_dim[1]
            v = y / input_dim[0]
            sample_x = math.floor(u * golden_dim[1])
            sample_y = math.floor(v * golden_dim[0])
            if semantic_data[y, x] == golden_semantic_data[sample_y, sample_x]:
                correct_count += 1
    return correct_count / (input_dim[0] * input_dim[1])


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestFlattenerSegmentationBakingVis(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        np.random.seed(1234)
        # Setup viewport
        self.viewport = get_active_viewport()

        await omni.kit.app.get_app_interface().next_update_async()

        filepath = os.path.join(FILE_DIR, "../data/scenes/OmniUe4-benchmark.usda")
        usd_context = omni.usd.get_context()
        await usd_context.open_stage_async(filepath)
        await omni.kit.app.get_app().next_update_async()

        syn.sensors.enable_sensors(self.viewport, [syn._syntheticdata.SensorType.SemanticSegmentation])
        
    async def _wait_for_data(self):
        data = np.empty(0)
        start = time()

        settings = carb.settings.get_settings()

        # wait until flattener is done loading in assets
        while not settings.get_as_bool("/app/captureFrame/ready"):
            await omni.kit.app.get_app_interface().next_update_async()

        # stall a couple of frames until samplerFeedback kicks off baking work.
        # NOTE: If we don't stall here, then we won't bake at all, because the ready flag will be falsely set
        #       since samplerFeedback hasn't seen any tiles yet, so flattener thinks scene is ready for capture.
        for i in range(0, 20):
            await omni.kit.app.get_app_interface().next_update_async()

        # wait until baking to be done
        while not settings.get_as_bool("/app/captureFrame/ready"):
            await omni.kit.app.get_app_interface().next_update_async()

    async def test_baking(self):
        """ Test that flattener correctly bakes semantic information into vtex
        """
        settings = carb.settings.get_settings()
        settings.set("/app/hydraEngine/waitIdle", True)
        # start flattener baking
        settings.set("/rtx/materialflattener/bake", True)
        settings.set("/rtx/materialflattener/rebaking", True)
        await omni.kit.app.get_app_interface().next_update_async()

        await self._wait_for_data()
        await syn.sensors.next_sensor_data_async(self.viewport)
        semantic_data = syn.sensors.get_semantic_segmentation(self.viewport)
        unique_classes = np.unique(semantic_data)

        # visual debug code
        #from PIL import Image
        #semantic_image = syn.visualize.colorize_segmentation(semantic_data)
        #semantic_image = np.uint8(semantic_image[:,:,:3])
        #im = Image.fromarray(semantic_image)
        #im.save('/home/chen/work/debug_segmentation.png')

        golden_filepath = os.path.join(FILE_DIR, "../data/golden/baked_segmentation.npz")
        golden_semantic_data = np.load(golden_filepath)["array"]
        unique_classes = np.unique(semantic_data)

        carb.log_warn(f'unique classes = {unique_classes}')

        assert len(unique_classes) == 3
        if len(unique_classes) == 3:
            accuracy = test_against_golden(semantic_data, golden_semantic_data)
            carb.log_warn(f'1st accuracy = {accuracy}')
            # it's possible semantic labels are flipped between road and lanemark, so redo the test
            # if accuracy is strikingly low
            if accuracy < BAKE_ACCURACY_THRESHOLD:
                for y in range(0, semantic_data.shape[0]):
                    for x in range(0, semantic_data.shape[1]):
                        # flip classes
                        if semantic_data[y, x] == unique_classes[1]:
                            semantic_data[y, x] = unique_classes[2]
                        elif semantic_data[y, x] == unique_classes[2]:
                            semantic_data[y, x] = unique_classes[1]
                accuracy = test_against_golden(semantic_data, golden_semantic_data)

                # visual debug code
                #semantic_image = syn.visualize.colorize_segmentation(semantic_data)
                #semantic_image = np.uint8(semantic_image[:,:,:3])
                #im = Image.fromarray(semantic_image)
                #im.save('/home/chen/work/debug_segmentation_2nd_try.png')

                carb.log_warn(f'2nd accuracy = {accuracy}')

            assert accuracy >= BAKE_ACCURACY_THRESHOLD

    # After running each test
    async def tearDown(self):
       pass