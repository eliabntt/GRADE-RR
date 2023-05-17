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

from pxr import UsdPhysics

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.syntheticdata as syn

from ..utils import add_semantics


FILE_DIR = os.path.dirname(os.path.realpath(__file__))


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestHelpersInstanceMappings(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        # Setup viewport
        self.viewport = get_active_viewport()

        await omni.usd.get_context().new_stage_async()
        self.stage = omni.usd.get_context().get_stage()
        prim = self.stage.DefinePrim("/World", "Xform")
        self.stage.SetDefaultPrim(prim)

    async def test_non_semantic_schemas(self):
        """ Test mixture of applied schemas including non-semantics.
        """
        prim = self.stage.DefinePrim("/World/Cone", "Cone")

        # Add semantics schema
        add_semantics(prim, "Je ne suis pas un cone.")

        # Non-semantics schema
        UsdPhysics.RigidBodyAPI.Apply(prim)
    
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        # Get instance mappings
        instance_mappings = syn.helpers.get_instance_mappings()

        # Validate
        cone_im = instance_mappings[0]
        assert cone_im["uniqueId"] == 1
        assert cone_im["name"] == "/World/Cone"
        assert cone_im["semanticId"] == 1
        assert cone_im["semanticLabel"] == "Je ne suis pas un cone."

    # After running each test
    async def tearDown(self):
        pass
