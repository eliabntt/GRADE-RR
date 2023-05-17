# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import os
import unittest
import omni.kit.test
from omni.kit.viewport.utility import get_active_viewport
from omni.syntheticdata import SyntheticData

from ..utils import add_semantics

import numpy as np

# Test the semantic filter

class TestSemanticFilter(omni.kit.test.AsyncTestCase):

    def __init__(self, methodName: str) -> None:
        super().__init__(methodName=methodName)

    async def setUp(self):

        await omni.usd.get_context().new_stage_async()
        stage = omni.usd.get_context().get_stage()

        # scene 
        # /World [belong_to:world]
        #    /Cube [class:cube]
        #        /Sphere [class:sphere]
        #    /Sphere [class:sphere]
        #    /Capsule [class:capsule]
        # /Cube [class:cube]
        # /Capsule [class:capsule]
        # /Nothing [belong_to:nothing]

        world_prim = stage.DefinePrim("/World", "Plane")
        add_semantics(world_prim, "world", "belong_to")
        
        world_cube_prim = stage.DefinePrim("/World/Cube", "Cube")
        add_semantics(world_cube_prim, "cube", "class")

        world_cube_sphere_prim = stage.DefinePrim("/World/Cube/Sphere", "Sphere")
        add_semantics(world_cube_sphere_prim, "sphere", "class")

        world_sphere_prim = stage.DefinePrim("/World/Sphere", "Sphere")
        add_semantics(world_sphere_prim, "sphere", "class")

        world_capsule_prim = stage.DefinePrim("/World/Capsule", "Capsule")
        add_semantics(world_capsule_prim, "capsule", "class")

        cube_prim = stage.DefinePrim("/Cube", "Cube")
        add_semantics(cube_prim, "cube", "class")
        
        capsule_prim = stage.DefinePrim("/Capsule", "Capsule")
        add_semantics(capsule_prim, "capsule", "class")

        nothing_prim = stage.DefinePrim("/Nothing", "Plane")
        add_semantics(nothing_prim, "nothing", "belong_to")

        self.render_product_path = get_active_viewport().render_product_path
        SyntheticData.Get().activate_node_template("SemanticLabelTokenSDExportRawArray", 0, [self.render_product_path])

        await omni.kit.app.get_app().next_update_async()


    def fetch_semantic_label_tokens(self):
        output_names = ["outputs:data","outputs:bufferSize"]
        outputs = SyntheticData.Get().get_node_attributes("SemanticLabelTokenSDExportRawArray", output_names, self.render_product_path)
        assert outputs
        return outputs["outputs:data"].view(np.uint64)

    async def check_num_valid_labels(self, expected_num_valid_labels):
        wait_iterations = 6
        for _ in range(wait_iterations):
            await omni.kit.app.get_app().next_update_async()
        num_valid_labels = np.count_nonzero(self.fetch_semantic_label_tokens())
        assert num_valid_labels == expected_num_valid_labels

    async def test_semantic_filter_all(self):
        SyntheticData.Get().set_default_semantic_filter("*:*", True)
        await self.check_num_valid_labels(8)    
    
    async def test_semantic_filter_no_world(self):
        SyntheticData.Get().set_default_semantic_filter("!belong_to:world", True)
        # /Cube /Capsule /Nothing
        await self.check_num_valid_labels(3)    
        
    async def test_semantic_filter_all_class_test(self):
        SyntheticData.Get().set_default_semantic_filter("class:*", True)
        await self.check_num_valid_labels(6)

    async def test_semantic_filter_all_class_no_cube_test(self):
        SyntheticData.Get().set_default_semantic_filter("class:!cube&*", True)
        await self.check_num_valid_labels(3)

    async def test_semantic_filter_only_sphere_or_cube_test(self):
        SyntheticData.Get().set_default_semantic_filter("class:cube|sphere", True)
        await self.check_num_valid_labels(4)

    async def test_semantic_filter_sphere_and_cube_test(self):
        SyntheticData.Get().set_default_semantic_filter("class:cube&sphere", True)
        # /World/Cube/Sphere
        await self.check_num_valid_labels(1)

    async def test_semantic_filter_world_and_sphere_test(self):
        SyntheticData.Get().set_default_semantic_filter("class:sphere,belong_to:world", True)
        await self.check_num_valid_labels(2)

    async def test_semantic_filter_no_belong_test(self):
        SyntheticData.Get().set_default_semantic_filter("belong_to:!*", True)
        # /Cube /Capsule
        await self.check_num_valid_labels(2)

    async def test_semantic_filter_world_or_capsule_test(self):
        SyntheticData.Get().set_default_semantic_filter("belong_to:world;class:capsule", True)
        await self.check_num_valid_labels(6)

    async def test_semantic_filter_belong_to_nohierarchy(self):
        SyntheticData.Get().set_default_semantic_filter("belong_to:*", False)
        await self.check_num_valid_labels(2)

    async def tearDown(self):
        SyntheticData.Get().set_default_semantic_filter("*:*")
        