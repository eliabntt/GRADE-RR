# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import os
import carb
import random
from pxr import Gf, UsdGeom, UsdLux, Sdf
import unittest
import omni.kit.test
from omni.syntheticdata import SyntheticData, SyntheticDataStage
from omni.kit.viewport.utility import get_active_viewport

FILE_DIR = os.path.dirname(os.path.realpath(__file__))

# Test the ogn node repeatability under stage manipulation


class TestStageManipulation(omni.kit.test.AsyncTestCase):

    def __init__(self, methodName: str) -> None:
        super().__init__(methodName=methodName)

    async def setUp(self):

        path = os.path.join(FILE_DIR, "../data/scenes/scene_instance_test.usda")
        await omni.usd.get_context().open_stage_async(path)
        #await omni.usd.get_context().new_stage_async()

        viewport = get_active_viewport()
        self.render_product_path = viewport.render_product_path

        # SyntheticData singleton interface
        sdg_iface = SyntheticData.Get()

        if not sdg_iface.is_node_template_registered("TestStageManipulationScenarii"):
            sdg_iface.register_node_template(
                SyntheticData.NodeTemplate(
                    SyntheticDataStage.SIMULATION,
                    "omni.syntheticdata.SdTestStageManipulationScenarii",
                    attributes={"inputs:worldPrimPath":"/World"}
                ),
                template_name="TestStageManipulationScenarii"  # node template name
            )

        render_vars = [
            #"SemanticMapSD",
            #"SemanticPrimTokenSD",
            #"InstanceMapSD",
            #"InstancePrimTokenSD",
            #"SemanticLabelTokenSD",
            #"SemanticLocalTransformSD",
            #"SemanticWorldTransformSD",
            "SemanticBoundingBox2DExtentTightSD",
            #"SemanticBoundingBox2DInfosTightSD",
            "SemanticBoundingBox2DExtentLooseSD",
            #"SemanticBoundingBox2DInfosLooseSD",
            "SemanticBoundingBox3DExtentSD",
            "SemanticBoundingBox3DInfosSD"
        ]

        for rv in render_vars:
            template_name = "TestRawArray" + rv
            if not sdg_iface.is_node_template_registered(template_name):
                sdg_iface.register_node_template(
                    SyntheticData.NodeTemplate(
                        SyntheticDataStage.ON_DEMAND,
                        "omni.syntheticdata.SdTestPrintRawArray",
                        [SyntheticData.NodeConnectionTemplate(rv + "ExportRawArray")]
                    ),
                    template_name=template_name
                )

        self.num_loops = 37

    async def render_var_test(self, render_var, ref_values, num_references_values, element_type, rand_seed=0, mode="printReferences"):
        sdg_iface = SyntheticData.Get()
        sdg_iface.activate_node_template("TestStageManipulationScenarii")
        sdg_iface.activate_node_template("TestRawArray" + render_var, 0, [self.render_product_path],
                                         {"inputs:elementType": element_type, "inputs:referenceValues": ref_values, "inputs:randomSeed": rand_seed, "inputs:mode": mode,
                                          "inputs:referenceNumUniqueRandomValues": num_references_values})
        for _ in range(self.num_loops):
            await omni.kit.app.get_app().next_update_async()
        sdg_iface.deactivate_node_template("TestRawArray" + render_var, 0, [self.render_product_path])
        sdg_iface.deactivate_node_template("TestStageManipulationScenarii")


    @unittest.skip("Unimplemented")
    async def test_semantic_map(self):
        await self.render_var_test("SemanticMapSD", [], "uint16", 2)

    async def test_semantic_bbox3d_extent(self):
        await self.render_var_test("SemanticBoundingBox3DExtentSD", 
        [
            87.556404,  223.83577,  -129.42677,  -155.79227,   -49.999996,  421.41083, 88.13742,   -50.000004,   49.999905,   39.782856,  -50.000004, -155.52794, -16.202198,
            -50.0, 136.29709,  -104.94976,  -155.52792,    87.556404,  -50.000008, 223.83577,  49.99991,   -87.8103,    -50.0,        -50.00001,   276.29846,  50.000004,
            421.41083,   -50.0,         60.42457,   223.83574,  -129.42676,   312.2204,  277.44424,   -50.000004,  -37.84166,    87.556404,  188.92877,   136.2971,  50.000004
        ], 13, "float32", 3, mode="testReferences")

    # async def test_semantic_bbox3d_infos(self):
    #     await self.render_var_test("SemanticBoundingBox3DInfosSD", 
    #     [
    #         -50.000008,  57.119793,  49.9999, -50.000004, -50.000015, -50.000004, 62.03122,
    #         -50.000008, -50.000004, -50.000004, -50.0,        50.0,       -50.0,  57.119793,
    #         9.5100141e-01, -4.7552836e-01,  6.1506079e+02,  1.0000000e+00, -1.0000000e+00,  1.3421423e+03,  4.9999901e+01
    #     ], 11, "int32", 4, mode="printReferences")

    async def test_semantic_bbox2d_extent_loose(self):
        await self.render_var_test("SemanticBoundingBox2DExtentLooseSD", 
        [
            733, 479, 532, 507, 460, 611, 763, 309,  17, 827, 789,
            698, 554, 947, 789, 581, 534, 156, 582, 323, 825, 298,
            562, 959, 595, 299, 117, 445, 572,  31, 622, 609, 228
        ], 11, "int32", 5, mode="testReferences")

    async def test_semantic_bbox2d_extent_tight(self):
        await self.render_var_test("SemanticBoundingBox2DExtentTightSD", 
        [
            0.0000000e+00, 5.0700000e+02, 1.1600000e+02, 7.4600000e+02, 5.9500000e+02, 2.1474836e+09, 2.1474836e+09, 2.5300000e+02, 3.6100000e+02, 1.7000000e+01,  0.0000000e+00,
            2.1474836e+09, 2.1474836e+09, 2.1474836e+09, 2.1474836e+09, 2.1474836e+09, 0.0000000e+00, 0.0000000e+00, 0.0000000e+00, 2.1474836e+09, 0.0000000e+00,  2.1474836e+09,
            0.0000000e+00, 3.1000000e+01, 5.3900000e+02, 2.3600000e+02, 2.1474836e+09, 5.7200000e+02, 8.9200000e+02, 9.0500000e+02, 5.6200000e+02, 5.1300000e+02,  0.0000000e+00
        ], 11, "int32", 9, mode="testReferences")

    async def tearDown(self):
        pass
