import omni.kit.test
import omni.graph.core as og
import omni.graph.core.tests as ogts
import os


class TestOgn(ogts.OmniGraphTestCase):

    async def test_data_access(self):
        from omni.syntheticdata.ogn.OgnSdPostRenderVarDisplayTextureDatabase import OgnSdPostRenderVarDisplayTextureDatabase
        test_file_name = "OgnSdPostRenderVarDisplayTextureTemplate.usda"
        usd_path = os.path.join(os.path.dirname(__file__), "usd", test_file_name)
        if not os.path.exists(usd_path):
            self.assertTrue(False, f"{usd_path} not found for loading test")
        (result, error) = await ogts.load_test_file(usd_path)
        self.assertTrue(result, f'{error} on {usd_path}')
        test_node = og.Controller.node("/TestGraph/Template_omni_syntheticdata_SdPostRenderVarDisplayTexture")
        database = OgnSdPostRenderVarDisplayTextureDatabase(test_node)
        self.assertTrue(test_node.is_valid())
        node_type_name = test_node.get_type_name()
        self.assertEqual(og.GraphRegistry().get_node_type_version(node_type_name), 1)

        def _attr_error(attribute: og.Attribute, usd_test: bool) -> str:
            test_type = "USD Load" if usd_test else "Database Access"
            return f"{node_type_name} {test_type} Test - {attribute.get_name()} value error"


        self.assertTrue(test_node.get_attribute_exists("inputs:cameraFisheyeParams"))
        attribute = test_node.get_attribute("inputs:cameraFisheyeParams")
        db_value = database.inputs.cameraFisheyeParams
        expected_value = []
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:cameraModel"))
        attribute = test_node.get_attribute("inputs:cameraModel")
        db_value = database.inputs.cameraModel
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:cameraNearFar"))
        attribute = test_node.get_attribute("inputs:cameraNearFar")
        db_value = database.inputs.cameraNearFar
        expected_value = [0.0, 0.0]
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:exec"))
        attribute = test_node.get_attribute("inputs:exec")
        db_value = database.inputs.exec

        self.assertTrue(test_node.get_attribute_exists("inputs:gpu"))
        attribute = test_node.get_attribute("inputs:gpu")
        db_value = database.inputs.gpu
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:instanceMapSDCudaPtr"))
        attribute = test_node.get_attribute("inputs:instanceMapSDCudaPtr")
        db_value = database.inputs.instanceMapSDCudaPtr
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:instanceMappingInfoSDPtr"))
        attribute = test_node.get_attribute("inputs:instanceMappingInfoSDPtr")
        db_value = database.inputs.instanceMappingInfoSDPtr
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:metersPerSceneUnit"))
        attribute = test_node.get_attribute("inputs:metersPerSceneUnit")
        db_value = database.inputs.metersPerSceneUnit
        expected_value = 0.0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:mode"))
        attribute = test_node.get_attribute("inputs:mode")
        db_value = database.inputs.mode
        expected_value = "autoMode"
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:parameters"))
        attribute = test_node.get_attribute("inputs:parameters")
        db_value = database.inputs.parameters
        expected_value = [0.0, 5.0, 0.33, 0.27]
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:renderVar"))
        attribute = test_node.get_attribute("inputs:renderVar")
        db_value = database.inputs.renderVar
        expected_value = ""
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:renderVarDisplay"))
        attribute = test_node.get_attribute("inputs:renderVarDisplay")
        db_value = database.inputs.renderVarDisplay
        expected_value = ""
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:rp"))
        attribute = test_node.get_attribute("inputs:rp")
        db_value = database.inputs.rp
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:sdDisplayHeight"))
        attribute = test_node.get_attribute("inputs:sdDisplayHeight")
        db_value = database.inputs.sdDisplayHeight
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:sdDisplayWidth"))
        attribute = test_node.get_attribute("inputs:sdDisplayWidth")
        db_value = database.inputs.sdDisplayWidth
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:sdSemBBox3dCamCornersCudaPtr"))
        attribute = test_node.get_attribute("inputs:sdSemBBox3dCamCornersCudaPtr")
        db_value = database.inputs.sdSemBBox3dCamCornersCudaPtr
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:sdSemBBox3dCamExtentCudaPtr"))
        attribute = test_node.get_attribute("inputs:sdSemBBox3dCamExtentCudaPtr")
        db_value = database.inputs.sdSemBBox3dCamExtentCudaPtr
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:sdSemBBoxExtentCudaPtr"))
        attribute = test_node.get_attribute("inputs:sdSemBBoxExtentCudaPtr")
        db_value = database.inputs.sdSemBBoxExtentCudaPtr
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:sdSemBBoxInfosCudaPtr"))
        attribute = test_node.get_attribute("inputs:sdSemBBoxInfosCudaPtr")
        db_value = database.inputs.sdSemBBoxInfosCudaPtr
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:semanticLabelTokenSDCudaPtr"))
        attribute = test_node.get_attribute("inputs:semanticLabelTokenSDCudaPtr")
        db_value = database.inputs.semanticLabelTokenSDCudaPtr
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:semanticMapSDCudaPtr"))
        attribute = test_node.get_attribute("inputs:semanticMapSDCudaPtr")
        db_value = database.inputs.semanticMapSDCudaPtr
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:semanticPrimTokenSDCudaPtr"))
        attribute = test_node.get_attribute("inputs:semanticPrimTokenSDCudaPtr")
        db_value = database.inputs.semanticPrimTokenSDCudaPtr
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:semanticWorldTransformSDCudaPtr"))
        attribute = test_node.get_attribute("inputs:semanticWorldTransformSDCudaPtr")
        db_value = database.inputs.semanticWorldTransformSDCudaPtr
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:swhFrameNumber"))
        attribute = test_node.get_attribute("inputs:swhFrameNumber")
        db_value = database.inputs.swhFrameNumber
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("outputs:cudaPtr"))
        attribute = test_node.get_attribute("outputs:cudaPtr")
        db_value = database.outputs.cudaPtr

        self.assertTrue(test_node.get_attribute_exists("outputs:exec"))
        attribute = test_node.get_attribute("outputs:exec")
        db_value = database.outputs.exec

        self.assertTrue(test_node.get_attribute_exists("outputs:format"))
        attribute = test_node.get_attribute("outputs:format")
        db_value = database.outputs.format

        self.assertTrue(test_node.get_attribute_exists("outputs:height"))
        attribute = test_node.get_attribute("outputs:height")
        db_value = database.outputs.height

        self.assertTrue(test_node.get_attribute_exists("outputs:renderVarDisplay"))
        attribute = test_node.get_attribute("outputs:renderVarDisplay")
        db_value = database.outputs.renderVarDisplay

        self.assertTrue(test_node.get_attribute_exists("outputs:width"))
        attribute = test_node.get_attribute("outputs:width")
        db_value = database.outputs.width
