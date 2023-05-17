import omni.kit.test
import omni.graph.core as og
import omni.graph.core.tests as ogts
import os


class TestOgn(ogts.OmniGraphTestCase):

    async def test_data_access(self):
        from omni.syntheticdata.ogn.OgnSdInstanceMappingPtrDatabase import OgnSdInstanceMappingPtrDatabase
        test_file_name = "OgnSdInstanceMappingPtrTemplate.usda"
        usd_path = os.path.join(os.path.dirname(__file__), "usd", test_file_name)
        if not os.path.exists(usd_path):
            self.assertTrue(False, f"{usd_path} not found for loading test")
        (result, error) = await ogts.load_test_file(usd_path)
        self.assertTrue(result, f'{error} on {usd_path}')
        test_node = og.Controller.node("/TestGraph/Template_omni_syntheticdata_SdInstanceMappingPtr")
        database = OgnSdInstanceMappingPtrDatabase(test_node)
        self.assertTrue(test_node.is_valid())
        node_type_name = test_node.get_type_name()
        self.assertEqual(og.GraphRegistry().get_node_type_version(node_type_name), 1)

        def _attr_error(attribute: og.Attribute, usd_test: bool) -> str:
            test_type = "USD Load" if usd_test else "Database Access"
            return f"{node_type_name} {test_type} Test - {attribute.get_name()} value error"


        self.assertTrue(test_node.get_attribute_exists("inputs:cudaPtr"))
        attribute = test_node.get_attribute("inputs:cudaPtr")
        db_value = database.inputs.cudaPtr
        expected_value = False
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:exec"))
        attribute = test_node.get_attribute("inputs:exec")
        db_value = database.inputs.exec

        self.assertTrue(test_node.get_attribute_exists("inputs:renderResults"))
        attribute = test_node.get_attribute("inputs:renderResults")
        db_value = database.inputs.renderResults
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:semanticFilerTokens"))
        attribute = test_node.get_attribute("inputs:semanticFilerTokens")
        db_value = database.inputs.semanticFilerTokens
        expected_value = []
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

        self.assertTrue(test_node.get_attribute_exists("outputs:exec"))
        attribute = test_node.get_attribute("outputs:exec")
        db_value = database.outputs.exec

        self.assertTrue(test_node.get_attribute_exists("outputs:instanceMapPtr"))
        attribute = test_node.get_attribute("outputs:instanceMapPtr")
        db_value = database.outputs.instanceMapPtr

        self.assertTrue(test_node.get_attribute_exists("outputs:instancePrimPathPtr"))
        attribute = test_node.get_attribute("outputs:instancePrimPathPtr")
        db_value = database.outputs.instancePrimPathPtr

        self.assertTrue(test_node.get_attribute_exists("outputs:minInstanceIndex"))
        attribute = test_node.get_attribute("outputs:minInstanceIndex")
        db_value = database.outputs.minInstanceIndex

        self.assertTrue(test_node.get_attribute_exists("outputs:minSemanticIndex"))
        attribute = test_node.get_attribute("outputs:minSemanticIndex")
        db_value = database.outputs.minSemanticIndex

        self.assertTrue(test_node.get_attribute_exists("outputs:numInstances"))
        attribute = test_node.get_attribute("outputs:numInstances")
        db_value = database.outputs.numInstances

        self.assertTrue(test_node.get_attribute_exists("outputs:numSemantics"))
        attribute = test_node.get_attribute("outputs:numSemantics")
        db_value = database.outputs.numSemantics

        self.assertTrue(test_node.get_attribute_exists("outputs:semanticLabelTokenPtrs"))
        attribute = test_node.get_attribute("outputs:semanticLabelTokenPtrs")
        db_value = database.outputs.semanticLabelTokenPtrs

        self.assertTrue(test_node.get_attribute_exists("outputs:semanticLocalTransformPtr"))
        attribute = test_node.get_attribute("outputs:semanticLocalTransformPtr")
        db_value = database.outputs.semanticLocalTransformPtr

        self.assertTrue(test_node.get_attribute_exists("outputs:semanticMapPtr"))
        attribute = test_node.get_attribute("outputs:semanticMapPtr")
        db_value = database.outputs.semanticMapPtr

        self.assertTrue(test_node.get_attribute_exists("outputs:semanticPrimPathPtr"))
        attribute = test_node.get_attribute("outputs:semanticPrimPathPtr")
        db_value = database.outputs.semanticPrimPathPtr

        self.assertTrue(test_node.get_attribute_exists("outputs:semanticWorldTransformPtr"))
        attribute = test_node.get_attribute("outputs:semanticWorldTransformPtr")
        db_value = database.outputs.semanticWorldTransformPtr

        self.assertTrue(test_node.get_attribute_exists("outputs:swhFrameNumber"))
        attribute = test_node.get_attribute("outputs:swhFrameNumber")
        db_value = database.outputs.swhFrameNumber
