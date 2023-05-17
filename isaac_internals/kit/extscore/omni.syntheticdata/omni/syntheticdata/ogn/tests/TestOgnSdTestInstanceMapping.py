import omni.kit.test
import omni.graph.core as og
import omni.graph.core.tests as ogts
import os


class TestOgn(ogts.OmniGraphTestCase):

    async def test_data_access(self):
        from omni.syntheticdata.ogn.OgnSdTestInstanceMappingDatabase import OgnSdTestInstanceMappingDatabase
        test_file_name = "OgnSdTestInstanceMappingTemplate.usda"
        usd_path = os.path.join(os.path.dirname(__file__), "usd", test_file_name)
        if not os.path.exists(usd_path):
            self.assertTrue(False, f"{usd_path} not found for loading test")
        (result, error) = await ogts.load_test_file(usd_path)
        self.assertTrue(result, f'{error} on {usd_path}')
        test_node = og.Controller.node("/TestGraph/Template_omni_syntheticdata_SdTestInstanceMapping")
        database = OgnSdTestInstanceMappingDatabase(test_node)
        self.assertTrue(test_node.is_valid())
        node_type_name = test_node.get_type_name()
        self.assertEqual(og.GraphRegistry().get_node_type_version(node_type_name), 1)

        def _attr_error(attribute: og.Attribute, usd_test: bool) -> str:
            test_type = "USD Load" if usd_test else "Database Access"
            return f"{node_type_name} {test_type} Test - {attribute.get_name()} value error"


        self.assertTrue(test_node.get_attribute_exists("inputs:exec"))
        attribute = test_node.get_attribute("inputs:exec")
        db_value = database.inputs.exec

        self.assertTrue(test_node.get_attribute_exists("inputs:instanceMapPtr"))
        attribute = test_node.get_attribute("inputs:instanceMapPtr")
        db_value = database.inputs.instanceMapPtr
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:instancePrimPathPtr"))
        attribute = test_node.get_attribute("inputs:instancePrimPathPtr")
        db_value = database.inputs.instancePrimPathPtr
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:minInstanceIndex"))
        attribute = test_node.get_attribute("inputs:minInstanceIndex")
        db_value = database.inputs.minInstanceIndex
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:minSemanticIndex"))
        attribute = test_node.get_attribute("inputs:minSemanticIndex")
        db_value = database.inputs.minSemanticIndex
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:numInstances"))
        attribute = test_node.get_attribute("inputs:numInstances")
        db_value = database.inputs.numInstances
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:numSemantics"))
        attribute = test_node.get_attribute("inputs:numSemantics")
        db_value = database.inputs.numSemantics
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:semanticLabelTokenPtrs"))
        attribute = test_node.get_attribute("inputs:semanticLabelTokenPtrs")
        db_value = database.inputs.semanticLabelTokenPtrs
        expected_value = []
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:semanticLocalTransformPtr"))
        attribute = test_node.get_attribute("inputs:semanticLocalTransformPtr")
        db_value = database.inputs.semanticLocalTransformPtr
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:semanticMapPtr"))
        attribute = test_node.get_attribute("inputs:semanticMapPtr")
        db_value = database.inputs.semanticMapPtr
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:semanticPrimPathPtr"))
        attribute = test_node.get_attribute("inputs:semanticPrimPathPtr")
        db_value = database.inputs.semanticPrimPathPtr
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:semanticWorldTransformPtr"))
        attribute = test_node.get_attribute("inputs:semanticWorldTransformPtr")
        db_value = database.inputs.semanticWorldTransformPtr
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:stage"))
        attribute = test_node.get_attribute("inputs:stage")
        db_value = database.inputs.stage
        expected_value = ""
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

        self.assertTrue(test_node.get_attribute_exists("inputs:testCaseIndex"))
        attribute = test_node.get_attribute("inputs:testCaseIndex")
        db_value = database.inputs.testCaseIndex
        expected_value = -1
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("outputs:exec"))
        attribute = test_node.get_attribute("outputs:exec")
        db_value = database.outputs.exec

        self.assertTrue(test_node.get_attribute_exists("outputs:semanticFilterPredicate"))
        attribute = test_node.get_attribute("outputs:semanticFilterPredicate")
        db_value = database.outputs.semanticFilterPredicate

        self.assertTrue(test_node.get_attribute_exists("outputs:success"))
        attribute = test_node.get_attribute("outputs:success")
        db_value = database.outputs.success
