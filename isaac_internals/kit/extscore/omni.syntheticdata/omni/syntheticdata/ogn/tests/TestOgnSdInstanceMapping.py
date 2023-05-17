import omni.kit.test
import omni.graph.core as og
import omni.graph.core.tests as ogts
import os


class TestOgn(ogts.OmniGraphTestCase):

    async def test_data_access(self):
        from omni.syntheticdata.ogn.OgnSdInstanceMappingDatabase import OgnSdInstanceMappingDatabase
        test_file_name = "OgnSdInstanceMappingTemplate.usda"
        usd_path = os.path.join(os.path.dirname(__file__), "usd", test_file_name)
        if not os.path.exists(usd_path):
            self.assertTrue(False, f"{usd_path} not found for loading test")
        (result, error) = await ogts.load_test_file(usd_path)
        self.assertTrue(result, f'{error} on {usd_path}')
        test_node = og.Controller.node("/TestGraph/Template_omni_syntheticdata_SdInstanceMapping")
        database = OgnSdInstanceMappingDatabase(test_node)
        self.assertTrue(test_node.is_valid())
        node_type_name = test_node.get_type_name()
        self.assertEqual(og.GraphRegistry().get_node_type_version(node_type_name), 1)

        def _attr_error(attribute: og.Attribute, usd_test: bool) -> str:
            test_type = "USD Load" if usd_test else "Database Access"
            return f"{node_type_name} {test_type} Test - {attribute.get_name()} value error"


        self.assertTrue(test_node.get_attribute_exists("inputs:exec"))
        attribute = test_node.get_attribute("inputs:exec")
        db_value = database.inputs.exec

        self.assertTrue(test_node.get_attribute_exists("inputs:lazy"))
        attribute = test_node.get_attribute("inputs:lazy")
        db_value = database.inputs.lazy
        expected_value = True
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:renderResults"))
        attribute = test_node.get_attribute("inputs:renderResults")
        db_value = database.inputs.renderResults
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

        self.assertTrue(test_node.get_attribute_exists("outputs:exec"))
        attribute = test_node.get_attribute("outputs:exec")
        db_value = database.outputs.exec

        self.assertTrue(test_node.get_attribute_exists("outputs:sdIMInstanceSemanticMap"))
        attribute = test_node.get_attribute("outputs:sdIMInstanceSemanticMap")
        db_value = database.outputs.sdIMInstanceSemanticMap

        self.assertTrue(test_node.get_attribute_exists("outputs:sdIMInstanceTokens"))
        attribute = test_node.get_attribute("outputs:sdIMInstanceTokens")
        db_value = database.outputs.sdIMInstanceTokens

        self.assertTrue(test_node.get_attribute_exists("outputs:sdIMMaxSemanticHierarchyDepth"))
        attribute = test_node.get_attribute("outputs:sdIMMaxSemanticHierarchyDepth")
        db_value = database.outputs.sdIMMaxSemanticHierarchyDepth

        self.assertTrue(test_node.get_attribute_exists("outputs:sdIMMinInstanceIndex"))
        attribute = test_node.get_attribute("outputs:sdIMMinInstanceIndex")
        db_value = database.outputs.sdIMMinInstanceIndex

        self.assertTrue(test_node.get_attribute_exists("outputs:sdIMMinSemanticIndex"))
        attribute = test_node.get_attribute("outputs:sdIMMinSemanticIndex")
        db_value = database.outputs.sdIMMinSemanticIndex

        self.assertTrue(test_node.get_attribute_exists("outputs:sdIMNumInstances"))
        attribute = test_node.get_attribute("outputs:sdIMNumInstances")
        db_value = database.outputs.sdIMNumInstances

        self.assertTrue(test_node.get_attribute_exists("outputs:sdIMNumSemanticTokens"))
        attribute = test_node.get_attribute("outputs:sdIMNumSemanticTokens")
        db_value = database.outputs.sdIMNumSemanticTokens

        self.assertTrue(test_node.get_attribute_exists("outputs:sdIMNumSemantics"))
        attribute = test_node.get_attribute("outputs:sdIMNumSemantics")
        db_value = database.outputs.sdIMNumSemantics

        self.assertTrue(test_node.get_attribute_exists("outputs:sdIMSemanticLocalTransform"))
        attribute = test_node.get_attribute("outputs:sdIMSemanticLocalTransform")
        db_value = database.outputs.sdIMSemanticLocalTransform

        self.assertTrue(test_node.get_attribute_exists("outputs:sdIMSemanticTokenMap"))
        attribute = test_node.get_attribute("outputs:sdIMSemanticTokenMap")
        db_value = database.outputs.sdIMSemanticTokenMap

        self.assertTrue(test_node.get_attribute_exists("outputs:sdIMSemanticWorldTransform"))
        attribute = test_node.get_attribute("outputs:sdIMSemanticWorldTransform")
        db_value = database.outputs.sdIMSemanticWorldTransform

        self.assertTrue(test_node.get_attribute_exists("outputs:swhFrameNumber"))
        attribute = test_node.get_attribute("outputs:swhFrameNumber")
        db_value = database.outputs.swhFrameNumber
