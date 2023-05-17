import omni.kit.test
import omni.graph.core as og
import omni.graph.core.tests as ogts
import os


class TestOgn(ogts.OmniGraphTestCase):

    async def test_data_access(self):
        from omni.syntheticdata.ogn.OgnSdPostInstanceMappingDatabase import OgnSdPostInstanceMappingDatabase
        test_file_name = "OgnSdPostInstanceMappingTemplate.usda"
        usd_path = os.path.join(os.path.dirname(__file__), "usd", test_file_name)
        if not os.path.exists(usd_path):
            self.assertTrue(False, f"{usd_path} not found for loading test")
        (result, error) = await ogts.load_test_file(usd_path)
        self.assertTrue(result, f'{error} on {usd_path}')
        test_node = og.Controller.node("/TestGraph/Template_omni_syntheticdata_SdPostInstanceMapping")
        database = OgnSdPostInstanceMappingDatabase(test_node)
        self.assertTrue(test_node.is_valid())
        node_type_name = test_node.get_type_name()
        self.assertEqual(og.GraphRegistry().get_node_type_version(node_type_name), 2)

        def _attr_error(attribute: og.Attribute, usd_test: bool) -> str:
            test_type = "USD Load" if usd_test else "Database Access"
            return f"{node_type_name} {test_type} Test - {attribute.get_name()} value error"


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

        self.assertTrue(test_node.get_attribute_exists("inputs:rp"))
        attribute = test_node.get_attribute("inputs:rp")
        db_value = database.inputs.rp
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:semanticFilterName"))
        attribute = test_node.get_attribute("inputs:semanticFilterName")
        db_value = database.inputs.semanticFilterName
        expected_value = "default"
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

        self.assertTrue(test_node.get_attribute_exists("outputs:instanceMapSDCudaPtr"))
        attribute = test_node.get_attribute("outputs:instanceMapSDCudaPtr")
        db_value = database.outputs.instanceMapSDCudaPtr

        self.assertTrue(test_node.get_attribute_exists("outputs:instanceMappingInfoSDPtr"))
        attribute = test_node.get_attribute("outputs:instanceMappingInfoSDPtr")
        db_value = database.outputs.instanceMappingInfoSDPtr

        self.assertTrue(test_node.get_attribute_exists("outputs:instancePrimTokenSDCudaPtr"))
        attribute = test_node.get_attribute("outputs:instancePrimTokenSDCudaPtr")
        db_value = database.outputs.instancePrimTokenSDCudaPtr

        self.assertTrue(test_node.get_attribute_exists("outputs:semanticLabelTokenSDCudaPtr"))
        attribute = test_node.get_attribute("outputs:semanticLabelTokenSDCudaPtr")
        db_value = database.outputs.semanticLabelTokenSDCudaPtr

        self.assertTrue(test_node.get_attribute_exists("outputs:semanticLocalTransformSDCudaPtr"))
        attribute = test_node.get_attribute("outputs:semanticLocalTransformSDCudaPtr")
        db_value = database.outputs.semanticLocalTransformSDCudaPtr

        self.assertTrue(test_node.get_attribute_exists("outputs:semanticMapSDCudaPtr"))
        attribute = test_node.get_attribute("outputs:semanticMapSDCudaPtr")
        db_value = database.outputs.semanticMapSDCudaPtr

        self.assertTrue(test_node.get_attribute_exists("outputs:semanticPrimTokenSDCudaPtr"))
        attribute = test_node.get_attribute("outputs:semanticPrimTokenSDCudaPtr")
        db_value = database.outputs.semanticPrimTokenSDCudaPtr

        self.assertTrue(test_node.get_attribute_exists("outputs:semanticWorldTransformSDCudaPtr"))
        attribute = test_node.get_attribute("outputs:semanticWorldTransformSDCudaPtr")
        db_value = database.outputs.semanticWorldTransformSDCudaPtr
