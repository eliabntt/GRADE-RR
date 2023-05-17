import omni.kit.test
import omni.graph.core as og
import omni.graph.core.tests as ogts
import os


class TestOgn(ogts.OmniGraphTestCase):

    async def test_data_access(self):
        from omni.syntheticdata.ogn.OgnSdPostSemantic3dBoundingBoxFilterDatabase import OgnSdPostSemantic3dBoundingBoxFilterDatabase
        test_file_name = "OgnSdPostSemantic3dBoundingBoxFilterTemplate.usda"
        usd_path = os.path.join(os.path.dirname(__file__), "usd", test_file_name)
        if not os.path.exists(usd_path):
            self.assertTrue(False, f"{usd_path} not found for loading test")
        (result, error) = await ogts.load_test_file(usd_path)
        self.assertTrue(result, f'{error} on {usd_path}')
        test_node = og.Controller.node("/TestGraph/Template_omni_syntheticdata_SdPostSemantic3dBoundingBoxFilter")
        database = OgnSdPostSemantic3dBoundingBoxFilterDatabase(test_node)
        self.assertTrue(test_node.is_valid())
        node_type_name = test_node.get_type_name()
        self.assertEqual(og.GraphRegistry().get_node_type_version(node_type_name), 1)

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
        expected_value = 0.01
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

        self.assertTrue(test_node.get_attribute_exists("inputs:sdSemBBox3dCamCornersCudaPtr"))
        attribute = test_node.get_attribute("inputs:sdSemBBox3dCamCornersCudaPtr")
        db_value = database.inputs.sdSemBBox3dCamCornersCudaPtr
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

        self.assertTrue(test_node.get_attribute_exists("inputs:viewportNearFar"))
        attribute = test_node.get_attribute("inputs:viewportNearFar")
        db_value = database.inputs.viewportNearFar
        expected_value = [0.0, -1.0]
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("outputs:exec"))
        attribute = test_node.get_attribute("outputs:exec")
        db_value = database.outputs.exec

        self.assertTrue(test_node.get_attribute_exists("outputs:sdSemBBoxInfosCudaPtr"))
        attribute = test_node.get_attribute("outputs:sdSemBBoxInfosCudaPtr")
        db_value = database.outputs.sdSemBBoxInfosCudaPtr
