import omni.kit.test
import omni.graph.core as og
import omni.graph.core.tests as ogts
import os


class TestOgn(ogts.OmniGraphTestCase):

    async def test_data_access(self):
        from omni.syntheticdata.ogn.OgnSdPostSemantic3dBoundingBoxCameraProjectionDatabase import OgnSdPostSemantic3dBoundingBoxCameraProjectionDatabase
        test_file_name = "OgnSdPostSemantic3dBoundingBoxCameraProjectionTemplate.usda"
        usd_path = os.path.join(os.path.dirname(__file__), "usd", test_file_name)
        if not os.path.exists(usd_path):
            self.assertTrue(False, f"{usd_path} not found for loading test")
        (result, error) = await ogts.load_test_file(usd_path)
        self.assertTrue(result, f'{error} on {usd_path}')
        test_node = og.Controller.node("/TestGraph/Template_omni_syntheticdata_SdPostSemantic3dBoundingBoxCameraProjection")
        database = OgnSdPostSemantic3dBoundingBoxCameraProjectionDatabase(test_node)
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

        self.assertTrue(test_node.get_attribute_exists("inputs:renderProductResolution"))
        attribute = test_node.get_attribute("inputs:renderProductResolution")
        db_value = database.inputs.renderProductResolution
        expected_value = [0, 0]
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

        self.assertTrue(test_node.get_attribute_exists("inputs:semanticWorldTransformSDCudaPtr"))
        attribute = test_node.get_attribute("inputs:semanticWorldTransformSDCudaPtr")
        db_value = database.inputs.semanticWorldTransformSDCudaPtr
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:viewportNearFar"))
        attribute = test_node.get_attribute("inputs:viewportNearFar")
        db_value = database.inputs.viewportNearFar
        expected_value = [1.0, 10000000.0]
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:viewportResolution"))
        attribute = test_node.get_attribute("inputs:viewportResolution")
        db_value = database.inputs.viewportResolution
        expected_value = [65536, 65536]
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("outputs:exec"))
        attribute = test_node.get_attribute("outputs:exec")
        db_value = database.outputs.exec

        self.assertTrue(test_node.get_attribute_exists("outputs:sdSemBBox3dCamCornersCudaPtr"))
        attribute = test_node.get_attribute("outputs:sdSemBBox3dCamCornersCudaPtr")
        db_value = database.outputs.sdSemBBox3dCamCornersCudaPtr

        self.assertTrue(test_node.get_attribute_exists("outputs:sdSemBBox3dCamExtentCudaPtr"))
        attribute = test_node.get_attribute("outputs:sdSemBBox3dCamExtentCudaPtr")
        db_value = database.outputs.sdSemBBox3dCamExtentCudaPtr
