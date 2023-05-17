import omni.kit.test
import omni.graph.core as og
import omni.graph.core.tests as ogts
import os


class TestOgn(ogts.OmniGraphTestCase):

    async def test_data_access(self):
        from omni.syntheticdata.ogn.OgnSdRenderProductCameraDatabase import OgnSdRenderProductCameraDatabase
        test_file_name = "OgnSdRenderProductCameraTemplate.usda"
        usd_path = os.path.join(os.path.dirname(__file__), "usd", test_file_name)
        if not os.path.exists(usd_path):
            self.assertTrue(False, f"{usd_path} not found for loading test")
        (result, error) = await ogts.load_test_file(usd_path)
        self.assertTrue(result, f'{error} on {usd_path}')
        test_node = og.Controller.node("/TestGraph/Template_omni_syntheticdata_SdRenderProductCamera")
        database = OgnSdRenderProductCameraDatabase(test_node)
        self.assertTrue(test_node.is_valid())
        node_type_name = test_node.get_type_name()
        self.assertEqual(og.GraphRegistry().get_node_type_version(node_type_name), 2)

        def _attr_error(attribute: og.Attribute, usd_test: bool) -> str:
            test_type = "USD Load" if usd_test else "Database Access"
            return f"{node_type_name} {test_type} Test - {attribute.get_name()} value error"


        self.assertTrue(test_node.get_attribute_exists("inputs:exec"))
        attribute = test_node.get_attribute("inputs:exec")
        db_value = database.inputs.exec

        self.assertTrue(test_node.get_attribute_exists("inputs:renderProductPath"))
        attribute = test_node.get_attribute("inputs:renderProductPath")
        db_value = database.inputs.renderProductPath
        expected_value = ""
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

        self.assertTrue(test_node.get_attribute_exists("outputs:cameraApertureOffset"))
        attribute = test_node.get_attribute("outputs:cameraApertureOffset")
        db_value = database.outputs.cameraApertureOffset

        self.assertTrue(test_node.get_attribute_exists("outputs:cameraApertureSize"))
        attribute = test_node.get_attribute("outputs:cameraApertureSize")
        db_value = database.outputs.cameraApertureSize

        self.assertTrue(test_node.get_attribute_exists("outputs:cameraFStop"))
        attribute = test_node.get_attribute("outputs:cameraFStop")
        db_value = database.outputs.cameraFStop

        self.assertTrue(test_node.get_attribute_exists("outputs:cameraFisheyeParams"))
        attribute = test_node.get_attribute("outputs:cameraFisheyeParams")
        db_value = database.outputs.cameraFisheyeParams

        self.assertTrue(test_node.get_attribute_exists("outputs:cameraFocalLength"))
        attribute = test_node.get_attribute("outputs:cameraFocalLength")
        db_value = database.outputs.cameraFocalLength

        self.assertTrue(test_node.get_attribute_exists("outputs:cameraFocusDistance"))
        attribute = test_node.get_attribute("outputs:cameraFocusDistance")
        db_value = database.outputs.cameraFocusDistance

        self.assertTrue(test_node.get_attribute_exists("outputs:cameraModel"))
        attribute = test_node.get_attribute("outputs:cameraModel")
        db_value = database.outputs.cameraModel

        self.assertTrue(test_node.get_attribute_exists("outputs:cameraNearFar"))
        attribute = test_node.get_attribute("outputs:cameraNearFar")
        db_value = database.outputs.cameraNearFar

        self.assertTrue(test_node.get_attribute_exists("outputs:cameraProjection"))
        attribute = test_node.get_attribute("outputs:cameraProjection")
        db_value = database.outputs.cameraProjection

        self.assertTrue(test_node.get_attribute_exists("outputs:cameraViewTransform"))
        attribute = test_node.get_attribute("outputs:cameraViewTransform")
        db_value = database.outputs.cameraViewTransform

        self.assertTrue(test_node.get_attribute_exists("outputs:exec"))
        attribute = test_node.get_attribute("outputs:exec")
        db_value = database.outputs.exec

        self.assertTrue(test_node.get_attribute_exists("outputs:metersPerSceneUnit"))
        attribute = test_node.get_attribute("outputs:metersPerSceneUnit")
        db_value = database.outputs.metersPerSceneUnit

        self.assertTrue(test_node.get_attribute_exists("outputs:renderProductResolution"))
        attribute = test_node.get_attribute("outputs:renderProductResolution")
        db_value = database.outputs.renderProductResolution

        self.assertTrue(test_node.get_attribute_exists("outputs:swhFrameNumber"))
        attribute = test_node.get_attribute("outputs:swhFrameNumber")
        db_value = database.outputs.swhFrameNumber
