# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import os
import carb
from pxr import Gf, UsdGeom, Sdf, UsdLux
from omni.kit.viewport.utility import get_active_viewport, create_viewport_window
import omni.kit.test

from omni.syntheticdata import SyntheticData, SyntheticDataStage

# Test the RenderProductCamera nodes  
class TestRenderProductCamera(omni.kit.test.AsyncTestCase):
    
    def __init__(self, methodName: str) -> None:
        super().__init__(methodName=methodName)
        
    async def setUp(self):

        settings = carb.settings.get_settings()
        settings.set_string("/rtx/rendermode", "PathTracing")
        settings.set_int("/rtx/pathtracing/spp", 32)
        settings.set_int("/persistent/app/viewport/displayOptions", 0)

        self.numLoops = 7
        self.multiViewport = False

        # Setup the scene
        await omni.usd.get_context().new_stage_async()
        stage = omni.usd.get_context().get_stage()

        # Setup viewports / renderproduct
        # first default viewport with the default perspective camera
        viewport_0 = get_active_viewport()
        resolution_0 = viewport_0.resolution
        camera_0 = UsdGeom.Camera.Define(stage, "/Camera0").GetPrim()
        viewport_0.camera_path = camera_0.GetPath()
        render_product_path_0 = viewport_0.render_product_path
        self.render_product_path_0 = render_product_path_0

        # second viewport with a ftheta camera
        if self.multiViewport:
            resolution_1 = (512, 512)
            viewport_window = create_viewport_window(width=resolution_1[0], height=resolution_1[1])
            viewport_1 = viewport_window.viewport_api
            viewport_1.resolution = resolution_1
            camera_1 = UsdGeom.Camera.Define(stage, "/Camera1").GetPrim()
            viewport_1.camera_path = camera_1.GetPath()
            render_product_path_1 = viewport_1.render_product_path
            self.render_product_path_1 = render_product_path_1 

        # SyntheticData singleton interface
        sdg_iface = SyntheticData.Get()

        if not sdg_iface.is_node_template_registered("TestSimRpCam"):
                sdg_iface.register_node_template( 
                    SyntheticData.NodeTemplate(
                        SyntheticDataStage.SIMULATION, 
                        "omni.syntheticdata.SdTestRenderProductCamera",
                        attributes={"inputs:stage":"simulation"}
                    ),
                    template_name="TestSimRpCam"  
                )
        if not sdg_iface.is_node_template_registered("TestPostRpCam"):
            sdg_iface.register_node_template( 
                SyntheticData.NodeTemplate(
                    SyntheticDataStage.POST_RENDER, 
                    "omni.syntheticdata.SdTestRenderProductCamera", 
                    [SyntheticData.NodeConnectionTemplate("PostRenderProductCamera")],
                    attributes={"inputs:stage":"postRender"}
                ), 
                template_name="TestPostRpCam"
            )
        if not sdg_iface.is_node_template_registered("TestOnDemandRpCam"):
            sdg_iface.register_node_template( 
                SyntheticData.NodeTemplate(
                    SyntheticDataStage.ON_DEMAND, 
                    "omni.syntheticdata.SdTestRenderProductCamera",
                    [SyntheticData.NodeConnectionTemplate("PostProcessRenderProductCamera")],
                    attributes={"inputs:stage":"onDemand"}
                ), 
                template_name="TestOnDemandRpCam"
            )

        attributes_0 = {
            "inputs:renderProductCameraPath":camera_0.GetPath().pathString,
            "inputs:width":resolution_0[0],
            "inputs:height":resolution_0[1]
        }
        sdg_iface.activate_node_template("TestSimRpCam", 0, [render_product_path_0], attributes_0)
        sdg_iface.activate_node_template("TestPostRpCam", 0, [render_product_path_0], attributes_0)
        sdg_iface.activate_node_template("TestOnDemandRpCam", 0, [render_product_path_0],attributes_0)

        if self.multiViewport: 
            attributes_1 = {
                "inputs:renderProductCameraPath":camera_1.GetPath().pathString,
                "inputs:width":resolution_1[0],
                "inputs:height":resolution_1[1]
            }
            sdg_iface.activate_node_template("TestSimRpCam", 0, [render_product_path_1], attributes_1)
            sdg_iface.activate_node_template("TestPostRpCam", 0, [render_product_path_1], attributes_1)
            sdg_iface.activate_node_template("TestOnDemandRpCam", 0, [render_product_path_1],attributes_1)


    async def test_renderproduct_camera(self):
        """ Test render product camera pipeline
        """
        sdg_iface = SyntheticData.Get()

        test_outname = "outputs:test"
        test_attributes_names = [test_outname]

        for _ in range(3):
            await omni.kit.app.get_app().next_update_async()

        for _ in range(self.numLoops):
            await omni.kit.app.get_app().next_update_async()

            assert sdg_iface.get_node_attributes("TestSimRpCam", test_attributes_names, self.render_product_path_0)[test_outname]
            assert sdg_iface.get_node_attributes("TestPostRpCam", test_attributes_names, self.render_product_path_0)[test_outname]
            assert sdg_iface.get_node_attributes("TestOnDemandRpCam", test_attributes_names, self.render_product_path_0)[test_outname]
            if self.multiViewport:     
                assert sdg_iface.get_node_attributes("TestSimRpCam", test_attributes_names, self.render_product_path_1)[test_outname]
                assert sdg_iface.get_node_attributes("TestPostRpCam", test_attributes_names, self.render_product_path_1)[test_outname]
                assert sdg_iface.get_node_attributes("TestOnDemandRpCam", test_attributes_names, self.render_product_path_1)[test_outname]

    async def tearDown(self):
        pass
