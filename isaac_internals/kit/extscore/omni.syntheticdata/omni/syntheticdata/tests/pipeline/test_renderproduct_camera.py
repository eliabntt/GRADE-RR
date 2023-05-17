import carb

from pxr import Gf, UsdGeom, UsdLux, Sdf

import omni.hydratexture
import omni.kit.test
from omni.syntheticdata import SyntheticData, SyntheticDataStage


# Test the instance mapping pipeline  
class TestRenderProductCamera(omni.kit.test.AsyncTestCase):
    
    def __init__(self, methodName: str) -> None:
        super().__init__(methodName=methodName)

    def render_product_path(self, hydra_texture) -> str:
        '''Return a string to the UsdRender.Product used by the texture'''
        render_product = hydra_texture.get_render_product_path()
        if render_product and (not render_product.startswith('/')):
            render_product = '/Render/RenderProduct_' + render_product
        return render_product    

    def register_test_rp_cam_pipeline(self):
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

    def activate_test_rp_cam_pipeline(self, test_case_index):
        sdg_iface = SyntheticData.Get()
        attributes = {
            "inputs:renderProductCameraPath": self._camera_path,
            "inputs:width": self._resolution[0],
            "inputs:height": self._resolution[1],
            "inputs:traceError": True
        }
        sdg_iface.activate_node_template("TestSimRpCam", 0, [self.render_product_path(self._hydra_texture_0)], attributes)
        sdg_iface.activate_node_template("TestPostRpCam", 0, [self.render_product_path(self._hydra_texture_0)], attributes)
        sdg_iface.activate_node_template("TestOnDemandRpCam", 0, [self.render_product_path(self._hydra_texture_0)],attributes)

        
    async def wait_for_num_frames(self, num_frames, max_num_frames=5000):
        self._hydra_texture_rendered_counter = 0
        wait_frames_left = max_num_frames
        while (self._hydra_texture_rendered_counter < num_frames) and (wait_frames_left > 0):
            await omni.kit.app.get_app().next_update_async()
            wait_frames_left -= 1

    async def setUp(self):
        self._settings = carb.settings.acquire_settings_interface()
        self._hydra_texture_factory = omni.hydratexture.acquire_hydra_texture_factory_interface()

        self._usd_context_name = ''
        self._usd_context = omni.usd.get_context(self._usd_context_name)
        await self._usd_context.new_stage_async()
        
        # camera
        self._camera_path = "/TestRPCamera"
        UsdGeom.Camera.Define(omni.usd.get_context().get_stage(), self._camera_path)
        self._resolution = [980,540]
        
        # renderer
        renderer = "rtx"
        if renderer not in self._usd_context.get_attached_hydra_engine_names():
            omni.usd.add_hydra_engine(renderer, self._usd_context)

        # create the hydra textures
        self._hydra_texture_0 = self._hydra_texture_factory.create_hydra_texture(
            "TEX0",
            width=self._resolution[0],
            height=self._resolution[1],
            usd_context_name=self._usd_context_name,
            usd_camera_path=self._camera_path, 
            hydra_engine_name=renderer,
            is_async=self._settings.get("/app/asyncRendering")
        )
        
        self._hydra_texture_rendered_counter = 0
        def on_hydra_texture_0(event: carb.events.IEvent):
            self._hydra_texture_rendered_counter += 1
        self._hydra_texture_rendered_counter_sub = self._hydra_texture_0.get_event_stream().create_subscription_to_push_by_type(
            omni.hydratexture.EVENT_TYPE_DRAWABLE_CHANGED,
            on_hydra_texture_0,
            name='async rendering test drawable update',
        )

        self.register_test_rp_cam_pipeline()
        
    async def tearDown(self):
        self._hydra_texture_rendered_counter_sub = None
        self._hydra_texture_0 = None
        
        self._usd_context.close_stage()
        omni.usd.release_all_hydra_engines(self._usd_context)

        self._hydra_texture_factory = None
        self._settings = None

        wait_iterations = 6
        for _ in range(wait_iterations):
            await omni.kit.app.get_app().next_update_async()

    async def test_case_0(self):
        print("test actual camera pipeline here.")
        self.activate_test_rp_cam_pipeline(0)
        await self.wait_for_num_frames(33)
