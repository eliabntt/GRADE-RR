import carb

from pxr import Gf, UsdGeom, UsdLux, Sdf

import omni.hydratexture
import omni.kit.test
from omni.syntheticdata import SyntheticData, SyntheticDataStage


# Test the instance mapping pipeline  
class TestInstanceMapping(omni.kit.test.AsyncTestCase):
    
    def __init__(self, methodName: str) -> None:
        super().__init__(methodName=methodName)

    def render_product_path(self, hydra_texture) -> str:
        '''Return a string to the UsdRender.Product used by the texture'''
        render_product = hydra_texture.get_render_product_path()
        if render_product and (not render_product.startswith('/')):
            render_product = '/Render/RenderProduct_' + render_product
        return render_product    

    def register_test_instance_mapping_pipeline(self):
        sdg_iface = SyntheticData.Get()
        if not sdg_iface.is_node_template_registered("TestSimSWHFrameNumber"):
            sdg_iface.register_node_template( 
                    SyntheticData.NodeTemplate(
                        SyntheticDataStage.SIMULATION,  
                        "omni.syntheticdata.SdUpdateSwFrameNumber"
                    ), 
                    template_name="TestSimSWHFrameNumber" 
                )
        if not sdg_iface.is_node_template_registered("TestSimInstanceMapping"):
            sdg_iface.register_node_template( 
                SyntheticData.NodeTemplate(
                    SyntheticDataStage.SIMULATION, 
                    "omni.syntheticdata.SdTestInstanceMapping",
                    [
                        SyntheticData.NodeConnectionTemplate("TestSimSWHFrameNumber", ())
                    ],
                    {"inputs:stage":"simulation"}  
                ), 
                template_name="TestSimInstanceMapping" 
            )
        if not sdg_iface.is_node_template_registered("TestOnDemandInstanceMapping"):
            sdg_iface.register_node_template( 
                SyntheticData.NodeTemplate(
                    SyntheticDataStage.ON_DEMAND, 
                    "omni.syntheticdata.SdTestInstanceMapping",
                    [
                        SyntheticData.NodeConnectionTemplate("InstanceMappingPtrWithTransforms"),
                        SyntheticData.NodeConnectionTemplate("TestSimInstanceMapping", (), attributes_mapping={"outputs:exec": "inputs:exec"})
                    ],
                    {"inputs:stage":"ondemand"} 
                ), 
                template_name="TestOnDemandInstanceMapping" 
            )

    def activate_test_instance_mapping_pipeline(self, case_index):
        sdg_iface = SyntheticData.Get()
        sdg_iface.activate_node_template("TestSimInstanceMapping", attributes={"inputs:testCaseIndex":case_index})
        sdg_iface.activate_node_template("TestOnDemandInstanceMapping", 0, 
                                         [self.render_product_path(self._hydra_texture_0)],
                                         {"inputs:testCaseIndex":case_index})
        sdg_iface.connect_node_template("TestSimInstanceMapping", 
                                        "InstanceMappingPre", None, 
                                        {"outputs:semanticFilterPredicate":"inputs:semanticFilterPredicate"})    

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
        
        # renderer
        renderer = "rtx"
        if renderer not in self._usd_context.get_attached_hydra_engine_names():
            omni.usd.add_hydra_engine(renderer, self._usd_context)

        # create the hydra textures
        self._hydra_texture_0 = self._hydra_texture_factory.create_hydra_texture(
            "TEX0",
            1920,
            1080,
            self._usd_context_name,
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

        self.register_test_instance_mapping_pipeline()
        
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
        self.activate_test_instance_mapping_pipeline(0)
        await self.wait_for_num_frames(11)

    async def test_case_1(self):
        self.activate_test_instance_mapping_pipeline(1)
        await self.wait_for_num_frames(11)

    async def test_case_2(self):
        self.activate_test_instance_mapping_pipeline(2)
        await self.wait_for_num_frames(11)

    async def test_case_3(self):
        self.activate_test_instance_mapping_pipeline(3)
        await self.wait_for_num_frames(11)

    async def test_case_4(self):
        self.activate_test_instance_mapping_pipeline(4)
        await self.wait_for_num_frames(11)



        