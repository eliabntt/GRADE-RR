import carb

from pxr import Gf, UsdGeom, UsdLux, Sdf

import omni.hydratexture
import omni.kit.test
from omni.syntheticdata import SyntheticData, SyntheticDataStage


# Test the instance mapping pipeline  
class TestGraphManipulation(omni.kit.test.AsyncTestCase):
    
    def __init__(self, methodName: str) -> None:
        super().__init__(methodName=methodName)

    def render_product_path(self, hydra_texture) -> str:
        '''Return a string to the UsdRender.Product used by the texture'''
        render_product = hydra_texture.get_render_product_path()
        if render_product and (not render_product.startswith('/')):
            render_product = '/Render/RenderProduct_' + render_product
        return render_product    
 
    async def setUp(self):
        
        self._settings = carb.settings.acquire_settings_interface()
        self._hydra_texture_factory = omni.hydratexture.acquire_hydra_texture_factory_interface()

        self._usd_context_name = ''
        self._usd_context = omni.usd.get_context(self._usd_context_name)
        await self._usd_context.new_stage_async()
        self._stage = omni.usd.get_context().get_stage()
        
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
        self._render_product_path_0 = self.render_product_path(self._hydra_texture_0)
        
        self._hydra_texture_rendered_counter = 0
        def on_hydra_texture_0(event: carb.events.IEvent):
            self._hydra_texture_rendered_counter += 1
        self._hydra_texture_rendered_counter_sub = self._hydra_texture_0.get_event_stream().create_subscription_to_push_by_type(
            omni.hydratexture.EVENT_TYPE_DRAWABLE_CHANGED,
            on_hydra_texture_0,
            name='async rendering test drawable update',
        )
        
        
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

    async def test_rendervar_enable(self):
        isdg = SyntheticData.Get()
        render_var = "BoundingBox3DSD"
        assert(not isdg.is_rendervar_enabled(self._render_product_path_0, render_var, False, self._stage))
        assert(not isdg.is_rendervar_used(self._render_product_path_0, render_var))
        isdg.enable_rendervar(self._render_product_path_0, render_var, self._stage)
        assert(isdg.is_rendervar_enabled(self._render_product_path_0, render_var, False, self._stage))
        assert(not isdg.is_rendervar_used(self._render_product_path_0, render_var))
        isdg.disable_rendervar(self._render_product_path_0, render_var, self._stage)
        assert(not isdg.is_rendervar_enabled(self._render_product_path_0, render_var, False, self._stage))
        assert(not isdg.is_rendervar_used(self._render_product_path_0, render_var))

    async def test_rendervar_auto_activation(self):
        isdg = SyntheticData.Get()
        render_var = "BoundingBox3DSD"
        assert(not isdg.is_rendervar_enabled(self._render_product_path_0, render_var, True, self._stage))
        assert(not isdg.is_rendervar_used(self._render_product_path_0, render_var))
        isdg.activate_node_template("BoundingBox3DReduction",0, [self._render_product_path_0], {}, self._stage, True)
        assert(isdg.is_rendervar_enabled(self._render_product_path_0, render_var, False, self._stage))
        assert(isdg.is_rendervar_used(self._render_product_path_0, render_var))
        assert(isdg.is_rendervar_enabled(self._render_product_path_0, render_var, True, self._stage))
        isdg.deactivate_node_template("BoundingBox3DReduction",0, [self._render_product_path_0], self._stage, True)
        assert(not isdg.is_rendervar_enabled(self._render_product_path_0, render_var, False, self._stage))
        assert(not isdg.is_rendervar_used(self._render_product_path_0, render_var))
    
    async def test_rendervar_manual_activation(self):
        isdg = SyntheticData.Get()
        render_var = "BoundingBox3DSD"
        assert(not isdg.is_rendervar_enabled(self._render_product_path_0, render_var, False, self._stage))
        assert(not isdg.is_rendervar_used(self._render_product_path_0, render_var))
        assert(not isdg.is_node_template_activated("BoundingBox3DReduction",self._render_product_path_0,False))
        isdg.activate_node_template("BoundingBox3DReduction",0, [self._render_product_path_0], {}, self._stage, False)
        assert(isdg.is_node_template_activated("BoundingBox3DReduction",self._render_product_path_0,False))
        assert(isdg.is_node_template_activated("BoundingBox3DReduction",self._render_product_path_0,True))
        assert(not isdg.is_rendervar_enabled(self._render_product_path_0, render_var, False, self._stage))
        assert(isdg.is_rendervar_used(self._render_product_path_0, render_var))
        isdg.enable_rendervar(self._render_product_path_0, render_var, self._stage)
        assert(isdg.is_rendervar_enabled(self._render_product_path_0, render_var, False, self._stage))
        assert(isdg.is_rendervar_used(self._render_product_path_0, render_var))
        assert(not isdg.is_rendervar_enabled(self._render_product_path_0, render_var, True, self._stage))
        isdg.deactivate_node_template("BoundingBox3DReduction",0, [self._render_product_path_0], self._stage, False)
        assert(not isdg.is_node_template_activated("BoundingBox3DReduction",self._render_product_path_0,True))
        assert(isdg.is_rendervar_enabled(self._render_product_path_0, render_var, False, self._stage))
        assert(not isdg.is_rendervar_used(self._render_product_path_0, render_var))
        isdg.disable_rendervar(self._render_product_path_0, render_var, self._stage)
        assert(not isdg.is_rendervar_enabled(self._render_product_path_0, render_var, False, self._stage))
        assert(not isdg.is_rendervar_used(self._render_product_path_0, render_var))

    async def test_rendervar_hybrid_activation(self):
        isdg = SyntheticData.Get()
        render_var = "BoundingBox3DSD"
        assert(not isdg.is_rendervar_enabled(self._render_product_path_0, render_var, False, self._stage))
        assert(not isdg.is_rendervar_used(self._render_product_path_0, render_var))
        isdg.activate_node_template("BoundingBox3DReduction",0, [self._render_product_path_0], {}, self._stage, False)
        assert(not isdg.is_rendervar_enabled(self._render_product_path_0, render_var, False, self._stage))
        assert(isdg.is_rendervar_used(self._render_product_path_0, render_var))
        isdg.enable_rendervar(self._render_product_path_0, render_var, self._stage)
        assert(isdg.is_rendervar_enabled(self._render_product_path_0, render_var, False, self._stage))
        assert(isdg.is_rendervar_used(self._render_product_path_0, render_var))
        isdg.deactivate_node_template("BoundingBox3DReduction",0, [self._render_product_path_0], self._stage, True)
        assert(isdg.is_rendervar_enabled(self._render_product_path_0, render_var, False, self._stage))
        assert(not isdg.is_rendervar_used(self._render_product_path_0, render_var))
        assert(not isdg.is_rendervar_enabled(self._render_product_path_0, render_var, True, self._stage))
        isdg.disable_rendervar(self._render_product_path_0, render_var, self._stage)
        assert(not isdg.is_rendervar_enabled(self._render_product_path_0, render_var, False, self._stage))
        
    async def test_rendervar_initially_activated(self):
        isdg = SyntheticData.Get()
        render_var = "BoundingBox3DSD"
        assert(not isdg.is_rendervar_enabled(self._render_product_path_0, render_var, False, self._stage))
        assert(not isdg.is_rendervar_used(self._render_product_path_0, render_var))
        isdg.enable_rendervar(self._render_product_path_0, render_var, self._stage)
        assert(isdg.is_rendervar_enabled(self._render_product_path_0, render_var, False, self._stage))
        assert(not isdg.is_rendervar_used(self._render_product_path_0, render_var))
        isdg.activate_node_template("BoundingBox3DReduction",0, [self._render_product_path_0], {}, self._stage, True)
        assert(isdg.is_rendervar_enabled(self._render_product_path_0, render_var, False, self._stage))
        assert(isdg.is_rendervar_used(self._render_product_path_0, render_var))
        assert(not isdg.is_rendervar_enabled(self._render_product_path_0, render_var, True, self._stage))
        isdg.deactivate_node_template("BoundingBox3DReduction",0, [self._render_product_path_0], self._stage, True)
        assert(isdg.is_rendervar_enabled(self._render_product_path_0, render_var, False, self._stage))
        assert(not isdg.is_rendervar_used(self._render_product_path_0, render_var))
        isdg.disable_rendervar(self._render_product_path_0, render_var, self._stage)
        assert(not isdg.is_rendervar_enabled(self._render_product_path_0, render_var, False, self._stage))
        
    async def test_rendervar_multiple_activation(self):
        isdg = SyntheticData.Get()
        render_var = "BoundingBox3DSD"
        if not isdg.is_node_template_registered("BoundingBox3DDisplayPostDuplicate"):
            isdg.register_node_template( 
                SyntheticData.NodeTemplate(
                    SyntheticDataStage.POST_RENDER,
                    "omni.syntheticdata.SdPostRenderVarDisplayTexture",
                    [
                        SyntheticData.NodeConnectionTemplate("LdrColorSD"),
                        SyntheticData.NodeConnectionTemplate("Camera3dPositionSD"),
                        SyntheticData.NodeConnectionTemplate("PostRenderProductCamera"),
                        SyntheticData.NodeConnectionTemplate("InstanceMappingPost"),
                        SyntheticData.NodeConnectionTemplate("BoundingBox3DReduction")
                    ],
                    {
                        "inputs:renderVar": "LdrColorSD",
                        "inputs:renderVarDisplay": "BoundingBox3DSDDisplay",
                        "inputs:mode": "semanticBoundingBox3dMode",
                        "inputs:parameters": [0.0, 5.0, 0.027, 0.27]
                    }
                ), # node template default attribute values (when differs from the default value specified in the .ogn)
                template_name="BoundingBox3DDisplayPostDuplicate" # node template name 
            )
        assert(not isdg.is_rendervar_enabled(self._render_product_path_0, render_var, False, self._stage))
        assert(not isdg.is_rendervar_used(self._render_product_path_0, render_var))
        isdg.activate_node_template("BoundingBox3DDisplayPost",0, [self._render_product_path_0], {}, self._stage, True)
        assert(not isdg.is_node_template_activated("BoundingBox3DReduction",self._render_product_path_0,True))
        assert(isdg.is_node_template_activated("BoundingBox3DReduction",self._render_product_path_0,False))
        assert(isdg.is_rendervar_enabled(self._render_product_path_0, render_var, True, self._stage))
        assert(isdg.is_rendervar_used(self._render_product_path_0, render_var))
        isdg.activate_node_template("BoundingBox3DDisplayPostDuplicate",0, [self._render_product_path_0], {}, self._stage, True)
        isdg.deactivate_node_template("BoundingBox3DDisplayPost",0, [self._render_product_path_0], self._stage, True)
        assert(isdg.is_rendervar_enabled(self._render_product_path_0, render_var, True, self._stage))
        assert(isdg.is_rendervar_used(self._render_product_path_0, render_var))
        assert(not isdg.is_node_template_activated("BoundingBox3DReduction",self._render_product_path_0,True))
        assert(isdg.is_node_template_activated("BoundingBox3DReduction",self._render_product_path_0,False))
        isdg.deactivate_node_template("BoundingBox3DDisplayPostDuplicate",0, [self._render_product_path_0], self._stage, True)
        assert(not isdg.is_node_template_activated("BoundingBox3DReduction",self._render_product_path_0,False))
        assert(not isdg.is_rendervar_enabled(self._render_product_path_0, render_var, True, self._stage))
        assert(not isdg.is_rendervar_used(self._render_product_path_0, render_var))
        