# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import os
import carb
from pxr import Gf, UsdGeom, UsdLux, Sdf
import unittest
import omni.kit.test
from omni.syntheticdata import SyntheticData, SyntheticDataStage

from ..utils import add_semantics

class TestPostVisualization(omni.kit.test.AsyncTestCase):
    
    def __init__(self, methodName: str) -> None:
        super().__init__(methodName=methodName)
    
    def activate_post_vis(self,render_product_path, render_var):
        sdg_iface = SyntheticData.Get()
        render_var_post_display = "Test" + render_var + "PostDisplay"
        if not sdg_iface.is_node_template_registered(render_var_post_display):
            sdg_iface.register_node_template( 
                SyntheticData.NodeTemplate(
                    SyntheticDataStage.ON_DEMAND, # node tempalte stage 
                    "omni.syntheticdata.SdLinearArrayToTexture", # node template type
                    # node template connections
                    [
                        SyntheticData.NodeConnectionTemplate(render_var),     
                    ]),
                template_name=render_var_post_display 
            )

        sdg_iface.activate_node_template(render_var_post_display, 0, [render_product_path])


    async def setUp(self):
        
        # Setup the scene
        await omni.usd.get_context().new_stage_async()
        stage = omni.usd.get_context().get_stage()
        world_prim = UsdGeom.Xform.Define(stage,"/World")
        UsdGeom.Xformable(world_prim).AddTranslateOp().Set((0, 0, 0))
        UsdGeom.Xformable(world_prim).AddRotateXYZOp().Set((0, 0, 0))
            
        capsule0_prim = stage.DefinePrim("/World/Capsule0", "Capsule")
        add_semantics(capsule0_prim, "capsule_0")
        UsdGeom.Xformable(capsule0_prim).AddTranslateOp().Set((100, 0, 0))
        UsdGeom.Xformable(capsule0_prim).AddScaleOp().Set((30, 30, 30))
        UsdGeom.Xformable(capsule0_prim).AddRotateXYZOp().Set((-90, 0, 0))
        capsule0_prim.GetAttribute("primvars:displayColor").Set([(0.3, 1, 0)])

        capsule1_prim = stage.DefinePrim("/World/Capsule1", "Capsule")
        add_semantics(capsule0_prim, "capsule_1")
        UsdGeom.Xformable(capsule1_prim).AddTranslateOp().Set((-100, 0, 0))
        UsdGeom.Xformable(capsule1_prim).AddScaleOp().Set((30, 30, 30))
        UsdGeom.Xformable(capsule1_prim).AddRotateXYZOp().Set((-90, 0, 0))
        capsule1_prim.GetAttribute("primvars:displayColor").Set([(0, 1, 0.3)])

        spherelight = UsdLux.SphereLight.Define(stage, "/SphereLight")
        spherelight.GetIntensityAttr().Set(30000)
        spherelight.GetRadiusAttr().Set(30)
            
        # Setup viewports / renderproduct
        vp_iface = omni.kit.viewport_legacy.get_viewport_interface()    
        viewport = vp_iface.get_viewport_window()
        render_product_path = viewport.get_render_product_path()
        
        self.activate_post_vis("LdrColorSD")

        self.numLoops = 100


    async def run_loop(self):
        
        # ensuring that the setup is taken into account
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

        for _ in range(self.numLoops):    
            await omni.kit.app.get_app().next_update_async()
        
        
    async def test_display(self):
        """ Test display
        """
        await self.run_loop()

    async def tearDown(self):
        pass
