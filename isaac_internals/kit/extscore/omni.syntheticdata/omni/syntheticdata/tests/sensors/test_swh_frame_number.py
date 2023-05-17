# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import os
import carb
from pxr import Gf, UsdGeom, UsdLux, Sdf
import unittest
import omni.kit.test
from omni.kit.viewport.utility import get_active_viewport, create_viewport_window
from omni.syntheticdata import SyntheticData, SyntheticDataStage

# Test the Fabric frame number synchronization  
class TestSWHFrameNumber(omni.kit.test.AsyncTestCase):
    
    def __init__(self, methodName: str) -> None:
        super().__init__(methodName=methodName)
        
    async def setUp(self):
        
        settings = carb.settings.get_settings()
        settings.set_string("/rtx/rendermode", "PathTracing")
        settings.set_int("/rtx/pathtracing/spp", 32)
        settings.set_int("/persistent/app/viewport/displayOptions", 0)

        # Setup the scene
        await omni.usd.get_context().new_stage_async()
        stage = omni.usd.get_context().get_stage()
        world_prim = UsdGeom.Xform.Define(stage,"/World")
        UsdGeom.Xformable(world_prim).AddTranslateOp().Set((0, 0, 0))
        UsdGeom.Xformable(world_prim).AddRotateXYZOp().Set((0, 0, 0))
            
        capsule0_prim = stage.DefinePrim("/World/Capsule0", "Capsule")
        UsdGeom.Xformable(capsule0_prim).AddTranslateOp().Set((100, 0, 0))
        UsdGeom.Xformable(capsule0_prim).AddScaleOp().Set((30, 30, 30))
        UsdGeom.Xformable(capsule0_prim).AddRotateXYZOp().Set((-90, 0, 0))
        capsule0_prim.GetAttribute("primvars:displayColor").Set([(0.3, 1, 0)])

        capsule1_prim = stage.DefinePrim("/World/Capsule1", "Capsule")
        UsdGeom.Xformable(capsule1_prim).AddTranslateOp().Set((-100, 0, 0))
        UsdGeom.Xformable(capsule1_prim).AddScaleOp().Set((30, 30, 30))
        UsdGeom.Xformable(capsule1_prim).AddRotateXYZOp().Set((-90, 0, 0))
        capsule1_prim.GetAttribute("primvars:displayColor").Set([(0, 1, 0.3)])

        spherelight = UsdLux.SphereLight.Define(stage, "/SphereLight")
        spherelight.GetIntensityAttr().Set(30000)
        spherelight.GetRadiusAttr().Set(30)

        # first default viewport with the default perspective camera
        viewport_0 = get_active_viewport()
        render_product_path_0 = viewport_0.render_product_path
        
        # second viewport with a ftheta camera 
        viewport_1_window = create_viewport_window(width=512, height=512)
        viewport_1 = viewport_1_window.viewport_api
        camera_1 = stage.DefinePrim("/Camera1", "Camera")
        camera_1.CreateAttribute("cameraProjectionType", Sdf.ValueTypeNames.Token).Set("fisheyePolynomial")
        UsdGeom.Xformable(camera_1).AddTranslateOp().Set((0, 250, 0))
        UsdGeom.Xformable(camera_1).AddRotateXYZOp().Set((-90, 0, 0))
        viewport_1.camera_path = camera_1.GetPath()
        render_product_path_1 = viewport_1.render_product_path

        # SyntheticData singleton interface
        sdg_iface = SyntheticData.Get()

        # Register node templates in the SyntheticData register 
        # (a node template is a template for creating a node specified by its type and its connections)
        # 
        # to illustrate we are using the generic omni.syntheticdata.SdTestStageSynchronization node type which supports every stage of the SyntheticData pipeline. When executed it logs the fabric frame number.
        #  
        # register a node template in the simulation stage
        # NB : this node template has no connections
        if not sdg_iface.is_node_template_registered("TestSyncSim"):
                sdg_iface.register_node_template( 
                    SyntheticData.NodeTemplate(
                        SyntheticDataStage.SIMULATION, # node tempalte stage 
                        "omni.syntheticdata.SdTestStageSynchronization", # node template type
                        attributes={"inputs:tag":"0"}), # node template default attribute values (when differs from the default value specified in the .ogn)
                    template_name="TestSyncSim" # node template name 
                )
        # register a node template in the postrender stage
        # NB : this template may be activated for several different renderproducts
        if not sdg_iface.is_node_template_registered("TestSyncPost"):
            sdg_iface.register_node_template( 
                SyntheticData.NodeTemplate(
                    SyntheticDataStage.POST_RENDER, # node template stage
                    "omni.syntheticdata.SdTestStageSynchronization", # node template type
                    # node template connections
                    [
                        # connected to a TestSyncSim node (a TestSyncSim node will be activated when activating this template)
                        SyntheticData.NodeConnectionTemplate("TestSyncSim", (), None), 
                        # connected to a LdrColorSD rendervar (the renderVar will be activated when activating this template)
                        SyntheticData.NodeConnectionTemplate("LdrColorSD"), 
                        # connected to a BoundingBox3DSD rendervar (the renderVar will be activated when activating this template)
                        SyntheticData.NodeConnectionTemplate("BoundingBox3DSD") 
                    ]), 
                template_name="TestSyncPost" # node template name
            )
        # register a node template in the postprocess stage
        # NB : this template may be activated for several different renderproducts
        if not sdg_iface.is_node_template_registered("TestSyncOnDemand"):
            sdg_iface.register_node_template( 
                SyntheticData.NodeTemplate(
                    SyntheticDataStage.ON_DEMAND, # node template stage
                    "omni.syntheticdata.SdTestStageSynchronization", # node template type
                    # node template connections
                    [
                        # connected to a TestSyncSim node (a TestSyncSim node will be activated when activating this template)
                        SyntheticData.NodeConnectionTemplate("TestSyncSim", (), None),
                        # connected to a PostProcessDispatch node : the PostProcessDispatch node trigger the execution of its downstream connections for every rendered frame
                        # (a PostProcessDispatch node will be activated when activating this template)
                        SyntheticData.NodeConnectionTemplate("PostProcessDispatch")
                    ]
                ), 
                template_name="TestSyncOnDemand" # node template name
            )
        # register a node template in the postprocess stage
        # NB : this template may be activated for any combination of renderproduct pairs
        if not sdg_iface.is_node_template_registered("TestSyncCross"):
            # register an accumulator which trigger once when all its upstream connections have triggered
            sdg_iface.register_node_template( 
                SyntheticData.NodeTemplate(
                    SyntheticDataStage.ON_DEMAND, # node template stage
                    "omni.graph.action.SyncGate", # node template type
                    # node template connections
                    [
                        # connected to the PostProcessDispatcher for the synchronization value
                        SyntheticData.NodeConnectionTemplate(
                            "PostProcessDispatcher", 
                            (), 
                            {"outputs:swhFrameNumber":"inputs:syncValue"}
                        ),
                        # connected to a TestSyncOnDemand node for the first renderproduct (a TestSyncSim node will be activated when activating this template)
                        SyntheticData.NodeConnectionTemplate(
                            "TestSyncOnDemand", 
                            (0,), 
                            {"outputs:exec":"inputs:execIn"}
                        ),
                        # connected to a TestSyncOnDemand node for the second renderproduct (a TestSyncSim node will be activated when activating this template)
                        SyntheticData.NodeConnectionTemplate(
                            "TestSyncOnDemand", 
                            (1,), 
                            {"outputs:exec":"inputs:execIn"}
                        ),
                    ]
                ), 
                template_name="TestSyncAccum" # node template name
            )
            sdg_iface.register_node_template( 
                SyntheticData.NodeTemplate(
                    SyntheticDataStage.ON_DEMAND,  # node template stage
                    "omni.syntheticdata.SdTestStageSynchronization",  # node template type
                    # node template connections
                    [
                        # connected to a TestSyncAccum node (a TestSyncAccum node will be activated when activating this template)
                        SyntheticData.NodeConnectionTemplate(
                            "TestSyncAccum", 
                            (0,1), 
                            {
                                "outputs:execOut":"inputs:exec",
                                "outputs:syncValue":"inputs:swhFrameNumber"
                            }
                        ),
                    ]
                ), 
                template_name="TestSyncCross" # node template name
            )

        # Activate the node templates for the renderproducts 
        # this will create the node (and all their missing dependencies) within the associated graphs
        #  
        # activate the TestSyncPost for the renderpoduct renderpoduct_0
        # this will also activate the TestSyncSim node and the LdrColorSD and BoundingBox3DSD renderVars for the renderpoduct renderpoduct_0
        # this will set the tag node attribute to "1"
        sdg_iface.activate_node_template("TestSyncPost", 0, [render_product_path_0],{"inputs:tag":"1"})
        # activate the TestSyncPost for the renderpoduct renderpoduct_1
        # this will also activate the LdrColorSD and BoundingBox3DSD renderVars for the renderpoduct renderpoduct_1 
        # NB TestSyncSim has already been activated
        # this will set the tag node attribute to "2"
        sdg_iface.activate_node_template("TestSyncPost", 0, [render_product_path_1],{"inputs:tag":"2"})
        # activate the TestSyncCross for the renderpoducts [renderproduct_0, renderproduct_1]
        # this will also activate :
        # - TestSyncAccum for the renderpoducts [renderproduct_0, renderproduct_1]
        # - PostProcessDispatch for the renderpoduct renderproduct_0
        # - TestSyncOnDemand for the renderproduct renderproduct_0
        # - TestSyncOnDemand for the renderproduct renderproduct_1
        # - PostProcessDispatch for the renderpoduct renderproduct_1
        # this will set the tag node attribute to "5" 
        sdg_iface.activate_node_template("TestSyncCross", 0, [render_product_path_0,render_product_path_1],{"inputs:tag":"5"})

        # Set some specific attributes to nodes that have been automatically activated
        # set the tag to the TestSyncOnDemand for renderproduct renderproduct_0
        sdg_iface.set_node_attributes("TestSyncOnDemand",{"inputs:tag":"3"},render_product_path_0)
        # set the tag to the TestSyncOnDemand for renderproduct renderproduct_1
        sdg_iface.set_node_attributes("TestSyncOnDemand",{"inputs:tag":"4"},render_product_path_1)


        # setup members
        self.render_product_path_0 = render_product_path_0 
        self.render_product_path_1 = render_product_path_1 
        self.numLoops = 33

    async def run_loop(self):
        
        sdg_iface = SyntheticData.Get()

        render_product_path_0 = self.render_product_path_0
        render_product_path_1 = self.render_product_path_1

        test_attributes_names = ["outputs:swhFrameNumber","outputs:fabricSWHFrameNumber"]
        
        # ensuring that the setup is taken into account
        for _ in range(5):
        
            await omni.kit.app.get_app().next_update_async()

        for _ in range(self.numLoops):
            
            await omni.kit.app.get_app().next_update_async()
        
            # test the post-render pipeline synchronization
            sync_post_attributes = sdg_iface.get_node_attributes(
                "TestSyncPost",test_attributes_names,render_product_path_0)
            assert sync_post_attributes and all(attr in sync_post_attributes for attr in test_attributes_names)
            assert sync_post_attributes["outputs:swhFrameNumber"] == sync_post_attributes["outputs:fabricSWHFrameNumber"]

            # test the on-demand pipeline synchronization
            sync_ondemand_attributes = sdg_iface.get_node_attributes(
                "TestSyncOnDemand",test_attributes_names,render_product_path_1)
            assert sync_ondemand_attributes and all(attr in sync_ondemand_attributes for attr in test_attributes_names)
            assert sync_ondemand_attributes["outputs:swhFrameNumber"] == sync_ondemand_attributes["outputs:fabricSWHFrameNumber"]

            # test the on-demand cross renderproduct synchronization
            sync_cross_ondemand_attributes = sdg_iface.get_node_attributes(
                "TestSyncCross",test_attributes_names,render_product_path_0)
            assert sync_cross_ondemand_attributes and all(attr in sync_cross_ondemand_attributes for attr in test_attributes_names)
            assert sync_cross_ondemand_attributes["outputs:swhFrameNumber"] == sync_cross_ondemand_attributes["outputs:fabricSWHFrameNumber"]

    async def test_sync_idle(self):
        """ Test swh frame synhronization with :
            - asyncRendering Off 
            - waitIdle On
        """
        settings = carb.settings.get_settings()
        settings.set_bool("/app/asyncRendering",False)
        settings.set_int("/app/settings/flatCacheStageFrameHistoryCount",3)
        settings.set_bool("/app/renderer/waitIdle",True) 
        settings.set_bool("/app/hydraEngine/waitIdle",True) 
        
        await self.run_loop()

    @unittest.skip("DRIVE-3247 : SyntheticData does not support async rendering.")
    async def test_sync(self):
        """ Test swh frame synhronization with :
            - asyncRendering Off 
            - waitIdle Off
        """
        settings = carb.settings.get_settings()
        settings.set_bool("/app/asyncRendering",False)
        settings.set_int("/app/settings/flatCacheStageFrameHistoryCount",3)
        settings.set_bool("/app/renderer/waitIdle",False) 
        settings.set_bool("/app/hydraEngine/waitIdle",False) 

        await self.run_loop()

    @unittest.skip("DRIVE-3247 : SyntheticData does not support async rendering.")
    async def test_async(self):
        """ Test swh frame synhronization with :
            - asyncRendering On 
            - waitIdle Off
        """
        settings = carb.settings.get_settings()
        settings.set_bool("/app/asyncRendering",True)
        settings.set_int("/app/settings/flatCacheStageFrameHistoryCount",3)
        settings.set_bool("/app/renderer/waitIdle",False) 
        settings.set_bool("/app/hydraEngine/waitIdle",False) 

        await self.run_loop()
            
    async def tearDown(self):
        # reset to the default params
        settings = carb.settings.get_settings()
        settings.set_bool("/app/asyncRendering",False)
        settings.set_bool("/app/renderer/waitIdle",True) 
        settings.set_bool("/app/hydraEngine/waitIdle",True) 
