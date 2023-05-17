import carb

from pxr import Gf, UsdGeom, UsdLux, Sdf

import omni.hydratexture
import omni.kit.test
from omni.syntheticdata import SyntheticData, SyntheticDataStage


# Test the Fabric frame number synchronization  
class TestSWHFrameNumber(omni.kit.test.AsyncTestCase):
    
    def __init__(self, methodName: str) -> None:
        super().__init__(methodName=methodName)

    def render_product_path(self, hydra_texture) -> str:
        '''Return a string to the UsdRender.Product used by the texture'''
        render_product = hydra_texture.get_render_product_path()
        if render_product and (not render_product.startswith('/')):
            render_product = '/Render/RenderProduct_' + render_product
        return render_product    

    async def wait_for_num_sims(self, num_sims, max_num_sims=5000):
        self._hydra_texture_rendered_counter = 0
        wait_sims_left = max_num_sims
        while (self._hydra_texture_rendered_counter < num_sims) and (wait_sims_left > 0):
            await omni.kit.app.get_app().next_update_async()
            wait_sims_left -= 1

    async def setUp(self):
        
        self._settings = carb.settings.acquire_settings_interface()
        self._hydra_texture_factory = omni.hydratexture.acquire_hydra_texture_factory_interface()

        self._usd_context_name = ''
        self._usd_context = omni.usd.get_context(self._usd_context_name)
        await self._usd_context.new_stage_async()

        # Setup the scene
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

        camera_1 = stage.DefinePrim("/Camera1", "Camera")
        camera_1.CreateAttribute("cameraProjectionType", Sdf.ValueTypeNames.Token).Set("fisheyePolynomial")
        UsdGeom.Xformable(camera_1).AddTranslateOp().Set((0, 250, 0))
        UsdGeom.Xformable(camera_1).AddRotateXYZOp().Set((-90, 0, 0))

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
        render_product_path_0 = self.render_product_path(self._hydra_texture_0)
        
        self._hydra_texture_rendered_counter = 0
        def on_hydra_texture_0(event: carb.events.IEvent):
            self._hydra_texture_rendered_counter += 1
        self._hydra_texture_rendered_counter_sub = self._hydra_texture_0.get_event_stream().create_subscription_to_push_by_type(
            omni.hydratexture.EVENT_TYPE_DRAWABLE_CHANGED,
            on_hydra_texture_0,
            name='async rendering test drawable update',
        )

        self._hydra_texture_1 = self._hydra_texture_factory.create_hydra_texture(
            "TEX1",
            512,
            512,
            self._usd_context_name,
            str(camera_1.GetPath()),
            hydra_engine_name=renderer,
            is_async=self._settings.get("/app/asyncRendering")
        )
        render_product_path_1 = self.render_product_path(self._hydra_texture_1)
        
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
                        attributes={
                            "inputs:tag":"0", 
                            "inputs:randomSeed": 13,
                            "inputs:randomMaxProcessingTimeUs": 33333, 
                            "inputs:traceError": True
                        }
                    ), # node template default attribute values (when differs from the default value specified in the .ogn)
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
                    ],
                    attributes={
                        "inputs:randomSeed": 27,
                        "inputs:randomMaxProcessingTimeUs": 33333, 
                        "inputs:traceError": True
                    }
                ), 
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
                    ],
                    attributes={
                        "inputs:randomSeed": 51,
                        "inputs:randomMaxProcessingTimeUs": 33333, 
                        "inputs:traceError": True
                    } # node template default attribute values (when differs from the default value specified in the .ogn)
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
                    ],
                    attributes={
                        "inputs:randomSeed": 62,
                        "inputs:randomMaxProcessingTimeUs": 33333, 
                        "inputs:traceError": True
                    }
                ), 
                template_name="TestSyncCross" # node template name
            )

        # Activate the node templates for the renderproducts 
        # this will create the node (and all their missing dependencies) within the associated graphs
        #  
        # activate the TestSyncSim
        sdg_iface.activate_node_template("TestSyncSim")
        # wait for the next update to make sure the simulation node is activated when activating the post-render and post-process nodes
        
        # activate the TestSyncPost for the renderpoduct renderpoduct_0
        # this will also activate the LdrColorSD and BoundingBox3DSD renderVars for the renderpoduct renderpoduct_0
        # this will set the tag node attribute to "1"
        sdg_iface.activate_node_template("TestSyncPost", 0, [render_product_path_0],{"inputs:tag":"1"})
        
        # activate the TestSyncPost for the renderpoduct renderpoduct_1
        # this will also activate the LdrColorSD and BoundingBox3DSD renderVars for the renderpoduct renderpoduct_1 
        # NB TestSyncSim has already been activated
        # this will set the tag node attribute to "2"
        sdg_iface.activate_node_template("TestSyncPost", 0, [render_product_path_1],{"inputs:tag":"2"})
        
        # FIXME : wait a couple of simulation updates as a workaround of an issue with the first 
        #         syncGate not being activated
        await self.wait_for_num_sims(3)

        # activate the TestSyncCross for the renderpoducts [renderproduct_0, renderproduct_1]
        # this will also activate :
        # - TestSyncAccum for the renderpoducts [renderproduct_0, renderproduct_1]
        # - PostProcessDispatch for the renderpoduct renderproduct_0
        # - TestSyncOnDemand for the renderproduct renderproduct_0
        # - TestSyncOnDemand for the renderproduct renderproduct_1
        # - PostProcessDispatch for the renderpoduct renderproduct_1
        # this will set the tag node attribute to "5" and processingTime to 30000
        sdg_iface.activate_node_template("TestSyncCross", 0, [render_product_path_0,render_product_path_1],{"inputs:tag":"5"})

        # Set some specific attributes to nodes that have been automatically activated
        # set the tag to the TestSyncOnDemand for renderproduct renderproduct_0
        sdg_iface.set_node_attributes("TestSyncOnDemand",{"inputs:tag":"3"},render_product_path_0)
        # set the tag to the TestSyncOnDemand for renderproduct renderproduct_1
        sdg_iface.set_node_attributes("TestSyncOnDemand",{"inputs:tag":"4"},render_product_path_1)

        # setup members
        self._num_sims = 555

    async def tearDown(self):
        self._hydra_texture_rendered_counter_sub = None
        self._hydra_texture_0 = None
        self._hydra_texture_1 = None

        self._usd_context.close_stage()
        omni.usd.release_all_hydra_engines(self._usd_context)

        self._hydra_texture_factory = None
        self._settings = None

        wait_iterations = 6
        for _ in range(wait_iterations):
            await omni.kit.app.get_app().next_update_async()

    async def test_pipline(self):
        """ Test swh frame synhronization
        """
        await self.wait_for_num_sims(self._num_sims)
        
        