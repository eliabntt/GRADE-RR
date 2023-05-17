from sqlite3 import connect
from pxr import Sdf, Usd, UsdRender

import carb
import omni.graph.core as og
import omni.usd
import omni.kit

from dataclasses import dataclass, field

""" SyntheticData class is the prototype interface implementation (will be eventually integrated in SynthetiData C++ interface )
    - contains the definition of all omnigraphs
    - expose methods for the user to
        - add / remove custom nodes to graphs 
"""

_sdg_iface = None


class SyntheticDataException(Exception):
    def __init__(self, message="error"):
        self.message = message
        super().__init__(self.message)


class SyntheticDataStage:
    # stage is set automatically from the node connections' stages
    AUTO = -1
    # global simulation : node scheduled in the simulation graph
    SIMULATION = 0
    # prerender : node scheduled in the prerender graph
    PRE_RENDER = 1
    # postrender : node scheduled in the postrender graph for a specific renderproduct
    POST_RENDER = 2
    # on demand : node scheduled in the postprocess graph
    ON_DEMAND = 3


class SyntheticData:

    _graphPathRoot = "/Render"
    _graphName = "SDGPipeline"
    _simulationGraphPath = "Simulation/" + _graphName
    _preRenderGraphPath = "PreRender/" + _graphName
    _postRenderGraphPath = "PostRender/" + _graphName
    _postProcessGraphPath = "PostProcess/" + _graphName

    _rendererTemplateName = "GpuInteropEntry"
    _renderVarBuffSuffix = "buff"
    _renderVarHostSuffix = "host"
    _renderVarToHostTemplateName = "PostRenderVarToHost"
    _renderProductAttributeName = "inputs:renderProductPath"

    _instanceMappingCtrl = "InstanceMappingPre"
    _defaultSemanticFilterName = "DefaultSemanticFilter"

    # graph registry : contains node templates used to construct a graph
    #  node template name / id
    #  list containing :
    #  - node type
    #  - list of template dependencies description :
    #       - connection node template name or renderVar name
    #       - index of the render product in the list provided during activation
    #       - dictionnary of inputs / outputs mapping
    # - node attributes name/value dictionnary to be set during the activation
    #

    @dataclass
    class NodeConnectionTemplate:
        node_template_id: str
        render_product_idxs: tuple = (0,)
        attributes_mapping: dict = field(default_factory=dict)

    @dataclass
    class NodeTemplate:
        pipeline_stage: int
        node_type_id: str
        connections: list = field(default_factory=list)
        attributes: dict = field(default_factory=dict)

    _ogn_templates_registry = {
        # --- Camera
        "RenderProductCameraPrimPath": NodeTemplate(
            SyntheticDataStage.SIMULATION,
            "omni.syntheticdata.SdSimRenderProductCamera"
        ),
        "PostRenderProductCamera": NodeTemplate(
            SyntheticDataStage.POST_RENDER,
            "omni.syntheticdata.SdRenderProductCamera",
            [
                NodeConnectionTemplate(
                    _rendererTemplateName,
                    attributes_mapping={
                        "outputs:rp": "inputs:renderResults",
                        "outputs:swhFrameNumber": "inputs:swhFrameNumber",
                        "outputs:exec": "inputs:exec"
                    }),
                NodeConnectionTemplate("RenderProductCameraPrimPath", attributes_mapping={
                                       "outputs:exec": "inputs:exec"})
            ]
        ),
        # --- GPUInterop
        _rendererTemplateName: NodeTemplate(SyntheticDataStage.POST_RENDER, "omni.graph.nodes.GpuInteropRenderProductEntry"),
        # --- InstanceMapping
        _instanceMappingCtrl : NodeTemplate(
            SyntheticDataStage.SIMULATION,
            "omni.syntheticdata.SdSimInstanceMapping",
            attributes={"inputs:needTransform": False, "inputs:semanticFilterPredicate":"*:*"}
        ),
        _defaultSemanticFilterName: NodeTemplate(
            SyntheticDataStage.SIMULATION,
            "omni.syntheticdata.SdSemanticFilter",
            attributes={"inputs:name": "default", "inputs:predicate": "*:*"}
        ),
        "InstanceMappingTransforms": NodeTemplate(
            SyntheticDataStage.SIMULATION,
            "omni.syntheticdata.SdSimInstanceMapping",
            [
                NodeConnectionTemplate(_instanceMappingCtrl, render_product_idxs=())
            ],
            {"inputs:needTransform": True}
        ),
        "InstanceMappingPost": NodeTemplate(
            SyntheticDataStage.POST_RENDER,
            "omni.syntheticdata.SdPostInstanceMapping",
            [
                NodeConnectionTemplate("InstanceIdTokenMapSD"),
                NodeConnectionTemplate(_instanceMappingCtrl, attributes_mapping={"outputs:exec": "inputs:exec"}, render_product_idxs=())
            ],
            {},
        ),
        # --- NoOp node used to expose the semantic transforms renderVars
        "InstanceMappingPostWithTransforms": NodeTemplate(
            SyntheticDataStage.POST_RENDER,
            "omni.syntheticdata.SdNoOp",
            [    
                NodeConnectionTemplate("InstanceMappingTransforms", attributes_mapping={"outputs:exec": "inputs:exec"}, render_product_idxs=()),
                NodeConnectionTemplate("InstanceMappingPost", attributes_mapping={"outputs:exec": "inputs:exec"})
            ],
            {},
        ),
        # --- BoundingBoxes
        "BoundingBox2DTightReduction": NodeTemplate(
            SyntheticDataStage.POST_RENDER,
            "omni.syntheticdata.SdPostSemanticBoundingBox",
            [
                NodeConnectionTemplate("BoundingBox2DTightSD"),
                NodeConnectionTemplate("InstanceMappingPost")
            ],
            {"inputs:renderVar": "BoundingBox2DTightSD"},
        ),
        "BoundingBox2DLooseReduction": NodeTemplate(
            SyntheticDataStage.POST_RENDER,
            "omni.syntheticdata.SdPostSemanticBoundingBox",
            [
                NodeConnectionTemplate("BoundingBox2DLooseSD"),
                NodeConnectionTemplate("InstanceMappingPost")
            ],
            {"inputs:renderVar": "BoundingBox2DLooseSD"},
        ),
        "BoundingBox3DReduction": NodeTemplate(
            SyntheticDataStage.POST_RENDER,
            "omni.syntheticdata.SdPostSemanticBoundingBox",
            [
                NodeConnectionTemplate("BoundingBox3DSD"),
                NodeConnectionTemplate("InstanceMappingTransforms", attributes_mapping={"outputs:exec": "inputs:exec"}, render_product_idxs=()),
                NodeConnectionTemplate("InstanceMappingPost")
            ],
            {"inputs:renderVar": "BoundingBox3DSD"},
        ),
        "BoundingBox3DCameraProjection": NodeTemplate(
            SyntheticDataStage.POST_RENDER,
            "omni.syntheticdata.SdPostSemantic3dBoundingBoxCameraProjection",
            [
                NodeConnectionTemplate("BoundingBox3DSD"),
                NodeConnectionTemplate("BoundingBox3DReduction"),
                NodeConnectionTemplate("PostRenderProductCamera"),
                NodeConnectionTemplate("InstanceMappingTransforms", attributes_mapping={"outputs:exec": "inputs:exec"}, render_product_idxs=()),
                NodeConnectionTemplate("InstanceMappingPost")
            ]
        ),
        "BoundingBox3DFilter": NodeTemplate(
            SyntheticDataStage.POST_RENDER,
            "omni.syntheticdata.SdPostSemantic3dBoundingBoxFilter",
            [
                NodeConnectionTemplate("BoundingBox3DSD"),
                NodeConnectionTemplate("BoundingBox3DCameraProjection"),
                NodeConnectionTemplate("PostRenderProductCamera"),
                NodeConnectionTemplate("BoundingBox3DReduction"),
                NodeConnectionTemplate("InstanceMappingPost")
            ]
        ),
        # --- PostRenderVarDisplay
        "LdrColorDisplayPost": NodeTemplate(
            SyntheticDataStage.POST_RENDER,
            "omni.syntheticdata.SdPostRenderVarDisplayTexture",
            [NodeConnectionTemplate("LdrColorSD")],
            {"inputs:renderVar": "LdrColorSD"},
        ),
        "DistanceToImagePlaneDisplayPost": NodeTemplate(
            SyntheticDataStage.POST_RENDER,
            "omni.syntheticdata.SdPostRenderVarDisplayTexture",
            [NodeConnectionTemplate("DistanceToImagePlaneSD")],
            {
                "inputs:renderVar": "DistanceToImagePlaneSD",
                "inputs:parameters": [0.0, 100.0, 0.0, 0.0]
            },
        ),
        "DistanceToCameraDisplayPost": NodeTemplate(
            SyntheticDataStage.POST_RENDER,
            "omni.syntheticdata.SdPostRenderVarDisplayTexture",
            [NodeConnectionTemplate("DistanceToCameraSD")],
            {
                "inputs:renderVar": "DistanceToCameraSD",
                "inputs:parameters": [0.0, 100.0, 0.0, 0.0]
            },
        ),
        "Camera3dPositionDisplayPost": NodeTemplate(
            SyntheticDataStage.POST_RENDER,
            "omni.syntheticdata.SdPostRenderVarDisplayTexture",
            [NodeConnectionTemplate("Camera3dPositionSD")],
            {"inputs:renderVar": "Camera3dPositionSD"},
        ),
        "NormalDisplayPost": NodeTemplate(
            SyntheticDataStage.POST_RENDER,
            "omni.syntheticdata.SdPostRenderVarDisplayTexture",
            [NodeConnectionTemplate("NormalSD")],
            {"inputs:renderVar": "NormalSD"},
        ),
        "CrossCorrespondenceDisplayPost": NodeTemplate(
            SyntheticDataStage.POST_RENDER,
            "omni.syntheticdata.SdPostRenderVarDisplayTexture",
            [NodeConnectionTemplate("CrossCorrespondenceSD")],
            {"inputs:renderVar": "CrossCorrespondenceSD"},
        ),
        "TargetMotionDisplayPost": NodeTemplate(
            SyntheticDataStage.POST_RENDER,
            "omni.syntheticdata.SdPostRenderVarDisplayTexture",
            [NodeConnectionTemplate("TargetMotionSD")],
            {
                "inputs:renderVar": "TargetMotionSD",
                "inputs:parameters": [1.0, 5.0, 0.0, 0.0]
            },
        ),
        "InstanceIdSegmentationDisplayPost": NodeTemplate(
            SyntheticDataStage.POST_RENDER,
            "omni.syntheticdata.SdPostRenderVarDisplayTexture",
            [NodeConnectionTemplate("InstanceSegmentationSD")],
            {"inputs:renderVar": "InstanceSegmentationSD",
                "inputs:renderVarDisplay": "RawInstanceSegmentationSDDisplay", "inputs:mode": "segmentationMapMode"},
        ),
        "InstanceSegmentationDisplayPost": NodeTemplate(
            SyntheticDataStage.POST_RENDER,
            "omni.syntheticdata.SdPostRenderVarDisplayTexture",
            [
                NodeConnectionTemplate("InstanceSegmentationSD"),
                NodeConnectionTemplate("InstanceMappingPost")
            ],
            {"inputs:renderVar": "InstanceSegmentationSD", "inputs:mode": "semanticPathMode"},
        ),
        "SemanticSegmentationDisplayPost": NodeTemplate(
            SyntheticDataStage.POST_RENDER,
            "omni.syntheticdata.SdPostRenderVarDisplayTexture",
            [
                NodeConnectionTemplate("InstanceSegmentationSD"),
                NodeConnectionTemplate("InstanceMappingPost"),
            ],
            {"inputs:renderVar": "InstanceSegmentationSD",
                "inputs:renderVarDisplay": "SemanticSegmentationSDDisplay", "inputs:mode": "semanticLabelMode"},
        ),
        "SemanticIdSegmentationDisplayPost": NodeTemplate(
            SyntheticDataStage.POST_RENDER,
            "omni.syntheticdata.SdPostRenderVarDisplayTexture",
            [NodeConnectionTemplate("SemanticSegmentationSD")],
            {"inputs:renderVar": "SemanticSegmentationSD",
                "inputs:renderVarDisplay": "RawSemanticSegmentationSDDisplay", "inputs:mode": "segmentationMapMode"},
        ),
        "BoundingBox2DTightDisplayPost": NodeTemplate(
            SyntheticDataStage.POST_RENDER,
            "omni.syntheticdata.SdPostRenderVarDisplayTexture",
            [
                NodeConnectionTemplate("LdrColorSD"),
                NodeConnectionTemplate("InstanceMappingPost"),
                NodeConnectionTemplate("BoundingBox2DTightReduction"),
            ],
            {"inputs:renderVar": "LdrColorSD", "inputs:renderVarDisplay": "BoundingBox2DTightSDDisplay",
                "inputs:mode": "semanticBoundingBox2dMode"},
        ),
        "BoundingBox2DLooseDisplayPost": NodeTemplate(
            SyntheticDataStage.POST_RENDER,
            "omni.syntheticdata.SdPostRenderVarDisplayTexture",
            [
                NodeConnectionTemplate("LdrColorSD"),
                NodeConnectionTemplate("InstanceMappingPost"),
                NodeConnectionTemplate("BoundingBox2DLooseReduction"),
            ],
            {"inputs:renderVar": "LdrColorSD", "inputs:renderVarDisplay": "BoundingBox2DLooseSDDisplay",
                "inputs:mode": "semanticBoundingBox2dMode"},
        ),
        "BoundingBox3DDisplayPost": NodeTemplate(
            SyntheticDataStage.POST_RENDER,
            "omni.syntheticdata.SdPostRenderVarDisplayTexture",
            [
                NodeConnectionTemplate("LdrColorSD"),
                NodeConnectionTemplate("Camera3dPositionSD"),
                NodeConnectionTemplate("PostRenderProductCamera"),
                NodeConnectionTemplate("InstanceMappingPost"),
                NodeConnectionTemplate("BoundingBox3DFilter"),
                NodeConnectionTemplate("BoundingBox3DCameraProjection"),
                NodeConnectionTemplate("BoundingBox3DReduction"),
            ],
            {
                "inputs:renderVar": "LdrColorSD",
                "inputs:renderVarDisplay": "BoundingBox3DSDDisplay",
                "inputs:mode": "semanticBoundingBox3dMode",
                "inputs:parameters": [0.0, 5.0, 0.027, 0.27]
            },
        ),
        # --- PostProcess
        "PostProcessDispatcher": NodeTemplate(
            SyntheticDataStage.ON_DEMAND,
            "omni.syntheticdata.SdOnNewFrame"
        ),
        "PostProcessDispatch": NodeTemplate(
            SyntheticDataStage.ON_DEMAND,
            "omni.syntheticdata.SdOnNewRenderProductFrame",
            [NodeConnectionTemplate("PostProcessDispatcher", render_product_idxs=())]
        ),
        "PostProcessRenderProductCamera": NodeTemplate(
            SyntheticDataStage.ON_DEMAND,
            "omni.syntheticdata.SdRenderProductCamera",
            [
                NodeConnectionTemplate("PostProcessDispatch"),
                NodeConnectionTemplate("RenderProductCameraPrimPath", attributes_mapping={
                                       "outputs:exec": "inputs:exec"}),
                NodeConnectionTemplate(_rendererTemplateName, attributes_mapping={
                                       "outputs:exec": "inputs:exec"})  # provide the renderResults
            ]
        ),
        "InstanceMapping": NodeTemplate(
            SyntheticDataStage.ON_DEMAND,
            "omni.syntheticdata.SdInstanceMapping",
            [
                NodeConnectionTemplate("PostProcessDispatch"),
                NodeConnectionTemplate("InstanceMappingPost", attributes_mapping={"outputs:exec": "inputs:exec"})
            ]
        ),
        "InstanceMappingWithTransforms": NodeTemplate(
            SyntheticDataStage.ON_DEMAND,
            "omni.syntheticdata.SdInstanceMapping",
            [
                NodeConnectionTemplate("PostProcessDispatch"),
                NodeConnectionTemplate("InstanceMappingTransforms", attributes_mapping={"outputs:exec": "inputs:exec"}, render_product_idxs=()),
                NodeConnectionTemplate("InstanceMappingPost", attributes_mapping={"outputs:exec": "inputs:exec"})
            ]
        ),
        "InstanceMappingPtr": NodeTemplate(
            SyntheticDataStage.ON_DEMAND,
            "omni.syntheticdata.SdInstanceMappingPtr",
            [
                NodeConnectionTemplate("PostProcessDispatch"),
                NodeConnectionTemplate("InstanceMappingPost", attributes_mapping={"outputs:exec": "inputs:exec"})
            ]
        ),
        "InstanceMappingPtrWithTransforms": NodeTemplate(
            SyntheticDataStage.ON_DEMAND,
            "omni.syntheticdata.SdInstanceMappingPtr",
            [
                NodeConnectionTemplate("PostProcessDispatch"),
                NodeConnectionTemplate("InstanceMappingTransforms", attributes_mapping={"outputs:exec": "inputs:exec"}, render_product_idxs=()),
                NodeConnectionTemplate("InstanceMappingPost", attributes_mapping={"outputs:exec": "inputs:exec"})
            ]
        )
    }

    # set of rendervars associated to the node exposing them :
    # - renderVar generated by the renderer are exposed by the GpuInteropEntry
    # - others renderVars are generated by some postRender nodes
    # FIXME : the list of renderer rendervars should be queried from the renderer
    _ogn_rendervars = {
        # renderer renderVars
        "LdrColorSD": _rendererTemplateName,
        "Camera3dPositionSD": _rendererTemplateName,
        "DistanceToImagePlaneSD": _rendererTemplateName,
        "DistanceToCameraSD": _rendererTemplateName,
        "DepthSD": _rendererTemplateName,
        "DepthLinearSD": _rendererTemplateName,
        "InstanceSegmentationSD": _rendererTemplateName,
        "SemanticSegmentationSD": _rendererTemplateName,
        "NormalSD": _rendererTemplateName,
        "TargetMotionSD": _rendererTemplateName,
        "BoundingBox2DTightSD": _rendererTemplateName,
        "BoundingBox2DLooseSD": _rendererTemplateName,
        "BoundingBox3DSD": _rendererTemplateName,
        "OcclusionSD": _rendererTemplateName,
        "TruncationSD": _rendererTemplateName,
        "CrossCorrespondenceSD": _rendererTemplateName,
        "InstanceIdTokenMapSD": _rendererTemplateName,
        "SemanticIdTokenMapSD": _rendererTemplateName,
        # postRender nodes rendervars
        "InstanceMappingInfoSDhost": "InstanceMappingPost",
        "SemanticMapSD": "InstanceMappingPost",
        "SemanticMapSDhost": "InstanceMappingPost",
        "SemanticPrimTokenSD": "InstanceMappingPost",
        "SemanticPrimTokenSDhost": "InstanceMappingPost",
        "InstanceMapSD": "InstanceMappingPost",
        "InstanceMapSDhost": "InstanceMappingPost",
        "InstancePrimTokenSD": "InstanceMappingPost",
        "InstancePrimTokenSDhost": "InstanceMappingPost",
        "SemanticLabelTokenSD": "InstanceMappingPost",
        "SemanticLabelTokenSDhost": "InstanceMappingPost",
        "SemanticLocalTransformSD": "InstanceMappingPostWithTransforms",
        "SemanticLocalTransformSDhost": "InstanceMappingPostWithTransforms",
        "SemanticWorldTransformSD": "InstanceMappingPostWithTransforms",
        "SemanticWorldTransformSDhost": "InstanceMappingPostWithTransforms",
        "SemanticBoundingBox2DExtentTightSD": "BoundingBox2DTightReduction",
        "SemanticBoundingBox2DInfosTightSD": "BoundingBox2DTightReduction",
        "SemanticBoundingBox2DExtentLooseSD": "BoundingBox2DLooseReduction",
        "SemanticBoundingBox2DInfosLooseSD": "BoundingBox2DLooseReduction",
        "SemanticBoundingBox3DExtentSD": "BoundingBox3DReduction",
        "SemanticBoundingBox3DInfosSD": "BoundingBox3DReduction",
        "SemanticBoundingBox3DCamCornersSD": "BoundingBox3DCameraProjection",
        "SemanticBoundingBox3DCamExtentSD": "BoundingBox3DCameraProjection",
        "SemanticBoundingBox3DFilterInfosSD": "BoundingBox3DFilter"
    }

    _ogn_post_display_types = [
        "omni.syntheticdata.SdPostRenderVarDisplayTexture"
    ]

    _ogn_display_types = [
        "omni.syntheticdata.SdRenderVarDisplayTexture",
        "omni.syntheticdata.SdLinearArrayToTexture"
    ]
    """lst: List of omnigraph node types conforming the display api. 
    Todo : use reflexivity on the node outputs."""

    @staticmethod
    def register_display_rendervar_templates() -> None:
        """Automatically register SdRenderVarDisplayTexture node template for all registerd nodes whose type is in the post display type.

        The function is called for every statically registered nodes during the interface initialization    . 
        It may be called after having registered nodes whose type is omni.syntheticdata.SdPostRenderVarDisplayTexture. 

        """
        ogn_registry_keys = [key for key in SyntheticData._ogn_templates_registry.keys()]
        for tplName in ogn_registry_keys:
            tplParams = SyntheticData._ogn_templates_registry[tplName]
            tplNameDisplay = tplName[:-11] + "Display"
            if (tplParams.node_type_id in SyntheticData._ogn_post_display_types) and (tplNameDisplay not in SyntheticData._ogn_templates_registry):
                SyntheticData.register_node_template(SyntheticData.NodeTemplate(
                    SyntheticDataStage.ON_DEMAND,
                    "omni.syntheticdata.SdRenderVarDisplayTexture",
                    [
                        SyntheticData.NodeConnectionTemplate("PostProcessDispatch"),
                        SyntheticData.NodeConnectionTemplate(tplName)
                    ],
                ),
                    template_name=tplNameDisplay
                )

    @staticmethod
    def register_combine_rendervar_templates() -> None:
        """Automatically register SdPostCompRenderVarTextures node template for all registerd nodes whose type is in the post display type list.

        The function is called for every statically registered nodes during the interface initialization    . 
        It may be called after having registered nodes whose type is in the post display type list. 

        """
        ogn_registry_keys = [key for key in SyntheticData._ogn_templates_registry.keys()]
        for tplName in ogn_registry_keys:
            tplParams = SyntheticData._ogn_templates_registry[tplName]
            if (tplParams.node_type_id in SyntheticData._ogn_post_display_types) and (
                tplName + "Combine" not in SyntheticData._ogn_templates_registry
            ):
                SyntheticData.register_combine_rendervar_template(tplName)

    @staticmethod
    def register_combine_rendervar_template(template_name: str) -> None:
        """Automatically register SdPostCompRenderVarTextures node template for the given template name.

            Args:
                template_name: name of the node template for which registering a SdPostCompRenderVarTextures template
        """
        if not template_name in SyntheticData._ogn_templates_registry:
            raise SyntheticDataException(f'graph node template "{template_name}" not registered')
        # cannot combine node results from the ondemand graph
        if SyntheticData._ogn_templates_registry[template_name].pipeline_stage > SyntheticDataStage.POST_RENDER:
            return
        templateParams = SyntheticData._ogn_templates_registry[template_name]
        if templateParams.node_type_id not in SyntheticData._ogn_post_display_types:
            raise SyntheticDataException(f'graph node template "{template_name}" not registered as a display node')
        templateNameCombine = template_name + "Combine"
        if templateNameCombine not in SyntheticData._ogn_templates_registry:
            SyntheticData.register_node_template(SyntheticData.NodeTemplate(
                SyntheticDataStage.POST_RENDER,
                "omni.syntheticdata.SdPostCompRenderVarTextures",
                [
                    SyntheticData.NodeConnectionTemplate(SyntheticData._rendererTemplateName),
                    SyntheticData.NodeConnectionTemplate(
                        template_name,
                        attributes_mapping={
                            "outputs:cudaPtr": "inputs:cudaPtr",
                            "outputs:width": "inputs:width",
                            "outputs:height": "inputs:height",
                            "outputs:format": "inputs:format"
                        }
                    )
                ]
            ),
                template_name=templateNameCombine,
            )

    @staticmethod
    def register_device_rendervar_to_host_templates(rendervars: list) -> None:
        """Automatically register SdPostRenderVarToHost node templates for the given rendervars

            Args:
                rendervars: list of renderVar names to register the rendervar device to host copy node template

        """
        # copy the rendervars list since the registration may modify the list
        rendervars_copy = rendervars.copy()
        for rv in rendervars_copy:
            rv_host = rv+SyntheticData._renderVarHostSuffix
            if rv.endswith(SyntheticData._renderVarHostSuffix) or (rv_host in SyntheticData._ogn_rendervars):
                continue
            template_name = rv + "PostCopyToHost"
            if template_name not in SyntheticData._ogn_templates_registry:
                SyntheticData.register_node_template(
                    SyntheticData.NodeTemplate(
                        SyntheticDataStage.POST_RENDER,
                        "omni.syntheticdata.SdPostRenderVarToHost",
                        [
                            SyntheticData.NodeConnectionTemplate(rv),
                            SyntheticData.NodeConnectionTemplate(
                                SyntheticData._rendererTemplateName,
                                attributes_mapping={
                                    "outputs:rp": "inputs:rp",
                                    "outputs:gpu": "inputs:gpu"
                                }
                            )
                        ],
                        {
                            "inputs:renderVar": rv, 
                            "inputs:renderVarHostSuffix" : SyntheticData._renderVarHostSuffix
                        }
                    ),
                    rendervars=[rv_host],
                    template_name=template_name,
                )

    @staticmethod
    def register_device_rendervar_tex_to_buff_templates(rendervars: list) -> None:
        """Automatically register SdPostRenderVarTextureToBuffer node templates for the given rendervars

            Args:
                rendervars: list of renderVar names to register the rendervar device texture to buffer copy node template

        """
        # copy the rendervars list since the registration may modify the list
        rendervars_copy = rendervars.copy()
        for rv in rendervars_copy:
            rv_buff = rv+SyntheticData._renderVarBuffSuffix
            if rv.endswith(SyntheticData._renderVarBuffSuffix) or (rv_buff in SyntheticData._ogn_rendervars):
                continue
            template_name = rv + "PostCopyToBuff"
            if template_name not in SyntheticData._ogn_templates_registry:
                SyntheticData.register_node_template(
                    SyntheticData.NodeTemplate(
                        SyntheticDataStage.POST_RENDER,
                        "omni.syntheticdata.SdPostRenderVarTextureToBuffer",
                        [
                            SyntheticData.NodeConnectionTemplate(rv),
                            SyntheticData.NodeConnectionTemplate(
                                SyntheticData._rendererTemplateName,
                                attributes_mapping={
                                    "outputs:rp": "inputs:rp",
                                    "outputs:gpu": "inputs:gpu"
                                }
                            )
                        ],
                        {
                            "inputs:renderVar": rv, 
                            "inputs:renderVarBufferSuffix" : SyntheticData._renderVarBuffSuffix
                        }
                    ),
                    rendervars=[rv_buff],
                    template_name=template_name,
                )

    @staticmethod
    def register_export_rendervar_ptr_templates(rendervars: list) -> None:
        """Automatically register SdRenderVarPtr node templates for the given rendervars

            Args:
                rendervars: list of renderVar names to register the ptr node template

        """
        for rv in rendervars:
            template_name = rv + "Ptr"
            if template_name not in SyntheticData._ogn_templates_registry:
                SyntheticData.register_node_template(
                    SyntheticData.NodeTemplate(
                        SyntheticDataStage.ON_DEMAND,
                        "omni.syntheticdata.SdRenderVarPtr",
                        [
                            SyntheticData.NodeConnectionTemplate(rv, (0,), None),
                            SyntheticData.NodeConnectionTemplate("PostProcessDispatch")
                        ],
                        {"inputs:renderVar": rv}
                    ),
                    template_name=template_name,
                )

    @staticmethod
    def register_export_rendervar_array_templates(rendervars: list) -> None:
        """Automatically register SdRenderVarToRawArray node templates for the given rendervars

            Args:
                rendervars: list of renderVar names to register the export raw array node template

        """
        for rv in rendervars:
            template_name = rv + "ExportRawArray"
            if template_name not in SyntheticData._ogn_templates_registry:
                SyntheticData.register_node_template(
                    SyntheticData.NodeTemplate(
                        SyntheticDataStage.ON_DEMAND,
                        "omni.syntheticdata.SdRenderVarToRawArray",
                        [
                            SyntheticData.NodeConnectionTemplate(rv, (0,), None),
                            SyntheticData.NodeConnectionTemplate("PostProcessDispatch")
                        ],
                        {"inputs:renderVar": rv}
                    ),
                    template_name=template_name,
                )

    @staticmethod
    def convert_sensor_type_to_rendervar(legacy_type_name: str) -> None:
        """Convert of legacy sensor type name to its rendervar name

            Args:
                legacy_type_name: legacy sensor type name to convert

            Returns:
                the name of the renderVar correspoding to the legacy name
        
        """
        if legacy_type_name == "Rgb":
            return "LdrColorSD"
        elif legacy_type_name == "MotionVector":
            return "TargetMotionSD"
        else:
            return legacy_type_name + "SD"

    @staticmethod
    def disable_async_rendering():
        """Disable asynchronous rendering 
           Since asyncRendering is not supported by the fabric, graphs are currently not compatible with this mode.
        """
        settings = carb.settings.get_settings()
        if settings.get("/app/asyncRendering") or settings.get("/app/asyncRenderingLowLatency"):
            carb.log_warn(f"SyntheticData is not supporting asyncRendering : switching it off.")
            settings.set("/app/asyncRendering", False)
            settings.set("/app/asyncRenderingLowLatency", False)

    @staticmethod
    def _has_rendervar(renderProductPath: str, renderVar: str, usdStage: Usd.Stage = None) -> bool:
        if not usdStage:
            usdStage = omni.usd.get_context().get_stage()
            if not usdStage:
                raise SyntheticDataException("No stage provided or in use by default UsdContext")

        renderProductPrim = usdStage.GetPrimAtPath(renderProductPath)
        if not renderProductPrim:
            raise SyntheticDataException(f"invalid renderProduct {renderProductPath}")
        renderVarPrimPath = f"/Render/Vars/{renderVar}"
        renderVarPrim = usdStage.GetPrimAtPath(renderVarPrimPath)
        if not renderVarPrim:
            return False
        renderProductRenderVarRel = renderProductPrim.GetRelationship("orderedVars")
        if not renderProductRenderVarRel:
            return False
        return renderVarPrimPath in renderProductRenderVarRel.GetTargets()

    @staticmethod
    def _add_rendervar(renderProductPath: str, renderVar: str, usdStage: Usd.Stage = None) -> None:
        # FIXME : we have to use the legacy Viewport interface to modify the renderproduct, otherwise changes may be overwritten
        vp_1found = False
        try:
            import omni.kit.viewport_legacy
            vp_iface = omni.kit.viewport_legacy.get_viewport_interface()
            viewports = vp_iface.get_instance_list()
            for viewport in viewports:
                vpw = vp_iface.get_viewport_window(viewport)
                if vpw.get_render_product_path() == renderProductPath:
                    vpw.add_aov(renderVar, False)
                    vp_1found = True
        except ImportError:
            pass
        # Both Viewport-1 and Viewport-2 won't share a common renderProductPath
        if vp_1found:
            return

        if not usdStage:
            usdStage = omni.usd.get_context().get_stage()
            if not usdStage:
                raise SyntheticDataException("No stage provided or in use by default UsdContext")

        with Usd.EditContext(usdStage, usdStage.GetSessionLayer()):
            renderProductPrim = usdStage.GetPrimAtPath(renderProductPath)
            if not renderProductPrim:
                raise SyntheticDataException(f"invalid renderProduct {renderProductPath}")
            renderVarPrimPath = f"/Render/Vars/{renderVar}"
            renderVarPrim = usdStage.GetPrimAtPath(renderVarPrimPath)
            if not renderVarPrim:
                renderVarPrim = usdStage.DefinePrim(renderVarPrimPath)
            if not renderVarPrim:
                raise SyntheticDataException(f"cannot create renderVar {renderVarPrimPath}")
            renderVarPrim.CreateAttribute("sourceName", Sdf.ValueTypeNames.String).Set(renderVar)
            renderVarPrim.SetMetadata("hide_in_stage_window", True)
            renderVarPrim.SetMetadata("no_delete", True)
            renderProductRenderVarRel = renderProductPrim.GetRelationship("orderedVars")
            if not renderProductRenderVarRel:
                renderProductRenderVarRel = renderProductPrim.CreateRelationship("orderedVars")
            if not renderProductRenderVarRel:
                raise SyntheticDataException(
                    f"cannot set orderedVars relationship for renderProduct {renderProductPath}")
            renderProductRenderVarRel.AddTarget(renderVarPrimPath)

    @staticmethod
    def _remove_rendervar(renderProductPath: str, renderVar: str, usdStage: Usd.Stage = None) -> None:
        # we should not remove the LdrColor since it is the default renderVar
        if renderVar == "LdrColor":
            return

        # FIXME : we have to use the legacy Viewport interface to modify the renderproduct, otherwise changes may be overwritten
        vp_1found = False
        try:
            import omni.kit.viewport_legacy
            vp_iface = omni.kit.viewport_legacy.get_viewport_interface()
            viewports = vp_iface.get_instance_list()
            for viewport in viewports:
                vpw = vp_iface.get_viewport_window(viewport)
                if vpw.get_render_product_path() == renderProductPath:
                    vpw.add_aov(renderVar, False)
                    vp_1found = True
        except ImportError:
            pass
        # Both Viewport-1 and Viewport-2 won't share a common renderProductPath
        if vp_1found:
            return

        if not usdStage:
            usdStage = omni.usd.get_context().get_stage()
            if not usdStage:
                raise SyntheticDataException("No stage provided or in use by default UsdContext")

        with Usd.EditContext(usdStage, usdStage.GetSessionLayer()):
            renderProductPrim = usdStage.GetPrimAtPath(renderProductPath)
            if not renderProductPrim:
                raise SyntheticDataException(f"invalid renderProduct {renderProductPath}")
            renderVarPrimPath = f"/Render/Vars/{renderVar}"
            renderProductRenderVarRel = renderProductPrim.GetRelationship("orderedVars")
            if not renderProductRenderVarRel:
                return
            renderProductRenderVarRel.RemoveTarget(renderVarPrimPath)

    @staticmethod
    def get_registered_visualization_template_names() -> list:
        """Get the registered node template names which types are in the display type list

            Returns:
                list of registered template names which types are in the display type list
        """
        registeredTemplateName = []
        for name, val in SyntheticData._ogn_templates_registry.items():
            if val.node_type_id in SyntheticData._ogn_display_types:
                registeredTemplateName.append(name)
        return registeredTemplateName

    @staticmethod
    def get_registered_visualization_template_names_for_display() -> list:
        """Get the registered node template names which types are in the display type list and their display name

            Returns:
                list of tuples of registered template names which types are in the display type list and their display name
        """
        for sensor in SyntheticData.get_registered_visualization_template_names():
            # by convention visualization sensors end with "Display"
            yield (sensor[0:-7] if sensor.endswith("Display") else sensor, sensor)

    @staticmethod
    def _get_graph_path(stage: int, renderProductPath: str = None) -> str:
        # simulation stages live in the same graph
        if stage == SyntheticDataStage.SIMULATION:
            return f"{SyntheticData._graphPathRoot}/{SyntheticData._simulationGraphPath}"
        elif stage == SyntheticDataStage.PRE_RENDER:
            # check if the renderProductPath has already an associated graph
            usdStage = omni.usd.get_context().get_stage()
            prim = usdStage.GetPrimAtPath(renderProductPath)
            ogpreprocesspath_attribute = prim.GetAttribute("ogPreProcessPath")
            if ogpreprocesspath_attribute:
                return f"{ogpreprocesspath_attribute.Get()}/{SyntheticData._graphName}"
            else:
                return f"{renderProductPath}/{SyntheticData._preRenderGraphPath}"
        # postprocess stages live in the same graph
        elif stage == SyntheticDataStage.ON_DEMAND:
            return f"{SyntheticData._graphPathRoot}/{SyntheticData._postProcessGraphPath}"
        elif stage == SyntheticDataStage.POST_RENDER:
            # check if the renderProductPath has already an associated graph
            usdStage = omni.usd.get_context().get_stage()
            prim = usdStage.GetPrimAtPath(renderProductPath)
            ogpostprocesspath_attribute = prim.GetAttribute("ogPostProcessPath")
            if ogpostprocesspath_attribute:
                return f"{ogpostprocesspath_attribute.Get()}/{SyntheticData._graphName}"
            else:
                return f"{renderProductPath}/{SyntheticData._postRenderGraphPath}"

    @staticmethod
    def _get_node_path(templateName: str, renderProductPath: str = None) -> str:
        if templateName not in SyntheticData._ogn_templates_registry:
            raise SyntheticDataException(f'graph node template "{templateName}" not registered')
        nodeStage = SyntheticData._ogn_templates_registry[templateName].pipeline_stage
        graphPath = SyntheticData._get_graph_path(nodeStage, renderProductPath)
        # prefix the node name by the renderproduct name for nodes living in the same graph
        # (simulation and postprocess graphs)
        nodeName = templateName
        if renderProductPath:
            renderProductName = renderProductPath.split("/")[-1]
            nodeName = f"{renderProductName}_{nodeName}"
        return f"{graphPath}/{nodeName}"

    @staticmethod
    def _unregister_node_template_rec(templateList: list) -> None:
        if not templateList:
            return

        templateDependenciesList = []

        for templateName in templateList:
            if templateName not in SyntheticData._ogn_templates_registry:
                continue

            dependencyNames = []
            for rv, tpl in SyntheticData._ogn_rendervars.items():
                if tpl == templateName:
                    dependencyNames.append(rv)

            for rv in dependencyNames:
                SyntheticData._ogn_rendervars.pop(rv)

            dependencyNames.append(templateName)

            SyntheticData._ogn_templates_registry.pop(templateName)

            for otherTemplateName, otherTemplateVal in SyntheticData._ogn_templates_registry.items():
                for otherTemplateConnection in otherTemplateVal.connections:
                    if otherTemplateConnection.node_template_id in dependencyNames:
                        templateDependenciesList.append(otherTemplateName)

        SyntheticData._unregister_node_template_rec(templateDependenciesList)

    @staticmethod
    def _connect_nodes(srcNode, dstNode, connectionMap, enable) -> bool:
        success = True
        for srcAttrName, dstAttrName in connectionMap.items():
            if (not srcNode.get_attribute_exists(srcAttrName)) or (not dstNode.get_attribute_exists(dstAttrName)):
                carb.log_error(
                    f"SyntheticData failed to (dis)connect node {srcNode.get_prim_path()}:{srcAttrName} to {dstNode.get_prim_path()}:{dstAttrName}"
                )
                success = False
                # best effort
                continue
            dstAttr = dstNode.get_attribute(dstAttrName)
            srcAttr = srcNode.get_attribute(srcAttrName)
            if enable:
                srcAttr.connect(dstAttr, True)
            else:
                srcAttr.disconnect(dstAttr, True)
        return success

    @staticmethod
    def _auto_connect_nodes(srcNode, dstNode, enable, srcIndex=0) -> bool:
        """Connect a source node to destination node
           The connections are made by matching outputs / inputs node attributes names
           In case of outputs attributes name clashing, the first node in the list is connected
           Optionnally outputs attributes name could be indexed : terminated by underscore followed by the srcNode list index (no leading zero)
           Indexed outputs attributes names take precedence 
        """
        success = False
        for attr in srcNode.get_attributes():
            srcAttrName = attr.get_name()
            if not srcAttrName.startswith("outputs:"):
                continue
            dstAttrName = "inputs:%s_%d" % (srcAttrName[8:], srcIndex)
            if (
                not dstNode.get_attribute_exists(dstAttrName)
                or dstNode.get_attribute(dstAttrName).get_upstream_connection_count()
            ):
                dstAttrName = "inputs:%s" % srcAttrName[8:]
            if (
                not dstNode.get_attribute_exists(dstAttrName)
                or dstNode.get_attribute(dstAttrName).get_upstream_connection_count()
            ):
                continue
            dstAttr = dstNode.get_attribute(dstAttrName)
            srcAttr = srcNode.get_attribute(srcAttrName)
            if enable:
                srcAttr.connect(dstAttr, True)
            else:
                srcAttr.disconnect(dstAttr, True)
            success = True
        return success

    @staticmethod
    def Initialize():
        """Initialize interface singleton instance."""
        global _sdg_iface
        if _sdg_iface is None:
            SyntheticData.register_device_rendervar_tex_to_buff_templates(SyntheticData._ogn_rendervars)
            SyntheticData.register_device_rendervar_to_host_templates(SyntheticData._ogn_rendervars)
            SyntheticData.register_display_rendervar_templates()
            SyntheticData.register_combine_rendervar_templates()
            SyntheticData.register_export_rendervar_ptr_templates(SyntheticData._ogn_rendervars)
            SyntheticData.register_export_rendervar_array_templates(SyntheticData._ogn_rendervars)
            _sdg_iface = SyntheticData()

    @staticmethod
    def Get():
        """Get the interface singleton instance."""
        global _sdg_iface
        return _sdg_iface

    @staticmethod
    def Reset():
        """Reset the interface singleton """
        global _sdg_iface
        if _sdg_iface:
            _sdg_iface.reset()
            _sdg_iface = None

    @staticmethod
    def register_node_template(node_template: NodeTemplate, rendervars: list = None, template_name: str = None) -> str:
        """Register a node template.

        Add a node template in the node registry. After the template has been added it may be activated for being executed in its associated stage.

        Args:
            node_template : template to be added to the registry
            rendervars : list of renderVar the node is producing 
            template_name : unique name id of the template

        Returns:
            the unique name id of the registered template

        """

        # check type
        if og.GraphRegistry().get_node_type_version(node_template.node_type_id) is None:
            raise SyntheticDataException(
                f"failed to register node template. Type {node_template.node_type_id} is not in the registry")

        # check template_name
        if template_name is None:
            numTypeTemplates = 0
            for template in SyntheticData._ogn_templates_registry.values():
                if template.node_type_id == node_template.node_type_id:
                    numTypeTemplates += 1
            template_name = "%s_%04d" % (node_template.node_type_id.split(".")[-1], numTypeTemplates)
        elif template_name in SyntheticData._ogn_templates_registry:
            raise SyntheticDataException(
                f"failed to register node template. Template {template_name} is already in the registry")
        elif template_name in SyntheticData._ogn_rendervars:
            raise SyntheticDataException(
                f"failed to register node template. Template {template_name} is already registered as a renderVar")

        # check connections
        autoStage = SyntheticDataStage.POST_RENDER if rendervars else SyntheticDataStage.SIMULATION
        i_connections = node_template.connections if node_template.connections else []
        for conn in i_connections:
            conn_name = conn.node_template_id

            if conn_name in SyntheticData._ogn_rendervars:
                conn_name = SyntheticData._ogn_rendervars[conn_name]

            if conn_name not in SyntheticData._ogn_templates_registry:
                raise SyntheticDataException(
                    f"failed to register node template. Connection template name {conn_name} is not in the registry")
            conn_stage = SyntheticData._ogn_templates_registry[conn_name].pipeline_stage
            autoStage = max(autoStage, conn_stage)
            conn_map = conn.attributes_mapping if conn.attributes_mapping else {}
            if not type(conn_map) is dict:
                raise SyntheticDataException(
                    f"failed to register node template. connection attributes map is not a dictionnary")

        # check stage
        if node_template.pipeline_stage == SyntheticDataStage.AUTO:
            node_template.pipeline_stage = autoStage
        if node_template.pipeline_stage < autoStage:
            raise SyntheticDataException(
                f"failed to register node template. Stage {node_template.pipeline_stage} is not compatible with the connections")

        # check and register renderVars
        if rendervars:
            if node_template.pipeline_stage != SyntheticDataStage.POST_RENDER:
                raise SyntheticDataException(
                    f"failed to register node template. Only postRender nodes may produce renderVars")
            for rv in rendervars:
                if (rv in SyntheticData._ogn_templates_registry) or (rv in SyntheticData._ogn_rendervars):
                    raise SyntheticDataException(f"failed to register node template. RenderVar {rv} already registered")
                else:
                    SyntheticData._ogn_rendervars[rv] = template_name

        SyntheticData._ogn_templates_registry[template_name] = node_template

        return template_name

    @staticmethod
    def is_node_template_registered(template_name: str) -> bool:
        """Check if a node template has already been registered.

            Args:
                template_name: name of the node template to check

            Returns:
                True if the template_name specifie a node template within the registry, False otherwise

        """
        return template_name in SyntheticData._ogn_templates_registry

    @staticmethod
    def unregister_node_template(template_name: str) -> None:
        """Unregister a node template.

        Remove a node template from the registry and all its dependencies. After removing a template, it cannot be activated anymore, nor its dependent templates.

        """
        SyntheticData._unregister_node_template_rec([template_name])

    def _reset_node_graph(self, nodeGraph):
        graph = nodeGraph.get_wrapped_graph()
        for node in graph.get_nodes():
            graph.destroy_node(node.get_prim_path(), True)
        orchestration_graph = nodeGraph.get_graph()
        orchestration_graph.destroy_node(nodeGraph.get_prim_path(), True)

    def _clear_empty_graphs(self):
        emptyGraph = []
        for graphPath, nodeGraph in self._nodeGraphs.items():
            if nodeGraph.get_wrapped_graph().get_nodes():
                emptyGraph.append(graphPath)
        for graphPath in emptyGraph:
            self._reset_node_graph(nodeGraph)
            self._nodeGraphs.pop(graphPath)

    def _set_process_path(self, renderProductPath, graphPath, processPathAttribueName):
        if not renderProductPath:
            raise SyntheticDataException("invalid renderProductPath")
        usdStage = omni.usd.get_context().get_stage()
        prim = usdStage.GetPrimAtPath(renderProductPath)
        ogprocesspath_attribute = prim.GetAttribute(processPathAttribueName)
        if not ogprocesspath_attribute:
            assert graphPath.endswith("/" + SyntheticData._graphName)
            ogProcessPath = graphPath[: -len("/" + SyntheticData._graphName)]
            prim.CreateAttribute(processPathAttribueName, Sdf.ValueTypeNames.String).Set(ogProcessPath)

    def _get_or_create_graph(self, path: str, stage: int, renderProductPath: object) -> object:
        if path in self._nodeGraphs:
            return self._nodeGraphs[path]

        settings = carb.settings.get_settings()
        use_legacy_simulation_pipeline = settings.get("/persistent/omnigraph/useLegacySimulationPipeline")

        pipelineStage = og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION
        executionModel = "push"
        backingType = og.GraphBackingType.GRAPH_BACKING_TYPE_FLATCACHE_SHARED
        if (stage == SyntheticDataStage.PRE_RENDER) and (not use_legacy_simulation_pipeline):
            pipelineStage = og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_PRERENDER
            backingType = og.GraphBackingType.GRAPH_BACKING_TYPE_FLATCACHE_SHARED  # GRAPH_BACKING_TYPE_FLATCACHE_WITHOUT_HISTORY
        elif (stage == SyntheticDataStage.POST_RENDER) and (not use_legacy_simulation_pipeline):
            pipelineStage = og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_POSTRENDER
            # GRAPH_BACKING_TYPE_FLATCACHE_WITHOUT_HISTORY
            backingType = backingType = og.GraphBackingType.GRAPH_BACKING_TYPE_FLATCACHE_SHARED
        elif (stage == SyntheticDataStage.ON_DEMAND) or (stage == SyntheticDataStage.ON_DEMAND):
            pipelineStage = og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION  # ONDEMAND (FIXME)
            executionModel = "execution"

        usdStage = omni.usd.get_context().get_stage()
        primExistWorkaround = not usdStage.GetPrimAtPath(path)

        orchestration_graphs = og.get_global_orchestration_graphs_in_pipeline_stage(pipelineStage)
        nodeGraph = orchestration_graphs[0].create_graph_as_node(
            path.replace("/", "_"),
            path,
            executionModel,
            True,
            primExistWorkaround,
            backingType,
            pipelineStage,
        )

        if stage == SyntheticDataStage.PRE_RENDER:
            self._set_process_path(renderProductPath, path, "ogPreProcessPath")
        elif stage == SyntheticDataStage.POST_RENDER:
            self._set_process_path(renderProductPath, path, "ogPostProcessPath")

        self._nodeGraphs[path] = nodeGraph
        return nodeGraph

    def _activate_node_rec(self, templateName: str, renderProductIndex: int = -1, renderProductPaths: list = None, render_var_activations: dict = None) -> None:

        renderProductPath = renderProductPaths[renderProductIndex] if renderProductIndex > -1 else None

        #  renderVar template
        if templateName in SyntheticData._ogn_rendervars:
            renderVarName = templateName
            templateName = SyntheticData._ogn_rendervars[templateName]
            if (not render_var_activations is None) and renderProductPath and (templateName == SyntheticData._rendererTemplateName):
                if renderProductPath not in render_var_activations:
                    render_var_activations[renderProductPath]={renderVarName:0}
                elif renderVarName not in render_var_activations[renderProductPath]:
                    render_var_activations[renderProductPath][renderVarName]=0
                render_var_activations[renderProductPath][renderVarName]+=1

        if templateName not in SyntheticData._ogn_templates_registry:
            raise SyntheticDataException(f"graph node template depends on unregistered template {templateName}")

        nodePath = SyntheticData._get_node_path(templateName, renderProductPath)

        if nodePath in self._graphNodes:
            return templateName

        template = SyntheticData._ogn_templates_registry[templateName]

        nodeStage = template.pipeline_stage

        graphPath = SyntheticData._get_graph_path(nodeStage, renderProductPath)
        nodeGraph = self._get_or_create_graph(graphPath, nodeStage, renderProductPath)

        nodeType = template.node_type_id

        usdStage = omni.usd.get_context().get_stage()
        primExistWorkaround = not usdStage.GetPrimAtPath(nodePath)

        self._graphNodes[nodePath] = nodeGraph.get_wrapped_graph().create_node(nodePath, nodeType, primExistWorkaround)
        node = self._graphNodes[nodePath]

        # setup static attributes
        for attrName, attrVal in template.attributes.items():
            if node.get_attribute_exists(attrName):
                node.get_attribute(attrName).set(attrVal)
            else:
                carb.log_error(f"SyntheticData failed to set node {nodePath} static attribute {attrName}")
                # do not return error : the default value in the ogn spec will be used

        # set inputs:renderProductPathPath
        if renderProductPath and node.get_attribute_exists(SyntheticData._renderProductAttributeName):
            node.get_attribute(SyntheticData._renderProductAttributeName).set(renderProductPath)

        # recursive call for upstream connections    
        for connIndex in range(len(template.connections)):

            connection = template.connections[connIndex]
            connTemplateName = connection.node_template_id
            connRenderProductPaths = [renderProductPaths[idx] for idx in connection.render_product_idxs] if (
                renderProductPaths and connection.render_product_idxs) else None
           
            # activate the template
            connTemplateName = self._activate_node_rec(connTemplateName, 0 if connRenderProductPaths else -
                                                       1, connRenderProductPaths, render_var_activations)

            # setup connection attributes
            connRenderProductPath = connRenderProductPaths[0] if connRenderProductPaths else None
            connNodePath = SyntheticData._get_node_path(connTemplateName, connRenderProductPath)
            connNode = self._graphNodes[connNodePath]
            connMap = connection.attributes_mapping
            if not connMap is None:
                if connMap:
                    SyntheticData._connect_nodes(connNode, node, connMap, True)
                else:
                    SyntheticData._auto_connect_nodes(connNode, node, True, connIndex)

        return templateName

    def _deactivate_node_rec(
        self, 
        templateName: str, 
        renderProductIndex: int = -1, 
        renderProductPaths: list = None, 
        render_var_deactivations: dict = None, 
        only_automatically_activated_nodes: bool = True,
        manual_deactivation: bool = True
    ) -> None:

        renderProductPath = renderProductPaths[renderProductIndex] if renderProductIndex > -1 else None

        if templateName in SyntheticData._ogn_rendervars:
            renderVarName = templateName
            templateName = SyntheticData._ogn_rendervars[templateName]
            if (not render_var_deactivations is None) and renderProductPath and (templateName == SyntheticData._rendererTemplateName):
                if renderProductPath not in render_var_deactivations:
                    render_var_deactivations[renderProductPath]={renderVarName:0}
                elif renderVarName not in render_var_deactivations[renderProductPath]:
                    render_var_deactivations[renderProductPath][renderVarName]=0
                render_var_deactivations[renderProductPath][renderVarName]+=1
        
        nodePath = SyntheticData._get_node_path(templateName, renderProductPath)
        
        # prevent automatically deactivating manually activated node
        if (nodePath not in self._graphNodes) or (not manual_deactivation and only_automatically_activated_nodes and (nodePath in self._activatedNodePaths)):
            return templateName
            
        node = self._graphNodes[nodePath]
        template = SyntheticData._ogn_templates_registry[templateName]
        
        # abort if the node has a downstream connection
        for attr in node.get_attributes():
            if attr.get_downstream_connection_count():
                return templateName

        node.get_graph().destroy_node(nodePath, True)
        self._graphNodes.pop(nodePath)

        # remove unused connections
        for connection in template.connections:

            connTemplateName = connection.node_template_id
            connRenderProductPaths = [renderProductPaths[idx] for idx in connection.render_product_idxs] if (
                renderProductPaths and connection.render_product_idxs) else None
            
            # deactivate the template
            self._deactivate_node_rec(connTemplateName, 
                                      0 if connRenderProductPaths else -1, 
                                      connRenderProductPaths, 
                                      render_var_deactivations, 
                                      only_automatically_activated_nodes, 
                                      False)

        return templateName

    def _set_node_attributes(self, nodePath, attributes) -> None:
        if not attributes:
            return
        if not nodePath in self._graphNodes:
            raise SyntheticDataException(f"invalid node {nodePath}")
        node = self._graphNodes[nodePath]
        for attrName, attrVal in attributes.items():
            if node.get_attribute_exists(attrName):
                og.Controller(attribute=node.get_attribute(attrName)).set(value=attrVal)
            else:
                raise SyntheticDataException(f"invalid node attribute {nodePath}.{attrName}")

    def _get_node_attributes(self, nodePath, attribute_names: list, gpu=False) -> dict:
        if not nodePath in self._graphNodes:
            return None
        node = self._graphNodes[nodePath]
        attributes = {}
        for attrName in attribute_names:
            if node.get_attribute_exists(attrName):
                attributes[attrName] = og.Controller(attribute=node.get_attribute(attrName)).get(on_gpu=gpu)
        return attributes

    def __init__(self) -> None:
        self._nodeGraphs = {}
        self._graphNodes = {}
        self._activatedNodePaths = []
        self._render_product_var_activations = {}

    def reset(self, usd=True, remove_activated_render_vars=False) -> None:
        """Reset the SyntheticData instance

        Args:
            usd : if true reset the graph in the usd stage session layer
            remove_activated_render_vars : if True and usd is True remove the render vars activated by the node activation

        If the stage is valid it will destroy every graph created.

        """
        stage = omni.usd.get_context().get_stage()
        if stage and usd:
            session_layer = stage.GetSessionLayer()
            with Usd.EditContext(stage, session_layer):
                for nodeGraph in self._nodeGraphs.values():
                    self._reset_node_graph(nodeGraph)
                if remove_activated_render_vars:
                    for rp, rvs in self._render_product_var_activations.items():
                        for rv, num_act in rvs.items():
                            if num_act[1] and (num_act[0] > 0):
                                self._remove_rendervar(rp,rv,stage)

        self._render_product_var_activations = {}
        self._activatedNodePaths = []
        self._graphNodes = {}
        self._nodeGraphs = {}

    def get_graph(self, stage: int = SyntheticDataStage.ON_DEMAND, renderProductPath: str = None) -> object:
        """Return the graph at a given stage, for a given renderProduct.

        Gives access to the SyntheticData graphs.

        Args:
            stage : SyntheticDataStage of the queried graph
            renderProductPath : (for POST_RENDER stage only) the renderProductPath for which to get the POST_RENDER graph

        Returns:
            the graph at the given stage for the given renderProductPath.

        """
        if renderProductPath and stage != SyntheticDataStage.POST_RENDER:
            raise SyntheticDataException("invalid graph")
        graphPath = SyntheticData._get_graph_path(stage, renderProductPath)
        return self._get_or_create_graph(graphPath, stage, renderProductPath)

    def activate_node_template(
        self,
        template_name: str,
        render_product_path_index: int = -1,
        render_product_paths: list = None,
        attributes: dict = None,
        stage: Usd.Stage = None,
        activate_render_vars: bool = True
    ) -> None:
        """Activate a registered node.

        Create a node instance for the given node template and all its missing dependencies (including nodes and renderVar). 
        The node will be executed during the next stage execution.

        Args:
            template_name : name of the node template to be activate
            render_product_path_index : if the node template is associated to a render product, index of the associated render product in the render product path list
            render_product_paths : render product path list to be used for specifying the render product of the node template and its dependencies to activate
            attributes : dictionnary of attributes to set to the activated "template_name" node
            stage : the stage to change, if None use the stage of the current usd context
            activate_render_vars : if True activate the required render_vars, if False it is the user responsability to activate the required render_vars
        
        Return:
            A dictionnary containing for every render products the list of render var dependencies of this activation
            NB : if activate_render_vars is True those render vars are added

        """

        if (template_name not in SyntheticData._ogn_templates_registry) and (template_name not in SyntheticData._ogn_rendervars):
            raise SyntheticDataException(f'graph node template "{template_name}" unregistered')

        node_path = SyntheticData._get_node_path(
            template_name, render_product_paths[render_product_path_index] if render_product_path_index > -1 else None
        )
        if node_path in self._activatedNodePaths:
            return
        
        if not stage:
            stage = omni.usd.get_context().get_stage()
            if not stage:
                raise SyntheticDataException("invalid USD stage")
        session_layer = stage.GetSessionLayer()
        with Usd.EditContext(stage, session_layer):
            render_var_activations = {}
            self._activate_node_rec(template_name, render_product_path_index, render_product_paths, render_var_activations)
            self._set_node_attributes(node_path, attributes)
            self._activatedNodePaths.append(node_path)
            # maintain the render_vars activation number for every render products
            activated_render_vars = {}
            for rp, rvs in render_var_activations.items():
                if rp not in self._render_product_var_activations:
                    self._render_product_var_activations[rp]={}
                for rv, num in rvs.items():
                    need_activation = not self._has_rendervar(rp,rv,stage) 
                    if rv not in self._render_product_var_activations[rp]:
                        self._render_product_var_activations[rp][rv] = [num, need_activation and activate_render_vars]
                    else:
                        self._render_product_var_activations[rp][rv][0] += num            
                        self._render_product_var_activations[rp][rv][1] = need_activation and activate_render_vars           
                    if need_activation:
                        if rp not in activated_render_vars:
                            activated_render_vars[rp]=[]
                        if rv not in activated_render_vars[rp]:
                            activated_render_vars[rp].append(rv)
                    
            if activate_render_vars:
                for rp, rvs in activated_render_vars.items():
                    for rv in rvs: 
                        SyntheticData._add_rendervar(rp, rv, stage)

        return activated_render_vars

    def is_node_template_activated(
        self,
        template_name: str,
        render_product_path: str = None,
        only_manually_activated: bool = False
    ) -> None:
        """Query the activation status of a node template.

        Args:
            template_name : name of the node template to query the activation status
            render_product_path : render product path for which to check the template activation status (None if not applicable)
            only_manually_activated: if True check the activation for only the explicitely activated templates ( exclude the automatically activated template )
            
        Return:
            True if the node template is currently activated and, if only_explicitely_activated is True, if it has been explicitely activated

        """
        node_path = SyntheticData._get_node_path(template_name, render_product_path)
        return node_path in self._activatedNodePaths if only_manually_activated else node_path in self._graphNodes

    def deactivate_node_template(
        self, 
        template_name: str, 
        render_product_path_index: int = -1, 
        render_product_paths: list = [], 
        stage: Usd.Stage = None,
        deactivate_render_vars: bool = False,
        recurse_only_automatically_activated: bool = True
        
    ) -> None:
        """Deactivate a registered node.

        Delete a node instance for the given node template and all its automatically activated dependencies with no more downstream connections.
        The node won't be executed anymore starting with the next stage execution.

        Args:
            template_name : name of the node template to deactivate
            render_product_path_index : if the node template is associated to a render product, index of the associated render product in the render product path list
            render_product_paths : render product path list to be used for specifying the render product of the node template and its dependencies to deactivate
            stage : the stage to change, if None use the stage of the current usd context
            deactivate_render_vars : if True deactivate the render_vars that have been activated in a call to activate_node_template and which are not used anymore by the managed graphs.
                                     Beware that in some cases, some of these render vars maybe actually used by other graphs, hence it is False by default 
                                     if False it is the user responsability to deactivate the unused render_vars.
            recurse_only_automatically_activated : if True recursively deactivate only automatically activated upstream nodes without other connections 
                                                   if False recursively deactivate all upstream nodes without other connections 
        
        Return:
            A dictionnary containing for every render products path the list of render var dependencies that have been activated by activate_node_template and are not used anymore by the managed graphs.
            NB : if deactivate_render_vars is True those render vars are removed

        """
        if not stage:
            stage = omni.usd.get_context().get_stage()
            if not stage:
                raise SyntheticDataException("invalid USD stage")
        session_layer = stage.GetSessionLayer()
        with Usd.EditContext(stage, session_layer):
            render_var_deactivations = {}
            self._deactivate_node_rec(template_name, render_product_path_index, render_product_paths, render_var_deactivations, recurse_only_automatically_activated)
            node_path = SyntheticData._get_node_path(
                template_name, render_product_paths[render_product_path_index] if render_product_path_index > -1 else None
            )
            if (node_path in self._activatedNodePaths) and (node_path not in self._graphNodes):
                self._activatedNodePaths.remove(node_path)
            # maintain the render_vars activation number for every render products
            deactivated_render_vars = {}
            for rp, rvs in render_var_deactivations.items():
                valid_rp = rp in  self._render_product_var_activations
                for rv, num in rvs.items():
                    valid_rv = valid_rp and rv in self._render_product_var_activations[rp]
                    if valid_rv and (self._render_product_var_activations[rp][rv][0] <= num):
                        if self._render_product_var_activations[rp][rv][1]:
                            if rp not in deactivated_render_vars:
                                deactivated_render_vars[rp]=[rv]
                            else:
                                deactivated_render_vars[rp].append(rv)
                        self._render_product_var_activations[rp].pop(rv) 
                    elif valid_rv:
                        self._render_product_var_activations[rp][rv][0] -= num

            if deactivate_render_vars:
                for rp, rvs in deactivated_render_vars.items():
                    for rv in rvs: 
                        SyntheticData._remove_rendervar(rp, rv, stage)

        return deactivated_render_vars

    def connect_node_template(self, src_template_name: str, dst_template_name: str, render_product_path: str=None, connection_map: dict=None):
        """Connect the given source node template to the destination node template

        Args:
            src_template_name : name of the source node template
            dst_template_name : name of the destination node template
            render_product_path : render product path of the node templates (None if the node are not specific to a render product)
            connection_map : attribute mapping for the source inputs to the destination outputs. (None for an automatic mapping based on names) 
        """
        src_node_path = SyntheticData._get_node_path(src_template_name, render_product_path)
        if src_node_path not in self._graphNodes:
            raise SyntheticDataException(f'cannot connect node template : "{src_node_path}" not activated')
        else:
            src_node = self._graphNodes[src_node_path]
        dst_node_path = SyntheticData._get_node_path(dst_template_name, render_product_path)
        if dst_node_path not in self._graphNodes:
            raise SyntheticDataException(f'cannot connect node template : "{dst_node_path}" not activated')
        else:
            dst_node = self._graphNodes[dst_node_path]
        if connection_map:
            SyntheticData._connect_nodes(src_node, dst_node, connection_map, True)
        else:
            SyntheticData._auto_connect_nodes(src_node, dst_node, True)

    def disconnect_node_template(self, src_template_name: str, dst_template_name: str, render_product_path: str=None, connection_map: dict=None):
        """Disconnect the given source node template to the destination node template

        Args:
            src_template_name : name of the source node template
            dst_template_name : name of the destination node template
            render_product_path : render product path of the node templates (None if the node are not specific to a render product)
            connection_map : attribute mapping for the source inputs to the destination outputs. (None for an automatic mapping based on names) 
        """
        src_node_path = SyntheticData._get_node_path(src_template_name, render_product_path)
        if src_node_path not in self._graphNodes:
            raise SyntheticDataException(f'cannot disconnect node template : "{src_node_path}" not activated')
        else:
            src_node = self._graphNodes[src_node_path]
        dst_node_path = SyntheticData._get_node_path(dst_template_name, render_product_path)
        if dst_node_path not in self._graphNodes:
            raise SyntheticDataException(f'cannot disconnect node template : "{dst_node_path}" not activated')
        else:
            dst_node = self._graphNodes[dst_node_path]
        if connection_map:
            SyntheticData._connect_nodes(src_node, dst_node, connection_map, False)
        else:
            SyntheticData._auto_connect_nodes(src_node, dst_node, False)
    
    def set_node_attributes(self, template_name: str, attributes: dict, render_product_path: str=None) -> None:
        """Set the value of an activated node attribute.

        The function may be used to set the value of multiple activated node input attributes before the execution of its stage.

        Args:
            template_name : name of the activated node
            render_product_path : if the activated node is associated to a render product, provide its path 
            attributes : dictionnary of attribute name/value to set 

        """
        node_path = SyntheticData._get_node_path(template_name, render_product_path)
        self._set_node_attributes(node_path, attributes)

    def get_node_attributes(
        self, template_name: str, attribute_names: list, render_product_path=None, gpu=False
    ) -> dict:
        """Get the value of several activated node's attributes.

        The function may be used to retrieve the value of multiple activated node output attributes after the execution of its graph.

        Args:
            template_name : name of the activated node
            attribute_names : list of node attribute names to retrieve the value
            render_product_path : if the activated node is associated to a render product, provide its path 
            gpu : for array data attribute, get a gpu data
            
        Returns:
            A dictionnary of attribute name/value for every successfully retrieved attributes
            None if the node is not a valid activated node

        """
        node_path = SyntheticData._get_node_path(template_name, render_product_path)
        return self._get_node_attributes(node_path, attribute_names, gpu)

    def set_instance_mapping_semantic_filter(self, predicate="*:*"):
        """Set the semantic filter predicate to be applied to the instance mapping. Contrary to the default
           semantic filter this filter affect the instance mapping. All semantic data filtered at this level is
           not available in the instance mapping.

        Args:
            predicate : a semantic filter predicate.
            
            predicate examples : 
                "typeA : labelA & !labelB | labelC , typeB: labelA ; typeC: labelD" 
                "typeA : * ; * : labelA"
        """
        SyntheticData._ogn_templates_registry[SyntheticData._instanceMappingCtrl].attributes["inputs:semanticFilterPredicate"] = predicate    
        node_path = SyntheticData._get_node_path(SyntheticData._instanceMappingCtrl)
        if node_path in self._graphNodes:
            self.set_node_attributes(SyntheticData._instanceMappingCtrl, {"inputs:semanticFilterPredicate":predicate})
        
    def set_default_semantic_filter(self, predicate="*:*", hierarchical_labels=False, matching_labels=True):
        """Set the default semantic filter predicate.

        Args:
            predicate : a semantic filter predicate.
            hierarchical_labels : option to propagate semantic labels within the hiearchy, from parent to childrens
            matching_labels : option to remove from the set of labels the one that do not match the predicate

            predicate examples : 
                "typeA : labelA & !labelB | labelC , typeB: labelA ; typeC: labelD" 
                "typeA : * ; * : labelA"
        """
        node_path = SyntheticData._get_node_path(SyntheticData._defaultSemanticFilterName)
        attributes = {"inputs:predicate": predicate, "inputs:hierarchicalLabels": hierarchical_labels,
                      "inputs:matchingLabels": matching_labels}
        if node_path in self._graphNodes:
            self.set_node_attributes(SyntheticData._defaultSemanticFilterName, attributes)
        else:
            self.activate_node_template(SyntheticData._defaultSemanticFilterName, attributes=attributes)
    
    def enable_rendervar(self, render_product_path:str, render_var:str, usd_stage: Usd.Stage = None) -> None:
        """Explicitely enable the computation of a render_var for a given render_product.

        Args:
            render_product_path : the render_product for which to enable the given render_var computation
            render_var : the name of the render_var to enable 
            usd_stage : usd stage 
        """
        SyntheticData._add_rendervar(render_product_path, render_var, usd_stage)

    def disable_rendervar(self, render_product_path:str, render_var:str, usd_stage: Usd.Stage = None) -> None:
        """Explicitely disable the computation of a render_var for a given render_product.

        Args:
            render_product_path : the render_product for which to disable the given render_var computation
            render_var : the name of the render_var to disable
            usd_stage : usd stage 
        """
        SyntheticData._remove_rendervar(render_product_path, render_var, usd_stage)

    def is_rendervar_used(self, render_product_path:str, render_var:str) -> None:
        """ query the used status of a render var for a render product
        
        Args:
            render_product_path: the path of the render product
            renver_var: the name of the render_var
            
        Returns:
            True if the given render var is currently in use by the activated syntheticData nodes for the given render product

        """
        if (render_product_path in self._render_product_var_activations) and (render_var in self._render_product_var_activations[render_product_path]):
            return self._render_product_var_activations[render_product_path][render_var][0] > 0
        else:
            return False    

    def is_rendervar_enabled(self, render_product_path:str, render_var:str, only_sdg_activated: bool = False, usd_stage: Usd.Stage = None) -> None:
        """ query the enabled status of a render var for a render product
        
        Args:
            render_product_path: the path of the render product
            renver_var: the name of the render_var
            only_sdg_activated: consider only the render var automatically enabled by a call to activate_node_template
            usd_stage: the usd stage (if None use the current usd context stage)
        
        Returns:
            True if the given render var is currently enabled for the given render product 
                 and, if only_sdg_activated is True, if it has been enabled by a call to activate_node_template

        """
        if only_sdg_activated:
            if (render_product_path in self._render_product_var_activations) and (render_var in self._render_product_var_activations[render_product_path]):
                return self._render_product_var_activations[render_product_path][render_var][1]
            else:
                return False 
        else:
            return SyntheticData._has_rendervar(render_product_path, render_var, usd_stage)
