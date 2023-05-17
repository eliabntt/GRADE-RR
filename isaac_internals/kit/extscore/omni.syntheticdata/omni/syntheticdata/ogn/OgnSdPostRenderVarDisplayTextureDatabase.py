"""Support for simplified access to data on nodes of type omni.syntheticdata.SdPostRenderVarDisplayTexture

Synthetic Data node to copy the input aov texture into the corresponding visualization texture
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import numpy
class OgnSdPostRenderVarDisplayTextureDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.syntheticdata.SdPostRenderVarDisplayTexture

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.cameraFisheyeParams
            inputs.cameraModel
            inputs.cameraNearFar
            inputs.exec
            inputs.gpu
            inputs.instanceMapSDCudaPtr
            inputs.instanceMappingInfoSDPtr
            inputs.metersPerSceneUnit
            inputs.mode
            inputs.parameters
            inputs.renderVar
            inputs.renderVarDisplay
            inputs.rp
            inputs.sdDisplayHeight
            inputs.sdDisplayWidth
            inputs.sdSemBBox3dCamCornersCudaPtr
            inputs.sdSemBBox3dCamExtentCudaPtr
            inputs.sdSemBBoxExtentCudaPtr
            inputs.sdSemBBoxInfosCudaPtr
            inputs.semanticLabelTokenSDCudaPtr
            inputs.semanticMapSDCudaPtr
            inputs.semanticPrimTokenSDCudaPtr
            inputs.semanticWorldTransformSDCudaPtr
            inputs.swhFrameNumber
        Outputs:
            outputs.cudaPtr
            outputs.exec
            outputs.format
            outputs.height
            outputs.renderVarDisplay
            outputs.width

    Predefined Tokens:
        tokens.LdrColorSD
        tokens.Camera3dPositionSD
        tokens.DistanceToImagePlaneSD
        tokens.DistanceToCameraSD
        tokens.InstanceSegmentationSD
        tokens.SemanticSegmentationSD
        tokens.NormalSD
        tokens.TargetMotionSD
        tokens.BoundingBox2DTightSD
        tokens.BoundingBox2DLooseSD
        tokens.BoundingBox3DSD
        tokens.OcclusionSD
        tokens.TruncationSD
        tokens.CrossCorrespondenceSD
        tokens.SemanticBoundingBox2DExtentTightSD
        tokens.SemanticBoundingBox2DInfosTightSD
        tokens.SemanticBoundingBox2DExtentLooseSD
        tokens.SemanticBoundingBox2DInfosLooseSD
        tokens.SemanticBoundingBox3DExtentSD
        tokens.SemanticBoundingBox3DInfosSD
        tokens.SemanticBoundingBox3DCamCornersSD
        tokens.SemanticBoundingBox3DDisplayAxesSD
        tokens.autoMode
        tokens.colorMode
        tokens.scaled3dVectorMode
        tokens.clippedValueMode
        tokens.normalized3dVectorMode
        tokens.segmentationMapMode
        tokens.instanceMapMode
        tokens.semanticPathMode
        tokens.semanticLabelMode
        tokens.semanticBoundingBox2dMode
        tokens.rawBoundingBox2dMode
        tokens.semanticProjBoundingBox3dMode
        tokens.semanticBoundingBox3dMode
        tokens.rawBoundingBox3dMode
        tokens.pinhole
        tokens.perspective
        tokens.orthographic
        tokens.fisheyePolynomial
    """
    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}
    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:cameraFisheyeParams', 'float[]', 0, None, 'Camera fisheye projection parameters', {}, True, [], False, ''),
        ('inputs:cameraModel', 'int', 0, None, 'Camera model (pinhole or fisheye models)', {}, True, 0, False, ''),
        ('inputs:cameraNearFar', 'float2', 0, None, 'Camera near/far clipping range', {}, True, [0.0, 0.0], False, ''),
        ('inputs:exec', 'execution', 0, None, 'Trigger', {}, True, None, False, ''),
        ('inputs:gpu', 'uint64', 0, 'gpuFoundations', 'Pointer to shared context containing gpu foundations', {}, True, 0, False, ''),
        ('inputs:instanceMapSDCudaPtr', 'uint64', 0, None, 'cuda uint16_t buffer pointer of size numInstances containing the instance parent semantic index', {}, True, 0, False, ''),
        ('inputs:instanceMappingInfoSDPtr', 'uint64', 0, None, 'uint buffer pointer containing the following information : [numInstances, minInstanceId, numSemantics, minSemanticId, numProtoSemantic]', {}, True, 0, False, ''),
        ('inputs:metersPerSceneUnit', 'float', 0, None, 'Scene units to meters scale', {}, True, 0.0, False, ''),
        ('inputs:mode', 'token', 0, None, 'Display mode', {ogn.MetadataKeys.DEFAULT: '"autoMode"'}, True, 'autoMode', False, ''),
        ('inputs:parameters', 'float4', 0, None, 'Display parameters', {ogn.MetadataKeys.DEFAULT: '[0.0, 5.0, 0.33, 0.27]'}, True, [0.0, 5.0, 0.33, 0.27], False, ''),
        ('inputs:renderVar', 'token', 0, None, 'Name of the input RenderVar to display', {}, True, '', False, ''),
        ('inputs:renderVarDisplay', 'token', 0, None, 'Name of the output display RenderVar', {}, True, '', False, ''),
        ('inputs:rp', 'uint64', 0, 'renderProduct', 'Pointer to render product for this view', {}, True, 0, False, ''),
        ('inputs:sdDisplayHeight', 'uint', 0, None, 'Visualization texture Height', {}, True, 0, False, ''),
        ('inputs:sdDisplayWidth', 'uint', 0, None, 'Visualization texture width', {}, True, 0, False, ''),
        ('inputs:sdSemBBox3dCamCornersCudaPtr', 'uint64', 0, None, 'Cuda buffer containing the projection of the 3d bounding boxes on the camera plane represented as a float3=(u,v,z,a) for each bounding box corners', {}, True, 0, False, ''),
        ('inputs:sdSemBBox3dCamExtentCudaPtr', 'uint64', 0, None, 'Cuda buffer containing the 2d extent of the 3d bounding boxes on the camera plane represented as a float6=(u_min,u_max,v_min,v_max,z_min,z_max)', {}, True, 0, False, ''),
        ('inputs:sdSemBBoxExtentCudaPtr', 'uint64', 0, None, 'Cuda buffer containing the extent of the bounding boxes as a float4=(u_min,v_min,u_max,v_max) for 2D or a float6=(xmin,ymin,zmin,xmax,ymax,zmax) in object space for 3D', {}, True, 0, False, ''),
        ('inputs:sdSemBBoxInfosCudaPtr', 'uint64', 0, None, 'Cuda buffer containing valid bounding boxes infos', {}, True, 0, False, ''),
        ('inputs:semanticLabelTokenSDCudaPtr', 'uint64', 0, None, 'cuda uint64_t buffer pointer of size numSemantics containing the semantic label token', {}, True, 0, False, ''),
        ('inputs:semanticMapSDCudaPtr', 'uint64', 0, None, 'cuda uint16_t buffer pointer of size numSemantics containing the semantic parent semantic index', {}, True, 0, False, ''),
        ('inputs:semanticPrimTokenSDCudaPtr', 'uint64', 0, None, 'cuda uint64_t buffer pointer of size numSemantics containing the semantic path token', {}, True, 0, False, ''),
        ('inputs:semanticWorldTransformSDCudaPtr', 'uint64', 0, None, 'cuda float44 buffer pointer of size numSemantics containing the world semantic transform', {}, True, 0, False, ''),
        ('inputs:swhFrameNumber', 'uint64', 0, None, 'Fabric frame number', {}, True, 0, False, ''),
        ('outputs:cudaPtr', 'uint64', 0, None, 'Display texture CUDA pointer', {}, True, None, False, ''),
        ('outputs:exec', 'execution', 0, None, 'Trigger', {}, True, None, False, ''),
        ('outputs:format', 'uint64', 0, None, 'Display texture format', {}, True, None, False, ''),
        ('outputs:height', 'uint', 0, None, 'Display texture height', {}, True, None, False, ''),
        ('outputs:renderVarDisplay', 'token', 0, None, 'Name of the output display RenderVar', {}, True, None, False, ''),
        ('outputs:width', 'uint', 0, None, 'Display texture width', {}, True, None, False, ''),
    ])
    class tokens:
        LdrColorSD = "LdrColorSD"
        Camera3dPositionSD = "Camera3dPositionSD"
        DistanceToImagePlaneSD = "DistanceToImagePlaneSD"
        DistanceToCameraSD = "DistanceToCameraSD"
        InstanceSegmentationSD = "InstanceSegmentationSD"
        SemanticSegmentationSD = "SemanticSegmentationSD"
        NormalSD = "NormalSD"
        TargetMotionSD = "TargetMotionSD"
        BoundingBox2DTightSD = "BoundingBox2DTightSD"
        BoundingBox2DLooseSD = "BoundingBox2DLooseSD"
        BoundingBox3DSD = "BoundingBox3DSD"
        OcclusionSD = "OcclusionSD"
        TruncationSD = "TruncationSD"
        CrossCorrespondenceSD = "CrossCorrespondenceSD"
        SemanticBoundingBox2DExtentTightSD = "SemanticBoundingBox2DExtentTightSD"
        SemanticBoundingBox2DInfosTightSD = "SemanticBoundingBox2DInfosTightSD"
        SemanticBoundingBox2DExtentLooseSD = "SemanticBoundingBox2DExtentLooseSD"
        SemanticBoundingBox2DInfosLooseSD = "SemanticBoundingBox2DInfosLooseSD"
        SemanticBoundingBox3DExtentSD = "SemanticBoundingBox3DExtentSD"
        SemanticBoundingBox3DInfosSD = "SemanticBoundingBox3DInfosSD"
        SemanticBoundingBox3DCamCornersSD = "SemanticBoundingBox3DCamCornersSD"
        SemanticBoundingBox3DDisplayAxesSD = "SemanticBoundingBox3DDisplayAxesSD"
        autoMode = "autoMode"
        colorMode = "colorMode"
        scaled3dVectorMode = "scaled3dVectorMode"
        clippedValueMode = "clippedValueMode"
        normalized3dVectorMode = "normalized3dVectorMode"
        segmentationMapMode = "segmentationMapMode"
        instanceMapMode = "instanceMapMode"
        semanticPathMode = "semanticPathMode"
        semanticLabelMode = "semanticLabelMode"
        semanticBoundingBox2dMode = "semanticBoundingBox2dMode"
        rawBoundingBox2dMode = "rawBoundingBox2dMode"
        semanticProjBoundingBox3dMode = "semanticProjBoundingBox3dMode"
        semanticBoundingBox3dMode = "semanticBoundingBox3dMode"
        rawBoundingBox3dMode = "rawBoundingBox3dMode"
        pinhole = "pinhole"
        perspective = "perspective"
        orthographic = "orthographic"
        fisheyePolynomial = "fisheyePolynomial"
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.exec = og.Database.ROLE_EXECUTION
        role_data.outputs.exec = og.Database.ROLE_EXECUTION
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"cameraModel", "cameraNearFar", "exec", "gpu", "instanceMapSDCudaPtr", "instanceMappingInfoSDPtr", "metersPerSceneUnit", "mode", "parameters", "renderVar", "renderVarDisplay", "rp", "sdDisplayHeight", "sdDisplayWidth", "sdSemBBox3dCamCornersCudaPtr", "sdSemBBox3dCamExtentCudaPtr", "sdSemBBoxExtentCudaPtr", "sdSemBBoxInfosCudaPtr", "semanticLabelTokenSDCudaPtr", "semanticMapSDCudaPtr", "semanticPrimTokenSDCudaPtr", "semanticWorldTransformSDCudaPtr", "swhFrameNumber", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.cameraModel, self._attributes.cameraNearFar, self._attributes.exec, self._attributes.gpu, self._attributes.instanceMapSDCudaPtr, self._attributes.instanceMappingInfoSDPtr, self._attributes.metersPerSceneUnit, self._attributes.mode, self._attributes.parameters, self._attributes.renderVar, self._attributes.renderVarDisplay, self._attributes.rp, self._attributes.sdDisplayHeight, self._attributes.sdDisplayWidth, self._attributes.sdSemBBox3dCamCornersCudaPtr, self._attributes.sdSemBBox3dCamExtentCudaPtr, self._attributes.sdSemBBoxExtentCudaPtr, self._attributes.sdSemBBoxInfosCudaPtr, self._attributes.semanticLabelTokenSDCudaPtr, self._attributes.semanticMapSDCudaPtr, self._attributes.semanticPrimTokenSDCudaPtr, self._attributes.semanticWorldTransformSDCudaPtr, self._attributes.swhFrameNumber]
            self._batchedReadValues = [0, [0.0, 0.0], None, 0, 0, 0, 0.0, "autoMode", [0.0, 5.0, 0.33, 0.27], "", "", 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        @property
        def cameraFisheyeParams(self):
            data_view = og.AttributeValueHelper(self._attributes.cameraFisheyeParams)
            return data_view.get()

        @cameraFisheyeParams.setter
        def cameraFisheyeParams(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.cameraFisheyeParams)
            data_view = og.AttributeValueHelper(self._attributes.cameraFisheyeParams)
            data_view.set(value)
            self.cameraFisheyeParams_size = data_view.get_array_size()

        @property
        def cameraModel(self):
            return self._batchedReadValues[0]

        @cameraModel.setter
        def cameraModel(self, value):
            self._batchedReadValues[0] = value

        @property
        def cameraNearFar(self):
            return self._batchedReadValues[1]

        @cameraNearFar.setter
        def cameraNearFar(self, value):
            self._batchedReadValues[1] = value

        @property
        def exec(self):
            return self._batchedReadValues[2]

        @exec.setter
        def exec(self, value):
            self._batchedReadValues[2] = value

        @property
        def gpu(self):
            return self._batchedReadValues[3]

        @gpu.setter
        def gpu(self, value):
            self._batchedReadValues[3] = value

        @property
        def instanceMapSDCudaPtr(self):
            return self._batchedReadValues[4]

        @instanceMapSDCudaPtr.setter
        def instanceMapSDCudaPtr(self, value):
            self._batchedReadValues[4] = value

        @property
        def instanceMappingInfoSDPtr(self):
            return self._batchedReadValues[5]

        @instanceMappingInfoSDPtr.setter
        def instanceMappingInfoSDPtr(self, value):
            self._batchedReadValues[5] = value

        @property
        def metersPerSceneUnit(self):
            return self._batchedReadValues[6]

        @metersPerSceneUnit.setter
        def metersPerSceneUnit(self, value):
            self._batchedReadValues[6] = value

        @property
        def mode(self):
            return self._batchedReadValues[7]

        @mode.setter
        def mode(self, value):
            self._batchedReadValues[7] = value

        @property
        def parameters(self):
            return self._batchedReadValues[8]

        @parameters.setter
        def parameters(self, value):
            self._batchedReadValues[8] = value

        @property
        def renderVar(self):
            return self._batchedReadValues[9]

        @renderVar.setter
        def renderVar(self, value):
            self._batchedReadValues[9] = value

        @property
        def renderVarDisplay(self):
            return self._batchedReadValues[10]

        @renderVarDisplay.setter
        def renderVarDisplay(self, value):
            self._batchedReadValues[10] = value

        @property
        def rp(self):
            return self._batchedReadValues[11]

        @rp.setter
        def rp(self, value):
            self._batchedReadValues[11] = value

        @property
        def sdDisplayHeight(self):
            return self._batchedReadValues[12]

        @sdDisplayHeight.setter
        def sdDisplayHeight(self, value):
            self._batchedReadValues[12] = value

        @property
        def sdDisplayWidth(self):
            return self._batchedReadValues[13]

        @sdDisplayWidth.setter
        def sdDisplayWidth(self, value):
            self._batchedReadValues[13] = value

        @property
        def sdSemBBox3dCamCornersCudaPtr(self):
            return self._batchedReadValues[14]

        @sdSemBBox3dCamCornersCudaPtr.setter
        def sdSemBBox3dCamCornersCudaPtr(self, value):
            self._batchedReadValues[14] = value

        @property
        def sdSemBBox3dCamExtentCudaPtr(self):
            return self._batchedReadValues[15]

        @sdSemBBox3dCamExtentCudaPtr.setter
        def sdSemBBox3dCamExtentCudaPtr(self, value):
            self._batchedReadValues[15] = value

        @property
        def sdSemBBoxExtentCudaPtr(self):
            return self._batchedReadValues[16]

        @sdSemBBoxExtentCudaPtr.setter
        def sdSemBBoxExtentCudaPtr(self, value):
            self._batchedReadValues[16] = value

        @property
        def sdSemBBoxInfosCudaPtr(self):
            return self._batchedReadValues[17]

        @sdSemBBoxInfosCudaPtr.setter
        def sdSemBBoxInfosCudaPtr(self, value):
            self._batchedReadValues[17] = value

        @property
        def semanticLabelTokenSDCudaPtr(self):
            return self._batchedReadValues[18]

        @semanticLabelTokenSDCudaPtr.setter
        def semanticLabelTokenSDCudaPtr(self, value):
            self._batchedReadValues[18] = value

        @property
        def semanticMapSDCudaPtr(self):
            return self._batchedReadValues[19]

        @semanticMapSDCudaPtr.setter
        def semanticMapSDCudaPtr(self, value):
            self._batchedReadValues[19] = value

        @property
        def semanticPrimTokenSDCudaPtr(self):
            return self._batchedReadValues[20]

        @semanticPrimTokenSDCudaPtr.setter
        def semanticPrimTokenSDCudaPtr(self, value):
            self._batchedReadValues[20] = value

        @property
        def semanticWorldTransformSDCudaPtr(self):
            return self._batchedReadValues[21]

        @semanticWorldTransformSDCudaPtr.setter
        def semanticWorldTransformSDCudaPtr(self, value):
            self._batchedReadValues[21] = value

        @property
        def swhFrameNumber(self):
            return self._batchedReadValues[22]

        @swhFrameNumber.setter
        def swhFrameNumber(self, value):
            self._batchedReadValues[22] = value

        def __getattr__(self, item: str):
            if item in self.LOCAL_PROPERTY_NAMES:
                return object.__getattribute__(self, item)
            else:
                return super().__getattr__(item)

        def __setattr__(self, item: str, new_value):
            if item in self.LOCAL_PROPERTY_NAMES:
                object.__setattr__(self, item, new_value)
            else:
                super().__setattr__(item, new_value)

        def _prefetch(self):
            readAttributes = self._batchedReadAttributes
            newValues = _og._prefetch_input_attributes_data(readAttributes)
            if len(readAttributes) == len(newValues):
                self._batchedReadValues = newValues
    class ValuesForOutputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"cudaPtr", "exec", "format", "height", "renderVarDisplay", "width", "_batchedWriteValues"}
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedWriteValues = { }

        @property
        def cudaPtr(self):
            value = self._batchedWriteValues.get(self._attributes.cudaPtr)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.cudaPtr)
                return data_view.get()

        @cudaPtr.setter
        def cudaPtr(self, value):
            self._batchedWriteValues[self._attributes.cudaPtr] = value

        @property
        def exec(self):
            value = self._batchedWriteValues.get(self._attributes.exec)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.exec)
                return data_view.get()

        @exec.setter
        def exec(self, value):
            self._batchedWriteValues[self._attributes.exec] = value

        @property
        def format(self):
            value = self._batchedWriteValues.get(self._attributes.format)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.format)
                return data_view.get()

        @format.setter
        def format(self, value):
            self._batchedWriteValues[self._attributes.format] = value

        @property
        def height(self):
            value = self._batchedWriteValues.get(self._attributes.height)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.height)
                return data_view.get()

        @height.setter
        def height(self, value):
            self._batchedWriteValues[self._attributes.height] = value

        @property
        def renderVarDisplay(self):
            value = self._batchedWriteValues.get(self._attributes.renderVarDisplay)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.renderVarDisplay)
                return data_view.get()

        @renderVarDisplay.setter
        def renderVarDisplay(self, value):
            self._batchedWriteValues[self._attributes.renderVarDisplay] = value

        @property
        def width(self):
            value = self._batchedWriteValues.get(self._attributes.width)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.width)
                return data_view.get()

        @width.setter
        def width(self, value):
            self._batchedWriteValues[self._attributes.width] = value

        def __getattr__(self, item: str):
            if item in self.LOCAL_PROPERTY_NAMES:
                return object.__getattribute__(self, item)
            else:
                return super().__getattr__(item)

        def __setattr__(self, item: str, new_value):
            if item in self.LOCAL_PROPERTY_NAMES:
                object.__setattr__(self, item, new_value)
            else:
                super().__setattr__(item, new_value)

        def _commit(self):
            _og._commit_output_attributes_data(self._batchedWriteValues)
            self._batchedWriteValues = { }
    class ValuesForState(og.DynamicAttributeAccess):
        """Helper class that creates natural hierarchical access to state attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
    def __init__(self, node):
        super().__init__(node)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_INPUT)
        self.inputs = OgnSdPostRenderVarDisplayTextureDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnSdPostRenderVarDisplayTextureDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnSdPostRenderVarDisplayTextureDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
