"""Support for simplified access to data on nodes of type omni.syntheticdata.SdPostSemantic3dBoundingBoxCameraProjection

Synthetic Data node to project 3d bounding boxes data in camera space.
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import numpy
class OgnSdPostSemantic3dBoundingBoxCameraProjectionDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.syntheticdata.SdPostSemantic3dBoundingBoxCameraProjection

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.cameraFisheyeParams
            inputs.cameraModel
            inputs.cameraNearFar
            inputs.exec
            inputs.gpu
            inputs.instanceMappingInfoSDPtr
            inputs.metersPerSceneUnit
            inputs.renderProductResolution
            inputs.rp
            inputs.sdSemBBoxExtentCudaPtr
            inputs.sdSemBBoxInfosCudaPtr
            inputs.semanticWorldTransformSDCudaPtr
            inputs.viewportNearFar
            inputs.viewportResolution
        Outputs:
            outputs.exec
            outputs.sdSemBBox3dCamCornersCudaPtr
            outputs.sdSemBBox3dCamExtentCudaPtr

    Predefined Tokens:
        tokens.SemanticBoundingBox3DInfosSD
        tokens.SemanticBoundingBox3DCamCornersSD
        tokens.SemanticBoundingBox3DCamExtentSD
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
        ('inputs:instanceMappingInfoSDPtr', 'uint64', 0, None, 'uint buffer pointer containing the following information : [numInstances, minInstanceId, numSemantics, minSemanticId, numProtoSemantic]', {}, True, 0, False, ''),
        ('inputs:metersPerSceneUnit', 'float', 0, None, 'Scene units to meters scale', {ogn.MetadataKeys.DEFAULT: '0.01'}, True, 0.01, False, ''),
        ('inputs:renderProductResolution', 'int2', 0, None, 'RenderProduct resolution', {}, True, [0, 0], False, ''),
        ('inputs:rp', 'uint64', 0, 'renderProduct', 'Pointer to render product for this view', {}, True, 0, False, ''),
        ('inputs:sdSemBBoxExtentCudaPtr', 'uint64', 0, None, 'Cuda buffer containing the extent of the bounding boxes as a float4=(u_min,v_min,u_max,v_max) for 2D or a float6=(xmin,ymin,zmin,xmax,ymax,zmax) in object space for 3D', {}, True, 0, False, ''),
        ('inputs:sdSemBBoxInfosCudaPtr', 'uint64', 0, None, 'Cuda buffer containing valid bounding boxes infos', {}, True, 0, False, ''),
        ('inputs:semanticWorldTransformSDCudaPtr', 'uint64', 0, None, 'cuda float44 buffer pointer of size numSemantics containing the world semantic transform', {}, True, 0, False, ''),
        ('inputs:viewportNearFar', 'float2', 0, None, 'near and far plane (in scene units) used to clip the 3d bounding boxes.', {ogn.MetadataKeys.DEFAULT: '[1.0, 10000000.0]'}, True, [1.0, 10000000.0], False, ''),
        ('inputs:viewportResolution', 'int2', 0, None, 'viewport width and height (in pixels) used to clip the 3d bounding boxes.', {ogn.MetadataKeys.DEFAULT: '[65536, 65536]'}, True, [65536, 65536], False, ''),
        ('outputs:exec', 'execution', 0, None, 'Trigger', {}, True, None, False, ''),
        ('outputs:sdSemBBox3dCamCornersCudaPtr', 'uint64', 0, None, 'Cuda buffer containing the projection of the 3d bounding boxes on the camera plane represented as a float4=(u,v,z,a) for each bounding box corners', {}, True, None, False, ''),
        ('outputs:sdSemBBox3dCamExtentCudaPtr', 'uint64', 0, None, 'Cuda buffer containing the 2d extent of the 3d bounding boxes on the camera plane represented as a float6=(u_min,u_max,v_min,v_max,z_min,z_max)', {}, True, None, False, ''),
    ])
    class tokens:
        SemanticBoundingBox3DInfosSD = "SemanticBoundingBox3DInfosSD"
        SemanticBoundingBox3DCamCornersSD = "SemanticBoundingBox3DCamCornersSD"
        SemanticBoundingBox3DCamExtentSD = "SemanticBoundingBox3DCamExtentSD"
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.exec = og.Database.ROLE_EXECUTION
        role_data.outputs.exec = og.Database.ROLE_EXECUTION
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"cameraModel", "cameraNearFar", "exec", "gpu", "instanceMappingInfoSDPtr", "metersPerSceneUnit", "renderProductResolution", "rp", "sdSemBBoxExtentCudaPtr", "sdSemBBoxInfosCudaPtr", "semanticWorldTransformSDCudaPtr", "viewportNearFar", "viewportResolution", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.cameraModel, self._attributes.cameraNearFar, self._attributes.exec, self._attributes.gpu, self._attributes.instanceMappingInfoSDPtr, self._attributes.metersPerSceneUnit, self._attributes.renderProductResolution, self._attributes.rp, self._attributes.sdSemBBoxExtentCudaPtr, self._attributes.sdSemBBoxInfosCudaPtr, self._attributes.semanticWorldTransformSDCudaPtr, self._attributes.viewportNearFar, self._attributes.viewportResolution]
            self._batchedReadValues = [0, [0.0, 0.0], None, 0, 0, 0.01, [0, 0], 0, 0, 0, 0, [1.0, 10000000.0], [65536, 65536]]

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
        def instanceMappingInfoSDPtr(self):
            return self._batchedReadValues[4]

        @instanceMappingInfoSDPtr.setter
        def instanceMappingInfoSDPtr(self, value):
            self._batchedReadValues[4] = value

        @property
        def metersPerSceneUnit(self):
            return self._batchedReadValues[5]

        @metersPerSceneUnit.setter
        def metersPerSceneUnit(self, value):
            self._batchedReadValues[5] = value

        @property
        def renderProductResolution(self):
            return self._batchedReadValues[6]

        @renderProductResolution.setter
        def renderProductResolution(self, value):
            self._batchedReadValues[6] = value

        @property
        def rp(self):
            return self._batchedReadValues[7]

        @rp.setter
        def rp(self, value):
            self._batchedReadValues[7] = value

        @property
        def sdSemBBoxExtentCudaPtr(self):
            return self._batchedReadValues[8]

        @sdSemBBoxExtentCudaPtr.setter
        def sdSemBBoxExtentCudaPtr(self, value):
            self._batchedReadValues[8] = value

        @property
        def sdSemBBoxInfosCudaPtr(self):
            return self._batchedReadValues[9]

        @sdSemBBoxInfosCudaPtr.setter
        def sdSemBBoxInfosCudaPtr(self, value):
            self._batchedReadValues[9] = value

        @property
        def semanticWorldTransformSDCudaPtr(self):
            return self._batchedReadValues[10]

        @semanticWorldTransformSDCudaPtr.setter
        def semanticWorldTransformSDCudaPtr(self, value):
            self._batchedReadValues[10] = value

        @property
        def viewportNearFar(self):
            return self._batchedReadValues[11]

        @viewportNearFar.setter
        def viewportNearFar(self, value):
            self._batchedReadValues[11] = value

        @property
        def viewportResolution(self):
            return self._batchedReadValues[12]

        @viewportResolution.setter
        def viewportResolution(self, value):
            self._batchedReadValues[12] = value

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
        LOCAL_PROPERTY_NAMES = {"exec", "sdSemBBox3dCamCornersCudaPtr", "sdSemBBox3dCamExtentCudaPtr", "_batchedWriteValues"}
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedWriteValues = { }

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
        def sdSemBBox3dCamCornersCudaPtr(self):
            value = self._batchedWriteValues.get(self._attributes.sdSemBBox3dCamCornersCudaPtr)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.sdSemBBox3dCamCornersCudaPtr)
                return data_view.get()

        @sdSemBBox3dCamCornersCudaPtr.setter
        def sdSemBBox3dCamCornersCudaPtr(self, value):
            self._batchedWriteValues[self._attributes.sdSemBBox3dCamCornersCudaPtr] = value

        @property
        def sdSemBBox3dCamExtentCudaPtr(self):
            value = self._batchedWriteValues.get(self._attributes.sdSemBBox3dCamExtentCudaPtr)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.sdSemBBox3dCamExtentCudaPtr)
                return data_view.get()

        @sdSemBBox3dCamExtentCudaPtr.setter
        def sdSemBBox3dCamExtentCudaPtr(self, value):
            self._batchedWriteValues[self._attributes.sdSemBBox3dCamExtentCudaPtr] = value

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
        self.inputs = OgnSdPostSemantic3dBoundingBoxCameraProjectionDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnSdPostSemantic3dBoundingBoxCameraProjectionDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnSdPostSemantic3dBoundingBoxCameraProjectionDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
