"""Support for simplified access to data on nodes of type omni.syntheticdata.SdPostSemantic3dBoundingBoxFilter

Synthetic Data node to cull the semantic 3d bounding boxes.
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import numpy
class OgnSdPostSemantic3dBoundingBoxFilterDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.syntheticdata.SdPostSemantic3dBoundingBoxFilter

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.exec
            inputs.gpu
            inputs.instanceMappingInfoSDPtr
            inputs.metersPerSceneUnit
            inputs.rp
            inputs.sdSemBBox3dCamCornersCudaPtr
            inputs.sdSemBBoxInfosCudaPtr
            inputs.viewportNearFar
        Outputs:
            outputs.exec
            outputs.sdSemBBoxInfosCudaPtr

    Predefined Tokens:
        tokens.SemanticBoundingBox3DInfosSD
        tokens.SemanticBoundingBox3DFilterInfosSD
    """
    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}
    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:exec', 'execution', 0, None, 'Trigger', {}, True, None, False, ''),
        ('inputs:gpu', 'uint64', 0, 'gpuFoundations', 'Pointer to shared context containing gpu foundations', {}, True, 0, False, ''),
        ('inputs:instanceMappingInfoSDPtr', 'uint64', 0, None, 'uint buffer pointer containing the following information : [numInstances, minInstanceId, numSemantics, minSemanticId, numProtoSemantic]', {}, True, 0, False, ''),
        ('inputs:metersPerSceneUnit', 'float', 0, None, 'Scene units to meters scale', {ogn.MetadataKeys.DEFAULT: '0.01'}, True, 0.01, False, ''),
        ('inputs:rp', 'uint64', 0, 'renderProduct', 'Pointer to render product for this view', {}, True, 0, False, ''),
        ('inputs:sdSemBBox3dCamCornersCudaPtr', 'uint64', 0, None, 'Cuda buffer containing the projection of the 3d bounding boxes on the camera plane represented as a float3=(u,v,z,a) for each bounding box corners', {}, True, 0, False, ''),
        ('inputs:sdSemBBoxInfosCudaPtr', 'uint64', 0, None, 'Cuda buffer containing valid bounding boxes infos', {}, True, 0, False, ''),
        ('inputs:viewportNearFar', 'float2', 0, None, 'near and far plane (in scene units) used to clip the 3d bounding boxes.', {ogn.MetadataKeys.DEFAULT: '[0.0, -1.0]'}, True, [0.0, -1.0], False, ''),
        ('outputs:exec', 'execution', 0, None, 'Trigger', {}, True, None, False, ''),
        ('outputs:sdSemBBoxInfosCudaPtr', 'uint64', 0, None, 'Cuda buffer containing valid bounding boxes infos', {}, True, None, False, ''),
    ])
    class tokens:
        SemanticBoundingBox3DInfosSD = "SemanticBoundingBox3DInfosSD"
        SemanticBoundingBox3DFilterInfosSD = "SemanticBoundingBox3DFilterInfosSD"
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.exec = og.Database.ROLE_EXECUTION
        role_data.outputs.exec = og.Database.ROLE_EXECUTION
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"exec", "gpu", "instanceMappingInfoSDPtr", "metersPerSceneUnit", "rp", "sdSemBBox3dCamCornersCudaPtr", "sdSemBBoxInfosCudaPtr", "viewportNearFar", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.exec, self._attributes.gpu, self._attributes.instanceMappingInfoSDPtr, self._attributes.metersPerSceneUnit, self._attributes.rp, self._attributes.sdSemBBox3dCamCornersCudaPtr, self._attributes.sdSemBBoxInfosCudaPtr, self._attributes.viewportNearFar]
            self._batchedReadValues = [None, 0, 0, 0.01, 0, 0, 0, [0.0, -1.0]]

        @property
        def exec(self):
            return self._batchedReadValues[0]

        @exec.setter
        def exec(self, value):
            self._batchedReadValues[0] = value

        @property
        def gpu(self):
            return self._batchedReadValues[1]

        @gpu.setter
        def gpu(self, value):
            self._batchedReadValues[1] = value

        @property
        def instanceMappingInfoSDPtr(self):
            return self._batchedReadValues[2]

        @instanceMappingInfoSDPtr.setter
        def instanceMappingInfoSDPtr(self, value):
            self._batchedReadValues[2] = value

        @property
        def metersPerSceneUnit(self):
            return self._batchedReadValues[3]

        @metersPerSceneUnit.setter
        def metersPerSceneUnit(self, value):
            self._batchedReadValues[3] = value

        @property
        def rp(self):
            return self._batchedReadValues[4]

        @rp.setter
        def rp(self, value):
            self._batchedReadValues[4] = value

        @property
        def sdSemBBox3dCamCornersCudaPtr(self):
            return self._batchedReadValues[5]

        @sdSemBBox3dCamCornersCudaPtr.setter
        def sdSemBBox3dCamCornersCudaPtr(self, value):
            self._batchedReadValues[5] = value

        @property
        def sdSemBBoxInfosCudaPtr(self):
            return self._batchedReadValues[6]

        @sdSemBBoxInfosCudaPtr.setter
        def sdSemBBoxInfosCudaPtr(self, value):
            self._batchedReadValues[6] = value

        @property
        def viewportNearFar(self):
            return self._batchedReadValues[7]

        @viewportNearFar.setter
        def viewportNearFar(self, value):
            self._batchedReadValues[7] = value

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
        LOCAL_PROPERTY_NAMES = {"exec", "sdSemBBoxInfosCudaPtr", "_batchedWriteValues"}
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
        def sdSemBBoxInfosCudaPtr(self):
            value = self._batchedWriteValues.get(self._attributes.sdSemBBoxInfosCudaPtr)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.sdSemBBoxInfosCudaPtr)
                return data_view.get()

        @sdSemBBoxInfosCudaPtr.setter
        def sdSemBBoxInfosCudaPtr(self, value):
            self._batchedWriteValues[self._attributes.sdSemBBoxInfosCudaPtr] = value

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
        self.inputs = OgnSdPostSemantic3dBoundingBoxFilterDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnSdPostSemantic3dBoundingBoxFilterDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnSdPostSemantic3dBoundingBoxFilterDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
