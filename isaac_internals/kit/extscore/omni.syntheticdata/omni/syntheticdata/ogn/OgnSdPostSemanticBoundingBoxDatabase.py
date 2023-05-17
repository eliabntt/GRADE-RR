"""Support for simplified access to data on nodes of type omni.syntheticdata.SdPostSemanticBoundingBox

Synthetic Data node to compute the bounding boxes of the scene semantic entities.
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import numpy
class OgnSdPostSemanticBoundingBoxDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.syntheticdata.SdPostSemanticBoundingBox

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.exec
            inputs.gpu
            inputs.instanceMapSDCudaPtr
            inputs.instanceMappingInfoSDPtr
            inputs.renderProductResolution
            inputs.renderVar
            inputs.rp
            inputs.semanticLocalTransformSDCudaPtr
            inputs.semanticMapSDCudaPtr
        Outputs:
            outputs.exec
            outputs.sdSemBBoxExtentCudaPtr
            outputs.sdSemBBoxInfosCudaPtr

    Predefined Tokens:
        tokens.BoundingBox2DLooseSD
        tokens.BoundingBox2DTightSD
        tokens.SemanticBoundingBox2DExtentLooseSD
        tokens.SemanticBoundingBox2DInfosLooseSD
        tokens.SemanticBoundingBox2DExtentTightSD
        tokens.SemanticBoundingBox2DInfosTightSD
        tokens.BoundingBox3DSD
        tokens.SemanticBoundingBox3DExtentSD
        tokens.SemanticBoundingBox3DInfosSD
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
        ('inputs:instanceMapSDCudaPtr', 'uint64', 0, None, 'cuda uint16_t buffer pointer of size numInstances containing the instance parent semantic index', {}, True, 0, False, ''),
        ('inputs:instanceMappingInfoSDPtr', 'uint64', 0, None, 'uint buffer pointer containing the following information : [numInstances, minInstanceId, numSemantics, minSemanticId, numProtoSemantic]', {}, True, 0, False, ''),
        ('inputs:renderProductResolution', 'int2', 0, None, 'RenderProduct resolution', {}, True, [0, 0], False, ''),
        ('inputs:renderVar', 'token', 0, None, 'Name of the BoundingBox RenderVar to process', {}, True, '', False, ''),
        ('inputs:rp', 'uint64', 0, 'renderProduct', 'Pointer to render product for this view', {}, True, 0, False, ''),
        ('inputs:semanticLocalTransformSDCudaPtr', 'uint64', 0, None, 'cuda float44 buffer pointer of size numSemantics containing the local semantic transform', {}, True, 0, False, ''),
        ('inputs:semanticMapSDCudaPtr', 'uint64', 0, None, 'cuda uint16_t buffer pointer of size numSemantics containing the semantic parent semantic index', {}, True, 0, False, ''),
        ('outputs:exec', 'execution', 0, None, 'Trigger', {}, True, None, False, ''),
        ('outputs:sdSemBBoxExtentCudaPtr', 'uint64', 0, None, 'Cuda buffer containing the extent of the bounding boxes as a float4=(u_min,v_min,u_max,v_max) for 2D or a float6=(xmin,ymin,zmin,xmax,ymax,zmax) in object space for 3D', {}, True, None, False, ''),
        ('outputs:sdSemBBoxInfosCudaPtr', 'uint64', 0, None, 'Cuda buffer containing valid bounding boxes infos', {}, True, None, False, ''),
    ])
    class tokens:
        BoundingBox2DLooseSD = "BoundingBox2DLooseSD"
        BoundingBox2DTightSD = "BoundingBox2DTightSD"
        SemanticBoundingBox2DExtentLooseSD = "SemanticBoundingBox2DExtentLooseSD"
        SemanticBoundingBox2DInfosLooseSD = "SemanticBoundingBox2DInfosLooseSD"
        SemanticBoundingBox2DExtentTightSD = "SemanticBoundingBox2DExtentTightSD"
        SemanticBoundingBox2DInfosTightSD = "SemanticBoundingBox2DInfosTightSD"
        BoundingBox3DSD = "BoundingBox3DSD"
        SemanticBoundingBox3DExtentSD = "SemanticBoundingBox3DExtentSD"
        SemanticBoundingBox3DInfosSD = "SemanticBoundingBox3DInfosSD"
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.exec = og.Database.ROLE_EXECUTION
        role_data.outputs.exec = og.Database.ROLE_EXECUTION
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"exec", "gpu", "instanceMapSDCudaPtr", "instanceMappingInfoSDPtr", "renderProductResolution", "renderVar", "rp", "semanticLocalTransformSDCudaPtr", "semanticMapSDCudaPtr", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.exec, self._attributes.gpu, self._attributes.instanceMapSDCudaPtr, self._attributes.instanceMappingInfoSDPtr, self._attributes.renderProductResolution, self._attributes.renderVar, self._attributes.rp, self._attributes.semanticLocalTransformSDCudaPtr, self._attributes.semanticMapSDCudaPtr]
            self._batchedReadValues = [None, 0, 0, 0, [0, 0], "", 0, 0, 0]

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
        def instanceMapSDCudaPtr(self):
            return self._batchedReadValues[2]

        @instanceMapSDCudaPtr.setter
        def instanceMapSDCudaPtr(self, value):
            self._batchedReadValues[2] = value

        @property
        def instanceMappingInfoSDPtr(self):
            return self._batchedReadValues[3]

        @instanceMappingInfoSDPtr.setter
        def instanceMappingInfoSDPtr(self, value):
            self._batchedReadValues[3] = value

        @property
        def renderProductResolution(self):
            return self._batchedReadValues[4]

        @renderProductResolution.setter
        def renderProductResolution(self, value):
            self._batchedReadValues[4] = value

        @property
        def renderVar(self):
            return self._batchedReadValues[5]

        @renderVar.setter
        def renderVar(self, value):
            self._batchedReadValues[5] = value

        @property
        def rp(self):
            return self._batchedReadValues[6]

        @rp.setter
        def rp(self, value):
            self._batchedReadValues[6] = value

        @property
        def semanticLocalTransformSDCudaPtr(self):
            return self._batchedReadValues[7]

        @semanticLocalTransformSDCudaPtr.setter
        def semanticLocalTransformSDCudaPtr(self, value):
            self._batchedReadValues[7] = value

        @property
        def semanticMapSDCudaPtr(self):
            return self._batchedReadValues[8]

        @semanticMapSDCudaPtr.setter
        def semanticMapSDCudaPtr(self, value):
            self._batchedReadValues[8] = value

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
        LOCAL_PROPERTY_NAMES = {"exec", "sdSemBBoxExtentCudaPtr", "sdSemBBoxInfosCudaPtr", "_batchedWriteValues"}
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
        def sdSemBBoxExtentCudaPtr(self):
            value = self._batchedWriteValues.get(self._attributes.sdSemBBoxExtentCudaPtr)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.sdSemBBoxExtentCudaPtr)
                return data_view.get()

        @sdSemBBoxExtentCudaPtr.setter
        def sdSemBBoxExtentCudaPtr(self, value):
            self._batchedWriteValues[self._attributes.sdSemBBoxExtentCudaPtr] = value

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
        self.inputs = OgnSdPostSemanticBoundingBoxDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnSdPostSemanticBoundingBoxDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnSdPostSemanticBoundingBoxDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
