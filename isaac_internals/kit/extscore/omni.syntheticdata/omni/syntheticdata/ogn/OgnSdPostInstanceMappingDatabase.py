"""Support for simplified access to data on nodes of type omni.syntheticdata.SdPostInstanceMapping

Synthetic Data node to compute and store scene instances semantic hierarchy information
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
class OgnSdPostInstanceMappingDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.syntheticdata.SdPostInstanceMapping

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.exec
            inputs.gpu
            inputs.rp
            inputs.semanticFilterName
            inputs.swhFrameNumber
        Outputs:
            outputs.exec
            outputs.instanceMapSDCudaPtr
            outputs.instanceMappingInfoSDPtr
            outputs.instancePrimTokenSDCudaPtr
            outputs.semanticLabelTokenSDCudaPtr
            outputs.semanticLocalTransformSDCudaPtr
            outputs.semanticMapSDCudaPtr
            outputs.semanticPrimTokenSDCudaPtr
            outputs.semanticWorldTransformSDCudaPtr

    Predefined Tokens:
        tokens.InstanceMappingInfoSDhost
        tokens.SemanticMapSD
        tokens.SemanticMapSDhost
        tokens.SemanticPrimTokenSD
        tokens.SemanticPrimTokenSDhost
        tokens.InstanceMapSD
        tokens.InstanceMapSDhost
        tokens.InstancePrimTokenSD
        tokens.InstancePrimTokenSDhost
        tokens.SemanticLabelTokenSD
        tokens.SemanticLabelTokenSDhost
        tokens.SemanticLocalTransformSD
        tokens.SemanticLocalTransformSDhost
        tokens.SemanticWorldTransformSD
        tokens.SemanticWorldTransformSDhost
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
        ('inputs:rp', 'uint64', 0, 'renderProduct', 'Pointer to render product for this view', {}, True, 0, False, ''),
        ('inputs:semanticFilterName', 'token', 0, None, 'Name of the semantic filter to apply to the semanticLabelToken', {ogn.MetadataKeys.DEFAULT: '"default"'}, True, 'default', False, ''),
        ('inputs:swhFrameNumber', 'uint64', 0, None, 'Fabric frame number', {}, True, 0, False, ''),
        ('outputs:exec', 'execution', 0, None, 'Trigger', {}, True, None, False, ''),
        ('outputs:instanceMapSDCudaPtr', 'uint64', 0, None, 'cuda uint16_t buffer pointer of size numInstances containing the instance parent semantic index', {}, True, None, False, ''),
        ('outputs:instanceMappingInfoSDPtr', 'uint64', 0, None, 'uint buffer pointer containing the following information : [numInstances, minInstanceId, numSemantics, minSemanticId, numProtoSemantic]', {}, True, None, False, ''),
        ('outputs:instancePrimTokenSDCudaPtr', 'uint64', 0, None, 'cuda uint64_t buffer pointer of size numInstances containing the instance path token', {}, True, None, False, ''),
        ('outputs:semanticLabelTokenSDCudaPtr', 'uint64', 0, None, 'cuda uint64_t buffer pointer of size numSemantics containing the semantic label token', {}, True, None, False, ''),
        ('outputs:semanticLocalTransformSDCudaPtr', 'uint64', 0, None, 'cuda float44 buffer pointer of size numSemantics containing the local semantic transform', {}, True, None, False, ''),
        ('outputs:semanticMapSDCudaPtr', 'uint64', 0, None, 'cuda uint16_t buffer pointer of size numSemantics containing the semantic parent semantic index', {}, True, None, False, ''),
        ('outputs:semanticPrimTokenSDCudaPtr', 'uint64', 0, None, 'cuda uint32_t buffer pointer of size numSemantics containing the prim part of the semantic path token', {}, True, None, False, ''),
        ('outputs:semanticWorldTransformSDCudaPtr', 'uint64', 0, None, 'cuda float44 buffer pointer of size numSemantics containing the world semantic transform', {}, True, None, False, ''),
    ])
    class tokens:
        InstanceMappingInfoSDhost = "InstanceMappingInfoSDhost"
        SemanticMapSD = "SemanticMapSD"
        SemanticMapSDhost = "SemanticMapSDhost"
        SemanticPrimTokenSD = "SemanticPrimTokenSD"
        SemanticPrimTokenSDhost = "SemanticPrimTokenSDhost"
        InstanceMapSD = "InstanceMapSD"
        InstanceMapSDhost = "InstanceMapSDhost"
        InstancePrimTokenSD = "InstancePrimTokenSD"
        InstancePrimTokenSDhost = "InstancePrimTokenSDhost"
        SemanticLabelTokenSD = "SemanticLabelTokenSD"
        SemanticLabelTokenSDhost = "SemanticLabelTokenSDhost"
        SemanticLocalTransformSD = "SemanticLocalTransformSD"
        SemanticLocalTransformSDhost = "SemanticLocalTransformSDhost"
        SemanticWorldTransformSD = "SemanticWorldTransformSD"
        SemanticWorldTransformSDhost = "SemanticWorldTransformSDhost"
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.exec = og.Database.ROLE_EXECUTION
        role_data.outputs.exec = og.Database.ROLE_EXECUTION
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"exec", "gpu", "rp", "semanticFilterName", "swhFrameNumber", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.exec, self._attributes.gpu, self._attributes.rp, self._attributes.semanticFilterName, self._attributes.swhFrameNumber]
            self._batchedReadValues = [None, 0, 0, "default", 0]

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
        def rp(self):
            return self._batchedReadValues[2]

        @rp.setter
        def rp(self, value):
            self._batchedReadValues[2] = value

        @property
        def semanticFilterName(self):
            return self._batchedReadValues[3]

        @semanticFilterName.setter
        def semanticFilterName(self, value):
            self._batchedReadValues[3] = value

        @property
        def swhFrameNumber(self):
            return self._batchedReadValues[4]

        @swhFrameNumber.setter
        def swhFrameNumber(self, value):
            self._batchedReadValues[4] = value

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
        LOCAL_PROPERTY_NAMES = {"exec", "instanceMapSDCudaPtr", "instanceMappingInfoSDPtr", "instancePrimTokenSDCudaPtr", "semanticLabelTokenSDCudaPtr", "semanticLocalTransformSDCudaPtr", "semanticMapSDCudaPtr", "semanticPrimTokenSDCudaPtr", "semanticWorldTransformSDCudaPtr", "_batchedWriteValues"}
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
        def instanceMapSDCudaPtr(self):
            value = self._batchedWriteValues.get(self._attributes.instanceMapSDCudaPtr)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.instanceMapSDCudaPtr)
                return data_view.get()

        @instanceMapSDCudaPtr.setter
        def instanceMapSDCudaPtr(self, value):
            self._batchedWriteValues[self._attributes.instanceMapSDCudaPtr] = value

        @property
        def instanceMappingInfoSDPtr(self):
            value = self._batchedWriteValues.get(self._attributes.instanceMappingInfoSDPtr)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.instanceMappingInfoSDPtr)
                return data_view.get()

        @instanceMappingInfoSDPtr.setter
        def instanceMappingInfoSDPtr(self, value):
            self._batchedWriteValues[self._attributes.instanceMappingInfoSDPtr] = value

        @property
        def instancePrimTokenSDCudaPtr(self):
            value = self._batchedWriteValues.get(self._attributes.instancePrimTokenSDCudaPtr)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.instancePrimTokenSDCudaPtr)
                return data_view.get()

        @instancePrimTokenSDCudaPtr.setter
        def instancePrimTokenSDCudaPtr(self, value):
            self._batchedWriteValues[self._attributes.instancePrimTokenSDCudaPtr] = value

        @property
        def semanticLabelTokenSDCudaPtr(self):
            value = self._batchedWriteValues.get(self._attributes.semanticLabelTokenSDCudaPtr)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.semanticLabelTokenSDCudaPtr)
                return data_view.get()

        @semanticLabelTokenSDCudaPtr.setter
        def semanticLabelTokenSDCudaPtr(self, value):
            self._batchedWriteValues[self._attributes.semanticLabelTokenSDCudaPtr] = value

        @property
        def semanticLocalTransformSDCudaPtr(self):
            value = self._batchedWriteValues.get(self._attributes.semanticLocalTransformSDCudaPtr)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.semanticLocalTransformSDCudaPtr)
                return data_view.get()

        @semanticLocalTransformSDCudaPtr.setter
        def semanticLocalTransformSDCudaPtr(self, value):
            self._batchedWriteValues[self._attributes.semanticLocalTransformSDCudaPtr] = value

        @property
        def semanticMapSDCudaPtr(self):
            value = self._batchedWriteValues.get(self._attributes.semanticMapSDCudaPtr)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.semanticMapSDCudaPtr)
                return data_view.get()

        @semanticMapSDCudaPtr.setter
        def semanticMapSDCudaPtr(self, value):
            self._batchedWriteValues[self._attributes.semanticMapSDCudaPtr] = value

        @property
        def semanticPrimTokenSDCudaPtr(self):
            value = self._batchedWriteValues.get(self._attributes.semanticPrimTokenSDCudaPtr)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.semanticPrimTokenSDCudaPtr)
                return data_view.get()

        @semanticPrimTokenSDCudaPtr.setter
        def semanticPrimTokenSDCudaPtr(self, value):
            self._batchedWriteValues[self._attributes.semanticPrimTokenSDCudaPtr] = value

        @property
        def semanticWorldTransformSDCudaPtr(self):
            value = self._batchedWriteValues.get(self._attributes.semanticWorldTransformSDCudaPtr)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.semanticWorldTransformSDCudaPtr)
                return data_view.get()

        @semanticWorldTransformSDCudaPtr.setter
        def semanticWorldTransformSDCudaPtr(self, value):
            self._batchedWriteValues[self._attributes.semanticWorldTransformSDCudaPtr] = value

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
        self.inputs = OgnSdPostInstanceMappingDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnSdPostInstanceMappingDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnSdPostInstanceMappingDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
