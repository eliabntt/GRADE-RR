"""Support for simplified access to data on nodes of type omni.syntheticdata.SdInstanceMappingPtr

Synthetic Data node to expose the scene instances semantic hierarchy information
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import numpy
class OgnSdInstanceMappingPtrDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.syntheticdata.SdInstanceMappingPtr

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.cudaPtr
            inputs.exec
            inputs.renderResults
            inputs.semanticFilerTokens
            inputs.swhFrameNumber
        Outputs:
            outputs.exec
            outputs.instanceMapPtr
            outputs.instancePrimPathPtr
            outputs.minInstanceIndex
            outputs.minSemanticIndex
            outputs.numInstances
            outputs.numSemantics
            outputs.semanticLabelTokenPtrs
            outputs.semanticLocalTransformPtr
            outputs.semanticMapPtr
            outputs.semanticPrimPathPtr
            outputs.semanticWorldTransformPtr
            outputs.swhFrameNumber

    Predefined Tokens:
        tokens.InstanceMappingInfoSDhost
        tokens.InstancePrimTokenSDhost
        tokens.InstancePrimTokenSD
        tokens.SemanticPrimTokenSDhost
        tokens.SemanticPrimTokenSD
        tokens.InstanceMapSDhost
        tokens.InstanceMapSD
        tokens.SemanticMapSDhost
        tokens.SemanticMapSD
        tokens.SemanticWorldTransformSDhost
        tokens.SemanticWorldTransformSD
        tokens.SemanticLocalTransformSDhost
        tokens.SemanticLocalTransformSD
        tokens.SemanticLabelTokenSDhost
        tokens.SemanticLabelTokenSD
    """
    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}
    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:cudaPtr', 'bool', 0, None, 'If true, return cuda device pointer instead of host pointer', {ogn.MetadataKeys.DEFAULT: 'false'}, True, False, False, ''),
        ('inputs:exec', 'execution', 0, None, 'Trigger', {}, True, None, False, ''),
        ('inputs:renderResults', 'uint64', 0, None, 'Render results pointer', {}, True, 0, False, ''),
        ('inputs:semanticFilerTokens', 'token[]', 0, None, 'Tokens identifying the semantic filters applied to the output semantic labels. Each token should correspond to an activated SdSemanticFilter node', {ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('inputs:swhFrameNumber', 'uint64', 0, None, 'Fabric frame number', {}, True, 0, False, ''),
        ('outputs:exec', 'execution', 0, 'Received', 'Executes when the event is received', {}, True, None, False, ''),
        ('outputs:instanceMapPtr', 'uint64', 0, None, 'Array pointer of numInstances uint16_t containing the semantic index of the instance prim first semantic prim parent', {}, True, None, False, ''),
        ('outputs:instancePrimPathPtr', 'uint64', 0, None, 'Array pointer of numInstances uint64_t containing the prim path tokens for every instance prims', {}, True, None, False, ''),
        ('outputs:minInstanceIndex', 'uint', 0, None, 'Instance index of the first instance prim in the instance arrays', {}, True, None, False, ''),
        ('outputs:minSemanticIndex', 'uint', 0, None, 'Semantic index of the first semantic prim in the semantic arrays', {}, True, None, False, ''),
        ('outputs:numInstances', 'uint', 0, None, 'Number of instances prim in the instance arrays', {}, True, None, False, ''),
        ('outputs:numSemantics', 'uint', 0, None, 'Number of semantic prim in the semantic arrays', {}, True, None, False, ''),
        ('outputs:semanticLabelTokenPtrs', 'uint64[]', 0, None, 'Array containing for every input semantic filters the corresponding array pointer of numSemantics uint64_t representing the semantic label of the semantic prim', {}, True, None, False, ''),
        ('outputs:semanticLocalTransformPtr', 'uint64', 0, None, 'Array pointer of numSemantics 4x4 float matrices containing the transform from world to object space for every semantic prims', {}, True, None, False, ''),
        ('outputs:semanticMapPtr', 'uint64', 0, None, 'Array pointer of numSemantics uint16_t containing the semantic index of the semantic prim first semantic prim parent', {}, True, None, False, ''),
        ('outputs:semanticPrimPathPtr', 'uint64', 0, None, 'Array pointer of numSemantics uint32_t containing the prim part of the prim path tokens for every semantic prims', {}, True, None, False, ''),
        ('outputs:semanticWorldTransformPtr', 'uint64', 0, None, 'Array pointer of numSemantics 4x4 float matrices containing the transform from local to world space for every semantic entity', {}, True, None, False, ''),
        ('outputs:swhFrameNumber', 'uint64', 0, None, 'Fabric frame number', {}, True, None, False, ''),
    ])
    class tokens:
        InstanceMappingInfoSDhost = "InstanceMappingInfoSDhost"
        InstancePrimTokenSDhost = "InstancePrimTokenSDhost"
        InstancePrimTokenSD = "InstancePrimTokenSD"
        SemanticPrimTokenSDhost = "SemanticPrimTokenSDhost"
        SemanticPrimTokenSD = "SemanticPrimTokenSD"
        InstanceMapSDhost = "InstanceMapSDhost"
        InstanceMapSD = "InstanceMapSD"
        SemanticMapSDhost = "SemanticMapSDhost"
        SemanticMapSD = "SemanticMapSD"
        SemanticWorldTransformSDhost = "SemanticWorldTransformSDhost"
        SemanticWorldTransformSD = "SemanticWorldTransformSD"
        SemanticLocalTransformSDhost = "SemanticLocalTransformSDhost"
        SemanticLocalTransformSD = "SemanticLocalTransformSD"
        SemanticLabelTokenSDhost = "SemanticLabelTokenSDhost"
        SemanticLabelTokenSD = "SemanticLabelTokenSD"
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.exec = og.Database.ROLE_EXECUTION
        role_data.outputs.exec = og.Database.ROLE_EXECUTION
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"cudaPtr", "exec", "renderResults", "swhFrameNumber", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.cudaPtr, self._attributes.exec, self._attributes.renderResults, self._attributes.swhFrameNumber]
            self._batchedReadValues = [False, None, 0, 0]

        @property
        def semanticFilerTokens(self):
            data_view = og.AttributeValueHelper(self._attributes.semanticFilerTokens)
            return data_view.get()

        @semanticFilerTokens.setter
        def semanticFilerTokens(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.semanticFilerTokens)
            data_view = og.AttributeValueHelper(self._attributes.semanticFilerTokens)
            data_view.set(value)
            self.semanticFilerTokens_size = data_view.get_array_size()

        @property
        def cudaPtr(self):
            return self._batchedReadValues[0]

        @cudaPtr.setter
        def cudaPtr(self, value):
            self._batchedReadValues[0] = value

        @property
        def exec(self):
            return self._batchedReadValues[1]

        @exec.setter
        def exec(self, value):
            self._batchedReadValues[1] = value

        @property
        def renderResults(self):
            return self._batchedReadValues[2]

        @renderResults.setter
        def renderResults(self, value):
            self._batchedReadValues[2] = value

        @property
        def swhFrameNumber(self):
            return self._batchedReadValues[3]

        @swhFrameNumber.setter
        def swhFrameNumber(self, value):
            self._batchedReadValues[3] = value

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
        LOCAL_PROPERTY_NAMES = {"exec", "instanceMapPtr", "instancePrimPathPtr", "minInstanceIndex", "minSemanticIndex", "numInstances", "numSemantics", "semanticLocalTransformPtr", "semanticMapPtr", "semanticPrimPathPtr", "semanticWorldTransformPtr", "swhFrameNumber", "_batchedWriteValues"}
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self.semanticLabelTokenPtrs_size = None
            self._batchedWriteValues = { }

        @property
        def semanticLabelTokenPtrs(self):
            data_view = og.AttributeValueHelper(self._attributes.semanticLabelTokenPtrs)
            return data_view.get(reserved_element_count=self.semanticLabelTokenPtrs_size)

        @semanticLabelTokenPtrs.setter
        def semanticLabelTokenPtrs(self, value):
            data_view = og.AttributeValueHelper(self._attributes.semanticLabelTokenPtrs)
            data_view.set(value)
            self.semanticLabelTokenPtrs_size = data_view.get_array_size()

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
        def instanceMapPtr(self):
            value = self._batchedWriteValues.get(self._attributes.instanceMapPtr)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.instanceMapPtr)
                return data_view.get()

        @instanceMapPtr.setter
        def instanceMapPtr(self, value):
            self._batchedWriteValues[self._attributes.instanceMapPtr] = value

        @property
        def instancePrimPathPtr(self):
            value = self._batchedWriteValues.get(self._attributes.instancePrimPathPtr)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.instancePrimPathPtr)
                return data_view.get()

        @instancePrimPathPtr.setter
        def instancePrimPathPtr(self, value):
            self._batchedWriteValues[self._attributes.instancePrimPathPtr] = value

        @property
        def minInstanceIndex(self):
            value = self._batchedWriteValues.get(self._attributes.minInstanceIndex)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.minInstanceIndex)
                return data_view.get()

        @minInstanceIndex.setter
        def minInstanceIndex(self, value):
            self._batchedWriteValues[self._attributes.minInstanceIndex] = value

        @property
        def minSemanticIndex(self):
            value = self._batchedWriteValues.get(self._attributes.minSemanticIndex)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.minSemanticIndex)
                return data_view.get()

        @minSemanticIndex.setter
        def minSemanticIndex(self, value):
            self._batchedWriteValues[self._attributes.minSemanticIndex] = value

        @property
        def numInstances(self):
            value = self._batchedWriteValues.get(self._attributes.numInstances)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.numInstances)
                return data_view.get()

        @numInstances.setter
        def numInstances(self, value):
            self._batchedWriteValues[self._attributes.numInstances] = value

        @property
        def numSemantics(self):
            value = self._batchedWriteValues.get(self._attributes.numSemantics)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.numSemantics)
                return data_view.get()

        @numSemantics.setter
        def numSemantics(self, value):
            self._batchedWriteValues[self._attributes.numSemantics] = value

        @property
        def semanticLocalTransformPtr(self):
            value = self._batchedWriteValues.get(self._attributes.semanticLocalTransformPtr)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.semanticLocalTransformPtr)
                return data_view.get()

        @semanticLocalTransformPtr.setter
        def semanticLocalTransformPtr(self, value):
            self._batchedWriteValues[self._attributes.semanticLocalTransformPtr] = value

        @property
        def semanticMapPtr(self):
            value = self._batchedWriteValues.get(self._attributes.semanticMapPtr)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.semanticMapPtr)
                return data_view.get()

        @semanticMapPtr.setter
        def semanticMapPtr(self, value):
            self._batchedWriteValues[self._attributes.semanticMapPtr] = value

        @property
        def semanticPrimPathPtr(self):
            value = self._batchedWriteValues.get(self._attributes.semanticPrimPathPtr)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.semanticPrimPathPtr)
                return data_view.get()

        @semanticPrimPathPtr.setter
        def semanticPrimPathPtr(self, value):
            self._batchedWriteValues[self._attributes.semanticPrimPathPtr] = value

        @property
        def semanticWorldTransformPtr(self):
            value = self._batchedWriteValues.get(self._attributes.semanticWorldTransformPtr)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.semanticWorldTransformPtr)
                return data_view.get()

        @semanticWorldTransformPtr.setter
        def semanticWorldTransformPtr(self, value):
            self._batchedWriteValues[self._attributes.semanticWorldTransformPtr] = value

        @property
        def swhFrameNumber(self):
            value = self._batchedWriteValues.get(self._attributes.swhFrameNumber)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.swhFrameNumber)
                return data_view.get()

        @swhFrameNumber.setter
        def swhFrameNumber(self, value):
            self._batchedWriteValues[self._attributes.swhFrameNumber] = value

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
        self.inputs = OgnSdInstanceMappingPtrDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnSdInstanceMappingPtrDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnSdInstanceMappingPtrDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
