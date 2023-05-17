"""Support for simplified access to data on nodes of type omni.syntheticdata.SdInstanceMapping

Synthetic Data node to expose the scene instances semantic hierarchy information
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import numpy
class OgnSdInstanceMappingDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.syntheticdata.SdInstanceMapping

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.exec
            inputs.lazy
            inputs.renderResults
            inputs.swhFrameNumber
        Outputs:
            outputs.exec
            outputs.sdIMInstanceSemanticMap
            outputs.sdIMInstanceTokens
            outputs.sdIMMaxSemanticHierarchyDepth
            outputs.sdIMMinInstanceIndex
            outputs.sdIMMinSemanticIndex
            outputs.sdIMNumInstances
            outputs.sdIMNumSemanticTokens
            outputs.sdIMNumSemantics
            outputs.sdIMSemanticLocalTransform
            outputs.sdIMSemanticTokenMap
            outputs.sdIMSemanticWorldTransform
            outputs.swhFrameNumber

    Predefined Tokens:
        tokens.InstanceMappingInfoSDhost
        tokens.InstanceMapSDhost
        tokens.SemanticLabelTokenSDhost
        tokens.InstancePrimTokenSDhost
        tokens.SemanticLocalTransformSDhost
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
        ('inputs:lazy', 'bool', 0, None, 'Compute outputs only when connected to a downstream node', {ogn.MetadataKeys.DEFAULT: 'true'}, True, True, False, ''),
        ('inputs:renderResults', 'uint64', 0, None, 'Render results pointer', {}, True, 0, False, ''),
        ('inputs:swhFrameNumber', 'uint64', 0, None, 'Fabric frame number', {}, True, 0, False, ''),
        ('outputs:exec', 'execution', 0, 'Received', 'Executes when the event is received', {}, True, None, False, ''),
        ('outputs:sdIMInstanceSemanticMap', 'uchar[]', 0, None, 'Raw array of uint16_t of size sdIMNumInstances*sdIMMaxSemanticHierarchyDepth containing the mapping from the instances index to their inherited semantic entities', {}, True, None, False, ''),
        ('outputs:sdIMInstanceTokens', 'token[]', 0, None, 'Instance array containing the token for every instances', {}, True, None, False, ''),
        ('outputs:sdIMMaxSemanticHierarchyDepth', 'uint', 0, None, 'Maximal number of semantic entities inherited by an instance', {}, True, None, False, ''),
        ('outputs:sdIMMinInstanceIndex', 'uint', 0, None, 'Instance id of the first instance in the instance arrays', {}, True, None, False, ''),
        ('outputs:sdIMMinSemanticIndex', 'uint', 0, None, 'Semantic id of the first semantic entity in the semantic arrays', {}, True, None, False, ''),
        ('outputs:sdIMNumInstances', 'uint', 0, None, 'Number of instances in the instance arrays', {}, True, None, False, ''),
        ('outputs:sdIMNumSemanticTokens', 'uint', 0, None, 'Number of semantics token including the semantic entity path, the semantic entity types and if the number of semantic types is greater than one a ', {}, True, None, False, ''),
        ('outputs:sdIMNumSemantics', 'uint', 0, None, 'Number of semantic entities in the semantic arrays', {}, True, None, False, ''),
        ('outputs:sdIMSemanticLocalTransform', 'float[]', 0, None, 'Semantic array of 4x4 float matrices containing the transform from world to local space for every semantic entity', {}, True, None, False, ''),
        ('outputs:sdIMSemanticTokenMap', 'token[]', 0, None, 'Semantic array of token of size numSemantics * numSemanticTypes containing the mapping from the semantic entities to the semantic entity path and semantic types', {}, True, None, False, ''),
        ('outputs:sdIMSemanticWorldTransform', 'float[]', 0, None, 'Semantic array of 4x4 float matrices containing the transform from local to world space for every semantic entity', {}, True, None, False, ''),
        ('outputs:swhFrameNumber', 'uint64', 0, None, 'Fabric frame number', {}, True, None, False, ''),
    ])
    class tokens:
        InstanceMappingInfoSDhost = "InstanceMappingInfoSDhost"
        InstanceMapSDhost = "InstanceMapSDhost"
        SemanticLabelTokenSDhost = "SemanticLabelTokenSDhost"
        InstancePrimTokenSDhost = "InstancePrimTokenSDhost"
        SemanticLocalTransformSDhost = "SemanticLocalTransformSDhost"
        SemanticWorldTransformSDhost = "SemanticWorldTransformSDhost"
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.exec = og.Database.ROLE_EXECUTION
        role_data.outputs.exec = og.Database.ROLE_EXECUTION
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"exec", "lazy", "renderResults", "swhFrameNumber", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.exec, self._attributes.lazy, self._attributes.renderResults, self._attributes.swhFrameNumber]
            self._batchedReadValues = [None, True, 0, 0]

        @property
        def exec(self):
            return self._batchedReadValues[0]

        @exec.setter
        def exec(self, value):
            self._batchedReadValues[0] = value

        @property
        def lazy(self):
            return self._batchedReadValues[1]

        @lazy.setter
        def lazy(self, value):
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
        LOCAL_PROPERTY_NAMES = {"exec", "sdIMMaxSemanticHierarchyDepth", "sdIMMinInstanceIndex", "sdIMMinSemanticIndex", "sdIMNumInstances", "sdIMNumSemanticTokens", "sdIMNumSemantics", "swhFrameNumber", "_batchedWriteValues"}
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self.sdIMInstanceSemanticMap_size = None
            self.sdIMInstanceTokens_size = None
            self.sdIMSemanticLocalTransform_size = None
            self.sdIMSemanticTokenMap_size = None
            self.sdIMSemanticWorldTransform_size = None
            self._batchedWriteValues = { }

        @property
        def sdIMInstanceSemanticMap(self):
            data_view = og.AttributeValueHelper(self._attributes.sdIMInstanceSemanticMap)
            return data_view.get(reserved_element_count=self.sdIMInstanceSemanticMap_size)

        @sdIMInstanceSemanticMap.setter
        def sdIMInstanceSemanticMap(self, value):
            data_view = og.AttributeValueHelper(self._attributes.sdIMInstanceSemanticMap)
            data_view.set(value)
            self.sdIMInstanceSemanticMap_size = data_view.get_array_size()

        @property
        def sdIMInstanceTokens(self):
            data_view = og.AttributeValueHelper(self._attributes.sdIMInstanceTokens)
            return data_view.get(reserved_element_count=self.sdIMInstanceTokens_size)

        @sdIMInstanceTokens.setter
        def sdIMInstanceTokens(self, value):
            data_view = og.AttributeValueHelper(self._attributes.sdIMInstanceTokens)
            data_view.set(value)
            self.sdIMInstanceTokens_size = data_view.get_array_size()

        @property
        def sdIMSemanticLocalTransform(self):
            data_view = og.AttributeValueHelper(self._attributes.sdIMSemanticLocalTransform)
            return data_view.get(reserved_element_count=self.sdIMSemanticLocalTransform_size)

        @sdIMSemanticLocalTransform.setter
        def sdIMSemanticLocalTransform(self, value):
            data_view = og.AttributeValueHelper(self._attributes.sdIMSemanticLocalTransform)
            data_view.set(value)
            self.sdIMSemanticLocalTransform_size = data_view.get_array_size()

        @property
        def sdIMSemanticTokenMap(self):
            data_view = og.AttributeValueHelper(self._attributes.sdIMSemanticTokenMap)
            return data_view.get(reserved_element_count=self.sdIMSemanticTokenMap_size)

        @sdIMSemanticTokenMap.setter
        def sdIMSemanticTokenMap(self, value):
            data_view = og.AttributeValueHelper(self._attributes.sdIMSemanticTokenMap)
            data_view.set(value)
            self.sdIMSemanticTokenMap_size = data_view.get_array_size()

        @property
        def sdIMSemanticWorldTransform(self):
            data_view = og.AttributeValueHelper(self._attributes.sdIMSemanticWorldTransform)
            return data_view.get(reserved_element_count=self.sdIMSemanticWorldTransform_size)

        @sdIMSemanticWorldTransform.setter
        def sdIMSemanticWorldTransform(self, value):
            data_view = og.AttributeValueHelper(self._attributes.sdIMSemanticWorldTransform)
            data_view.set(value)
            self.sdIMSemanticWorldTransform_size = data_view.get_array_size()

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
        def sdIMMaxSemanticHierarchyDepth(self):
            value = self._batchedWriteValues.get(self._attributes.sdIMMaxSemanticHierarchyDepth)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.sdIMMaxSemanticHierarchyDepth)
                return data_view.get()

        @sdIMMaxSemanticHierarchyDepth.setter
        def sdIMMaxSemanticHierarchyDepth(self, value):
            self._batchedWriteValues[self._attributes.sdIMMaxSemanticHierarchyDepth] = value

        @property
        def sdIMMinInstanceIndex(self):
            value = self._batchedWriteValues.get(self._attributes.sdIMMinInstanceIndex)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.sdIMMinInstanceIndex)
                return data_view.get()

        @sdIMMinInstanceIndex.setter
        def sdIMMinInstanceIndex(self, value):
            self._batchedWriteValues[self._attributes.sdIMMinInstanceIndex] = value

        @property
        def sdIMMinSemanticIndex(self):
            value = self._batchedWriteValues.get(self._attributes.sdIMMinSemanticIndex)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.sdIMMinSemanticIndex)
                return data_view.get()

        @sdIMMinSemanticIndex.setter
        def sdIMMinSemanticIndex(self, value):
            self._batchedWriteValues[self._attributes.sdIMMinSemanticIndex] = value

        @property
        def sdIMNumInstances(self):
            value = self._batchedWriteValues.get(self._attributes.sdIMNumInstances)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.sdIMNumInstances)
                return data_view.get()

        @sdIMNumInstances.setter
        def sdIMNumInstances(self, value):
            self._batchedWriteValues[self._attributes.sdIMNumInstances] = value

        @property
        def sdIMNumSemanticTokens(self):
            value = self._batchedWriteValues.get(self._attributes.sdIMNumSemanticTokens)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.sdIMNumSemanticTokens)
                return data_view.get()

        @sdIMNumSemanticTokens.setter
        def sdIMNumSemanticTokens(self, value):
            self._batchedWriteValues[self._attributes.sdIMNumSemanticTokens] = value

        @property
        def sdIMNumSemantics(self):
            value = self._batchedWriteValues.get(self._attributes.sdIMNumSemantics)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.sdIMNumSemantics)
                return data_view.get()

        @sdIMNumSemantics.setter
        def sdIMNumSemantics(self, value):
            self._batchedWriteValues[self._attributes.sdIMNumSemantics] = value

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
        self.inputs = OgnSdInstanceMappingDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnSdInstanceMappingDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnSdInstanceMappingDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
