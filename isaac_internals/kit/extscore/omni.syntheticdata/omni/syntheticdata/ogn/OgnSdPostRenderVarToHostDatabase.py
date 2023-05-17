"""Support for simplified access to data on nodes of type omni.syntheticdata.SdPostRenderVarToHost

Expose a host renderVar from the input device renderVar.
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
class OgnSdPostRenderVarToHostDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.syntheticdata.SdPostRenderVarToHost

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.gpu
            inputs.renderVar
            inputs.renderVarHostSuffix
            inputs.rp
        Outputs:
            outputs.renderVar
    """
    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}
    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:gpu', 'uint64', 0, None, 'Pointer to shared context containing gpu foundations', {}, True, 0, False, ''),
        ('inputs:renderVar', 'token', 0, None, 'Name of the device renderVar to expose on the host', {}, True, '', False, ''),
        ('inputs:renderVarHostSuffix', 'string', 0, None, 'Suffix appended to the renderVar name', {ogn.MetadataKeys.DEFAULT: '"host"'}, True, 'host', False, ''),
        ('inputs:rp', 'uint64', 0, None, 'Pointer to render product for this view', {}, True, 0, False, ''),
        ('outputs:renderVar', 'token', 0, None, 'Name of the resulting renderVar on the host', {}, True, None, False, ''),
    ])
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"gpu", "renderVar", "renderVarHostSuffix", "rp", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.gpu, self._attributes.renderVar, self._attributes.renderVarHostSuffix, self._attributes.rp]
            self._batchedReadValues = [0, "", "host", 0]

        @property
        def gpu(self):
            return self._batchedReadValues[0]

        @gpu.setter
        def gpu(self, value):
            self._batchedReadValues[0] = value

        @property
        def renderVar(self):
            return self._batchedReadValues[1]

        @renderVar.setter
        def renderVar(self, value):
            self._batchedReadValues[1] = value

        @property
        def renderVarHostSuffix(self):
            return self._batchedReadValues[2]

        @renderVarHostSuffix.setter
        def renderVarHostSuffix(self, value):
            self._batchedReadValues[2] = value

        @property
        def rp(self):
            return self._batchedReadValues[3]

        @rp.setter
        def rp(self, value):
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
        LOCAL_PROPERTY_NAMES = {"renderVar", "_batchedWriteValues"}
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedWriteValues = { }

        @property
        def renderVar(self):
            value = self._batchedWriteValues.get(self._attributes.renderVar)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.renderVar)
                return data_view.get()

        @renderVar.setter
        def renderVar(self, value):
            self._batchedWriteValues[self._attributes.renderVar] = value

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
        self.inputs = OgnSdPostRenderVarToHostDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnSdPostRenderVarToHostDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnSdPostRenderVarToHostDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
