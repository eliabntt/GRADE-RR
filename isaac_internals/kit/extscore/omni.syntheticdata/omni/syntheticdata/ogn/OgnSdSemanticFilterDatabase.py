"""Support for simplified access to data on nodes of type omni.syntheticdata.SdSemanticFilter

Synthetic Data node to declare a semantic filter.
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
class OgnSdSemanticFilterDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.syntheticdata.SdSemanticFilter

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.exec
            inputs.hierarchicalLabels
            inputs.matchingLabels
            inputs.name
            inputs.predicate
        Outputs:
            outputs.exec
            outputs.name
            outputs.predicate
    """
    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}
    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:exec', 'execution', 0, None, 'Dependency', {}, True, None, False, ''),
        ('inputs:hierarchicalLabels', 'bool', 0, None, 'If true the filter consider all labels in the semantic hierarchy above the prims', {ogn.MetadataKeys.DEFAULT: 'false'}, True, False, False, ''),
        ('inputs:matchingLabels', 'bool', 0, None, 'If true output only the labels matching the filter (if false keep all labels of the matching prims)', {ogn.MetadataKeys.DEFAULT: 'true'}, True, True, False, ''),
        ('inputs:name', 'token', 0, None, 'Filter unique identifier [if empty, use the normalized predicate as an identifier]', {ogn.MetadataKeys.DEFAULT: '""'}, True, '', False, ''),
        ('inputs:predicate', 'token', 0, None, 'The semantic filter specification : a disjunctive normal form of semantic type and label', {ogn.MetadataKeys.DEFAULT: '""'}, True, '', False, ''),
        ('outputs:exec', 'execution', 0, None, 'Trigger', {}, True, None, False, ''),
        ('outputs:name', 'token', 0, None, 'The semantic filter name identifier', {ogn.MetadataKeys.DEFAULT: '""'}, True, '', False, ''),
        ('outputs:predicate', 'token', 0, None, 'The semantic filter predicate in normalized form', {ogn.MetadataKeys.DEFAULT: '""'}, True, '', False, ''),
    ])
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.exec = og.Database.ROLE_EXECUTION
        role_data.outputs.exec = og.Database.ROLE_EXECUTION
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"exec", "hierarchicalLabels", "matchingLabels", "name", "predicate", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.exec, self._attributes.hierarchicalLabels, self._attributes.matchingLabels, self._attributes.name, self._attributes.predicate]
            self._batchedReadValues = [None, False, True, "", ""]

        @property
        def exec(self):
            return self._batchedReadValues[0]

        @exec.setter
        def exec(self, value):
            self._batchedReadValues[0] = value

        @property
        def hierarchicalLabels(self):
            return self._batchedReadValues[1]

        @hierarchicalLabels.setter
        def hierarchicalLabels(self, value):
            self._batchedReadValues[1] = value

        @property
        def matchingLabels(self):
            return self._batchedReadValues[2]

        @matchingLabels.setter
        def matchingLabels(self, value):
            self._batchedReadValues[2] = value

        @property
        def name(self):
            return self._batchedReadValues[3]

        @name.setter
        def name(self, value):
            self._batchedReadValues[3] = value

        @property
        def predicate(self):
            return self._batchedReadValues[4]

        @predicate.setter
        def predicate(self, value):
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
        LOCAL_PROPERTY_NAMES = {"exec", "name", "predicate", "_batchedWriteValues"}
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
        def name(self):
            value = self._batchedWriteValues.get(self._attributes.name)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.name)
                return data_view.get()

        @name.setter
        def name(self, value):
            self._batchedWriteValues[self._attributes.name] = value

        @property
        def predicate(self):
            value = self._batchedWriteValues.get(self._attributes.predicate)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.predicate)
                return data_view.get()

        @predicate.setter
        def predicate(self, value):
            self._batchedWriteValues[self._attributes.predicate] = value

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
        self.inputs = OgnSdSemanticFilterDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnSdSemanticFilterDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnSdSemanticFilterDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
