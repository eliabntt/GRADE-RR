"""Support for simplified access to data on nodes of type omni.syntheticdata.SdSimInstanceMapping

Synthetic Data node to update and cache the instance mapping data
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
class OgnSdSimInstanceMappingDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.syntheticdata.SdSimInstanceMapping

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.needTransform
            inputs.semanticFilterPredicate
        Outputs:
            outputs.exec
            outputs.semanticFilterPredicate
    """
    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}
    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:needTransform', 'bool', 0, None, 'If true compute the semantic entities world and object transforms', {ogn.MetadataKeys.DEFAULT: 'true'}, True, True, False, ''),
        ('inputs:semanticFilterPredicate', 'token', 0, None, 'The semantic filter predicate : a disjunctive normal form of semantic type and label', {ogn.MetadataKeys.DEFAULT: '"*:*"'}, True, '*:*', False, ''),
        ('outputs:exec', 'execution', 0, None, 'Trigger', {}, True, None, False, ''),
        ('outputs:semanticFilterPredicate', 'token', 0, None, 'The semantic filter predicate in normalized form', {}, True, None, False, ''),
    ])
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.outputs.exec = og.Database.ROLE_EXECUTION
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"needTransform", "semanticFilterPredicate", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.needTransform, self._attributes.semanticFilterPredicate]
            self._batchedReadValues = [True, "*:*"]

        @property
        def needTransform(self):
            return self._batchedReadValues[0]

        @needTransform.setter
        def needTransform(self, value):
            self._batchedReadValues[0] = value

        @property
        def semanticFilterPredicate(self):
            return self._batchedReadValues[1]

        @semanticFilterPredicate.setter
        def semanticFilterPredicate(self, value):
            self._batchedReadValues[1] = value

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
        LOCAL_PROPERTY_NAMES = {"exec", "semanticFilterPredicate", "_batchedWriteValues"}
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
        def semanticFilterPredicate(self):
            value = self._batchedWriteValues.get(self._attributes.semanticFilterPredicate)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.semanticFilterPredicate)
                return data_view.get()

        @semanticFilterPredicate.setter
        def semanticFilterPredicate(self, value):
            self._batchedWriteValues[self._attributes.semanticFilterPredicate] = value

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
        self.inputs = OgnSdSimInstanceMappingDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnSdSimInstanceMappingDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnSdSimInstanceMappingDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
