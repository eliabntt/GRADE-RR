"""Support for simplified access to data on nodes of type omni.syntheticdata.SdOnNewFrame

Synthetic Data postprocess node to execute pipeline after the NewFrame event has been received
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import numpy
class OgnSdOnNewFrameDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.syntheticdata.SdOnNewFrame

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Outputs:
            outputs.cudaStream
            outputs.exec
            outputs.renderProductDataPtrs
            outputs.renderProductPaths
            outputs.swhFrameNumber
    """
    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}
    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('outputs:cudaStream', 'uint64', 0, None, 'Cuda stream', {}, True, None, False, ''),
        ('outputs:exec', 'execution', 0, None, 'Executes for each newFrame event received', {}, True, None, False, ''),
        ('outputs:renderProductDataPtrs', 'uint64[]', 0, None, 'HydraRenderProduct data pointer.', {}, True, None, False, ''),
        ('outputs:renderProductPaths', 'token[]', 0, None, 'Render product path tokens.', {}, True, None, False, ''),
        ('outputs:swhFrameNumber', 'uint64', 0, None, 'Fabric frame number', {}, True, None, False, ''),
    ])
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.outputs.exec = og.Database.ROLE_EXECUTION
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = { }
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = []
            self._batchedReadValues = []

        def _prefetch(self):
            readAttributes = self._batchedReadAttributes
            newValues = _og._prefetch_input_attributes_data(readAttributes)
            if len(readAttributes) == len(newValues):
                self._batchedReadValues = newValues
    class ValuesForOutputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"cudaStream", "exec", "swhFrameNumber", "_batchedWriteValues"}
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self.renderProductDataPtrs_size = None
            self.renderProductPaths_size = None
            self._batchedWriteValues = { }

        @property
        def renderProductDataPtrs(self):
            data_view = og.AttributeValueHelper(self._attributes.renderProductDataPtrs)
            return data_view.get(reserved_element_count=self.renderProductDataPtrs_size)

        @renderProductDataPtrs.setter
        def renderProductDataPtrs(self, value):
            data_view = og.AttributeValueHelper(self._attributes.renderProductDataPtrs)
            data_view.set(value)
            self.renderProductDataPtrs_size = data_view.get_array_size()

        @property
        def renderProductPaths(self):
            data_view = og.AttributeValueHelper(self._attributes.renderProductPaths)
            return data_view.get(reserved_element_count=self.renderProductPaths_size)

        @renderProductPaths.setter
        def renderProductPaths(self, value):
            data_view = og.AttributeValueHelper(self._attributes.renderProductPaths)
            data_view.set(value)
            self.renderProductPaths_size = data_view.get_array_size()

        @property
        def cudaStream(self):
            value = self._batchedWriteValues.get(self._attributes.cudaStream)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.cudaStream)
                return data_view.get()

        @cudaStream.setter
        def cudaStream(self, value):
            self._batchedWriteValues[self._attributes.cudaStream] = value

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
        self.inputs = OgnSdOnNewFrameDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnSdOnNewFrameDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnSdOnNewFrameDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)