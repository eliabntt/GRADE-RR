"""Support for simplified access to data on nodes of type omni.syntheticdata.SdLinearArrayToTexture

Synthetic Data node to copy the input buffer array into a texture for visualization
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import numpy
class OgnSdLinearArrayToTextureDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.syntheticdata.SdLinearArrayToTexture

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.data
            inputs.exec
            inputs.height
            inputs.sdDisplayCudaMipmappedArray
            inputs.sdDisplayFormat
            inputs.sdDisplayHeight
            inputs.sdDisplayStream
            inputs.sdDisplayWidth
            inputs.stream
            inputs.width
        Outputs:
            outputs.cudaPtr
            outputs.exec
            outputs.format
            outputs.handlePtr
            outputs.height
            outputs.stream
            outputs.width
    """
    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}
    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:data', 'float4[]', 0, None, 'Buffer array data', {ogn.MetadataKeys.MEMORY_TYPE: 'cuda'}, True, [], False, ''),
        ('inputs:exec', 'execution', 0, None, 'Trigger', {}, True, None, False, ''),
        ('inputs:height', 'uint', 0, None, 'Buffer array height', {}, True, 0, False, ''),
        ('inputs:sdDisplayCudaMipmappedArray', 'uint64', 0, None, 'Visualization texture CUDA mipmapped array pointer', {}, True, 0, False, ''),
        ('inputs:sdDisplayFormat', 'uint64', 0, None, 'Visualization texture format', {}, True, 0, False, ''),
        ('inputs:sdDisplayHeight', 'uint', 0, None, 'Visualization texture Height', {}, True, 0, False, ''),
        ('inputs:sdDisplayStream', 'uint64', 0, None, 'Visualization texture CUDA stream pointer', {}, True, 0, False, ''),
        ('inputs:sdDisplayWidth', 'uint', 0, None, 'Visualization texture width', {}, True, 0, False, ''),
        ('inputs:stream', 'uint64', 0, None, 'Pointer to the CUDA Stream', {}, True, 0, False, ''),
        ('inputs:width', 'uint', 0, None, 'Buffer array width', {}, True, 0, False, ''),
        ('outputs:cudaPtr', 'uint64', 0, None, 'Display texture CUDA pointer', {}, True, None, False, ''),
        ('outputs:exec', 'execution', 0, 'Received', 'Executes when the event is received', {}, True, None, False, ''),
        ('outputs:format', 'uint64', 0, None, 'Display texture format', {}, True, None, False, ''),
        ('outputs:handlePtr', 'uint64', 0, None, 'Display texture handle reference', {}, True, None, False, ''),
        ('outputs:height', 'uint', 0, None, 'Display texture height', {}, True, None, False, ''),
        ('outputs:stream', 'uint64', 0, None, 'Output texture CUDA stream pointer', {}, True, None, False, ''),
        ('outputs:width', 'uint', 0, None, 'Display texture width', {}, True, None, False, ''),
    ])
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.exec = og.Database.ROLE_EXECUTION
        role_data.outputs.exec = og.Database.ROLE_EXECUTION
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"exec", "height", "sdDisplayCudaMipmappedArray", "sdDisplayFormat", "sdDisplayHeight", "sdDisplayStream", "sdDisplayWidth", "stream", "width", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.exec, self._attributes.height, self._attributes.sdDisplayCudaMipmappedArray, self._attributes.sdDisplayFormat, self._attributes.sdDisplayHeight, self._attributes.sdDisplayStream, self._attributes.sdDisplayWidth, self._attributes.stream, self._attributes.width]
            self._batchedReadValues = [None, 0, 0, 0, 0, 0, 0, 0, 0]

        @property
        def data(self):
            data_view = og.AttributeValueHelper(self._attributes.data)
            return data_view.get(on_gpu=True)

        @data.setter
        def data(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.data)
            data_view = og.AttributeValueHelper(self._attributes.data)
            data_view.set(value, on_gpu=True)
            self.data_size = data_view.get_array_size()

        @property
        def exec(self):
            return self._batchedReadValues[0]

        @exec.setter
        def exec(self, value):
            self._batchedReadValues[0] = value

        @property
        def height(self):
            return self._batchedReadValues[1]

        @height.setter
        def height(self, value):
            self._batchedReadValues[1] = value

        @property
        def sdDisplayCudaMipmappedArray(self):
            return self._batchedReadValues[2]

        @sdDisplayCudaMipmappedArray.setter
        def sdDisplayCudaMipmappedArray(self, value):
            self._batchedReadValues[2] = value

        @property
        def sdDisplayFormat(self):
            return self._batchedReadValues[3]

        @sdDisplayFormat.setter
        def sdDisplayFormat(self, value):
            self._batchedReadValues[3] = value

        @property
        def sdDisplayHeight(self):
            return self._batchedReadValues[4]

        @sdDisplayHeight.setter
        def sdDisplayHeight(self, value):
            self._batchedReadValues[4] = value

        @property
        def sdDisplayStream(self):
            return self._batchedReadValues[5]

        @sdDisplayStream.setter
        def sdDisplayStream(self, value):
            self._batchedReadValues[5] = value

        @property
        def sdDisplayWidth(self):
            return self._batchedReadValues[6]

        @sdDisplayWidth.setter
        def sdDisplayWidth(self, value):
            self._batchedReadValues[6] = value

        @property
        def stream(self):
            return self._batchedReadValues[7]

        @stream.setter
        def stream(self, value):
            self._batchedReadValues[7] = value

        @property
        def width(self):
            return self._batchedReadValues[8]

        @width.setter
        def width(self, value):
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
        LOCAL_PROPERTY_NAMES = {"cudaPtr", "exec", "format", "handlePtr", "height", "stream", "width", "_batchedWriteValues"}
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedWriteValues = { }

        @property
        def cudaPtr(self):
            value = self._batchedWriteValues.get(self._attributes.cudaPtr)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.cudaPtr)
                return data_view.get()

        @cudaPtr.setter
        def cudaPtr(self, value):
            self._batchedWriteValues[self._attributes.cudaPtr] = value

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
        def format(self):
            value = self._batchedWriteValues.get(self._attributes.format)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.format)
                return data_view.get()

        @format.setter
        def format(self, value):
            self._batchedWriteValues[self._attributes.format] = value

        @property
        def handlePtr(self):
            value = self._batchedWriteValues.get(self._attributes.handlePtr)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.handlePtr)
                return data_view.get()

        @handlePtr.setter
        def handlePtr(self, value):
            self._batchedWriteValues[self._attributes.handlePtr] = value

        @property
        def height(self):
            value = self._batchedWriteValues.get(self._attributes.height)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.height)
                return data_view.get()

        @height.setter
        def height(self, value):
            self._batchedWriteValues[self._attributes.height] = value

        @property
        def stream(self):
            value = self._batchedWriteValues.get(self._attributes.stream)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.stream)
                return data_view.get()

        @stream.setter
        def stream(self, value):
            self._batchedWriteValues[self._attributes.stream] = value

        @property
        def width(self):
            value = self._batchedWriteValues.get(self._attributes.width)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.width)
                return data_view.get()

        @width.setter
        def width(self, value):
            self._batchedWriteValues[self._attributes.width] = value

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
        self.inputs = OgnSdLinearArrayToTextureDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnSdLinearArrayToTextureDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnSdLinearArrayToTextureDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
