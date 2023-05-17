"""Support for simplified access to data on nodes of type omni.syntheticdata.SdRenderVarToRawArray

Synthetic Data action node to copy the input rendervar into an output raw array
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import numpy
class OgnSdRenderVarToRawArrayDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.syntheticdata.SdRenderVarToRawArray

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.cudaStream
            inputs.exec
            inputs.renderResults
            inputs.renderVar
            inputs.swhFrameNumber
        Outputs:
            outputs.bufferSize
            outputs.cudaStream
            outputs.data
            outputs.exec
            outputs.format
            outputs.height
            outputs.swhFrameNumber
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
        ('inputs:cudaStream', 'uint64', 0, None, 'Pointer to the CUDA stream', {}, True, 0, False, ''),
        ('inputs:exec', 'execution', 0, None, 'Trigger', {}, True, None, False, ''),
        ('inputs:renderResults', 'uint64', 0, None, 'Render results pointer', {}, True, 0, False, ''),
        ('inputs:renderVar', 'token', 0, None, 'Name of the renderVar', {}, True, '', False, ''),
        ('inputs:swhFrameNumber', 'uint64', 0, None, 'Fabric frame number', {}, True, 0, False, ''),
        ('outputs:bufferSize', 'uint64', 0, None, 'Size (in bytes) of the buffer (0 if the input is a texture)', {}, True, None, False, ''),
        ('outputs:cudaStream', 'uint64', 0, None, 'Pointer to the CUDA stream', {}, True, None, False, ''),
        ('outputs:data', 'uchar[]', 0, None, 'Buffer array data', {ogn.MetadataKeys.MEMORY_TYPE: 'any', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:exec', 'execution', 0, 'Received', 'Executes when the event is received', {}, True, None, False, ''),
        ('outputs:format', 'uint64', 0, None, 'Format', {}, True, None, False, ''),
        ('outputs:height', 'uint', 0, None, 'Height  (0 if the input is a buffer)', {}, True, None, False, ''),
        ('outputs:swhFrameNumber', 'uint64', 0, None, 'Frame number', {}, True, None, False, ''),
        ('outputs:width', 'uint', 0, None, 'Width  (0 if the input is a buffer)', {}, True, None, False, ''),
    ])
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.exec = og.Database.ROLE_EXECUTION
        role_data.outputs.exec = og.Database.ROLE_EXECUTION
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"cudaStream", "exec", "renderResults", "renderVar", "swhFrameNumber", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.cudaStream, self._attributes.exec, self._attributes.renderResults, self._attributes.renderVar, self._attributes.swhFrameNumber]
            self._batchedReadValues = [0, None, 0, "", 0]

        @property
        def cudaStream(self):
            return self._batchedReadValues[0]

        @cudaStream.setter
        def cudaStream(self, value):
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
        def renderVar(self):
            return self._batchedReadValues[3]

        @renderVar.setter
        def renderVar(self, value):
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
        LOCAL_PROPERTY_NAMES = {"bufferSize", "cudaStream", "exec", "format", "height", "swhFrameNumber", "width", "_batchedWriteValues"}
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self.data_size = 0
            self._batchedWriteValues = { }

        class __data:
            def __init__(self, parent):
                self._parent = parent

            @property
            def cpu(self):
                data_view = og.AttributeValueHelper(self._parent._attributes.data)
                return data_view.get(reserved_element_count=self._parent.data_size)

            @cpu.setter
            def cpu(self, value):
                data_view = og.AttributeValueHelper(self._parent._attributes.cpu)
                data_view.set(value)
                self._parent.cpu_size = data_view.get_array_size()

            @property
            def gpu(self):
                data_view = og.AttributeValueHelper(self._parent._attributes.data)
                data_view.gpu_ptr_kind = og.PtrToPtrKind.CPU
                return data_view.get(reserved_element_count=self._parent.data_size, on_gpu=True)

            @gpu.setter
            def gpu(self, value):
                data_view = og.AttributeValueHelper(self._parent._attributes.gpu)
                data_view.set(value)
                self._parent.gpu_size = data_view.get_array_size()

        @property
        def data(self):
            return self.__class__.__data(self)

        @property
        def bufferSize(self):
            value = self._batchedWriteValues.get(self._attributes.bufferSize)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.bufferSize)
                return data_view.get()

        @bufferSize.setter
        def bufferSize(self, value):
            self._batchedWriteValues[self._attributes.bufferSize] = value

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
        self.inputs = OgnSdRenderVarToRawArrayDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnSdRenderVarToRawArrayDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnSdRenderVarToRawArrayDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
