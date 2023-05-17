"""Support for simplified access to data on nodes of type omni.syntheticdata.SdTextureToLinearArray

SyntheticData node to copy the input texture into a linear array buffer
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import numpy
class OgnSdTextureToLinearArrayDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.syntheticdata.SdTextureToLinearArray

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.cudaMipmappedArray
            inputs.format
            inputs.height
            inputs.hydraTime
            inputs.mipCount
            inputs.outputHeight
            inputs.outputWidth
            inputs.simTime
            inputs.stream
            inputs.width
        Outputs:
            outputs.data
            outputs.height
            outputs.hydraTime
            outputs.simTime
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
        ('inputs:cudaMipmappedArray', 'uint64', 0, None, 'Pointer to the CUDA Mipmapped Array', {}, True, 0, False, ''),
        ('inputs:format', 'uint64', 0, None, 'Format', {}, True, 0, False, ''),
        ('inputs:height', 'uint', 0, None, 'Height', {}, True, 0, False, ''),
        ('inputs:hydraTime', 'double', 0, None, 'Hydra time in stage', {}, True, 0.0, False, ''),
        ('inputs:mipCount', 'uint', 0, None, 'Mip Count', {}, True, 0, False, ''),
        ('inputs:outputHeight', 'uint', 0, None, 'Requested output height', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('inputs:outputWidth', 'uint', 0, None, 'Requested output width', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('inputs:simTime', 'double', 0, None, 'Simulation time', {}, True, 0.0, False, ''),
        ('inputs:stream', 'uint64', 0, None, 'Pointer to the CUDA Stream', {}, True, 0, False, ''),
        ('inputs:width', 'uint', 0, None, 'Width', {}, True, 0, False, ''),
        ('outputs:data', 'float4[]', 0, None, 'Buffer array data', {ogn.MetadataKeys.MEMORY_TYPE: 'cuda', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:height', 'uint', 0, None, 'Buffer array height', {}, True, None, False, ''),
        ('outputs:hydraTime', 'double', 0, None, 'Hydra time in stage', {}, True, None, False, ''),
        ('outputs:simTime', 'double', 0, None, 'Simulation time', {}, True, None, False, ''),
        ('outputs:stream', 'uint64', 0, None, 'Pointer to the CUDA Stream', {}, True, None, False, ''),
        ('outputs:width', 'uint', 0, None, 'Buffer array width', {}, True, None, False, ''),
    ])
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"cudaMipmappedArray", "format", "height", "hydraTime", "mipCount", "outputHeight", "outputWidth", "simTime", "stream", "width", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.cudaMipmappedArray, self._attributes.format, self._attributes.height, self._attributes.hydraTime, self._attributes.mipCount, self._attributes.outputHeight, self._attributes.outputWidth, self._attributes.simTime, self._attributes.stream, self._attributes.width]
            self._batchedReadValues = [0, 0, 0, 0.0, 0, 0, 0, 0.0, 0, 0]

        @property
        def cudaMipmappedArray(self):
            return self._batchedReadValues[0]

        @cudaMipmappedArray.setter
        def cudaMipmappedArray(self, value):
            self._batchedReadValues[0] = value

        @property
        def format(self):
            return self._batchedReadValues[1]

        @format.setter
        def format(self, value):
            self._batchedReadValues[1] = value

        @property
        def height(self):
            return self._batchedReadValues[2]

        @height.setter
        def height(self, value):
            self._batchedReadValues[2] = value

        @property
        def hydraTime(self):
            return self._batchedReadValues[3]

        @hydraTime.setter
        def hydraTime(self, value):
            self._batchedReadValues[3] = value

        @property
        def mipCount(self):
            return self._batchedReadValues[4]

        @mipCount.setter
        def mipCount(self, value):
            self._batchedReadValues[4] = value

        @property
        def outputHeight(self):
            return self._batchedReadValues[5]

        @outputHeight.setter
        def outputHeight(self, value):
            self._batchedReadValues[5] = value

        @property
        def outputWidth(self):
            return self._batchedReadValues[6]

        @outputWidth.setter
        def outputWidth(self, value):
            self._batchedReadValues[6] = value

        @property
        def simTime(self):
            return self._batchedReadValues[7]

        @simTime.setter
        def simTime(self, value):
            self._batchedReadValues[7] = value

        @property
        def stream(self):
            return self._batchedReadValues[8]

        @stream.setter
        def stream(self, value):
            self._batchedReadValues[8] = value

        @property
        def width(self):
            return self._batchedReadValues[9]

        @width.setter
        def width(self, value):
            self._batchedReadValues[9] = value

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
        LOCAL_PROPERTY_NAMES = {"height", "hydraTime", "simTime", "stream", "width", "_batchedWriteValues"}
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self.data_size = 0
            self._batchedWriteValues = { }

        @property
        def data(self):
            data_view = og.AttributeValueHelper(self._attributes.data)
            return data_view.get(reserved_element_count=self.data_size, on_gpu=True)

        @data.setter
        def data(self, value):
            data_view = og.AttributeValueHelper(self._attributes.data)
            data_view.set(value, on_gpu=True)
            self.data_size = data_view.get_array_size()

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
        def hydraTime(self):
            value = self._batchedWriteValues.get(self._attributes.hydraTime)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.hydraTime)
                return data_view.get()

        @hydraTime.setter
        def hydraTime(self, value):
            self._batchedWriteValues[self._attributes.hydraTime] = value

        @property
        def simTime(self):
            value = self._batchedWriteValues.get(self._attributes.simTime)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.simTime)
                return data_view.get()

        @simTime.setter
        def simTime(self, value):
            self._batchedWriteValues[self._attributes.simTime] = value

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
        self.inputs = OgnSdTextureToLinearArrayDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnSdTextureToLinearArrayDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnSdTextureToLinearArrayDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
