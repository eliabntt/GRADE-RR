"""Support for simplified access to data on nodes of type omni.syntheticdata.SdPostCompRenderVarTextures

Synthetic Data node to compose a front renderVar texture into a back renderVar texture
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import numpy
class OgnSdPostCompRenderVarTexturesDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.syntheticdata.SdPostCompRenderVarTextures

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.cudaPtr
            inputs.format
            inputs.gpu
            inputs.height
            inputs.mode
            inputs.parameters
            inputs.renderVar
            inputs.rp
            inputs.width

    Predefined Tokens:
        tokens.line
        tokens.grid
    """
    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}
    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:cudaPtr', 'uint64', 0, None, 'Front texture CUDA pointer', {}, True, 0, False, ''),
        ('inputs:format', 'uint64', 0, None, 'Front texture format', {}, True, 0, False, ''),
        ('inputs:gpu', 'uint64', 0, 'gpuFoundations', 'Pointer to shared context containing gpu foundations', {}, True, 0, False, ''),
        ('inputs:height', 'uint', 0, None, 'Front texture height', {}, True, 0, False, ''),
        ('inputs:mode', 'token', 0, None, 'Mode : grid, line', {ogn.MetadataKeys.DEFAULT: '"line"'}, True, 'line', False, ''),
        ('inputs:parameters', 'float3', 0, None, 'Parameters', {ogn.MetadataKeys.DEFAULT: '[0, 0, 0]'}, True, [0, 0, 0], False, ''),
        ('inputs:renderVar', 'token', 0, None, 'Name of the back RenderVar', {ogn.MetadataKeys.DEFAULT: '"LdrColor"'}, True, 'LdrColor', False, ''),
        ('inputs:rp', 'uint64', 0, 'renderProduct', 'Pointer to render product for this view', {}, True, 0, False, ''),
        ('inputs:width', 'uint', 0, None, 'Front texture width', {}, True, 0, False, ''),
    ])
    class tokens:
        line = "line"
        grid = "grid"
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"cudaPtr", "format", "gpu", "height", "mode", "parameters", "renderVar", "rp", "width", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.cudaPtr, self._attributes.format, self._attributes.gpu, self._attributes.height, self._attributes.mode, self._attributes.parameters, self._attributes.renderVar, self._attributes.rp, self._attributes.width]
            self._batchedReadValues = [0, 0, 0, 0, "line", [0, 0, 0], "LdrColor", 0, 0]

        @property
        def cudaPtr(self):
            return self._batchedReadValues[0]

        @cudaPtr.setter
        def cudaPtr(self, value):
            self._batchedReadValues[0] = value

        @property
        def format(self):
            return self._batchedReadValues[1]

        @format.setter
        def format(self, value):
            self._batchedReadValues[1] = value

        @property
        def gpu(self):
            return self._batchedReadValues[2]

        @gpu.setter
        def gpu(self, value):
            self._batchedReadValues[2] = value

        @property
        def height(self):
            return self._batchedReadValues[3]

        @height.setter
        def height(self, value):
            self._batchedReadValues[3] = value

        @property
        def mode(self):
            return self._batchedReadValues[4]

        @mode.setter
        def mode(self, value):
            self._batchedReadValues[4] = value

        @property
        def parameters(self):
            return self._batchedReadValues[5]

        @parameters.setter
        def parameters(self, value):
            self._batchedReadValues[5] = value

        @property
        def renderVar(self):
            return self._batchedReadValues[6]

        @renderVar.setter
        def renderVar(self, value):
            self._batchedReadValues[6] = value

        @property
        def rp(self):
            return self._batchedReadValues[7]

        @rp.setter
        def rp(self, value):
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
        LOCAL_PROPERTY_NAMES = { }
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedWriteValues = { }

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
        self.inputs = OgnSdPostCompRenderVarTexturesDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnSdPostCompRenderVarTexturesDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnSdPostCompRenderVarTexturesDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
