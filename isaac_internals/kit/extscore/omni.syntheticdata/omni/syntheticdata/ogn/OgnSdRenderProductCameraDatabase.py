"""Support for simplified access to data on nodes of type omni.syntheticdata.SdRenderProductCamera

Synthetic Data node to expose the camera data
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import numpy
class OgnSdRenderProductCameraDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.syntheticdata.SdRenderProductCamera

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.exec
            inputs.renderProductPath
            inputs.renderResults
            inputs.swhFrameNumber
        Outputs:
            outputs.cameraApertureOffset
            outputs.cameraApertureSize
            outputs.cameraFStop
            outputs.cameraFisheyeParams
            outputs.cameraFocalLength
            outputs.cameraFocusDistance
            outputs.cameraModel
            outputs.cameraNearFar
            outputs.cameraProjection
            outputs.cameraViewTransform
            outputs.exec
            outputs.metersPerSceneUnit
            outputs.renderProductResolution
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
        ('inputs:exec', 'execution', 0, None, 'Trigger', {}, True, None, False, ''),
        ('inputs:renderProductPath', 'token', 0, None, 'RenderProduct prim path', {}, True, '', False, ''),
        ('inputs:renderResults', 'uint64', 0, None, 'Render results', {}, True, 0, False, ''),
        ('inputs:swhFrameNumber', 'uint64', 0, None, 'Fabric frame number', {}, True, 0, False, ''),
        ('outputs:cameraApertureOffset', 'float2', 0, None, 'Camera horizontal and vertical aperture offset', {}, True, None, False, ''),
        ('outputs:cameraApertureSize', 'float2', 0, None, 'Camera horizontal and vertical aperture', {}, True, None, False, ''),
        ('outputs:cameraFStop', 'float', 0, None, 'Camera fStop', {}, True, None, False, ''),
        ('outputs:cameraFisheyeParams', 'float[]', 0, None, 'Camera fisheye projection parameters', {}, True, None, False, ''),
        ('outputs:cameraFocalLength', 'float', 0, None, 'Camera focal length', {}, True, None, False, ''),
        ('outputs:cameraFocusDistance', 'float', 0, None, 'Camera focus distance', {}, True, None, False, ''),
        ('outputs:cameraModel', 'int', 0, None, 'Camera model (pinhole or fisheye models)', {}, True, None, False, ''),
        ('outputs:cameraNearFar', 'float2', 0, None, 'Camera near/far clipping range', {}, True, None, False, ''),
        ('outputs:cameraProjection', 'matrix4d', 0, None, 'Camera projection matrix', {}, True, None, False, ''),
        ('outputs:cameraViewTransform', 'matrix4d', 0, None, 'Camera view matrix', {}, True, None, False, ''),
        ('outputs:exec', 'execution', 0, 'Received', 'Executes for each newFrame event received', {}, True, None, False, ''),
        ('outputs:metersPerSceneUnit', 'float', 0, None, 'Scene units to meters scale', {}, True, None, False, ''),
        ('outputs:renderProductResolution', 'int2', 0, None, 'RenderProduct resolution', {}, True, None, False, ''),
        ('outputs:swhFrameNumber', 'uint64', 0, None, 'Fabric frame number', {}, True, None, False, ''),
    ])
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.exec = og.Database.ROLE_EXECUTION
        role_data.outputs.cameraProjection = og.Database.ROLE_MATRIX
        role_data.outputs.cameraViewTransform = og.Database.ROLE_MATRIX
        role_data.outputs.exec = og.Database.ROLE_EXECUTION
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"exec", "renderProductPath", "renderResults", "swhFrameNumber", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.exec, self._attributes.renderProductPath, self._attributes.renderResults, self._attributes.swhFrameNumber]
            self._batchedReadValues = [None, "", 0, 0]

        @property
        def exec(self):
            return self._batchedReadValues[0]

        @exec.setter
        def exec(self, value):
            self._batchedReadValues[0] = value

        @property
        def renderProductPath(self):
            return self._batchedReadValues[1]

        @renderProductPath.setter
        def renderProductPath(self, value):
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
        LOCAL_PROPERTY_NAMES = {"cameraApertureOffset", "cameraApertureSize", "cameraFStop", "cameraFocalLength", "cameraFocusDistance", "cameraModel", "cameraNearFar", "cameraProjection", "cameraViewTransform", "exec", "metersPerSceneUnit", "renderProductResolution", "swhFrameNumber", "_batchedWriteValues"}
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self.cameraFisheyeParams_size = None
            self._batchedWriteValues = { }

        @property
        def cameraFisheyeParams(self):
            data_view = og.AttributeValueHelper(self._attributes.cameraFisheyeParams)
            return data_view.get(reserved_element_count=self.cameraFisheyeParams_size)

        @cameraFisheyeParams.setter
        def cameraFisheyeParams(self, value):
            data_view = og.AttributeValueHelper(self._attributes.cameraFisheyeParams)
            data_view.set(value)
            self.cameraFisheyeParams_size = data_view.get_array_size()

        @property
        def cameraApertureOffset(self):
            value = self._batchedWriteValues.get(self._attributes.cameraApertureOffset)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.cameraApertureOffset)
                return data_view.get()

        @cameraApertureOffset.setter
        def cameraApertureOffset(self, value):
            self._batchedWriteValues[self._attributes.cameraApertureOffset] = value

        @property
        def cameraApertureSize(self):
            value = self._batchedWriteValues.get(self._attributes.cameraApertureSize)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.cameraApertureSize)
                return data_view.get()

        @cameraApertureSize.setter
        def cameraApertureSize(self, value):
            self._batchedWriteValues[self._attributes.cameraApertureSize] = value

        @property
        def cameraFStop(self):
            value = self._batchedWriteValues.get(self._attributes.cameraFStop)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.cameraFStop)
                return data_view.get()

        @cameraFStop.setter
        def cameraFStop(self, value):
            self._batchedWriteValues[self._attributes.cameraFStop] = value

        @property
        def cameraFocalLength(self):
            value = self._batchedWriteValues.get(self._attributes.cameraFocalLength)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.cameraFocalLength)
                return data_view.get()

        @cameraFocalLength.setter
        def cameraFocalLength(self, value):
            self._batchedWriteValues[self._attributes.cameraFocalLength] = value

        @property
        def cameraFocusDistance(self):
            value = self._batchedWriteValues.get(self._attributes.cameraFocusDistance)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.cameraFocusDistance)
                return data_view.get()

        @cameraFocusDistance.setter
        def cameraFocusDistance(self, value):
            self._batchedWriteValues[self._attributes.cameraFocusDistance] = value

        @property
        def cameraModel(self):
            value = self._batchedWriteValues.get(self._attributes.cameraModel)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.cameraModel)
                return data_view.get()

        @cameraModel.setter
        def cameraModel(self, value):
            self._batchedWriteValues[self._attributes.cameraModel] = value

        @property
        def cameraNearFar(self):
            value = self._batchedWriteValues.get(self._attributes.cameraNearFar)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.cameraNearFar)
                return data_view.get()

        @cameraNearFar.setter
        def cameraNearFar(self, value):
            self._batchedWriteValues[self._attributes.cameraNearFar] = value

        @property
        def cameraProjection(self):
            value = self._batchedWriteValues.get(self._attributes.cameraProjection)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.cameraProjection)
                return data_view.get()

        @cameraProjection.setter
        def cameraProjection(self, value):
            self._batchedWriteValues[self._attributes.cameraProjection] = value

        @property
        def cameraViewTransform(self):
            value = self._batchedWriteValues.get(self._attributes.cameraViewTransform)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.cameraViewTransform)
                return data_view.get()

        @cameraViewTransform.setter
        def cameraViewTransform(self, value):
            self._batchedWriteValues[self._attributes.cameraViewTransform] = value

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
        def metersPerSceneUnit(self):
            value = self._batchedWriteValues.get(self._attributes.metersPerSceneUnit)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.metersPerSceneUnit)
                return data_view.get()

        @metersPerSceneUnit.setter
        def metersPerSceneUnit(self, value):
            self._batchedWriteValues[self._attributes.metersPerSceneUnit] = value

        @property
        def renderProductResolution(self):
            value = self._batchedWriteValues.get(self._attributes.renderProductResolution)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.renderProductResolution)
                return data_view.get()

        @renderProductResolution.setter
        def renderProductResolution(self, value):
            self._batchedWriteValues[self._attributes.renderProductResolution] = value

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
        self.inputs = OgnSdRenderProductCameraDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnSdRenderProductCameraDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnSdRenderProductCameraDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
