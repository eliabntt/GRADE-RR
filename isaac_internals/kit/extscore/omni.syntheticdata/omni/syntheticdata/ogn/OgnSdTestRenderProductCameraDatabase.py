"""Support for simplified access to data on nodes of type omni.syntheticdata.SdTestRenderProductCamera

Synthetic Data node to test the renderProduct camera pipeline
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import numpy
class OgnSdTestRenderProductCameraDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.syntheticdata.SdTestRenderProductCamera

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.cameraApertureOffset
            inputs.cameraApertureSize
            inputs.cameraFStop
            inputs.cameraFisheyeParams
            inputs.cameraFocalLength
            inputs.cameraFocusDistance
            inputs.cameraModel
            inputs.cameraNearFar
            inputs.cameraProjection
            inputs.cameraViewTransform
            inputs.exec
            inputs.height
            inputs.metersPerSceneUnit
            inputs.renderProductCameraPath
            inputs.renderProductResolution
            inputs.stage
            inputs.swhFrameNumber
            inputs.traceError
            inputs.width
        Outputs:
            outputs.test

    Predefined Tokens:
        tokens.simulation
        tokens.postRender
        tokens.onDemand
    """
    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}
    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:cameraApertureOffset', 'float2', 0, None, 'Camera horizontal and vertical aperture offset', {}, True, [0.0, 0.0], False, ''),
        ('inputs:cameraApertureSize', 'float2', 0, None, 'Camera horizontal and vertical aperture', {}, True, [0.0, 0.0], False, ''),
        ('inputs:cameraFStop', 'float', 0, None, 'Camera fStop', {}, True, 0.0, False, ''),
        ('inputs:cameraFisheyeParams', 'float[]', 0, None, 'Camera fisheye projection parameters', {}, True, [], False, ''),
        ('inputs:cameraFocalLength', 'float', 0, None, 'Camera focal length', {}, True, 0.0, False, ''),
        ('inputs:cameraFocusDistance', 'float', 0, None, 'Camera focus distance', {}, True, 0.0, False, ''),
        ('inputs:cameraModel', 'int', 0, None, 'Camera model (pinhole or fisheye models)', {}, True, 0, False, ''),
        ('inputs:cameraNearFar', 'float2', 0, None, 'Camera near/far clipping range', {}, True, [0.0, 0.0], False, ''),
        ('inputs:cameraProjection', 'matrix4d', 0, None, 'Camera projection matrix', {}, True, [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]], False, ''),
        ('inputs:cameraViewTransform', 'matrix4d', 0, None, 'Camera view matrix', {}, True, [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]], False, ''),
        ('inputs:exec', 'execution', 0, None, 'Trigger', {}, True, None, False, ''),
        ('inputs:height', 'uint', 0, None, 'Height of the frame', {}, True, 0, False, ''),
        ('inputs:metersPerSceneUnit', 'float', 0, None, 'Scene units to meters scale', {}, True, 0.0, False, ''),
        ('inputs:renderProductCameraPath', 'token', 0, None, 'RenderProduct camera prim path', {}, True, '', False, ''),
        ('inputs:renderProductResolution', 'int2', 0, None, 'RenderProduct resolution', {}, True, [0, 0], False, ''),
        ('inputs:stage', 'token', 0, None, 'Stage in {simulation, postrender, ondemand}', {}, True, '', False, ''),
        ('inputs:swhFrameNumber', 'uint64', 0, None, 'Fabric frame number', {}, True, 0, False, ''),
        ('inputs:traceError', 'bool', 0, None, 'If true print an error message when the frame numbers are out-of-sync', {ogn.MetadataKeys.DEFAULT: 'false'}, True, False, False, ''),
        ('inputs:width', 'uint', 0, None, 'Width of the frame', {}, True, 0, False, ''),
        ('outputs:test', 'bool', 0, None, 'Test value : false if failed', {}, True, None, False, ''),
    ])
    class tokens:
        simulation = "simulation"
        postRender = "postRender"
        onDemand = "onDemand"
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.cameraProjection = og.Database.ROLE_MATRIX
        role_data.inputs.cameraViewTransform = og.Database.ROLE_MATRIX
        role_data.inputs.exec = og.Database.ROLE_EXECUTION
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"cameraApertureOffset", "cameraApertureSize", "cameraFStop", "cameraFocalLength", "cameraFocusDistance", "cameraModel", "cameraNearFar", "cameraProjection", "cameraViewTransform", "exec", "height", "metersPerSceneUnit", "renderProductCameraPath", "renderProductResolution", "stage", "swhFrameNumber", "traceError", "width", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.cameraApertureOffset, self._attributes.cameraApertureSize, self._attributes.cameraFStop, self._attributes.cameraFocalLength, self._attributes.cameraFocusDistance, self._attributes.cameraModel, self._attributes.cameraNearFar, self._attributes.cameraProjection, self._attributes.cameraViewTransform, self._attributes.exec, self._attributes.height, self._attributes.metersPerSceneUnit, self._attributes.renderProductCameraPath, self._attributes.renderProductResolution, self._attributes.stage, self._attributes.swhFrameNumber, self._attributes.traceError, self._attributes.width]
            self._batchedReadValues = [[0.0, 0.0], [0.0, 0.0], 0.0, 0.0, 0.0, 0, [0.0, 0.0], [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0], [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0], None, 0, 0.0, "", [0, 0], "", 0, False, 0]

        @property
        def cameraFisheyeParams(self):
            data_view = og.AttributeValueHelper(self._attributes.cameraFisheyeParams)
            return data_view.get()

        @cameraFisheyeParams.setter
        def cameraFisheyeParams(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.cameraFisheyeParams)
            data_view = og.AttributeValueHelper(self._attributes.cameraFisheyeParams)
            data_view.set(value)
            self.cameraFisheyeParams_size = data_view.get_array_size()

        @property
        def cameraApertureOffset(self):
            return self._batchedReadValues[0]

        @cameraApertureOffset.setter
        def cameraApertureOffset(self, value):
            self._batchedReadValues[0] = value

        @property
        def cameraApertureSize(self):
            return self._batchedReadValues[1]

        @cameraApertureSize.setter
        def cameraApertureSize(self, value):
            self._batchedReadValues[1] = value

        @property
        def cameraFStop(self):
            return self._batchedReadValues[2]

        @cameraFStop.setter
        def cameraFStop(self, value):
            self._batchedReadValues[2] = value

        @property
        def cameraFocalLength(self):
            return self._batchedReadValues[3]

        @cameraFocalLength.setter
        def cameraFocalLength(self, value):
            self._batchedReadValues[3] = value

        @property
        def cameraFocusDistance(self):
            return self._batchedReadValues[4]

        @cameraFocusDistance.setter
        def cameraFocusDistance(self, value):
            self._batchedReadValues[4] = value

        @property
        def cameraModel(self):
            return self._batchedReadValues[5]

        @cameraModel.setter
        def cameraModel(self, value):
            self._batchedReadValues[5] = value

        @property
        def cameraNearFar(self):
            return self._batchedReadValues[6]

        @cameraNearFar.setter
        def cameraNearFar(self, value):
            self._batchedReadValues[6] = value

        @property
        def cameraProjection(self):
            return self._batchedReadValues[7]

        @cameraProjection.setter
        def cameraProjection(self, value):
            self._batchedReadValues[7] = value

        @property
        def cameraViewTransform(self):
            return self._batchedReadValues[8]

        @cameraViewTransform.setter
        def cameraViewTransform(self, value):
            self._batchedReadValues[8] = value

        @property
        def exec(self):
            return self._batchedReadValues[9]

        @exec.setter
        def exec(self, value):
            self._batchedReadValues[9] = value

        @property
        def height(self):
            return self._batchedReadValues[10]

        @height.setter
        def height(self, value):
            self._batchedReadValues[10] = value

        @property
        def metersPerSceneUnit(self):
            return self._batchedReadValues[11]

        @metersPerSceneUnit.setter
        def metersPerSceneUnit(self, value):
            self._batchedReadValues[11] = value

        @property
        def renderProductCameraPath(self):
            return self._batchedReadValues[12]

        @renderProductCameraPath.setter
        def renderProductCameraPath(self, value):
            self._batchedReadValues[12] = value

        @property
        def renderProductResolution(self):
            return self._batchedReadValues[13]

        @renderProductResolution.setter
        def renderProductResolution(self, value):
            self._batchedReadValues[13] = value

        @property
        def stage(self):
            return self._batchedReadValues[14]

        @stage.setter
        def stage(self, value):
            self._batchedReadValues[14] = value

        @property
        def swhFrameNumber(self):
            return self._batchedReadValues[15]

        @swhFrameNumber.setter
        def swhFrameNumber(self, value):
            self._batchedReadValues[15] = value

        @property
        def traceError(self):
            return self._batchedReadValues[16]

        @traceError.setter
        def traceError(self, value):
            self._batchedReadValues[16] = value

        @property
        def width(self):
            return self._batchedReadValues[17]

        @width.setter
        def width(self, value):
            self._batchedReadValues[17] = value

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
        LOCAL_PROPERTY_NAMES = {"test", "_batchedWriteValues"}
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedWriteValues = { }

        @property
        def test(self):
            value = self._batchedWriteValues.get(self._attributes.test)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.test)
                return data_view.get()

        @test.setter
        def test(self, value):
            self._batchedWriteValues[self._attributes.test] = value

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
        self.inputs = OgnSdTestRenderProductCameraDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnSdTestRenderProductCameraDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnSdTestRenderProductCameraDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
