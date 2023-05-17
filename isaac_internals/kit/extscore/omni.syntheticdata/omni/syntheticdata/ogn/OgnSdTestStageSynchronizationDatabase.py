"""Support for simplified access to data on nodes of type omni.syntheticdata.SdTestStageSynchronization

Synthetic Data node to test the pipeline stage synchronization
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
class OgnSdTestStageSynchronizationDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.syntheticdata.SdTestStageSynchronization

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.exec
            inputs.gpu
            inputs.randomMaxProcessingTimeUs
            inputs.randomSeed
            inputs.renderResults
            inputs.rp
            inputs.swhFrameNumber
            inputs.tag
            inputs.traceError
        Outputs:
            outputs.exec
            outputs.fabricSWHFrameNumber
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
        ('inputs:exec', 'execution', 0, None, 'OnDemand connection : trigger', {}, True, None, False, ''),
        ('inputs:gpu', 'uint64', 0, 'gpuFoundations', 'PostRender connection : pointer to shared context containing gpu foundations', {}, True, 0, False, ''),
        ('inputs:randomMaxProcessingTimeUs', 'uint', 0, None, 'Maximum number of micro-seconds to randomly (uniformely) wait for in order to simulate varying workload', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('inputs:randomSeed', 'uint', 0, None, 'Random seed for the randomization', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('inputs:renderResults', 'uint64', 0, None, 'OnDemand connection : pointer to render product results', {}, True, 0, False, ''),
        ('inputs:rp', 'uint64', 0, 'renderProduct', 'PostRender connection : pointer to render product for this view', {}, True, 0, False, ''),
        ('inputs:swhFrameNumber', 'uint64', 0, None, 'Fabric frame number', {}, True, 0, False, ''),
        ('inputs:tag', 'token', 0, None, 'A tag to identify the node', {}, True, '', False, ''),
        ('inputs:traceError', 'bool', 0, None, 'If true print an error message when the frame numbers are out-of-sync', {ogn.MetadataKeys.DEFAULT: 'false'}, True, False, False, ''),
        ('outputs:exec', 'execution', 0, None, 'OnDemand connection : trigger', {}, True, None, False, ''),
        ('outputs:fabricSWHFrameNumber', 'uint64', 0, None, 'Fabric frame number from the fabric', {}, True, None, False, ''),
        ('outputs:swhFrameNumber', 'uint64', 0, None, 'Fabric frame number', {}, True, None, False, ''),
    ])
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.exec = og.Database.ROLE_EXECUTION
        role_data.outputs.exec = og.Database.ROLE_EXECUTION
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"exec", "gpu", "randomMaxProcessingTimeUs", "randomSeed", "renderResults", "rp", "swhFrameNumber", "tag", "traceError", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.exec, self._attributes.gpu, self._attributes.randomMaxProcessingTimeUs, self._attributes.randomSeed, self._attributes.renderResults, self._attributes.rp, self._attributes.swhFrameNumber, self._attributes.tag, self._attributes.traceError]
            self._batchedReadValues = [None, 0, 0, 0, 0, 0, 0, "", False]

        @property
        def exec(self):
            return self._batchedReadValues[0]

        @exec.setter
        def exec(self, value):
            self._batchedReadValues[0] = value

        @property
        def gpu(self):
            return self._batchedReadValues[1]

        @gpu.setter
        def gpu(self, value):
            self._batchedReadValues[1] = value

        @property
        def randomMaxProcessingTimeUs(self):
            return self._batchedReadValues[2]

        @randomMaxProcessingTimeUs.setter
        def randomMaxProcessingTimeUs(self, value):
            self._batchedReadValues[2] = value

        @property
        def randomSeed(self):
            return self._batchedReadValues[3]

        @randomSeed.setter
        def randomSeed(self, value):
            self._batchedReadValues[3] = value

        @property
        def renderResults(self):
            return self._batchedReadValues[4]

        @renderResults.setter
        def renderResults(self, value):
            self._batchedReadValues[4] = value

        @property
        def rp(self):
            return self._batchedReadValues[5]

        @rp.setter
        def rp(self, value):
            self._batchedReadValues[5] = value

        @property
        def swhFrameNumber(self):
            return self._batchedReadValues[6]

        @swhFrameNumber.setter
        def swhFrameNumber(self, value):
            self._batchedReadValues[6] = value

        @property
        def tag(self):
            return self._batchedReadValues[7]

        @tag.setter
        def tag(self, value):
            self._batchedReadValues[7] = value

        @property
        def traceError(self):
            return self._batchedReadValues[8]

        @traceError.setter
        def traceError(self, value):
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
        LOCAL_PROPERTY_NAMES = {"exec", "fabricSWHFrameNumber", "swhFrameNumber", "_batchedWriteValues"}
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
        def fabricSWHFrameNumber(self):
            value = self._batchedWriteValues.get(self._attributes.fabricSWHFrameNumber)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.fabricSWHFrameNumber)
                return data_view.get()

        @fabricSWHFrameNumber.setter
        def fabricSWHFrameNumber(self, value):
            self._batchedWriteValues[self._attributes.fabricSWHFrameNumber] = value

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
        self.inputs = OgnSdTestStageSynchronizationDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnSdTestStageSynchronizationDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnSdTestStageSynchronizationDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
