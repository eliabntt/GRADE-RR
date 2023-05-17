"""Support for simplified access to data on nodes of type omni.syntheticdata.SdTestInstanceMapping

Synthetic Data node to test the instance mapping pipeline
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import numpy
class OgnSdTestInstanceMappingDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.syntheticdata.SdTestInstanceMapping

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.exec
            inputs.instanceMapPtr
            inputs.instancePrimPathPtr
            inputs.minInstanceIndex
            inputs.minSemanticIndex
            inputs.numInstances
            inputs.numSemantics
            inputs.semanticLabelTokenPtrs
            inputs.semanticLocalTransformPtr
            inputs.semanticMapPtr
            inputs.semanticPrimPathPtr
            inputs.semanticWorldTransformPtr
            inputs.stage
            inputs.swhFrameNumber
            inputs.testCaseIndex
        Outputs:
            outputs.exec
            outputs.semanticFilterPredicate
            outputs.success

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
        ('inputs:exec', 'execution', 0, None, 'Trigger', {}, True, None, False, ''),
        ('inputs:instanceMapPtr', 'uint64', 0, None, 'Array pointer of numInstances uint16_t containing the semantic index of the instance prim first semantic prim parent', {}, True, 0, False, ''),
        ('inputs:instancePrimPathPtr', 'uint64', 0, None, 'Array pointer of numInstances uint64_t containing the prim path tokens for every instance prims', {}, True, 0, False, ''),
        ('inputs:minInstanceIndex', 'uint', 0, None, 'Instance index of the first instance prim in the instance arrays', {}, True, 0, False, ''),
        ('inputs:minSemanticIndex', 'uint', 0, None, 'Semantic index of the first semantic prim in the semantic arrays', {}, True, 0, False, ''),
        ('inputs:numInstances', 'uint', 0, None, 'Number of instances prim in the instance arrays', {}, True, 0, False, ''),
        ('inputs:numSemantics', 'uint', 0, None, 'Number of semantic prim in the semantic arrays', {}, True, 0, False, ''),
        ('inputs:semanticLabelTokenPtrs', 'uint64[]', 0, None, 'Array containing for every input semantic filters the corresponding array pointer of numSemantics uint64_t representing the semantic label of the semantic prim', {}, True, [], False, ''),
        ('inputs:semanticLocalTransformPtr', 'uint64', 0, None, 'Array pointer of numSemantics 4x4 float matrices containing the transform from world to object space for every semantic prims', {}, True, 0, False, ''),
        ('inputs:semanticMapPtr', 'uint64', 0, None, 'Array pointer of numSemantics uint16_t containing the semantic index of the semantic prim first semantic prim parent', {}, True, 0, False, ''),
        ('inputs:semanticPrimPathPtr', 'uint64', 0, None, 'Array pointer of numSemantics uint32_t containing the prim part of the prim path tokens for every semantic prims', {}, True, 0, False, ''),
        ('inputs:semanticWorldTransformPtr', 'uint64', 0, None, 'Array pointer of numSemantics 4x4 float matrices containing the transform from local to world space for every semantic entity', {}, True, 0, False, ''),
        ('inputs:stage', 'token', 0, None, 'Stage in {simulation, postrender, ondemand}', {}, True, '', False, ''),
        ('inputs:swhFrameNumber', 'uint64', 0, None, 'Fabric frame number', {}, True, 0, False, ''),
        ('inputs:testCaseIndex', 'int', 0, None, 'Test case index', {ogn.MetadataKeys.DEFAULT: '-1'}, True, -1, False, ''),
        ('outputs:exec', 'execution', 0, 'Received', 'Executes when the event is received', {}, True, None, False, ''),
        ('outputs:semanticFilterPredicate', 'token', 0, None, 'The semantic filter predicate : a disjunctive normal form of semantic type and label', {}, True, None, False, ''),
        ('outputs:success', 'bool', 0, None, 'Test value : false if failed', {}, True, None, False, ''),
    ])
    class tokens:
        simulation = "simulation"
        postRender = "postRender"
        onDemand = "onDemand"
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.exec = og.Database.ROLE_EXECUTION
        role_data.outputs.exec = og.Database.ROLE_EXECUTION
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"exec", "instanceMapPtr", "instancePrimPathPtr", "minInstanceIndex", "minSemanticIndex", "numInstances", "numSemantics", "semanticLocalTransformPtr", "semanticMapPtr", "semanticPrimPathPtr", "semanticWorldTransformPtr", "stage", "swhFrameNumber", "testCaseIndex", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.exec, self._attributes.instanceMapPtr, self._attributes.instancePrimPathPtr, self._attributes.minInstanceIndex, self._attributes.minSemanticIndex, self._attributes.numInstances, self._attributes.numSemantics, self._attributes.semanticLocalTransformPtr, self._attributes.semanticMapPtr, self._attributes.semanticPrimPathPtr, self._attributes.semanticWorldTransformPtr, self._attributes.stage, self._attributes.swhFrameNumber, self._attributes.testCaseIndex]
            self._batchedReadValues = [None, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, "", 0, -1]

        @property
        def semanticLabelTokenPtrs(self):
            data_view = og.AttributeValueHelper(self._attributes.semanticLabelTokenPtrs)
            return data_view.get()

        @semanticLabelTokenPtrs.setter
        def semanticLabelTokenPtrs(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.semanticLabelTokenPtrs)
            data_view = og.AttributeValueHelper(self._attributes.semanticLabelTokenPtrs)
            data_view.set(value)
            self.semanticLabelTokenPtrs_size = data_view.get_array_size()

        @property
        def exec(self):
            return self._batchedReadValues[0]

        @exec.setter
        def exec(self, value):
            self._batchedReadValues[0] = value

        @property
        def instanceMapPtr(self):
            return self._batchedReadValues[1]

        @instanceMapPtr.setter
        def instanceMapPtr(self, value):
            self._batchedReadValues[1] = value

        @property
        def instancePrimPathPtr(self):
            return self._batchedReadValues[2]

        @instancePrimPathPtr.setter
        def instancePrimPathPtr(self, value):
            self._batchedReadValues[2] = value

        @property
        def minInstanceIndex(self):
            return self._batchedReadValues[3]

        @minInstanceIndex.setter
        def minInstanceIndex(self, value):
            self._batchedReadValues[3] = value

        @property
        def minSemanticIndex(self):
            return self._batchedReadValues[4]

        @minSemanticIndex.setter
        def minSemanticIndex(self, value):
            self._batchedReadValues[4] = value

        @property
        def numInstances(self):
            return self._batchedReadValues[5]

        @numInstances.setter
        def numInstances(self, value):
            self._batchedReadValues[5] = value

        @property
        def numSemantics(self):
            return self._batchedReadValues[6]

        @numSemantics.setter
        def numSemantics(self, value):
            self._batchedReadValues[6] = value

        @property
        def semanticLocalTransformPtr(self):
            return self._batchedReadValues[7]

        @semanticLocalTransformPtr.setter
        def semanticLocalTransformPtr(self, value):
            self._batchedReadValues[7] = value

        @property
        def semanticMapPtr(self):
            return self._batchedReadValues[8]

        @semanticMapPtr.setter
        def semanticMapPtr(self, value):
            self._batchedReadValues[8] = value

        @property
        def semanticPrimPathPtr(self):
            return self._batchedReadValues[9]

        @semanticPrimPathPtr.setter
        def semanticPrimPathPtr(self, value):
            self._batchedReadValues[9] = value

        @property
        def semanticWorldTransformPtr(self):
            return self._batchedReadValues[10]

        @semanticWorldTransformPtr.setter
        def semanticWorldTransformPtr(self, value):
            self._batchedReadValues[10] = value

        @property
        def stage(self):
            return self._batchedReadValues[11]

        @stage.setter
        def stage(self, value):
            self._batchedReadValues[11] = value

        @property
        def swhFrameNumber(self):
            return self._batchedReadValues[12]

        @swhFrameNumber.setter
        def swhFrameNumber(self, value):
            self._batchedReadValues[12] = value

        @property
        def testCaseIndex(self):
            return self._batchedReadValues[13]

        @testCaseIndex.setter
        def testCaseIndex(self, value):
            self._batchedReadValues[13] = value

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
        LOCAL_PROPERTY_NAMES = {"exec", "semanticFilterPredicate", "success", "_batchedWriteValues"}
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

        @property
        def success(self):
            value = self._batchedWriteValues.get(self._attributes.success)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.success)
                return data_view.get()

        @success.setter
        def success(self, value):
            self._batchedWriteValues[self._attributes.success] = value

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
        self.inputs = OgnSdTestInstanceMappingDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnSdTestInstanceMappingDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnSdTestInstanceMappingDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
