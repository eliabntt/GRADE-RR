"""Support for simplified access to data on nodes of type omni.syntheticdata.SdTestPrintRawArray

Synthetic Data test node printing the input linear array
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import numpy
import sys
import traceback
class OgnSdTestPrintRawArrayDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.syntheticdata.SdTestPrintRawArray

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.bufferSize
            inputs.data
            inputs.elementCount
            inputs.elementType
            inputs.exec
            inputs.height
            inputs.mode
            inputs.randomSeed
            inputs.referenceNumUniqueRandomValues
            inputs.referenceSWHFrameNumbers
            inputs.referenceTolerance
            inputs.referenceValues
            inputs.swhFrameNumber
            inputs.width
        Outputs:
            outputs.exec
            outputs.swhFrameNumber
        State:
            state.initialSWHFrameNumber

    Predefined Tokens:
        tokens.uint16
        tokens.int16
        tokens.uint32
        tokens.int32
        tokens.float32
        tokens.token
        tokens.printFormatted
        tokens.printReferences
    """
    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}
    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:bufferSize', 'uint', 0, None, 'Size (in bytes) of the buffer (0 if the input is a texture)', {}, True, 0, False, ''),
        ('inputs:data', 'uchar[]', 0, None, 'Buffer array data', {ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('inputs:elementCount', 'int', 0, None, 'Number of array element', {ogn.MetadataKeys.DEFAULT: '1'}, True, 1, False, ''),
        ('inputs:elementType', 'token', 0, None, 'Type of the array element', {ogn.MetadataKeys.DEFAULT: '"uint8"'}, True, 'uint8', False, ''),
        ('inputs:exec', 'execution', 0, None, 'Trigger', {}, True, None, False, ''),
        ('inputs:height', 'uint', 0, None, 'Height  (0 if the input is a buffer)', {}, True, 0, False, ''),
        ('inputs:mode', 'token', 0, None, 'Mode in [printFormatted, printReferences, testReferences]', {ogn.MetadataKeys.DEFAULT: '"printFormatted"'}, True, 'printFormatted', False, ''),
        ('inputs:randomSeed', 'int', 0, None, 'Random seed', {}, True, 0, False, ''),
        ('inputs:referenceNumUniqueRandomValues', 'int', 0, None, 'Number of reference unique random values to compare', {ogn.MetadataKeys.DEFAULT: '7'}, True, 7, False, ''),
        ('inputs:referenceSWHFrameNumbers', 'uint[]', 0, None, 'Reference swhFrameNumbers relative to the first one', {ogn.MetadataKeys.DEFAULT: '[11, 17, 29]'}, True, [11, 17, 29], False, ''),
        ('inputs:referenceTolerance', 'float', 0, None, 'Reference tolerance', {ogn.MetadataKeys.DEFAULT: '0.1'}, True, 0.1, False, ''),
        ('inputs:referenceValues', 'float[]', 0, None, 'Reference data point values', {}, True, [], False, ''),
        ('inputs:swhFrameNumber', 'uint64', 0, None, 'Frame number', {}, True, 0, False, ''),
        ('inputs:width', 'uint', 0, None, 'Width  (0 if the input is a buffer)', {}, True, 0, False, ''),
        ('outputs:exec', 'execution', 0, 'Received', 'Executes when the event is received', {}, True, None, False, ''),
        ('outputs:swhFrameNumber', 'uint64', 0, None, 'FrameNumber just rendered', {}, True, None, False, ''),
        ('state:initialSWHFrameNumber', 'int64', 0, None, 'Initial swhFrameNumber', {ogn.MetadataKeys.DEFAULT: '-1'}, True, -1, False, ''),
    ])
    class tokens:
        uint16 = "uint16"
        int16 = "int16"
        uint32 = "uint32"
        int32 = "int32"
        float32 = "float32"
        token = "token"
        printFormatted = "printFormatted"
        printReferences = "printReferences"
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.exec = og.Database.ROLE_EXECUTION
        role_data.outputs.exec = og.Database.ROLE_EXECUTION
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"bufferSize", "elementCount", "elementType", "exec", "height", "mode", "randomSeed", "referenceNumUniqueRandomValues", "referenceTolerance", "swhFrameNumber", "width", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.bufferSize, self._attributes.elementCount, self._attributes.elementType, self._attributes.exec, self._attributes.height, self._attributes.mode, self._attributes.randomSeed, self._attributes.referenceNumUniqueRandomValues, self._attributes.referenceTolerance, self._attributes.swhFrameNumber, self._attributes.width]
            self._batchedReadValues = [0, 1, "uint8", None, 0, "printFormatted", 0, 7, 0.1, 0, 0]

        @property
        def data(self):
            data_view = og.AttributeValueHelper(self._attributes.data)
            return data_view.get()

        @data.setter
        def data(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.data)
            data_view = og.AttributeValueHelper(self._attributes.data)
            data_view.set(value)
            self.data_size = data_view.get_array_size()

        @property
        def referenceSWHFrameNumbers(self):
            data_view = og.AttributeValueHelper(self._attributes.referenceSWHFrameNumbers)
            return data_view.get()

        @referenceSWHFrameNumbers.setter
        def referenceSWHFrameNumbers(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.referenceSWHFrameNumbers)
            data_view = og.AttributeValueHelper(self._attributes.referenceSWHFrameNumbers)
            data_view.set(value)
            self.referenceSWHFrameNumbers_size = data_view.get_array_size()

        @property
        def referenceValues(self):
            data_view = og.AttributeValueHelper(self._attributes.referenceValues)
            return data_view.get()

        @referenceValues.setter
        def referenceValues(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.referenceValues)
            data_view = og.AttributeValueHelper(self._attributes.referenceValues)
            data_view.set(value)
            self.referenceValues_size = data_view.get_array_size()

        @property
        def bufferSize(self):
            return self._batchedReadValues[0]

        @bufferSize.setter
        def bufferSize(self, value):
            self._batchedReadValues[0] = value

        @property
        def elementCount(self):
            return self._batchedReadValues[1]

        @elementCount.setter
        def elementCount(self, value):
            self._batchedReadValues[1] = value

        @property
        def elementType(self):
            return self._batchedReadValues[2]

        @elementType.setter
        def elementType(self, value):
            self._batchedReadValues[2] = value

        @property
        def exec(self):
            return self._batchedReadValues[3]

        @exec.setter
        def exec(self, value):
            self._batchedReadValues[3] = value

        @property
        def height(self):
            return self._batchedReadValues[4]

        @height.setter
        def height(self, value):
            self._batchedReadValues[4] = value

        @property
        def mode(self):
            return self._batchedReadValues[5]

        @mode.setter
        def mode(self, value):
            self._batchedReadValues[5] = value

        @property
        def randomSeed(self):
            return self._batchedReadValues[6]

        @randomSeed.setter
        def randomSeed(self, value):
            self._batchedReadValues[6] = value

        @property
        def referenceNumUniqueRandomValues(self):
            return self._batchedReadValues[7]

        @referenceNumUniqueRandomValues.setter
        def referenceNumUniqueRandomValues(self, value):
            self._batchedReadValues[7] = value

        @property
        def referenceTolerance(self):
            return self._batchedReadValues[8]

        @referenceTolerance.setter
        def referenceTolerance(self, value):
            self._batchedReadValues[8] = value

        @property
        def swhFrameNumber(self):
            return self._batchedReadValues[9]

        @swhFrameNumber.setter
        def swhFrameNumber(self, value):
            self._batchedReadValues[9] = value

        @property
        def width(self):
            return self._batchedReadValues[10]

        @width.setter
        def width(self, value):
            self._batchedReadValues[10] = value

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
        LOCAL_PROPERTY_NAMES = {"exec", "swhFrameNumber", "_batchedWriteValues"}
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

        @property
        def initialSWHFrameNumber(self):
            data_view = og.AttributeValueHelper(self._attributes.initialSWHFrameNumber)
            return data_view.get()

        @initialSWHFrameNumber.setter
        def initialSWHFrameNumber(self, value):
            data_view = og.AttributeValueHelper(self._attributes.initialSWHFrameNumber)
            data_view.set(value)
    def __init__(self, node):
        super().__init__(node)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_INPUT)
        self.inputs = OgnSdTestPrintRawArrayDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnSdTestPrintRawArrayDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnSdTestPrintRawArrayDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
    class abi:
        """Class defining the ABI interface for the node type"""
        @staticmethod
        def get_node_type():
            get_node_type_function = getattr(OgnSdTestPrintRawArrayDatabase.NODE_TYPE_CLASS, 'get_node_type', None)
            if callable(get_node_type_function):
                return get_node_type_function()
            return 'omni.syntheticdata.SdTestPrintRawArray'
        @staticmethod
        def compute(context, node):
            try:
                per_node_data = OgnSdTestPrintRawArrayDatabase.PER_NODE_DATA[node.node_id()]
                db = per_node_data.get('_db')
                if db is None:
                    db = OgnSdTestPrintRawArrayDatabase(node)
                    per_node_data['_db'] = db
            except:
                db = OgnSdTestPrintRawArrayDatabase(node)

            try:
                compute_function = getattr(OgnSdTestPrintRawArrayDatabase.NODE_TYPE_CLASS, 'compute', None)
                if callable(compute_function) and compute_function.__code__.co_argcount > 1:
                    return compute_function(context, node)

                db.inputs._prefetch()
                db.inputs._setting_locked = True
                with og.in_compute():
                    return OgnSdTestPrintRawArrayDatabase.NODE_TYPE_CLASS.compute(db)
            except Exception as error:
                stack_trace = "".join(traceback.format_tb(sys.exc_info()[2].tb_next))
                db.log_error(f'Assertion raised in compute - {error}\n{stack_trace}', add_context=False)
            finally:
                db.inputs._setting_locked = False
                db.outputs._commit()
            return False
        @staticmethod
        def initialize(context, node):
            OgnSdTestPrintRawArrayDatabase._initialize_per_node_data(node)
            initialize_function = getattr(OgnSdTestPrintRawArrayDatabase.NODE_TYPE_CLASS, 'initialize', None)
            if callable(initialize_function):
                initialize_function(context, node)
        @staticmethod
        def release(node):
            release_function = getattr(OgnSdTestPrintRawArrayDatabase.NODE_TYPE_CLASS, 'release', None)
            if callable(release_function):
                release_function(node)
            OgnSdTestPrintRawArrayDatabase._release_per_node_data(node)
        @staticmethod
        def update_node_version(context, node, old_version, new_version):
            update_node_version_function = getattr(OgnSdTestPrintRawArrayDatabase.NODE_TYPE_CLASS, 'update_node_version', None)
            if callable(update_node_version_function):
                return update_node_version_function(context, node, old_version, new_version)
            return False
        @staticmethod
        def initialize_type(node_type):
            initialize_type_function = getattr(OgnSdTestPrintRawArrayDatabase.NODE_TYPE_CLASS, 'initialize_type', None)
            needs_initializing = True
            if callable(initialize_type_function):
                needs_initializing = initialize_type_function(node_type)
            if needs_initializing:
                node_type.set_metadata(ogn.MetadataKeys.EXTENSION, "omni.syntheticdata")
                node_type.set_metadata(ogn.MetadataKeys.TOKENS, "[\"uint16\", \"int16\", \"uint32\", \"int32\", \"float32\", \"token\", \"printFormatted\", \"printReferences\"]")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORIES, "graph:action,internal:test")
                node_type.set_metadata(ogn.MetadataKeys.DESCRIPTION, "Synthetic Data test node printing the input linear array")
                node_type.set_metadata(ogn.MetadataKeys.EXCLUSIONS, "tests")
                node_type.set_metadata(ogn.MetadataKeys.LANGUAGE, "Python")
                OgnSdTestPrintRawArrayDatabase.INTERFACE.add_to_node_type(node_type)
                node_type.set_has_state(True)
        @staticmethod
        def on_connection_type_resolve(node):
            on_connection_type_resolve_function = getattr(OgnSdTestPrintRawArrayDatabase.NODE_TYPE_CLASS, 'on_connection_type_resolve', None)
            if callable(on_connection_type_resolve_function):
                on_connection_type_resolve_function(node)
    NODE_TYPE_CLASS = None
    GENERATOR_VERSION = (1, 17, 2)
    TARGET_VERSION = (2, 65, 4)
    @staticmethod
    def register(node_type_class):
        OgnSdTestPrintRawArrayDatabase.NODE_TYPE_CLASS = node_type_class
        og.register_node_type(OgnSdTestPrintRawArrayDatabase.abi, 1)
    @staticmethod
    def deregister():
        og.deregister_node_type("omni.syntheticdata.SdTestPrintRawArray")
