omni.syntheticdata
//////////////////////////#

Introduction
************

This extension provides both C++ and python bindings that allow users to extract ground truth data from scenes loaded
and rendered in Omniverse Kit and use it for DL/RL training purposes. Data can be accessed either in host memory or
directly on device memory to provide high performance training. The scene data is provided by generating USD data
that can be rendered through the Kit renderer.

Core Concepts
*************

Sensor
======

Ground truth data is accessed through various sensors that are associated with a view in the renderer. The sensors
generally provide access to synthetic data and are either represented as images or buffers of attribute data. Attribute
data elements are usually associated with a particular instance in a scene, which is usually represented by a mesh
specified in the USD data. Sensors are objects that are managed by the user either through the API or the UI.

Synthetic Image Data
====================

Synthetic image data is represented by sensors as a 2D image. Examples of synthetic image data include RGB data,
depth data, and segmentation data. The data can be in any valid image format supported by the renderer.

Synthetic Attribute Data
========================

Synthetic attribute data is represented by sensors as raw structured data that can be accessed as an array.
The data structures used to store array elements depend on the type of sensor. Examples of synthetic attribute data
include bounding boxes. See the data structures defined below to see how various attribute data arrays define their
data.

Instance
========

An instance is a single segmentation unit in a scene that is usually represented as a mesh. An instance is usually
represented in sensor data as a unique unsigned integer ID. The renderer currently limits scenes to having 2^24
unique instances.

Semantic Class
==============

A semantic class is a classification given to a scene instance that can be used for training purposes. It is provided
as a unique string and is usually represented in sensor data as a unique unsigned integer ID. Semantic class strings
can be anything that will be used to identify scene instances, such as "car", "tree", "large", "broken", etc. The
renderer currently limits scenes to having 2^16 unique semantic classes. Semantic class data is specified inside the
USD scene data through the Semantic API schema.

Segmentation
============

Segmentation data is usually represented by sensors as synthetic image data and is used to segment image data
within a view. Examples include instance segmentation which will represent each pixel in the image data with an
instance ID and semantic segmentation which will represent each pixel in the image data with a semantic ID.

Accessing Data on Device Memory
===============================

Device Memory is usually GPU memory. Synthetic data can be accessed directly on device memory with python by using
PyTorch tensors.

Accessing Data on Host Memory
=============================

Device Memory is usually system memory. Synthetic data can be accessed directly on host memory with python through
numpy arrays.

Data Structures
***************

Below are the various data structures specified by the C++ API and accessed through python using pybind.

SensorType
==========

.. code::

    enum class SensorType : uint32_t
    {
        // These sensors represent image data
        eRgb = 0, ///< RGB data
        eDepth, ///< depth data
        eDepthLinear, ///< linear depth data (in meters)
        eInstanceSegmentation, ///< instance segmentation data
        eSemanticSegmentation, ///< semantic segmentation data
        eNormal, ///< normal vector data
        eMotionVector, ///< motion vector data
        // These sensors represent instance attribute data
        eBoundingBox2DTight, ///< tight 2D bounding box data, only contains non-occluded pixels
        eBoundingBox2DLoose, ///< loose 2D bounding box data, also contains occluded pixels
        eBoundingBox3D, ///< 3D view space bounding box data
        eOcclusion, ///< occlusion data
        eTruncation, ///< truncation data
    };

SensorResourceType
==================

.. code::

    enum class SensorResourceType
    {
        eTexture, ///< image data sensors
        eBuffer ///< attribute data sensors
    };

SensorInfo
==========

.. code::

    struct SensorInfo
    {
        SensorType type; ///< sensor type
        SensorResourceType resType; ///< sensor resource type
        union
        {
            struct
            {
                uint32_t width; ///< sensor width of texture sensors
                uint32_t height; ///< sensor height of texture sensors
                uint32_t bpp; ///< bytes per pixel stored for texture sensors
                uint32_t rowSize; ///< texture row stride in bytes
            } tex;
            struct
            {
                size_t size; ///< size in bytes of buffer sensors
            } buff;
        }; ///< sensor parameters
    };

BoundingBox2DValues
===================

.. code::

    struct BoundingBox2DValues
    {
        uint32_t instanceId; ///< instance ID
        uint32_t semanticId; ///< semantic ID
        int32_t x_min; ///< left extent
        int32_t y_min; ///< top extent
        int32_t x_max; ///< right extent
        int32_t y_max; ///< bottom extent
    };

BoundingBox3DValues
===================

.. code::

    struct BoundingBox3DValues
    {
        uint32_t instanceId; ///< instance ID
        uint32_t semanticId; ///< semantic ID
        float x_min; ///< left extent
        float y_min; ///< top extent
        float z_min; ///< front extent
        float x_max; ///< right extent
        float y_max; ///< bottom extent
        float z_max; ///< back extent
    };

OcclusionValues
===============

.. code::

    struct OcclusionValues
    {
        uint32_t instanceId; ///< instance ID
        uint32_t semanticId; ///< semantic ID
        float occlusionRatio; ///< ratio of instance that is occluded
    };

TruncationValues
================

.. code::

    struct TruncationValues
    {
        uint32_t instanceId; ///< instance ID
        uint32_t semanticId; ///< semantic ID
        float truncationRatio; ///< ratio of instance that is truncated
    };


Python API Docs
****************

Pybind API
==========

.. code::

    // Creates a sensor of specified type if none exist otherwise return the existing sensor.
    //
    // Args:
    //
    // arg0 (type): The sensor type to return
    create_sensor(sensors::SensorType type)

.. code::

    // Destroys the specified sensor.
    //
    // Args:
    //
    // arg0 (type): The sensor type to destroy
    destroy_sensor(sensors::SensorType type)

.. code::

    // Returns the width of the specified image sensor.
    //
    // Args:
    //
    // arg0 (type): The sensor to retrieve the width for
    get_sensor_width(carb::sensors::SensorType type)

.. code::

    // Returns the height of the specified image sensor.
    //
    // Args:
    //
    // arg0 (type): The sensor to retrieve the height for
    get_sensor_height(carb::sensors::SensorType type)

.. code::

    // Returns the bytes per pixel of the specified image sensor.
    //
    // Args:
    //
    // arg0 (type): The sensor to retrieve the bytes per pixel for
    get_sensor_bpp(carb::sensors::SensorType type)

.. code::

    // Returns the row size in bytes of the specified image sensor.
    //
    // Args:
    //
    // arg0 (type): The sensor to retrieve the row size for
    get_sensor_row_size(carb::sensors::SensorType type)


.. code::

    // Returns the size in bytes of the specified attribute sensor.
    //
    // Args:
    //
    // arg0 (type): The sensor to retrieve the size for
    get_sensor_size(carb::sensors::SensorType type)


.. code::

    // Returns a pointer to the sensor's data on device memory
    //
    // Args:
    //
    // arg0 (type): The sensor to retrieve the data for
    get_sensor_device_data(carb::sensors::SensorType type)

.. code::

    // Returns a pointer to the sensor's data on host memory
    //
    // Args:
    //
    // arg0 (type): The sensor to retrieve the host data for
    get_sensor_host_data(carb::sensors::SensorType type)


.. code::

    // Returns floating point tensor data of the image sensor on device memory
    //
    // Args:
    //
    // arg0 (type): The image sensor to retrieve the tensor data for
    //
    // arg1 (width): The width of the image sensor
    //
    // arg2 (height): The height of the image sensor
    //
    // arg3 (rowSize): The row size in bytes of the image sensor
    get_sensor_device_float_2d_tensor(carb::sensors::SensorType type, size_t width, size_t height, size_t rowSize)

.. code::

    // Returns 32-bit integer tensor data of the image sensor on device memory
    //
    // Args:
    //
    // arg0 (type): The image sensor to retrieve the tensor data for
    //
    // arg1 (width): The width of the image sensor
    //
    // arg2 (height): The height of the image sensor
    //
    // arg3 (rowSize): The row size in bytes of the image sensor
    get_sensor_device_int32_2d_tensor(carb::sensors::SensorType type, size_t width, size_t height, size_t rowSize)

.. code::

    // Returns 8-bit integer vector tensor data of the image sensor on device memory
    //
    // Args:
    //
    // arg0 (type): The image sensor to retrieve the tensor data for
    //
    // arg1 (width): The width of the image sensor
    //
    // arg2 (height): The height of the image sensor
    //
    // arg3 (rowSize): The row size in bytes of the image sensor
    get_sensor_device_uint8_3d_tensor(carb::sensors::SensorType type, size_t width, size_t height, size_t rowSize)

.. code::

    // Returns 32-bit integer numpy array data of the image sensor on host memory
    //
    // Args:
    //
    // arg0 (type): The image sensor to retrieve the numpy data for
    //
    // arg1 (width): The width of the image sensor
    //
    // arg2 (height): The height of the image sensor
    //
    // arg3 (rowSize): The row size in bytes of the image sensor
    get_sensor_host_uint32_texture_array(carb::sensors::SensorType type, size_t width, size_t height, size_t rowSize)

.. code::

    // Returns floating point numpy array data of the image sensor on host memory
    //
    // Args:
    //
    // arg0 (type): The image sensor to retrieve the numpy data for
    //
    // arg1 (width): The width of the image sensor
    //
    // arg2 (height): The height of the image sensor
    //
    // arg3 (rowSize): The row size in bytes of the image sensor
    get_sensor_host_float_texture_array(carb::sensors::SensorType type, size_t width, size_t height, size_t rowSize) 

.. code::

    // Returns floating point numpy array data of the attribute sensor on host memory
    //
    // Args:
    //
    // arg0 (type): The attribute sensor to retrieve the numpy data for
    //
    // arg1 (size): The size of the attribute sensor in bytes
    get_sensor_host_float_buffer_array(carb::sensors::SensorType type, size_t size)

.. code::

    // Returns 32-bit unsigned integer numpy array data of the attribute sensor on host memory
    //
    // Args:
    //
    // arg0 (type): The attribute sensor to retrieve the numpy data for
    //
    // arg1 (size): The size of the attribute sensor in bytes
    get_sensor_host_uint32_buffer_array(carb::sensors::SensorType type, size_t size)

.. code::

    // Returns 32-bit signed integer numpy array data of the attribute sensor on host memory
    //
    // Args:
    //
    // arg0 (type): The attribute sensor to retrieve the numpy data for
    //
    // arg1 (size): The size of the attribute sensor in bytes
    get_sensor_host_int32_buffer_array(carb::sensors::SensorType type, size_t size)

.. code::

    // Returns a numpy array of BoundingBox2DValues data for the attribute sensor on host memory
    //
    // Args:
    //
    // arg0 (type): The attribute sensor to retrieve the numpy data for
    //
    // arg1 (size): The size of the attribute sensor in bytes
    get_sensor_host_bounding_box_2d_buffer_array(carb::sensors::SensorType type, size_t size)

.. code::

    // Returns a numpy array of BoundingBox3DValues data for the attribute sensor on host memory
    //
    // Args:
    //
    // arg0 (type): The attribute sensor to retrieve the numpy data for
    //
    // arg1 (size): The size of the attribute sensor in bytes
    get_sensor_host_bounding_box_3d_buffer_array(carb::sensors::SensorType type, size_t size)

.. code::

    // Returns a numpy array of OcclusionValues data for the attribute sensor on host memory
    //
    // Args:
    //
    // arg0 (type): The attribute sensor to retrieve the numpy data for
    //
    // arg1 (size): The size of the attribute sensor in bytes
    get_sensor_host_occlusion_buffer_array(carb::sensors::SensorType type, size_t size)

.. code::

    // Returns a numpy array of TruncationValues data for the attribute sensor on host memory (TODO)
    //
    // Args:
    //
    // arg0 (type): The attribute sensor to retrieve the numpy data for
    //
    // arg1 (size): The size of the attribute sensor in bytes
    get_sensor_host_truncation_buffer_array(carb::sensors::SensorType type, size_t size)

.. code::

    // Returns the instance ID of the specified mesh as represented by sensor data
    //
    // Args:
    //
    // arg0 (uri): The representation of the mesh in the USD scene
    get_instance_segmentation_id(const char* uri)

.. code::

    // Returns the semantic ID of the specified name and type as represented by sensor data
    //
    // Args:
    //
    // arg0 (type): The semantic type name
    //
    // arg1 (data): The semantic data name
    get_semantic_segmentation_id_from_data(const char* type, const char* data)

.. code::

    // Returns the semantic class name of the semantic ID represented by sensor data
    //
    // Args:
    //
    // arg0 (semanticId): The semantic ID
    get_semantic_segmentation_data_from_id(uint16_t semanticId)

.. code::

    // Specify which semantic classes to retrieve bounding boxes for
    //
    // Args:
    //
    // arg0 (semanticId): The semantic ID to retrieve bounding boxes for
    set_bounding_box_semantic_segmentation_id(uint16_t semanticId)

.. code::

    // Specify which semantic classes to retrieve bounding boxes for
    //
    // Args:
    //
    // arg0 (data): The semantic data class name to retrieve bounding boxes for
    set_bounding_box_semantic_segmentation_data(std::string data)

