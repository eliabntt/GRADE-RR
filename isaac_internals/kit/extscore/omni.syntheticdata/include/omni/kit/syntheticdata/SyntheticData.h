// Copyright (c) 2020-2021, NVIDIA CORPORATION. All rights reserved.
//
// NVIDIA CORPORATION and its licensors retain all intellectual property
// and proprietary rights in and to this software, related documentation
// and any modifications thereto. Any use, reproduction, disclosure or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA CORPORATION is strictly prohibited.
//
#pragma once

#include <carb/Interface.h>
#include <carb/sensors/SensorTypes.h>

#include <string>

namespace omni
{

// Forward declare deprecated viewport interface
namespace kit
{
    class IViewportWindow;
};

namespace syntheticdata
{

/**
 * Return code for functions that need to return information about a failure.
 */
enum class SyntheticDataResult : int32_t
{
    eSuccess = 0, ///< Successful completion.
    eFailure, ///< A general failure or error.
};

/**
 * Structure which contains a list mapping instance Ids to prims with semantic labels that
 * can be used to build bounding boxes for meshes with a common semantic class
 */
struct InstanceMapping
{
    uint32_t uniqueId; ///< a unique ID for this instance mapping entry
    std::string primPath; ///< the prim path for this instance mapping
    uint16_t semanticId; ///< the semantic ID for this instance mapping
    std::string semanticLabel; ///< the semantic class label for this mapping
    std::vector<uint32_t> instanceIds; ///< a list of instance Ids that inherit this semantic label
    void* metaData; ///< reserved for future use
};

struct SemanticInstanceMappingData
{
    using TInstanceIndex = uint32_t;
    using TInstanceToken = uint64_t; 
    static constexpr TInstanceToken invalidInstanceTypeToken = 0;

    TInstanceIndex numInstances; ///< number of instances in the scene
    TInstanceIndex minInstanceIndex; ///< minimum instance index
    /// array containing the prim path token for every instances (from minInstanceIndex to minInstanceIndex + numInstances - 1)
    TInstanceToken* instanceTokens;

    using TSemanticIndex = uint16_t;
    static constexpr TSemanticIndex invalidSemanticIndex = 0xFFFF;
    using TSemanticToken = uint64_t; 
    static constexpr TSemanticToken invalidSemanticToken = 0;

    using TSemanticTypeToken = uint64_t;
   
    TSemanticIndex numSemantics; ///< number of semantic prim in the scene
    TSemanticIndex minSemanticIndex; ///< minimum semantic index
    /// array containing the prim path token for every semantic prim (from minSemanticIndex to minSemanticIndex + numSemantics
    /// - 1)
    TSemanticToken* semanticTokens;

    using TTransform = float[4][4];

    TTransform* semanticLocalTransforms; ///< transform from world to semantic prim local space
    TTransform* semanticWorldTransforms; ///< transform from semantic prim local space to world

    /// array of size numInstances containing the mapping from the instance index to its first semantic prim parent index
    TSemanticIndex* instanceSemanticMap;
    /// array of size numSemantics containing the mapping from the semantic index to its first semantic prim parent index
    TSemanticIndex* semanticMap;
};

struct BoundingBox2DExtent
{
    int32_t x_min; ///< left extent
    int32_t y_min; ///< top extent
    int32_t x_max; ///< right extent
    int32_t y_max; ///< bottom extent
};

struct BoundingBox3DExtent
{
    float x_min; ///< left extent (local coords)
    float y_min; ///< top extent (local coords)
    float z_min; ///< front extent (local coords)
    float x_max; ///< right extent (local coords)
    float y_max; ///< bottom extent (local coords)
    float z_max; ///< back extent (local coords)
};

struct SyntheticData
{
    CARB_PLUGIN_INTERFACE("omni::syntheticdata::SyntheticData", 0, 8)

    /**
     * Enable a specific sensor type on a viewport.
     *
     * @param type the type of sensor to enable
     *
     * @param viewportWindow the viewport to enable the sensor on
     *
     * @return false
     * 
     * @deprecated removed since version 0.8
     */
    bool(CARB_ABI* createSensor)(carb::sensors::SensorType type,
                                 omni::kit::IViewportWindow* viewportWindow);

    /**
     * Disable a sensor on a viewport.
     *
     * @param type the type of sensor to disable
     *
     * @param viewportWindow the viewport to disable the sensor on
     *
     * @return false
     * 
     * @deprecated removed since version 0.8
     */
    bool(CARB_ABI* destroySensor)(carb::sensors::SensorType type,
                                  omni::kit::IViewportWindow* viewportWindow);

    /**
     * Retrieve resource information on the sensor type of a specific viewport
     *
     * @param type the type of sensor to retrieve information from
     *
     * @param viewportWindow the viewport to retrieve information from
     * 
     * @return dummy sensorInfo.
     * 
     * @deprecated removed since version 0.8
     */
    const carb::sensors::SensorInfo(CARB_ABI* getSensorInfo)(carb::sensors::SensorType type,
                                                             omni::kit::IViewportWindow* viewportWindow);

    /**
     * Get pointer the sensor data on the device
     *
     * @param type the type of sensor to retrieve data from
     *
     * @param viewportWindow the viewport you are retrieving data from
     *
     * @return nullptr
     * 
     * @deprecated removed since version 0.8
     */
    void*(CARB_ABI* getSensorDeviceData)(carb::sensors::SensorType type,
                                         omni::kit::IViewportWindow* viewportWindow);

    /**
     * Get pointer the sensor data on the host
     *
     * @param type the type of sensor to retrieve data from
     *
     * @param viewportWindow the viewport you are retrieving data from
     *
     * @return nullptr
     * 
     * @deprecated removed since version 0.8
     */
    void*(CARB_ABI* getSensorHostData)(carb::sensors::SensorType type,
                                       omni::kit::IViewportWindow* viewportWindow);

    /**
     * Get the number of sensors that are currently enabled on a viewport
     *
     * @param viewportWindow the viewport to return the count for
     *
     * @return 0
     * 
     * @deprecated removed since version 0.8
     */
    uint32_t(CARB_ABI* getSensorsCount)(omni::kit::IViewportWindow* viewportWindow);

    /**
     * Get a list of sensors bound to the viewport
     *
     * @param viewportWindow the viewport to retrieve sensors for
     *
     * @param sensorList an array of sensors in the viewport
     *
     * @param count number of sensors to retrieve for the given viewport
     *
     * @remarks It is up to the caller to allocate memory for the sensor
     *          list, using getSensorCount() to determind the list size.
     *          The call will fail if count is larger than the number of
     *          instance IDs at uri, or if instanceList is NULL.
     *
     * @return eFailure
     * 
     * @deprecated removed since version 0.8
     */
    SyntheticDataResult(CARB_ABI* getSensors)(omni::kit::IViewportWindow* viewportWindow,
                                              carb::sensors::SensorType* sensorList,
                                              uint32_t count);

    /**
     * Get instance ID count for an asset
     *
     * @param uri the object to retrieve the instance ID count for
     *
     * @return number of instance IDs listed at the uri
     *
     * @deprecated to be removed, replaced by InstanceMapping AOVs
     */
    size_t(CARB_ABI* getInstanceIdsCount)(const char* uri);

    /**
     * Get instance IDs for an asset
     *
     * @param uri the object to retrieve the instance ID list for
     *
     * @param instanceList An array of instance IDs for the given uri
     *
     * @param count number of instance IDs to retrieve at the uri
     *
     * @remarks It is up to the caller to allocate memory for the instance
     *          list, using getInstanceIdsCount() to determind the list size.
     *          The call will fail if count is larger than the number of
     *          instance IDs at uri, or if instanceList is NULL.
     *
     * @return eSuccess if successful, eFailure otherwise
     *
     * @deprecated to be removed, replaced by InstanceMapping AOVs
     */
    SyntheticDataResult(CARB_ABI* getInstanceIds)(const char* uri, uint32_t* instanceList, size_t count);

    /**
     * Get uri from an instance ID
     *
     * @param instanceId Instance ID to retrieve a uri for
     *
     * @return uri for the mesh specified by instanceId
     * 
     * @deprecated to be removed, replaced by InstanceMapping AOVs
     */
    const char*(CARB_ABI* getUriFromInstanceId)(uint32_t instanceId);

    /**
     * Get semantic ID for an asset
     *
     * @param type the semantic type to retrieve the semantic ID for

     * @param data the semantic data to retrieve the semantic ID for
     *
     * @return semantic ID retrieved by the segmentation sensor
     * 
     * @deprecated to be removed, replaced by InstanceMapping AOVs
     */
    uint16_t(CARB_ABI* getSemanticIdFromData)(const char* type, const char* data);

    /**
     * Get semantic type for an asset
     *
     * @param semanticId the Id to retrieve the semantic type for
     *
     * @return semantic type retrieved by the segmentation sensor
     * 
     * @deprecated to be removed, replaced by InstanceMapping AOVs
     */
    const char*(CARB_ABI* getSemanticTypeFromId)(uint16_t semanticId);

    /**
     * Get semantic data for an asset
     *
     * @param semanticId the Id to retrieve the semantic class fors
     *
     * @return semantic data retrieved by the segmentation sensor
     * 
     * @deprecated to be removed, replaced by InstanceMapping AOVs
     */
    const char*(CARB_ABI* getSemanticDataFromId)(uint16_t semanticId);

    /**
     * Specify the semantic ID of bounding boxes to return
     *
     * @param semanticId only retreive bounding boxes with this id
     *
     * @remarks 0 is the default value and retrieves all bounding boxes
     * 
     * @deprecated to be removed, will be replaced by OmniGraph node
     */
    void(CARB_ABI* setBoundingBoxSemanticId)(uint16_t semanticId);

    /**
     * Specify the semantic type and data of bounding boxes to return
     *
     * @param type only retrieve bounding boxes with this semantic type

     * @param data only retreive bounding boxes with this semantic data
     *
     * @remarks null string is the default value and retrieves all bounding boxes
     * 
     * @deprecated to be removed, will be replaced by OmniGraph node
     */
    void(CARB_ABI* setBoundingBoxSemantics)(const char* type, const char* data);

    /**
     * Build out a list that will containing mappings of instances and semantics
     * to prims
     *
     * @param uri the root prim to derive instance mappings from
     *
     * @param instanceMappings a list to output instance mappings to
     *
     * @remarks It is expected that you will pass in an empty vector to be filled
     *          in with instance mappings. It is the caller's responsibility to make
     *          sure the vector is freed when it is no longer needed
     * 
     * @deprecated to be removed, replaced by InstanceMapping AOVs
     */
    size_t(CARB_ABI* getInstanceMappings)(const char* uri, std::vector<InstanceMapping>& instanceMappings);

     /**
     * Callback to update the pipeline internal data just before flushing the fabric
     *
     * @param stausdStageId id of the current USD stage for which the data has to be updated
     */
    void(CARB_ABI* updatePipelineBeforeFabricFlush)(uint64_t usdStageId);

};
}
}
