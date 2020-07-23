//  ================================================================
//  Created by Gregory Kramida on 11/12/19.
//  Copyright (c) 2019 Gregory Kramida
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//  http://www.apache.org/licenses/LICENSE-2.0

//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//  ================================================================

// Inspired in part by InfiniTAM/ITMLib/Utils/ITMLibSettings of the original InfiniTAM repository, Oxford University

#pragma once

//stdlib
#include <cfloat>
#include <memory>

//boost
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>

//local
#include "../VoxelVolumeParameters.h"
#include "../SurfelVolumeParameters.h"
#include "../../../ORUtils/MemoryDeviceType.h"
#include "../Math.h"
#include "../../SurfaceTrackers/Interface/SlavchevaSurfaceTracker.h"
#include "../../SurfaceTrackers/WarpGradientFunctors/WarpGradientFunctor.h"
#include "../../Engines/Main/NonRigidTrackingParameters.h"
#include "../../Objects/Volume/VoxelBlockHash.h"
#include "../../Objects/Volume/PlainVoxelArray.h"
#include "LoggingSettings.h"

namespace po = boost::program_options;
namespace pt = boost::property_tree;

namespace ITMLib {
namespace configuration {


// region ============================================== SERIALIZABLE ENUMS ============================================

#define FAILUREMODE_ENUM_DESCRIPTION FailureMode, \
    (FAILUREMODE_RELOCALIZE, "relocalize"), \
    (FAILUREMODE_IGNORE, "ignore"), \
    (FAILUREMODE_STOP_INTEGRATION, "stop_integration")

DECLARE_SERIALIZABLE_ENUM(FAILUREMODE_ENUM_DESCRIPTION);

#define SWAPPINGMODE_ENUM_DESCRIPTION SwappingMode, \
    (SWAPPINGMODE_DISABLED, "disabled", "DISABLED", "SWAPPINGMODE_DISABLED"), \
    (SWAPPINGMODE_ENABLED, "enabled", "ENABLED", "SWAPPINGMODE_ENABLED"), \
    (SWAPPINGMODE_DELETE, "delete", "DELETE","SWAPPINGMODE_DELETE")

DECLARE_SERIALIZABLE_ENUM(SWAPPINGMODE_ENUM_DESCRIPTION);


#define VOLUME_ROLE_ENUM_DESCRIPTION VolumeRole, \
    (VOLUME_CANONICAL, "canonical", "CANONICAL"), \
    (VOLUME_LIVE, "live", "LIVE"), \
    (VOLUME_WARP, "warp", "WARP")

DECLARE_SERIALIZABLE_ENUM(VOLUME_ROLE_ENUM_DESCRIPTION);
//endregion ========================================================================================================

// region ======================================== SERIALIZABLE STRUCTS ============================================
#define PATHS_STRUCT_DESCRIPTION Paths,\
    (std::string, output_path, "output", PATH, "Path used for any diagnostic/experimental output."),\
    (std::string, calibration_file_path, "calib.txt", PATH, "Path to the calibration file."),\
    (std::string, openni_file_path, "", PATH, "Path to an openni recording to read from."),\
    (std::string, rgb_video_file_path, "", PATH, "Path to an RGB video file (must be used in combination with depth_video_file_path)"),\
    (std::string, depth_video_file_path, "", PATH, "Path to a depth video file (pixel intensity change of 1 signifies depth change of 1 mm), must be used in combibation with rgb_video_file_path"),\
    (std::string, rgb_image_path_mask, "", PATH, "Path to rgb image files (ex. %06i is a frame number bask for files with 6 digit frame numbers)."),\
    (std::string, depth_image_path_mask, "", PATH, \
        "Path to depth image files (ex. %06i is a frame number bask for files with 6 digit frame numbers, "\
        "where frame numbers must start with 000000)."),\
    (std::string, mask_image_path_mask, "", PATH,\
        "Path to mask image files. These must be binary images. RGB and depth pixels where mask pixels have 0" \
        " intensity will be ignored. (Formatting rules are the same as depth_image_path_mask)."),\
    (std::string, imu_input_path, "", PATH, "Path to imu input file/handle.")

DECLARE_SERIALIZABLE_STRUCT(PATHS_STRUCT_DESCRIPTION);





struct TrackerConfigurationStringPresets {
	static const std::string default_ICP_tracker_configuration;
	static const std::string default_ICP_tracker_configuration_loop_closure;
	static const std::string default_depth_only_extended_tracker_configuration;
	static const std::string default_intensity_depth_extended_tracker_configuration;
	static const std::string default_color_only_tracker_configuration;
	static const std::string default_IMU_ICP_tracker_configuration;
	static const std::string default_IMU_extended_tracker_configuration;
	static const std::string default_surfel_tracker_configuration;
};

#ifndef COMPILE_WITHOUT_CUDA
#define DEFAULT_DEVICE MEMORYDEVICE_CUDA
#else
#define DEFAULT_DEVICE MEMORYDEVICE_CPU
#endif

#define ARRAY_VOLUME_PARAMETERS_STRUCT_DESCRIPTION \
    ArrayVolumeParameters, \
    (GridAlignedBox, canonical, GridAlignedBox(), STRUCT, "Parameters specific to the canonical (target / reference) volume in array indexing configuration."), \
    (GridAlignedBox, live, GridAlignedBox(), STRUCT, "Parameters specific to the live (source) volume in array indexing configuration."), \
    (GridAlignedBox, warp, GridAlignedBox(), STRUCT, "Parameters specific to the volume holding warp vectors (motion information) in array indexing configuration.")

DECLARE_SERIALIZABLE_STRUCT(ARRAY_VOLUME_PARAMETERS_STRUCT_DESCRIPTION);

#define HASH_VOLUME_PARAMETERS_STRUCT_DESCRIPTION \
    HashVolumeParameters, \
    (VoxelBlockHash::VoxelBlockHashParameters, canonical, VoxelBlockHash::VoxelBlockHashParameters(), STRUCT, "Parameters specific to the canonical (target / reference) volume in voxel block hash indexing configuration."), \
    (VoxelBlockHash::VoxelBlockHashParameters, live, VoxelBlockHash::VoxelBlockHashParameters(), STRUCT, "Parameters specific to the live (source) volume in voxel block hash indexing configuration."), \
    (VoxelBlockHash::VoxelBlockHashParameters, warp, VoxelBlockHash::VoxelBlockHashParameters(), STRUCT, "Parameters specific to the volume holding warp vectors (motion information) in voxel block hash indexing configuration.")

DECLARE_SERIALIZABLE_STRUCT(HASH_VOLUME_PARAMETERS_STRUCT_DESCRIPTION);

#define SPECIFIC_VOLUME_PARAMETERS_STRUCT_DESCRIPTION \
    SpecificVolumeParameters, \
    (ArrayVolumeParameters, array, ArrayVolumeParameters(), STRUCT, "Specific parameters to use for different volumes with the array indexing method."), \
    (HashVolumeParameters, hash, HashVolumeParameters(), STRUCT, "Specific parameters to use for different volumes with the hash indexing method.")

DECLARE_SERIALIZABLE_STRUCT(SPECIFIC_VOLUME_PARAMETERS_STRUCT_DESCRIPTION);

#define CONFIGURATION_STRUCT_DESCRIPTION Configuration, \
    (Vector3i, focus_coordinates, Vector3i(0), VECTOR, \
    "3d coordinates (integer) that specify the voxel to log additional diagnostic information about (and " \
    "where to focus telemetry information records). "\
    "Only effective for logging when verbosity_level is set to focus_spots (alt. VERBOSITY_FOCUS_SPOTS) or above."), \
    (bool, record_reconstruction_video, false, PRIMITIVE, \
            "Whether to record video of the canonical reconstruction during automatic run "),\
	(bool, record_inputs_in_reconstruction_video, false, PRIMITIVE, \
            "Whether to concatenate input frames to the canonical reconstruction video frames (only useful with  '--record_reconstruction_video')"),\
    (VoxelVolumeParameters, general_voxel_volume_parameters, VoxelVolumeParameters(), STRUCT, "Voxel volume parameters, such as voxel size."),\
    (SurfelVolumeParameters, general_surfel_volume_parameters, SurfelVolumeParameters(), STRUCT, "Surfel volume parameters, such as surfel radius."),\
    (SpecificVolumeParameters, specific_volume_parameters, SpecificVolumeParameters(), STRUCT, "Parameters for specific volumes (multiple volumes are used in dynamic mode), may include information for different indexing methods."), \
    (SlavchevaSurfaceTracker::Parameters, slavcheva_parameters, SlavchevaSurfaceTracker::Parameters(), STRUCT,"Parameters pertaining to energy tuning for dynamic surface tracking."),\
    (SlavchevaSurfaceTracker::Switches, slavcheva_switches, SlavchevaSurfaceTracker::Switches(), STRUCT,"Switches pertaining to optimization for dynamic surface tracking."),\
    (LoggingSettings, logging_settings, LoggingSettings(), STRUCT, "Logging settings"),\
    (Paths, paths, Paths(), STRUCT,"Input / output paths"),\
    (NonRigidTrackingParameters, non_rigid_tracking_parameters, NonRigidTrackingParameters(), STRUCT,"Parameters pertaining to stopping conditions, gradient functor type, and momentum weight that are used for dynamic surface tracking."),\
    (bool, create_meshing_engine, true, PRIMITIVE, "Create all the things required for marching cubes and mesh extraction (uses lots of additional memory)"),\
    (MemoryDeviceType, device_type, DEFAULT_DEVICE, ENUM, "Type of device to use, i.e. CPU/GPU/Metal"),\
    (bool, use_approximate_raycast, false, PRIMITIVE, "Enables or disables approximate raycast."),\
    (bool, use_threshold_filter, false, PRIMITIVE, "Enables or disables threshold filtering, i.e. filtering out pixels whose difference from their neighbors exceeds a certain threshold"),\
    (bool, use_bilateral_filter, false, PRIMITIVE, "Enables or disables bilateral filtering on depth input images."),\
    (FailureMode, behavior_on_failure, FAILUREMODE_IGNORE, ENUM, "What to do on tracker failure: ignore, relocalize or stop integration - not supported in loop closure or dynamic libmode"),\
    (SwappingMode, swapping_mode, SWAPPINGMODE_DISABLED, ENUM, "Determines how swapping works: disabled, fully enabled (still with dragons) and delete what's not visible - not supported in loop closure version"),\
    (std::string, tracker_configuration, TrackerConfigurationStringPresets::default_depth_only_extended_tracker_configuration, PRIMITIVE, "Tracker configuration. (Better description still needs to be provided for this, already in TODO / issues)")


DECLARE_SERIALIZABLE_STRUCT(CONFIGURATION_STRUCT_DESCRIPTION);

Configuration& get();
template<typename TIndex>
typename TIndex::InitializationParameters for_volume_role(VolumeRole role);
void load_configuration_from_variable_map(const po::variables_map& vm);
void load_default();
void load_configuration_from_json_file(const std::string& path);
void save_configuration_to_json_file(const std::string& path);
void save_configuration_to_json_file(const std::string& path, const Configuration& configuration);

template<typename TDeferrableStruct>
static void AddDeferrableToSourceTree(configuration::Configuration& _configuration, const TDeferrableStruct& deferrable_struct){
    _configuration.source_tree.add_child(TDeferrableStruct::default_parse_path, deferrable_struct.ToPTree(_configuration.origin));
}

} // namespace configuration
} // namespace ITMLib