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
#include "VoxelVolumeParameters.h"
#include "SurfelVolumeParameters.h"
#include "../../ORUtils/MemoryDeviceType.h"
#include "Math.h"
#include "../SurfaceTrackers/Interface/SlavchevaSurfaceTracker.h"
#include "../SurfaceTrackers/WarpGradientFunctors/WarpGradientFunctor.h"
#include "../Engines/Main/NonRigidTrackingParameters.h"
#include "../Objects/Volume/VoxelBlockHash.h"
#include "../Objects/Volume/PlainVoxelArray.h"
#include "../Utils/Geometry/CardinalAxesAndPlanes.h"

namespace po = boost::program_options;
namespace pt = boost::property_tree;


namespace ITMLib {
namespace configuration {
// region ============================================== SERIALIZABLE ENUMS ============================================
#define VERBOSITY_LEVEL_ENUM_DESCRIPTION VerbosityLevel, \
    (VERBOSITY_SILENT, "silent", "SILENT", "VERBOSITY_SILENT"), \
    (VERBOSITY_FATAL, "fatal", "FATAL", "VERBOSITY_FATAL"), \
    (VERBOSITY_ERROR, "error", "ERROR", "VERBOSITY_ERROR"), \
    (VERBOSITY_WARNING, "warning", "WARNING", "VERBOSITY_WARNING"), \
    (VERBOSITY_INFO, "info", "information", "INFO", "VERBOSITY_INFO"), \
    (VERBOSITY_TOP_LEVEL, "top_level", "TOP_LEVEL", "Top-level operations", "VERBOSITY_TOP_LEVEL", "top-level", "top-level operations"), \
    (VERBOSITY_PER_FRAME, "per_frame", "PER_FRAME", "Per-frame operations", "VERBOSITY_PER_FRAME", "per-frame", "per-frame operations"), \
    (VERBOSITY_PER_ITERATION, "per_iteration", "PER_ITERATION", "Per-iteration operations", "VERBOSITY_PER_ITERATION", "per-iteration", "per-iteration operations"), \
    (VERBOSITY_FOCUS_SPOTS, "focus_spots", "FOCUS_SPOTS", "focus_coordinates", "Interesting details", "trouble spots"), \
    (VERBOSITY_DEBUG, "debug", "DEBUG", "VERBOSITY_DEBUG")


DECLARE_SERIALIZABLE_ENUM(VERBOSITY_LEVEL_ENUM_DESCRIPTION)

#define FAILUREMODE_ENUM_DESCRIPTION FailureMode, \
    (FAILUREMODE_RELOCALIZE, "relocalize"), \
    (FAILUREMODE_IGNORE, "ignore"), \
    (FAILUREMODE_STOP_INTEGRATION, "stop_integration")

DECLARE_SERIALIZABLE_ENUM(FAILUREMODE_ENUM_DESCRIPTION)

#define SWAPPINGMODE_ENUM_DESCRIPTION SwappingMode, \
    (SWAPPINGMODE_DISABLED, "disabled", "DISABLED", "SWAPPINGMODE_DISABLED"), \
    (SWAPPINGMODE_ENABLED, "enabled", "ENABLED", "SWAPPINGMODE_ENABLED"), \
    (SWAPPINGMODE_DELETE, "delete", "DELETE","SWAPPINGMODE_DELETE")

DECLARE_SERIALIZABLE_ENUM(SWAPPINGMODE_ENUM_DESCRIPTION)

#define LIBMODE_ENUM_DESCRIPTION LibMode, \
    (LIBMODE_BASIC, "basic"), \
    (LIBMODE_BASIC_SURFELS, "surfels"), \
    (LIBMODE_LOOPCLOSURE, "loop_closure"), \
    (LIBMODE_DYNAMIC, "dynamic")

DECLARE_SERIALIZABLE_ENUM(LIBMODE_ENUM_DESCRIPTION)

#define INDEXING_METHOD_ENUM_DESCRIPTION IndexingMethod, \
    (INDEX_HASH, "hash", "HASH"), \
    (INDEX_ARRAY, "array", "ARRAY")

DECLARE_SERIALIZABLE_ENUM(INDEXING_METHOD_ENUM_DESCRIPTION)

#define VOLUME_ROLE_ENUM_DESCRIPTION VolumeRole, \
    (VOLUME_CANONICAL, "canonical", "CANONICAL"), \
    (VOLUME_LIVE, "live", "LIVE"), \
    (VOLUME_WARP, "warp", "WARP")

DECLARE_SERIALIZABLE_ENUM(VOLUME_ROLE_ENUM_DESCRIPTION)
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


///For focus_coordinates to be used, VerbosityLevel must be set to VERBOSITY_FOCUS_SPOTS or above
#define TELEMETRY_SETTINGS_STRUCT_DESCRIPTION TelemetrySettings, \
    (Vector3i, focus_coordinates, Vector3i(0), VECTOR, \
    "Focus 3d coordinates (integer) that specify the voxel to print additional diagnostic information about (and " \
    "where to focus telemetry information records). "\
    "Only effective when verbosity_level is set to focus_spots (alt. VERBOSITY_FOCUS_SPOTS) or above."), \
    (bool, record_reconstruction_video, false, PRIMITIVE, \
            "Whether to record video of the canonical reconstruction during automatic run "\
            "(see number_of_frames_to_process_after_launch and index_of_frame_to_start_at)."), \
    (bool, log_to_disk, false, PRIMITIVE, "Print log to text file, in the output path (preserves older log files). "\
    "Can be used in combination with stdout."), \
    (bool, log_to_stdout, true, PRIMITIVE, "Print log to stdout. Can be used in combination with disk logging."), \
    (bool, log_benchmarks, false, PRIMITIVE, "Whether to log runtime benchmarks after automatic run."),\
    (bool, log_volume_statistics, false, PRIMITIVE, "Whether to output various volume statistics after some operations" \
    " (used only when verbosity_level is set at or above PER_FRAME)."),\
    (bool, log_trajectory_quaternions, false, PRIMITIVE, "Whether to log estimated camera trajectory quaternions"),\
    (bool, record_volume_memory_usage, false, PRIMITIVE, "Whether to record information required to debug memory" \
    " usage, e.g. used block locations for the VoxelBlockHash index."), \
    (bool, record_live_volume_as_2D_slices, false, PRIMITIVE, "Whether to record 2D slices (images, with pixel " \
    "representing TSDF value) of the live volume once per frame before the beginning of the surface tracking optimization."), \
    (bool, record_canonical_volume_as_2D_slices, false, PRIMITIVE, "Whether to record 2D slices (images, with pixel " \
    "representing TSDF value) of the canonical volume once per frame before the beginning of the surface tracking optimization."), \
    (bool, record_live_focus_point_TSDF_graph, false, PRIMITIVE, "Whether to record graphs of SDF value of a single " \
    "voxel plotted against iteration number of the surface tracking optimization."), \
    (bool, record_live_focus_layer_TSDF_heatmap, false, PRIMITIVE, "Whether to record TSDF heatmaps for the voxel layer in " \
    "the warped live volume (in the plane specified by TSDF_heatmap_plane parameter) for each iteration of the surface" \
    "tracking optimization."), \
    (Plane, TSDF_heatmap_plane, PLANE_ZX, ENUM, "Plane in which to record TSDF heatmaps " \
    "(see record_live_focus_layer_TSDF_heatmap parameter help)."), \
    (bool, record_focus_neighborhood_live_tsdf_sequence, false, PRIMITIVE, "Whether to record a sequence of TSDF " \
    "volumes representing the immediate neighborhood of the focus_coordinates (see focus_coordinates parameter) " \
    "in the warped live frame over the course of the entire surface tracking optimization. [WITH_VTK compilation required!]"), \
    (int, focus_neighborhood_size, 3, PRIMITIVE, "For focus neighborhood recording: a cube of size " \
    "2 x focus_neighborhood_size + 1 around the focus_coordinates will be recorded. [WITH_VTK compilation required!]"), \
    (bool, record_focus_neighborhood_warp_sequence, false, PRIMITIVE, "Whether to record a sequence of warp " \
    "vectors in the immediate neighborhood of the focus_coordinates (see focus_coordinates parameter) over " \
    "the course of the entire surface tracking optimization. [WITH_VTK compilation required!]"), \
    (bool, log_surface_tracking_optimization_energies, false, PRIMITIVE, "Whether to log optimization energies " \
    "for each iteration of the surface tracking optimization. Only works when non_rigid_tracking_parameters.functor_type " \
    "parameter is set to \"slavcheva_diagnostic\""), \
    (bool, record_frame_meshes, false, PRIMITIVE, "Whether to log three meshes at every frame: (a) from live volume " \
    "after camera tracking and before surface tracking, (b) from live volume after surface tracking, and (c) from " \
    "canonical volume after fusion."), \
    (bool, use_CPU_for_mesh_recording, false, PRIMITIVE, "Whether to ALWAYS use CPU & regular RAM when recording mesh telemetry. For CUDA runs, this will reduce GPU memory usage."), \
	(bool, record_camera_matrices, false, PRIMITIVE, "Whether to record estimated camera trajectory matrices in world space.")


DECLARE_SERIALIZABLE_STRUCT(TELEMETRY_SETTINGS_STRUCT_DESCRIPTION);

#define AUTOMATIC_RUN_SETTINGS_STRUCT_DESCRIPTION AutomaticRunSettings, \
    (int, number_of_frames_to_process, 0, PRIMITIVE, "This number of frames will be processed automatically after the program is launched (launches automatic run)."), \
    (int, index_of_frame_to_start_at, 0, PRIMITIVE, "Index of the first frame (or frame set) to read from disk (or, how many frames to skip). The remaining frames will be read in order."), \
    (bool, load_volume_before_processing, false, PRIMITIVE, "When this is set to true, the program will attempt to load the volume for the index_of_frame_to_start_with from the corresponding subfolder within output_folder."), \
    (bool, save_volumes_after_processing, false, PRIMITIVE, "Whether to save volume(s) after automatic processing"), \
    (bool, exit_after_automatic_processing, false, PRIMITIVE, "Whether to exit the program after the automatic run.")

DECLARE_SERIALIZABLE_STRUCT(AUTOMATIC_RUN_SETTINGS_STRUCT_DESCRIPTION);


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
    (VoxelVolumeParameters, general_voxel_volume_parameters, VoxelVolumeParameters(), STRUCT, "Voxel volume parameters, such as voxel size."),\
    (SurfelVolumeParameters, general_surfel_volume_parameters, SurfelVolumeParameters(), STRUCT, "Surfel volume parameters, such as surfel radius."),\
    (SpecificVolumeParameters, specific_volume_parameters, SpecificVolumeParameters(), STRUCT, "Parameters for specific volumes (multiple volumes are used in dynamic mode), may include information for different indexing methods."), \
    (SlavchevaSurfaceTracker::Parameters, slavcheva_parameters, SlavchevaSurfaceTracker::Parameters(), STRUCT,"Parameters pertaining to energy tuning for dynamic surface tracking."),\
    (SlavchevaSurfaceTracker::Switches, slavcheva_switches, SlavchevaSurfaceTracker::Switches(), STRUCT,"Switches pertaining to optimization for dynamic surface tracking."),\
    (TelemetrySettings, telemetry_settings, TelemetrySettings(), STRUCT, "Telemetry / diagnostics / logging settings"),\
    (Paths, paths, Paths(), STRUCT,"Input / output paths"),\
    (AutomaticRunSettings, automatic_run_settings, AutomaticRunSettings(), STRUCT, "Settings that dictate how the experiment is run, i.e. such as how much data to process automatically, how to behave after the automatic run, etc."),\
    (NonRigidTrackingParameters, non_rigid_tracking_parameters, NonRigidTrackingParameters(), STRUCT,"Parameters pertaining to stopping conditions, gradient functor type, and momentum weight that are used for dynamic surface tracking."),\
    (bool, skip_points, false, PRIMITIVE, "For ITMColorTracker: skips every other point when using the colour renderer for creating a point cloud"),\
    (bool, create_meshing_engine, true, PRIMITIVE, "Create all the things required for marching cubes and mesh extraction (uses lots of additional memory)"),\
    (MemoryDeviceType, device_type, DEFAULT_DEVICE, ENUM, "Type of device to use, i.e. CPU/GPU/Metal"),\
    (bool, use_approximate_raycast, false, PRIMITIVE, "Enables or disables approximate raycast."),\
    (bool, use_threshold_filter, false, PRIMITIVE, "Enables or disables threshold filtering, i.e. filtering out pixels whose difference from their neighbors exceeds a certain threshold"),\
    (bool, use_bilateral_filter, false, PRIMITIVE, "Enables or disables bilateral filtering on depth input images."),\
    (bool, enable_rigid_tracking, true, PRIMITIVE, "Enables or disables rigid tracking."),\
    (FailureMode, behavior_on_failure, FAILUREMODE_IGNORE, ENUM, "What to do on tracker failure: ignore, relocalize or stop integration - not supported in loop closure or dynamic libmode"),\
    (SwappingMode, swapping_mode, SWAPPINGMODE_DISABLED, ENUM, "Determines how swapping works: disabled, fully enabled (still with dragons) and delete what's not visible - not supported in loop closure version"),\
    (LibMode, library_mode, LIBMODE_DYNAMIC, ENUM, "Switch between various library modes - basic, with loop closure, etc."),\
    (IndexingMethod, indexing_method, INDEX_HASH, ENUM, "Indexing method to use in the 3D volumes, i.e. array or hash."),\
    (VerbosityLevel, verbosity_level, VERBOSITY_PER_FRAME, ENUM, "Verbosity level. "),\
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

} // namespace configuration
} // namespace ITMLib