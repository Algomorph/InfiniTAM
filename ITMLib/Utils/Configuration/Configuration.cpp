//  ================================================================
//  Created by Gregory Kramida on 11/08/19.
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

//stdlib
#include <unordered_map>
#include <utility>
#include <regex>

//boost
#include <boost/filesystem.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/preprocessor/stringize.hpp>

//local
#include "../Metacoding/DeferrableStructUtilities.h"
#include "Configuration.h"
#include "../FileIO/JSON_Utilities.h"
#include "TelemetrySettings.h"
#include "../../Engines/Indexing/IndexingSettings.h"
#include "../../Engines/Rendering/RenderingSettings.h"
#include "AutomaticRunSettings.h"
#include "../../Engines/Main/MainEngineSettings.h"

using namespace ITMLib::configuration;

// *** serializable enum definitions ***

DEFINE_SERIALIZABLE_ENUM(FAILUREMODE_ENUM_DESCRIPTION);

DEFINE_SERIALIZABLE_ENUM(SWAPPINGMODE_ENUM_DESCRIPTION);


// defined in other headers or externally
DEFINE_SERIALIZABLE_ENUM(MemoryDeviceType,
                         (MEMORYDEVICE_CPU, "cpu", "CPU", "MEMORYDEVICE_CPU"),
                         (MEMORYDEVICE_CUDA, "cuda", "CUDA", "MEMORYDEVICE_CUDA"),
                         (MEMORYDEVICE_METAL, "metal", "METAL", "MEMORYDEVICE_METAL"));

DEFINE_SERIALIZABLE_ENUM(VOLUME_ROLE_ENUM_DESCRIPTION);

namespace ITMLib {
namespace configuration {

// =====
// *** serializable struct definitions ***

DEFINE_SERIALIZABLE_STRUCT(PATHS_STRUCT_DESCRIPTION);

//_DEBUG
DEFINE_SERIALIZABLE_STRUCT(ARRAY_VOLUME_PARAMETERS_STRUCT_DESCRIPTION);


DEFINE_SERIALIZABLE_STRUCT(HASH_VOLUME_PARAMETERS_STRUCT_DESCRIPTION);

//_DEBUG
DEFINE_SERIALIZABLE_STRUCT(SPECIFIC_VOLUME_PARAMETERS_STRUCT_DESCRIPTION);

//_DEBUG
//DEFINE_SERIALIZABLE_STRUCT(CONFIGURATION_STRUCT_DESCRIPTION);
Configuration::Configuration() = default;
Configuration::Configuration(Configuration& other, const std::string& parse_path) : Configuration(other) { this->parse_path = parse_path; }
Configuration::Configuration(Vector3i focus_coordinates, bool record_reconstruction_video, bool record_inputs_in_reconstruction_video,
                             VoxelVolumeParameters general_voxel_volume_parameters, SurfelVolumeParameters general_surfel_volume_parameters,
                             SpecificVolumeParameters specific_volume_parameters, SlavchevaSurfaceTracker::Parameters slavcheva_parameters,
                             SlavchevaSurfaceTracker::Switches slavcheva_switches, LoggingSettings logging_settings, Paths paths,
                             NonRigidTrackingParameters non_rigid_tracking_parameters, bool create_meshing_engine, MemoryDeviceType device_type,
                             bool use_approximate_raycast, bool use_threshold_filter, bool use_bilateral_filter, FailureMode behavior_on_failure,
                             SwappingMode swapping_mode, std::string tracker_configuration, std::string origin, std::string parse_path)
		: focus_coordinates(focus_coordinates), record_reconstruction_video(record_reconstruction_video),
		  record_inputs_in_reconstruction_video(record_inputs_in_reconstruction_video),
		  general_voxel_volume_parameters(std::move(general_voxel_volume_parameters)),
		  general_surfel_volume_parameters(std::move(general_surfel_volume_parameters)),
		  specific_volume_parameters(std::move(specific_volume_parameters)), slavcheva_parameters(std::move(slavcheva_parameters)),
		  slavcheva_switches(std::move(slavcheva_switches)), logging_settings(std::move(logging_settings)), paths(std::move(paths)),
		  non_rigid_tracking_parameters(std::move(non_rigid_tracking_parameters)), create_meshing_engine(create_meshing_engine),
		  device_type(device_type), use_approximate_raycast(use_approximate_raycast), use_threshold_filter(use_threshold_filter),
		  use_bilateral_filter(use_bilateral_filter), behavior_on_failure(behavior_on_failure), swapping_mode(swapping_mode),
		  tracker_configuration(tracker_configuration), origin(std::move(origin)) {}
Configuration::Configuration(const boost::program_options::variables_map& vm, std::string origin, std::string parse_path) : focus_coordinates(
		vm["focus_coordinates"].empty() ? Vector3i(0) : variables_map_to_vector<Vector3i>(vm, "focus_coordinates")), record_reconstruction_video(
		vm["record_reconstruction_video"].empty() ? false : vm["record_reconstruction_video"].as<bool>()), record_inputs_in_reconstruction_video(
		vm["record_inputs_in_reconstruction_video"].empty() ? false : vm["record_inputs_in_reconstruction_video"].as<bool>()),
                                                                                                                            general_voxel_volume_parameters(
		                                                                                                                            vm),
                                                                                                                            general_surfel_volume_parameters(
		                                                                                                                            vm),
                                                                                                                            specific_volume_parameters(
		                                                                                                                            vm),
                                                                                                                            slavcheva_parameters(vm),
                                                                                                                            slavcheva_switches(vm),
                                                                                                                            logging_settings(vm),
                                                                                                                            paths(vm),
                                                                                                                            non_rigid_tracking_parameters(
		                                                                                                                            vm),
                                                                                                                            create_meshing_engine(
		                                                                                                                            vm["create_meshing_engine"].empty()
		                                                                                                                            ? true
		                                                                                                                            : vm["create_meshing_engine"].as<bool>()),
                                                                                                                            device_type(
		                                                                                                                            vm["device_type"].empty()
		                                                                                                                            ? MEMORYDEVICE_CUDA
		                                                                                                                            : string_to_enumerator<MemoryDeviceType>(
				                                                                                                                            vm["device_type"].as<std::string>())),
                                                                                                                            use_approximate_raycast(
		                                                                                                                            vm["use_approximate_raycast"].empty()
		                                                                                                                            ? false
		                                                                                                                            : vm["use_approximate_raycast"].as<bool>()),
                                                                                                                            use_threshold_filter(
		                                                                                                                            vm["use_threshold_filter"].empty()
		                                                                                                                            ? false
		                                                                                                                            : vm["use_threshold_filter"].as<bool>()),
                                                                                                                            use_bilateral_filter(
		                                                                                                                            vm["use_bilateral_filter"].empty()
		                                                                                                                            ? false
		                                                                                                                            : vm["use_bilateral_filter"].as<bool>()),
                                                                                                                            behavior_on_failure(
		                                                                                                                            vm["behavior_on_failure"].empty()
		                                                                                                                            ? FAILUREMODE_IGNORE
		                                                                                                                            : string_to_enumerator<FailureMode>(
				                                                                                                                            vm["behavior_on_failure"].as<std::string>())),
                                                                                                                            swapping_mode(
		                                                                                                                            vm["swapping_mode"].empty()
		                                                                                                                            ? SWAPPINGMODE_DISABLED
		                                                                                                                            : string_to_enumerator<SwappingMode>(
				                                                                                                                            vm["swapping_mode"].as<std::string>())),
                                                                                                                            tracker_configuration(
		                                                                                                                            vm["tracker_configuration"].empty()
		                                                                                                                            ? TrackerConfigurationStringPresets::default_depth_only_extended_tracker_configuration
		                                                                                                                            : vm["tracker_configuration"].as<std::string>()),
                                                                                                                            origin(std::move(
		                                                                                                                            origin)) {}
void Configuration::SetFromPTree(const boost::property_tree::ptree& tree) {
	Configuration temporary_instance = BuildFromPTree(tree);
	*this = temporary_instance;
}
void Configuration::UpdateFromVariablesMap(const boost::program_options::variables_map& vm) {
	if (!vm["focus_coordinates"].empty())this->focus_coordinates = variables_map_to_vector<Vector3i>(vm, "focus_coordinates");
	if (!vm["record_reconstruction_video"].empty())this->record_reconstruction_video = vm["record_reconstruction_video"].as<bool>();
	if (!vm["record_inputs_in_reconstruction_video"].empty())this->record_inputs_in_reconstruction_video = vm["record_inputs_in_reconstruction_video"].as<bool>();
	this->general_voxel_volume_parameters.UpdateFromVariablesMap(vm);
	this->general_surfel_volume_parameters.UpdateFromVariablesMap(vm);
	this->specific_volume_parameters.UpdateFromVariablesMap(vm);
	this->slavcheva_parameters.UpdateFromVariablesMap(vm);
	this->slavcheva_switches.UpdateFromVariablesMap(vm);
	this->logging_settings.UpdateFromVariablesMap(vm);
	this->paths.UpdateFromVariablesMap(vm);
	this->non_rigid_tracking_parameters.UpdateFromVariablesMap(vm);
	if (!vm["create_meshing_engine"].empty())this->create_meshing_engine = vm["create_meshing_engine"].as<bool>();
	if (!vm["device_type"].empty())this->device_type = string_to_enumerator<MemoryDeviceType>(vm["device_type"].as<std::string>());
	if (!vm["use_approximate_raycast"].empty())this->use_approximate_raycast = vm["use_approximate_raycast"].as<bool>();
	if (!vm["use_threshold_filter"].empty())this->use_threshold_filter = vm["use_threshold_filter"].as<bool>();
	if (!vm["use_bilateral_filter"].empty())this->use_bilateral_filter = vm["use_bilateral_filter"].as<bool>();
	if (!vm["behavior_on_failure"].empty())this->behavior_on_failure = string_to_enumerator<FailureMode>(vm["behavior_on_failure"].as<std::string>());
	if (!vm["swapping_mode"].empty())this->swapping_mode = string_to_enumerator<SwappingMode>(vm["swapping_mode"].as<std::string>());
	if (!vm["tracker_configuration"].empty())this->tracker_configuration = vm["tracker_configuration"].as<std::string>();
}
Configuration Configuration::BuildFromPTree(const boost::property_tree::ptree& tree, const std::string& origin, const std::string& parse_path) {
	Configuration default_instance;
	boost::optional<Vector3i> focus_coordinates = ptree_to_optional_serializable_vector<Vector3i>(tree, "focus_coordinates");
	boost::optional<bool> record_reconstruction_video = tree.get_optional<bool>("record_reconstruction_video");
	boost::optional<bool> record_inputs_in_reconstruction_video = tree.get_optional<bool>("record_inputs_in_reconstruction_video");
	boost::optional<VoxelVolumeParameters> general_voxel_volume_parameters = ptree_to_optional_serializable_struct<VoxelVolumeParameters>(tree,
	                                                                                                                                      "general_voxel_volume_parameters",
	                                                                                                                                      origin);
	boost::optional<SurfelVolumeParameters> general_surfel_volume_parameters = ptree_to_optional_serializable_struct<SurfelVolumeParameters>(tree,
	                                                                                                                                         "general_surfel_volume_parameters",
	                                                                                                                                         origin);
	boost::optional<SpecificVolumeParameters> specific_volume_parameters = ptree_to_optional_serializable_struct<SpecificVolumeParameters>(tree,
	                                                                                                                                       "specific_volume_parameters",
	                                                                                                                                       origin);
	boost::optional<SlavchevaSurfaceTracker::Parameters> slavcheva_parameters = ptree_to_optional_serializable_struct<SlavchevaSurfaceTracker::Parameters>(
			tree, "slavcheva_parameters", origin);
	boost::optional<SlavchevaSurfaceTracker::Switches> slavcheva_switches = ptree_to_optional_serializable_struct<SlavchevaSurfaceTracker::Switches>(
			tree, "slavcheva_switches", origin);
	boost::optional<LoggingSettings> logging_settings = ptree_to_optional_serializable_struct<LoggingSettings>(tree, "logging_settings", origin);
	boost::optional<Paths> paths = ptree_to_optional_serializable_struct<Paths>(tree, "paths", origin);
	boost::optional<NonRigidTrackingParameters> non_rigid_tracking_parameters = ptree_to_optional_serializable_struct<NonRigidTrackingParameters>(
			tree, "non_rigid_tracking_parameters", origin);
	boost::optional<bool> create_meshing_engine = tree.get_optional<bool>("create_meshing_engine");
	boost::optional<MemoryDeviceType> device_type = ptree_to_optional_enumerator<MemoryDeviceType>(tree, "device_type");
	boost::optional<bool> use_approximate_raycast = tree.get_optional<bool>("use_approximate_raycast");
	boost::optional<bool> use_threshold_filter = tree.get_optional<bool>("use_threshold_filter");
	boost::optional<bool> use_bilateral_filter = tree.get_optional<bool>("use_bilateral_filter");
	boost::optional<FailureMode> behavior_on_failure = ptree_to_optional_enumerator<FailureMode>(tree, "behavior_on_failure");
	boost::optional<SwappingMode> swapping_mode = ptree_to_optional_enumerator<SwappingMode>(tree, "swapping_mode");
	boost::optional<std::string> tracker_configuration = tree.get_optional<std::string>("tracker_configuration");
	Configuration instance = {focus_coordinates ? focus_coordinates.get() : default_instance.focus_coordinates,
	                          record_reconstruction_video ? record_reconstruction_video.get() : default_instance.record_reconstruction_video,
	                          record_inputs_in_reconstruction_video ? record_inputs_in_reconstruction_video.get()
	                                                                : default_instance.record_inputs_in_reconstruction_video,
	                          general_voxel_volume_parameters ? general_voxel_volume_parameters.get()
	                                                          : default_instance.general_voxel_volume_parameters,
	                          general_surfel_volume_parameters ? general_surfel_volume_parameters.get()
	                                                           : default_instance.general_surfel_volume_parameters,
	                          specific_volume_parameters ? specific_volume_parameters.get() : default_instance.specific_volume_parameters,
	                          slavcheva_parameters ? slavcheva_parameters.get() : default_instance.slavcheva_parameters,
	                          slavcheva_switches ? slavcheva_switches.get() : default_instance.slavcheva_switches,
	                          logging_settings ? logging_settings.get() : default_instance.logging_settings,
	                          paths ? paths.get() : default_instance.paths,
	                          non_rigid_tracking_parameters ? non_rigid_tracking_parameters.get() : default_instance.non_rigid_tracking_parameters,
	                          create_meshing_engine ? create_meshing_engine.get() : default_instance.create_meshing_engine,
	                          device_type ? device_type.get() : default_instance.device_type,
	                          use_approximate_raycast ? use_approximate_raycast.get() : default_instance.use_approximate_raycast,
	                          use_threshold_filter ? use_threshold_filter.get() : default_instance.use_threshold_filter,
	                          use_bilateral_filter ? use_bilateral_filter.get() : default_instance.use_bilateral_filter,
	                          behavior_on_failure ? behavior_on_failure.get() : default_instance.behavior_on_failure,
	                          swapping_mode ? swapping_mode.get() : default_instance.swapping_mode,
	                          tracker_configuration ? tracker_configuration.get() : default_instance.tracker_configuration, origin};
	instance.source_tree = tree;
	return instance;
}
boost::property_tree::ptree Configuration::ToPTree(const std::string& _origin) const {
	boost::property_tree::ptree tree;
	tree.add_child("focus_coordinates", serializable_vector_to_ptree(focus_coordinates));
	tree.add("record_reconstruction_video", record_reconstruction_video);
	tree.add("record_inputs_in_reconstruction_video", record_inputs_in_reconstruction_video);
	tree.add_child("general_voxel_volume_parameters", general_voxel_volume_parameters.ToPTree(origin));
	tree.add_child("general_surfel_volume_parameters", general_surfel_volume_parameters.ToPTree(origin));
	tree.add_child("specific_volume_parameters", specific_volume_parameters.ToPTree(origin));
	tree.add_child("slavcheva_parameters", slavcheva_parameters.ToPTree(origin));
	tree.add_child("slavcheva_switches", slavcheva_switches.ToPTree(origin));
	tree.add_child("logging_settings", logging_settings.ToPTree(origin));
	tree.add_child("paths", paths.ToPTree(origin));
	tree.add_child("non_rigid_tracking_parameters", non_rigid_tracking_parameters.ToPTree(origin));
	tree.add("create_meshing_engine", create_meshing_engine);
	tree.add("device_type", enumerator_to_string(device_type));
	tree.add("use_approximate_raycast", use_approximate_raycast);
	tree.add("use_threshold_filter", use_threshold_filter);
	tree.add("use_bilateral_filter", use_bilateral_filter);
	tree.add("behavior_on_failure", enumerator_to_string(behavior_on_failure));
	tree.add("swapping_mode", enumerator_to_string(swapping_mode));
	tree.add("tracker_configuration", tracker_configuration);
	return tree;
}
bool operator==(const Configuration& instance1, const Configuration& instance2) {
	return instance1.focus_coordinates == instance2.focus_coordinates &&
	       instance1.record_reconstruction_video == instance2.record_reconstruction_video &&
	       instance1.record_inputs_in_reconstruction_video == instance2.record_inputs_in_reconstruction_video &&
	       instance1.general_voxel_volume_parameters == instance2.general_voxel_volume_parameters &&
	       instance1.general_surfel_volume_parameters == instance2.general_surfel_volume_parameters &&
	       instance1.specific_volume_parameters == instance2.specific_volume_parameters &&
	       instance1.slavcheva_parameters == instance2.slavcheva_parameters && instance1.slavcheva_switches == instance2.slavcheva_switches &&
	       instance1.logging_settings == instance2.logging_settings && instance1.paths == instance2.paths &&
	       instance1.non_rigid_tracking_parameters == instance2.non_rigid_tracking_parameters &&
	       instance1.create_meshing_engine == instance2.create_meshing_engine && instance1.device_type == instance2.device_type &&
	       instance1.use_approximate_raycast == instance2.use_approximate_raycast &&
	       instance1.use_threshold_filter == instance2.use_threshold_filter && instance1.use_bilateral_filter == instance2.use_bilateral_filter &&
	       instance1.behavior_on_failure == instance2.behavior_on_failure && instance1.swapping_mode == instance2.swapping_mode &&
	       instance1.tracker_configuration == instance2.tracker_configuration;
}
std::ostream& operator<<(std::ostream& out, const Configuration& instance) {
	boost::property_tree::ptree tree(instance.ToPTree());
	boost::property_tree::write_json_no_quotes(out, tree, true);
	return out;
}
void Configuration::AddToOptionsDescription(boost::program_options::options_description& od) {
	od.add_options()(generate_cli_argument_identifiers_snake_case("focus_coordinates", od).c_str(),
	                 boost::program_options::value<std::vector<Vector3i::value_type>>()->multitoken()->default_value(
			                 serializable_vector_to_std_vector(Vector3i(0))),
	                 "3d coordinates (integer) that specify the voxel to log additional diagnostic information about (and " "where to focus telemetry information records). " "Only effective for logging when verbosity_level is set to focus_spots (alt. VERBOSITY_FOCUS_SPOTS) or above.");
	od.add_options()(generate_cli_argument_identifiers_snake_case("record_reconstruction_video", od).c_str(),
	                 boost::program_options::value<bool>()->default_value(false),
	                 "Whether to record video of the canonical reconstruction during automatic run ");
	od.add_options()(generate_cli_argument_identifiers_snake_case("record_inputs_in_reconstruction_video", od).c_str(),
	                 boost::program_options::value<bool>()->default_value(false),
	                 "Whether to concatenate input frames to the canonical reconstruction video frames (only useful with  '--record_reconstruction_video')");
	general_voxel_volume_parameters.AddToOptionsDescription(od);
	general_surfel_volume_parameters.AddToOptionsDescription(od);
	specific_volume_parameters.AddToOptionsDescription(od);
	slavcheva_parameters.AddToOptionsDescription(od);
	slavcheva_switches.AddToOptionsDescription(od);
	logging_settings.AddToOptionsDescription(od);
	paths.AddToOptionsDescription(od);
	non_rigid_tracking_parameters.AddToOptionsDescription(od);
	od.add_options()(generate_cli_argument_identifiers_snake_case("create_meshing_engine", od).c_str(),
	                 boost::program_options::value<bool>()->default_value(true),
	                 "Create all the things required for marching cubes and mesh extraction (uses lots of additional memory)");
	od.add_options()(generate_cli_argument_identifiers_snake_case("device_type", od).c_str(),
	                 boost::program_options::value<std::string>()->default_value(enumerator_to_string(MEMORYDEVICE_CUDA)),
	                 (std::string("\"Type of device to use, i.e. CPU/GPU/Metal\"") + enumerator_bracketed_list<MemoryDeviceType>()).c_str());
	od.add_options()(generate_cli_argument_identifiers_snake_case("use_approximate_raycast", od).c_str(),
	                 boost::program_options::value<bool>()->default_value(false), "Enables or disables approximate raycast.");
	od.add_options()(generate_cli_argument_identifiers_snake_case("use_threshold_filter", od).c_str(),
	                 boost::program_options::value<bool>()->default_value(false),
	                 "Enables or disables threshold filtering, i.e. filtering out pixels whose difference from their neighbors exceeds a certain threshold");
	od.add_options()(generate_cli_argument_identifiers_snake_case("use_bilateral_filter", od).c_str(),
	                 boost::program_options::value<bool>()->default_value(false), "Enables or disables bilateral filtering on depth input images.");
	od.add_options()(generate_cli_argument_identifiers_snake_case("behavior_on_failure", od).c_str(),
	                 boost::program_options::value<std::string>()->default_value(enumerator_to_string(FAILUREMODE_IGNORE)), (std::string(
					"\"What to do on tracker failure: ignore, relocalize or stop integration - not supported in loop closure or dynamic libmode\"") +
	                                                                                                                         enumerator_bracketed_list<FailureMode>()).c_str());
	od.add_options()(generate_cli_argument_identifiers_snake_case("swapping_mode", od).c_str(),
	                 boost::program_options::value<std::string>()->default_value(enumerator_to_string(SWAPPINGMODE_DISABLED)), (std::string(
					"\"Determines how swapping works: disabled, fully enabled (still with dragons) and delete what's not visible - not supported in loop closure version\"") +
	                                                                                                                            enumerator_bracketed_list<SwappingMode>()).c_str());
	od.add_options()(generate_cli_argument_identifiers_snake_case("tracker_configuration", od).c_str(),
	                 boost::program_options::value<std::string>()->default_value(
			                 TrackerConfigurationStringPresets::default_depth_only_extended_tracker_configuration),
	                 "Tracker configuration. (Better description still needs to be provided for this, already in TODO / issues)");
}
// =@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@

// region ===================================== TRACKING PRESET DEFINITIONS ============================================

const std::string TrackerConfigurationStringPresets::default_ICP_tracker_configuration =
		"type=icp,levels=rrrbb,minstep=1e-3,"
		"outlierC=0.01,outlierF=0.002,"
		"numiterC=10,numiterF=2,failureDec=5.0"; // 5 for normal, 20 for loop closure
const std::string TrackerConfigurationStringPresets::default_ICP_tracker_configuration_loop_closure =
		"type=icp,levels=rrrbb,minstep=1e-3,"
		"outlierC=0.01,outlierF=0.002,"
		"numiterC=10,numiterF=2,failureDec=20.0"; // 5 for normal, 20 for loop closure
const std::string TrackerConfigurationStringPresets::default_depth_only_extended_tracker_configuration =
		"type=extended,levels=rrbb,useDepth=1,minstep=1e-4,"
		"outlierSpaceC=0.1,outlierSpaceF=0.004,"
		"numiterC=20,numiterF=50,tukeyCutOff=8,"
		"framesToSkip=20,framesToWeight=50,failureDec=20.0";
const std::string TrackerConfigurationStringPresets::default_intensity_depth_extended_tracker_configuration =
		"type=extended,levels=bbb,useDepth=1,useColour=1,"
		"colourWeight=0.3,minstep=1e-4,"
		"outlierColourC=0.175,outlierColourF=0.005,"
		"outlierSpaceC=0.1,outlierSpaceF=0.004,"
		"numiterC=20,numiterF=50,tukeyCutOff=8,"
		"framesToSkip=20,framesToWeight=50,failureDec=20.0";
const std::string TrackerConfigurationStringPresets::default_color_only_tracker_configuration =
		"type=rgb,levels=rrbb";
const std::string TrackerConfigurationStringPresets::default_IMU_ICP_tracker_configuration =
		"type=imuicp,levels=tb,minstep=1e-3,outlierC=0.01,"
		"outlierF=0.005,numiterC=4,numiterF=2";
const std::string TrackerConfigurationStringPresets::default_IMU_extended_tracker_configuration =
		"type=extendedimu,levels=ttb,minstep=5e-4,outlierSpaceC=0.1,"
		"outlierSpaceF=0.004,numiterC=20,numiterF=5,tukeyCutOff=8,"
		"framesToSkip=20,framesToWeight=50,failureDec=20.0";

const std::string TrackerConfigurationStringPresets::default_surfel_tracker_configuration =
		"extended,levels=rrbb,minstep=1e-4,outlierSpaceC=0.1,outlierSpaceF=0.004,"
		"numiterC=20,numiterF=20,tukeyCutOff=8,framesToSkip=0,framesToWeight=1,failureDec=20.0";

// endregion ===========================================================================================================

// region ==== CONFIGURATION SINGLETON HANDLING ========================================================================

Configuration instance;

Configuration& Get() {
	return instance;
}

template<>
typename VoxelBlockHash::InitializationParameters ForVolumeRole<VoxelBlockHash>(VolumeRole role) {
	switch (role) {
		default:
		case VOLUME_CANONICAL:
			return instance.specific_volume_parameters.hash.canonical;
		case VOLUME_LIVE:
			return instance.specific_volume_parameters.hash.live;
		case VOLUME_WARP:
			return instance.specific_volume_parameters.hash.warp;
	}
}

template<>
typename PlainVoxelArray::InitializationParameters ForVolumeRole<PlainVoxelArray>(VolumeRole role) {
	switch (role) {
		default:
		case VOLUME_CANONICAL:
			return instance.specific_volume_parameters.array.canonical;
		case VOLUME_LIVE:
			return instance.specific_volume_parameters.array.live;
		case VOLUME_WARP:
			return instance.specific_volume_parameters.array.warp;
	}
}

static void AddAllDeferrableStructsFromVariablesMap(const po::variables_map& vm) {
	MainEngineSettings main_engine_settings(vm);
	AddDeferrableToTargetTree(instance.source_tree, main_engine_settings);
	TelemetrySettings telemetry_settings(vm);
	AddDeferrableToTargetTree(instance.source_tree, telemetry_settings);
	IndexingSettings indexing_settings(vm);
	AddDeferrableToTargetTree(instance.source_tree, indexing_settings);
	RenderingSettings rendering_settings(vm);
	AddDeferrableToTargetTree(instance.source_tree, rendering_settings);
	AutomaticRunSettings automatic_run_settings(vm);
	AddDeferrableToTargetTree(instance.source_tree, automatic_run_settings);
}

void LoadConfigurationFromVariableMap(const po::variables_map& vm) {
	instance = Configuration(vm);
	AddAllDeferrableStructsFromVariablesMap(vm);
}

void LoadDefault() {
	instance = Configuration();
}

namespace fs = boost::filesystem;

void LoadConfigurationFromJSONFile(const std::string& path) {
	pt::ptree tree;
	pt::read_json(path, tree);
	instance = Configuration::BuildFromPTree(tree, path);
}

static void UpdateAllDeferrableStructsFromVariablesMap(const po::variables_map& vm) {
	auto main_engine_settings = BuildDeferrableFromParentIfPresent<MainEngineSettings>(instance);
	main_engine_settings.UpdateFromVariablesMap(vm);
	AddDeferrableToTargetTree(instance.source_tree, main_engine_settings);
	auto telemetry_settings = BuildDeferrableFromParentIfPresent<TelemetrySettings>(instance);
	telemetry_settings.UpdateFromVariablesMap(vm);
	AddDeferrableToTargetTree(instance.source_tree, telemetry_settings);
	auto indexing_settings = BuildDeferrableFromParentIfPresent<IndexingSettings>(instance);
	indexing_settings.UpdateFromVariablesMap(vm);
	AddDeferrableToTargetTree(instance.source_tree, indexing_settings);
	auto rendering_settings = BuildDeferrableFromParentIfPresent<RenderingSettings>(instance);
	rendering_settings.UpdateFromVariablesMap(vm);
	AddDeferrableToTargetTree(instance.source_tree, rendering_settings);
	auto automatic_run_settings = BuildDeferrableFromParentIfPresent<AutomaticRunSettings>(instance);
	automatic_run_settings.UpdateFromVariablesMap(vm);
	AddDeferrableToTargetTree(instance.source_tree, automatic_run_settings);
}

void UpdateConfigurationFromVariableMap(const po::variables_map& vm) {
	UpdateAllDeferrableStructsFromVariablesMap(vm);
	instance.UpdateFromVariablesMap(vm);
}

static void AddAllDeferrableStructsFromSourceToTargetRootConfiguration(pt::ptree& target_tree, const pt::ptree& origin_tree, std::string origin) {
	AddDeferrableFromSourceToTargetTree<MainEngineSettings>(target_tree, origin_tree, origin);
	AddDeferrableFromSourceToTargetTree<TelemetrySettings>(target_tree, origin_tree, origin);
	AddDeferrableFromSourceToTargetTree<IndexingSettings>(target_tree, origin_tree, origin);
	AddDeferrableFromSourceToTargetTree<RenderingSettings>(target_tree, origin_tree, origin);
	AddDeferrableFromSourceToTargetTree<AutomaticRunSettings>(target_tree, origin_tree, origin);
}

void SaveConfigurationToJSONFile(const std::string& path) {
	pt::ptree target_tree = instance.ToPTree(path);
	AddAllDeferrableStructsFromSourceToTargetRootConfiguration(target_tree, instance.source_tree, instance.origin);
	pt::write_json_no_quotes(path, target_tree, true);
}

void SaveConfigurationToJSONFile(const std::string& path, const Configuration& configuration) {
	pt::ptree target_tree = configuration.ToPTree(path);
	AddAllDeferrableStructsFromSourceToTargetRootConfiguration(target_tree, configuration.source_tree, configuration.origin);
	pt::write_json_no_quotes(path, target_tree, true);
}
// endregion ===========================================================================================================
} // namespace ITMLib::configuration
} // namespace ITMLib