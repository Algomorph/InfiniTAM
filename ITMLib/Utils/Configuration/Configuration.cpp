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

DEFINE_SERIALIZABLE_STRUCT(ARRAY_VOLUME_PARAMETERS_STRUCT_DESCRIPTION);

DEFINE_SERIALIZABLE_STRUCT(HASH_VOLUME_PARAMETERS_STRUCT_DESCRIPTION);

DEFINE_SERIALIZABLE_STRUCT(SPECIFIC_VOLUME_PARAMETERS_STRUCT_DESCRIPTION);

DEFINE_SERIALIZABLE_STRUCT(CONFIGURATION_STRUCT_DESCRIPTION);

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

void CompileOptionDescription(po::options_description& od){
	Configuration::AddToOptionsDescription(od);
	MainEngineSettings::AddToOptionsDescription(od);
	TelemetrySettings::AddToOptionsDescription(od);
	IndexingSettings::AddToOptionsDescription(od);
	RenderingSettings::AddToOptionsDescription(od);
	AutomaticRunSettings::AddToOptionsDescription(od);
}

static void AddAllDeferrableStructsFromVariablesMap(const po::variables_map& vm) {
	MainEngineSettings::BuildDeferredFromVariablesMap(instance.source_tree, vm);
	TelemetrySettings::BuildDeferredFromVariablesMap(instance.source_tree, vm);
	IndexingSettings::BuildDeferredFromVariablesMap(instance.source_tree, vm);
	RenderingSettings::BuildDeferredFromVariablesMap(instance.source_tree, vm);
	AutomaticRunSettings::BuildDeferredFromVariablesMap(instance.source_tree, vm);
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
	MainEngineSettings::UpdateDeferredFromVariablesMap(instance.source_tree, vm, instance.origin);
	TelemetrySettings::UpdateDeferredFromVariablesMap(instance.source_tree, vm, instance.origin);
	IndexingSettings::UpdateDeferredFromVariablesMap(instance.source_tree, vm, instance.origin);
	RenderingSettings::UpdateDeferredFromVariablesMap(instance.source_tree, vm, instance.origin);
	AutomaticRunSettings::UpdateDeferredFromVariablesMap(instance.source_tree, vm, instance.origin);
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
	// because parent serializable struct has no information
	// about deferrables it contains, deferrables don't automatically
	// get serialized to the tree output from their parent.
	// We have to manually add them from source tree parsed
	// from the original file and add them to the target tree.
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