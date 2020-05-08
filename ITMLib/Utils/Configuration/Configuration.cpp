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
#include "Configuration.h"
#include "../FileIO/JSON_Utilities.h"

using namespace ITMLib::configuration;

// *** serializable enum definitions ***
DEFINE_SERIALIZABLE_ENUM(VERBOSITY_LEVEL_ENUM_DESCRIPTION)

DEFINE_SERIALIZABLE_ENUM(FAILUREMODE_ENUM_DESCRIPTION)

DEFINE_SERIALIZABLE_ENUM(SWAPPINGMODE_ENUM_DESCRIPTION)

DEFINE_SERIALIZABLE_ENUM(LIBMODE_ENUM_DESCRIPTION)

DEFINE_SERIALIZABLE_ENUM(INDEXING_METHOD_ENUM_DESCRIPTION)

// defined in other headers or externally
DEFINE_SERIALIZABLE_ENUM(MemoryDeviceType,
                         (MEMORYDEVICE_CPU, "cpu", "CPU", "MEMORYDEVICE_CPU"),
                         (MEMORYDEVICE_CUDA, "cuda", "CUDA", "MEMORYDEVICE_CUDA"),
                         (MEMORYDEVICE_METAL, "metal", "METAL", "MEMORYDEVICE_METAL")
)

DEFINE_SERIALIZABLE_ENUM(VOLUME_ROLE_ENUM_DESCRIPTION)

namespace ITMLib {
namespace configuration {

// =====
// *** serializable struct definitions ***
DEFINE_SERIALIZABLE_STRUCT(PATHS_STRUCT_DESCRIPTION)

DEFINE_SERIALIZABLE_STRUCT(TELEMETRY_SETTINGS_STRUCT_DESCRIPTION)

DEFINE_SERIALIZABLE_STRUCT(AUTOMATIC_RUN_SETTINGS_STRUCT_DESCRIPTION)

DEFINE_SERIALIZABLE_STRUCT(ARRAY_VOLUME_PARAMETERS_STRUCT_DESCRIPTION)

DEFINE_SERIALIZABLE_STRUCT(HASH_VOLUME_PARAMETERS_STRUCT_DESCRIPTION)

DEFINE_SERIALIZABLE_STRUCT(SPECIFIC_VOLUME_PARAMETERS_STRUCT_DESCRIPTION)

//#pragma message BOOST_PP_STRINGIZE((DEFINE_SERIALIZABLE_STRUCT(CONFIGURATION_STRUCT_DESCRIPTION)))

// * * *
//#pragma message BOOST_PP_STRINGIZE((DEFINE_SERIALIZABLE_STRUCT(CONFIGURATION_STRUCT_DESCRIPTION)))

DEFINE_SERIALIZABLE_STRUCT(CONFIGURATION_STRUCT_DESCRIPTION)

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

Configuration& get() {
	return instance;
}

template<>
typename VoxelBlockHash::InitializationParameters for_volume_role<VoxelBlockHash>(VolumeRole role){
	switch(role){
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
typename PlainVoxelArray::InitializationParameters for_volume_role<PlainVoxelArray>(VolumeRole role){
	switch(role){
		default:
		case VOLUME_CANONICAL:
			return instance.specific_volume_parameters.array.canonical;
		case VOLUME_LIVE:
			return instance.specific_volume_parameters.array.live;
		case VOLUME_WARP:
			return instance.specific_volume_parameters.array.warp;
	}
}

void load_configuration_from_variable_map(const po::variables_map& vm) {
	instance = Configuration(vm);
}

void load_default() {
	instance = Configuration();
}

namespace fs = boost::filesystem;

void load_configuration_from_json_file(const std::string& path) {
	pt::ptree tree;
	pt::read_json(path, tree);
	instance = Configuration::BuildFromPTree(tree, path);
}

void save_configuration_to_json_file(const std::string& path) {
	pt::write_json_no_quotes(path, instance.ToPTree(path), true);
}

void save_configuration_to_json_file(const std::string& path, const Configuration& configuration) {
	pt::write_json_no_quotes(path, configuration.ToPTree(path), true);
}
// endregion ===========================================================================================================
} // namespace ITMLib::configuration
} // namespace ITMLib