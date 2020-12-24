//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 12/24/20.
//  Copyright (c) 2020 Gregory Kramida
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
// === local ===
#include "GenerateTestConfigurations.h"

// === ITMLib ===
#include "../../ITMLib/Utils/Logging/Logging.h"
#include "../../ITMLib/Utils/Configuration/Configuration.h"
#include "../../ITMLib/Utils/Configuration/AutomaticRunSettings.h"
#include "../../ITMLib/Engines/Main/MainEngineSettings.h"
#include "../../ITMLib/Engines/Telemetry/TelemetrySettings.h"
#include "../../ITMLib/Engines/DepthFusion/DepthFusionSettings.h"
#include "../../ITMLib/Engines/VolumeFusion/VolumeFusionSettings.h"
#include "../../ITMLib/Engines/Rendering/RenderingSettings.h"
#include "../../ITMLib/Engines/Indexing/IndexingSettings.h"
#include "../../ITMLib/Engines/LevelSetAlignment/Interface/LevelSetAlignmentParameters.h"

// === Test Utilities ==
#include "../TestUtilities/TestUtilities.h"

using namespace ITMLib;
using namespace test;

configuration::Configuration GenerateDefaultSnoopyConfiguration() {
	using namespace configuration;
	configuration::Configuration
			default_snoopy_configuration(
			Vector3i(0, 0, 0),
			Vector2i(0, 0),
			true,
			true,
			VoxelVolumeParameters(0.004, 0.2, 1.0, 0.04, 100, false, 1.0f),
			SurfelVolumeParameters(),
			SpecificVolumeParameters(
					ArrayVolumeParameters(),
					HashVolumeParameters(
							VoxelBlockHashParameters(0x40000, 0x20000),
							VoxelBlockHashParameters(0x20000, 0x20000),
							VoxelBlockHashParameters(0x20000, 0x20000)
					)
			),
			LoggingSettings(VERBOSITY_WARNING,
			                true,
			                true,
			                true,
			                false,
			                false,
			                false,
			                true,
			                false,
			                false,
			                false,
			                false,
			                false,
			                false),
			Paths("<CONFIGURATION_DIRECTORY>",
			      "<CONFIGURATION_DIRECTORY>/snoopy_calib.txt",
			      "", "", "",
			      "<CONFIGURATION_DIRECTORY>/frames/color_%06i.png",
			      "<CONFIGURATION_DIRECTORY>/frames/depth_%06i.png",
			      "<CONFIGURATION_DIRECTORY>/frames/mask_%06i.png",
			      ""),
			true,
			MEMORYDEVICE_CUDA,
			false,
			false,
			false,
			configuration::FAILUREMODE_IGNORE,
			configuration::SWAPPINGMODE_DISABLED,
			configuration::TrackerConfigurationStringPresets::default_intensity_depth_extended_tracker_configuration
	);
	default_snoopy_configuration.source_tree = default_snoopy_configuration.ToPTree();
	MainEngineSettings default_snoopy_main_engine_settings(true, LIBMODE_DYNAMIC, INDEX_HASH, false, true);
	TelemetrySettings default_snoopy_telemetry_settings;
	IndexingSettings default_snoopy_indexing_settings;
	RenderingSettings default_snoopy_rendering_settings;
	AutomaticRunSettings default_snoopy_automatic_run_settings(716, 16, false, false, false, false, false);
	LevelSetAlignmentParameters default_snoopy_level_set_evolution_parameters(
			ExecutionMode::OPTIMIZED,
			LevelSetAlignmentWeights(
					0.2f,
					0.1f,
					2.0f,
					0.2f,
					0.2f,
					1e-5f),
			LevelSetAlignmentSwitches(
					true, false, true, false, true
			),
			LevelSetAlignmentTerminationConditions(MAXIMUM, 300, 10, 1e-06)
	);
	VolumeFusionSettings default_snoopy_volume_fusion_settings;
	DepthFusionSettings default_snoopy_depth_fusion_settings;

	AddDeferrableToSourceTree(default_snoopy_configuration, default_snoopy_main_engine_settings);
	AddDeferrableToSourceTree(default_snoopy_configuration, default_snoopy_telemetry_settings);
	AddDeferrableToSourceTree(default_snoopy_configuration, default_snoopy_indexing_settings);
	AddDeferrableToSourceTree(default_snoopy_configuration, default_snoopy_rendering_settings);
	AddDeferrableToSourceTree(default_snoopy_configuration, default_snoopy_automatic_run_settings);
	AddDeferrableToSourceTree(default_snoopy_configuration, default_snoopy_level_set_evolution_parameters);
	AddDeferrableToSourceTree(default_snoopy_configuration, default_snoopy_volume_fusion_settings);
	AddDeferrableToSourceTree(default_snoopy_configuration, default_snoopy_depth_fusion_settings);

	return default_snoopy_configuration;
}

void GenerateConfigurationTestData() {
	using namespace configuration;
	configuration::Configuration changed_up_configuration = GenerateChangedUpConfiguration();
	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(),
	               "Generating configuration test data ... ");
	configuration::Configuration default_snoopy_configuration = GenerateDefaultSnoopyConfiguration();
	test::ConstructGeneratedConfigurationDirectoryIfMissing();
	configuration::SaveConfigurationToJSONFile(STATIC_TEST_DATA_PREFIX
	"../Files/infinitam_snoopy_config.json",
			default_snoopy_configuration);
	configuration::Configuration default_configuration;
	default_configuration.device_type = MEMORYDEVICE_CPU;
	configuration::SaveConfigurationToJSONFile(GENERATED_TEST_DATA_PREFIX
	"TestData/configuration/default_config_cpu.json",
			default_configuration);
	default_configuration.device_type = MEMORYDEVICE_CUDA;
	configuration::SaveConfigurationToJSONFile(GENERATED_TEST_DATA_PREFIX
	"TestData/configuration/default_config_cuda.json",
			default_configuration);
	configuration::SaveConfigurationToJSONFile(GENERATED_TEST_DATA_PREFIX
	"TestData/configuration/config1.json", changed_up_configuration);
}