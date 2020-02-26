//  ================================================================
//  Created by Gregory Kramida on 11/13/19.
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

#define BOOST_TEST_MODULE Configuration
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//boost
#include <boost/test/unit_test.hpp>

//local
#include "../ITMLib/Utils/Configuration.h"

namespace pt = boost::property_tree;

using namespace ITMLib;
using namespace ITMLib::configuration;

configuration::Configuration generate_default_snoopy_configuration();

//#define GENERATE_TEST_DATA
#ifdef _MSC_VER
#define SOURCE_DIRECTORY "../../../"
#else
#define SOURCE_DIRECTORY "../../"
#endif
BOOST_AUTO_TEST_CASE(ConfigurationTest) {
	configuration::Configuration default_configuration;

	configuration::Configuration configuration1(
			VoxelVolumeParameters(0.005, 0.12, 4.12, 0.05, 200, true, 1.2f),
			SurfelVolumeParameters(0.4f, 0.5f, static_cast<float>(22 * M_PI / 180), 0.008f, 0.0003f, 3.4f, 26.0f, 5,
			                       1.1f, 4.5f, 21, 300, false, false),
			SpecificVolumeParameters(
					ArrayVolumeParameters(),
					HashVolumeParameters(
							VoxelBlockHash::VoxelBlockHashParameters(0x40000, 0x20000),
							VoxelBlockHash::VoxelBlockHashParameters(0x20000, 0x20000),
							VoxelBlockHash::VoxelBlockHashParameters(0x20000, 0x20000))
			),
			SlavchevaSurfaceTracker::Parameters(0.11f, 0.09f, 2.0f, 0.3f, 0.1f, 1e-6f),
			SlavchevaSurfaceTracker::Switches(false, true, false, true, false),
			TelemetrySettings(Vector3i(20, 23, 0), true, true, true),
			Paths("TestData/output1",
			      "TestData/calib_file1.txt",
			      "", "", "",
			      "TestData/frame_color_%%06i.png",
			      "TestData/frame_depth_%%06i.png",
			      "TestData/frame_mask_%%06i.png",
			      ""),
			AutomaticRunSettings(50, 16, true, true, true),
			NonRigidTrackingParameters(ITMLib::TRACKER_SLAVCHEVA_DIAGNOSTIC, 300, 0.0002f, 0.4f),
			true,
			false,
			MEMORYDEVICE_CPU,
			true,
			true,
			true,
			false,
			configuration::FAILUREMODE_RELOCALIZE,
			configuration::SWAPPINGMODE_ENABLED,
			configuration::LIBMODE_BASIC,
			configuration::INDEX_ARRAY,
			configuration::VERBOSITY_SILENT,
			"type=rgb,levels=rrbb"
	);
#ifdef GENERATE_TEST_DATA
	std::cout << "Saving test data..." << std::endl;
	configuration::Configuration default_snoopy_configuration = generate_default_snoopy_configuration();
	configuration::save_configuration_to_json_file(SOURCE_DIRECTORY "Files/infinitam_snoopy_config.json", default_snoopy_configuration);
	default_configuration.device_type = MEMORYDEVICE_CPU;
	configuration::save_configuration_to_json_file(SOURCE_DIRECTORY "Tests/TestData/default_config_cpu.json", default_configuration);
	default_configuration.device_type = MEMORYDEVICE_CUDA;
	configuration::save_configuration_to_json_file(SOURCE_DIRECTORY "Tests/TestData/default_config_cuda.json", default_configuration);
	configuration::save_configuration_to_json_file(SOURCE_DIRECTORY "Tests/TestData/config1.json", configuration1);
#endif

#ifdef COMPILE_WITHOUT_CUDA
	configuration::load_configuration_from_json_file("TestData/default_config_cpu.json");
#else
	configuration::load_configuration_from_json_file("TestData/default_config_cuda.json");
#endif

	BOOST_REQUIRE_EQUAL(default_configuration.general_voxel_volume_parameters, configuration::get().general_voxel_volume_parameters);
	BOOST_REQUIRE_EQUAL(default_configuration.general_surfel_volume_parameters, configuration::get().general_surfel_volume_parameters);
	BOOST_REQUIRE_EQUAL(default_configuration.slavcheva_parameters, configuration::get().slavcheva_parameters);
	BOOST_REQUIRE_EQUAL(default_configuration.slavcheva_switches, configuration::get().slavcheva_switches);
	BOOST_REQUIRE_EQUAL(default_configuration.telemetry_settings, configuration::get().telemetry_settings);
	BOOST_REQUIRE_EQUAL(default_configuration.paths, configuration::get().paths);
	BOOST_REQUIRE_EQUAL(default_configuration.automatic_run_settings, configuration::get().automatic_run_settings);
	BOOST_REQUIRE_EQUAL(default_configuration.non_rigid_tracking_parameters, configuration::get().non_rigid_tracking_parameters);
	BOOST_REQUIRE_EQUAL(default_configuration, configuration::get());

	configuration::load_configuration_from_json_file("TestData/config1.json");
	BOOST_REQUIRE_EQUAL(configuration1.general_voxel_volume_parameters, configuration::get().general_voxel_volume_parameters);
	BOOST_REQUIRE_EQUAL(configuration1.general_surfel_volume_parameters, configuration::get().general_surfel_volume_parameters);
	BOOST_REQUIRE_EQUAL(configuration1.slavcheva_parameters, configuration::get().slavcheva_parameters);
	BOOST_REQUIRE_EQUAL(configuration1.slavcheva_switches, configuration::get().slavcheva_switches);
	BOOST_REQUIRE_EQUAL(configuration1.telemetry_settings, configuration::get().telemetry_settings);
	BOOST_REQUIRE_EQUAL(configuration1.paths, configuration::get().paths);
	BOOST_REQUIRE_EQUAL(configuration1.automatic_run_settings, configuration::get().automatic_run_settings);
	BOOST_REQUIRE_EQUAL(configuration1.non_rigid_tracking_parameters, configuration::get().non_rigid_tracking_parameters);
	BOOST_REQUIRE_EQUAL(configuration1, configuration::get());
	configuration::save_configuration_to_json_file("TestData/config2.json", configuration1);
	configuration::load_configuration_from_json_file("TestData/config2.json");
	BOOST_REQUIRE_EQUAL(configuration1.general_voxel_volume_parameters, configuration::get().general_voxel_volume_parameters);
	BOOST_REQUIRE_EQUAL(configuration1.general_surfel_volume_parameters, configuration::get().general_surfel_volume_parameters);
	BOOST_REQUIRE_EQUAL(configuration1.slavcheva_parameters, configuration::get().slavcheva_parameters);
	BOOST_REQUIRE_EQUAL(configuration1.slavcheva_switches, configuration::get().slavcheva_switches);
	BOOST_REQUIRE_EQUAL(configuration1.telemetry_settings, configuration::get().telemetry_settings);
	BOOST_REQUIRE_EQUAL(configuration1.paths, configuration::get().paths);
	BOOST_REQUIRE_EQUAL(configuration1.automatic_run_settings, configuration::get().automatic_run_settings);
	BOOST_REQUIRE_EQUAL(configuration1.non_rigid_tracking_parameters, configuration::get().non_rigid_tracking_parameters);
	BOOST_REQUIRE_EQUAL(configuration1, configuration::get());


}

configuration::Configuration generate_default_snoopy_configuration(){
	configuration::Configuration default_snoopy_configuration(
			VoxelVolumeParameters(0.004, 0.2, 3.0, 0.04, 100, false, 1.0f),
			SurfelVolumeParameters(),
			SpecificVolumeParameters(
					ArrayVolumeParameters(),
					HashVolumeParameters(
							VoxelBlockHash::VoxelBlockHashParameters(0x40000, 0x20000),
							VoxelBlockHash::VoxelBlockHashParameters(0x20000, 0x20000),
							VoxelBlockHash::VoxelBlockHashParameters(0x20000, 0x20000))
			),
			SlavchevaSurfaceTracker::Parameters(
					0.2f,
					0.1f,
					2.0f,
					0.2f,
					0.2f,
					1e-5f),
			SlavchevaSurfaceTracker::Switches(true, false, true, false, true),
			TelemetrySettings(Vector3i(0), true, true, true),
			Paths("<CONFIGURATION_DIRECTORY>",
			      "<CONFIGURATION_DIRECTORY>/snoopy_calib.txt",
			      "", "", "",
			      "<CONFIGURATION_DIRECTORY>/frames/color_%06i.png",
			      "<CONFIGURATION_DIRECTORY>/frames/depth_%06i.png",
			      "<CONFIGURATION_DIRECTORY>/frames/omask_%06i.png",
			      ""),
			AutomaticRunSettings(50, 16, false, false, false),
			NonRigidTrackingParameters(
					ITMLib::TRACKER_SLAVCHEVA_OPTIMIZED,
					300,
					1e-06,
					0.5f),
			false,
			true,
			MEMORYDEVICE_CUDA,
			false,
			false,
			false,
			true,
			configuration::FAILUREMODE_IGNORE,
			configuration::SWAPPINGMODE_DISABLED,
			configuration::LIBMODE_DYNAMIC,
			configuration::INDEX_HASH,
			configuration::VERBOSITY_SILENT,
			configuration::TrackerConfigurationStringPresets::default_intensity_depth_extended_tracker_configuration
	);
	return default_snoopy_configuration;
}