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

//ITMLib
#include "../ITMLib/Utils/Configuration/Configuration.h"

//test_utilities
#include "TestUtilities/TestUtilities.h"

namespace pt = boost::property_tree;

using namespace ITMLib;
using namespace ITMLib::configuration;
using namespace test_utilities;

configuration::Configuration GenerateDefaultSnoopyConfiguration();

BOOST_AUTO_TEST_CASE(ConfigurationTest) {
	configuration::Configuration default_configuration;

	configuration::Configuration configuration1 = GenerateChangedUpConfiguration();

#ifdef COMPILE_WITHOUT_CUDA
	configuration::load_configuration_from_json_file("TestData/configuration/default_config_cpu.json");
#else
	configuration::load_configuration_from_json_file("TestData/configuration/default_config_cuda.json");
#endif

	BOOST_REQUIRE_EQUAL(default_configuration.general_voxel_volume_parameters,
	                    configuration::get().general_voxel_volume_parameters);
	BOOST_REQUIRE_EQUAL(default_configuration.general_surfel_volume_parameters,
	                    configuration::get().general_surfel_volume_parameters);
	BOOST_REQUIRE_EQUAL(default_configuration.slavcheva_parameters, configuration::get().slavcheva_parameters);
	BOOST_REQUIRE_EQUAL(default_configuration.slavcheva_switches, configuration::get().slavcheva_switches);
	BOOST_REQUIRE_EQUAL(default_configuration.telemetry_settings, configuration::get().telemetry_settings);
	BOOST_REQUIRE_EQUAL(default_configuration.paths, configuration::get().paths);
	BOOST_REQUIRE_EQUAL(default_configuration.automatic_run_settings, configuration::get().automatic_run_settings);
	BOOST_REQUIRE_EQUAL(default_configuration.non_rigid_tracking_parameters,
	                    configuration::get().non_rigid_tracking_parameters);
	BOOST_REQUIRE_EQUAL(default_configuration, configuration::get());

	configuration::load_configuration_from_json_file("TestData/configuration/config1.json");
	BOOST_REQUIRE_EQUAL(configuration1.general_voxel_volume_parameters,
	                    configuration::get().general_voxel_volume_parameters);
	BOOST_REQUIRE_EQUAL(configuration1.general_surfel_volume_parameters,
	                    configuration::get().general_surfel_volume_parameters);
	BOOST_REQUIRE_EQUAL(configuration1.slavcheva_parameters, configuration::get().slavcheva_parameters);
	BOOST_REQUIRE_EQUAL(configuration1.slavcheva_switches, configuration::get().slavcheva_switches);
	BOOST_REQUIRE_EQUAL(configuration1.telemetry_settings, configuration::get().telemetry_settings);
	BOOST_REQUIRE_EQUAL(configuration1.paths, configuration::get().paths);
	BOOST_REQUIRE_EQUAL(configuration1.automatic_run_settings, configuration::get().automatic_run_settings);
	BOOST_REQUIRE_EQUAL(configuration1.non_rigid_tracking_parameters,
	                    configuration::get().non_rigid_tracking_parameters);
	BOOST_REQUIRE_EQUAL(configuration1, configuration::get());
	configuration::save_configuration_to_json_file("TestData/configuration/config2.json", configuration1);
	configuration::load_configuration_from_json_file("TestData/configuration/config2.json");
	BOOST_REQUIRE_EQUAL(configuration1.general_voxel_volume_parameters,
	                    configuration::get().general_voxel_volume_parameters);
	BOOST_REQUIRE_EQUAL(configuration1.general_surfel_volume_parameters,
	                    configuration::get().general_surfel_volume_parameters);
	BOOST_REQUIRE_EQUAL(configuration1.slavcheva_parameters, configuration::get().slavcheva_parameters);
	BOOST_REQUIRE_EQUAL(configuration1.slavcheva_switches, configuration::get().slavcheva_switches);
	BOOST_REQUIRE_EQUAL(configuration1.telemetry_settings, configuration::get().telemetry_settings);
	BOOST_REQUIRE_EQUAL(configuration1.paths, configuration::get().paths);
	BOOST_REQUIRE_EQUAL(configuration1.automatic_run_settings, configuration::get().automatic_run_settings);
	BOOST_REQUIRE_EQUAL(configuration1.non_rigid_tracking_parameters,
	                    configuration::get().non_rigid_tracking_parameters);
	BOOST_REQUIRE_EQUAL(configuration1, configuration::get());


}
