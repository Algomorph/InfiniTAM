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

//test_utilities
#include "TestUtilities/TestUtilities.h"
#include "../ITMLib/Utils/Configuration/TelemetrySettings.h"

//ITMLib
#include "../ITMLib/Utils/Configuration/Configuration.h"
#include "../ITMLib/Engines/Indexing/IndexingSettings.h"
#include "../ITMLib/Utils/Metacoding/DeferrableStructUtilities.h"
#include "../ITMLib/Engines/Rendering/RenderingSettings.h"
#include "../ITMLib/Utils/Configuration/AutomaticRunSettings.h"

namespace pt = boost::property_tree;

using namespace ITMLib;
using namespace ITMLib::configuration;
using namespace test_utilities;

struct DeferrableStructCollection {
	TelemetrySettings telemetry_settings;
	IndexingSettings indexing_settings;
	RenderingSettings rendering_settings;
	AutomaticRunSettings automatic_run_settings;

	DeferrableStructCollection(const configuration::Configuration& source_configuration = configuration::get()) :
			telemetry_settings(BuildDeferrableFromParentIfPresent<TelemetrySettings>(source_configuration)),
			indexing_settings(BuildDeferrableFromParentIfPresent<IndexingSettings>(source_configuration)),
			rendering_settings(BuildDeferrableFromParentIfPresent<RenderingSettings>(source_configuration)),
			automatic_run_settings(BuildDeferrableFromParentIfPresent<AutomaticRunSettings>(source_configuration)){}

	friend void RequireEqualDeferrables(const DeferrableStructCollection& l, const DeferrableStructCollection& r){
		BOOST_REQUIRE_EQUAL(l.telemetry_settings, r.telemetry_settings);
		BOOST_REQUIRE_EQUAL(l.indexing_settings, r.indexing_settings);
		BOOST_REQUIRE_EQUAL(l.rendering_settings, r.rendering_settings);
	}

};

configuration::Configuration GenerateDefaultSnoopyConfiguration();

BOOST_AUTO_TEST_CASE(ConfigurationTest) {
	configuration::Configuration default_configuration;
	DeferrableStructCollection default_deferrables(default_configuration);

	configuration::Configuration configuration1 = GenerateChangedUpConfiguration();
	DeferrableStructCollection deferrables1(configuration1);

#ifdef COMPILE_WITHOUT_CUDA
	configuration::load_configuration_from_json_file( GENERATED_TEST_DATA_PREFIX "TestData/configuration/default_config_cpu.json");
#else
	configuration::load_configuration_from_json_file( GENERATED_TEST_DATA_PREFIX "TestData/configuration/default_config_cuda.json");
#endif

	DeferrableStructCollection loaded_deferrables;

	BOOST_REQUIRE_EQUAL(default_configuration.general_voxel_volume_parameters,
	                    configuration::get().general_voxel_volume_parameters);
	BOOST_REQUIRE_EQUAL(default_configuration.general_surfel_volume_parameters,
	                    configuration::get().general_surfel_volume_parameters);
	BOOST_REQUIRE_EQUAL(default_configuration.slavcheva_parameters, configuration::get().slavcheva_parameters);
	BOOST_REQUIRE_EQUAL(default_configuration.slavcheva_switches, configuration::get().slavcheva_switches);
	BOOST_REQUIRE_EQUAL(default_configuration.logging_settings, configuration::get().logging_settings);
	RequireEqualDeferrables(default_deferrables, loaded_deferrables);
	BOOST_REQUIRE_EQUAL(default_configuration.paths, configuration::get().paths);
	BOOST_REQUIRE_EQUAL(default_configuration.non_rigid_tracking_parameters,
	                    configuration::get().non_rigid_tracking_parameters);
	BOOST_REQUIRE_EQUAL(default_configuration, configuration::get());

	configuration::load_configuration_from_json_file(GENERATED_TEST_DATA_PREFIX "TestData/configuration/config1.json");
	loaded_deferrables = DeferrableStructCollection();

	BOOST_REQUIRE_EQUAL(configuration1.general_voxel_volume_parameters,
	                    configuration::get().general_voxel_volume_parameters);
	BOOST_REQUIRE_EQUAL(configuration1.general_surfel_volume_parameters,
	                    configuration::get().general_surfel_volume_parameters);
	BOOST_REQUIRE_EQUAL(configuration1.slavcheva_parameters, configuration::get().slavcheva_parameters);
	BOOST_REQUIRE_EQUAL(configuration1.slavcheva_switches, configuration::get().slavcheva_switches);
	BOOST_REQUIRE_EQUAL(configuration1.logging_settings, configuration::get().logging_settings);
	RequireEqualDeferrables(deferrables1, loaded_deferrables);
	BOOST_REQUIRE_EQUAL(configuration1.paths, configuration::get().paths);
	BOOST_REQUIRE_EQUAL(configuration1.non_rigid_tracking_parameters,
	                    configuration::get().non_rigid_tracking_parameters);
	BOOST_REQUIRE_EQUAL(configuration1, configuration::get());
	configuration::save_configuration_to_json_file(GENERATED_TEST_DATA_PREFIX "TestData/configuration/config2.json", configuration1);
	configuration::load_configuration_from_json_file(GENERATED_TEST_DATA_PREFIX "TestData/configuration/config2.json");
	loaded_deferrables = DeferrableStructCollection();

	BOOST_REQUIRE_EQUAL(configuration1.general_voxel_volume_parameters,
	                    configuration::get().general_voxel_volume_parameters);
	BOOST_REQUIRE_EQUAL(configuration1.general_surfel_volume_parameters,
	                    configuration::get().general_surfel_volume_parameters);
	BOOST_REQUIRE_EQUAL(configuration1.slavcheva_parameters, configuration::get().slavcheva_parameters);
	BOOST_REQUIRE_EQUAL(configuration1.slavcheva_switches, configuration::get().slavcheva_switches);
	BOOST_REQUIRE_EQUAL(configuration1.logging_settings, configuration::get().logging_settings);
	RequireEqualDeferrables(deferrables1, loaded_deferrables);
	BOOST_REQUIRE_EQUAL(configuration1.paths, configuration::get().paths);
	BOOST_REQUIRE_EQUAL(configuration1.non_rigid_tracking_parameters,
	                    configuration::get().non_rigid_tracking_parameters);
	BOOST_REQUIRE_EQUAL(configuration1, configuration::get());
}
