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

//stdlib
#include <string>
#include <thread>

//boost
#include <boost/test/unit_test.hpp>

//test_utilities
#include "TestUtilities/TestUtilities.h"
#include "../ITMLib/Engines/Telemetry/TelemetrySettings.h"

//ITMLib
#include "../ITMLib/Utils/Configuration/Configuration.h"
#include "../ITMLib/Engines/Indexing/IndexingSettings.h"
#include "../ITMLib/Utils/Metacoding/DeferrableStructUtilities.h"
#include "../ITMLib/Engines/Rendering/RenderingSettings.h"
#include "../ITMLib/Utils/Configuration/AutomaticRunSettings.h"
#include "../ITMLib/Engines/Main/MainEngineSettings.h"
#include "../ITMLib/Engines/LevelSetAlignment/Interface/LevelSetAlignmentParameters.h"
#include "../ITMLib/Engines/VolumeFusion/VolumeFusionSettings.h"

namespace pt = boost::property_tree;

using namespace ITMLib;
using namespace ITMLib::configuration;
using namespace test_utilities;

struct DeferrableStructCollection {
	MainEngineSettings main_engine_settings;
	TelemetrySettings telemetry_settings;
	IndexingSettings indexing_settings;
	RenderingSettings rendering_settings;
	AutomaticRunSettings automatic_run_settings;
	LevelSetAlignmentParameters level_set_evolution_parameters;
	VolumeFusionSettings volume_fusion_settings;


	DeferrableStructCollection(const configuration::Configuration& source_configuration = configuration::Get()) :
			main_engine_settings(BuildDeferrableFromParentIfPresent<MainEngineSettings>(source_configuration)),
			telemetry_settings(BuildDeferrableFromParentIfPresent<TelemetrySettings>(source_configuration)),
			indexing_settings(BuildDeferrableFromParentIfPresent<IndexingSettings>(source_configuration)),
			rendering_settings(BuildDeferrableFromParentIfPresent<RenderingSettings>(source_configuration)),
			automatic_run_settings(BuildDeferrableFromParentIfPresent<AutomaticRunSettings>(source_configuration)),
			level_set_evolution_parameters(BuildDeferrableFromParentIfPresent<LevelSetAlignmentParameters>(source_configuration)),
			volume_fusion_settings(BuildDeferrableFromParentIfPresent<VolumeFusionSettings>(source_configuration)){}

	friend void RequireEqualDeferrables(const DeferrableStructCollection& l, const DeferrableStructCollection& r) {
		BOOST_REQUIRE_EQUAL(l.main_engine_settings, r.main_engine_settings);
		BOOST_REQUIRE_EQUAL(l.telemetry_settings, r.telemetry_settings);
		BOOST_REQUIRE_EQUAL(l.indexing_settings, r.indexing_settings);
		BOOST_REQUIRE_EQUAL(l.rendering_settings, r.rendering_settings);
		BOOST_REQUIRE_EQUAL(l.automatic_run_settings, r.automatic_run_settings);
		BOOST_REQUIRE_EQUAL(l.level_set_evolution_parameters, r.level_set_evolution_parameters);
		BOOST_REQUIRE_EQUAL(l.volume_fusion_settings, r.volume_fusion_settings);
	}

};

configuration::Configuration GenerateDefaultSnoopyConfiguration();

BOOST_AUTO_TEST_CASE(ConfigurationFileTest) {
	configuration::Configuration default_configuration;
	DeferrableStructCollection default_deferrables(default_configuration);

	configuration::Configuration configuration1 = GenerateChangedUpConfiguration();
	DeferrableStructCollection deferrables1(configuration1);

#ifdef COMPILE_WITHOUT_CUDA
	configuration::load_configuration_from_json_file( GENERATED_TEST_DATA_PREFIX "TestData/configuration/default_config_cpu.json");
#else
	configuration::LoadConfigurationFromJSONFile(GENERATED_TEST_DATA_PREFIX "TestData/configuration/default_config_cuda.json");
#endif

	DeferrableStructCollection loaded_deferrables;

	BOOST_REQUIRE_EQUAL(default_configuration.general_voxel_volume_parameters,
	                    configuration::Get().general_voxel_volume_parameters);
	BOOST_REQUIRE_EQUAL(default_configuration.general_surfel_volume_parameters,
	                    configuration::Get().general_surfel_volume_parameters);
	BOOST_REQUIRE_EQUAL(default_configuration.logging_settings, configuration::Get().logging_settings);
	RequireEqualDeferrables(default_deferrables, loaded_deferrables);
	BOOST_REQUIRE_EQUAL(default_configuration.paths, configuration::Get().paths);
	BOOST_REQUIRE_EQUAL(default_configuration, configuration::Get());

	configuration::LoadConfigurationFromJSONFile(GENERATED_TEST_DATA_PREFIX "TestData/configuration/config1.json");
	loaded_deferrables = DeferrableStructCollection();

	BOOST_REQUIRE_EQUAL(configuration1.general_voxel_volume_parameters,
	                    configuration::Get().general_voxel_volume_parameters);
	BOOST_REQUIRE_EQUAL(configuration1.general_surfel_volume_parameters,
	                    configuration::Get().general_surfel_volume_parameters);
	BOOST_REQUIRE_EQUAL(configuration1.logging_settings, configuration::Get().logging_settings);
	RequireEqualDeferrables(deferrables1, loaded_deferrables);
	BOOST_REQUIRE_EQUAL(configuration1.paths, configuration::Get().paths);
	BOOST_REQUIRE_EQUAL(configuration1, configuration::Get());
	configuration::SaveConfigurationToJSONFile(GENERATED_TEST_DATA_PREFIX "TestData/configuration/config2.json", configuration1);
	configuration::LoadConfigurationFromJSONFile(GENERATED_TEST_DATA_PREFIX "TestData/configuration/config2.json");
	loaded_deferrables = DeferrableStructCollection();

	BOOST_REQUIRE_EQUAL(configuration1.general_voxel_volume_parameters,
	                    configuration::Get().general_voxel_volume_parameters);
	BOOST_REQUIRE_EQUAL(configuration1.general_surfel_volume_parameters,
	                    configuration::Get().general_surfel_volume_parameters);
	BOOST_REQUIRE_EQUAL(configuration1.logging_settings, configuration::Get().logging_settings);
	RequireEqualDeferrables(deferrables1, loaded_deferrables);
	BOOST_REQUIRE_EQUAL(configuration1.paths, configuration::Get().paths);
	BOOST_REQUIRE_EQUAL(configuration1, configuration::Get());
}

void ExecuteExternalProcess(const std::string& command) {
	system(command.c_str());
}

static std::string GetCLI_optionsSubtestExecutablePath() {
#ifdef _MSC_VER
#ifdef NDEBUG
	return std::string(GENERATED_TEST_DATA_PREFIX "Release/cli_options_subtest");
#else
	return std::string(GENERATED_TEST_DATA_PREFIX "Debug/cli_options_subtest");
#endif
#else
	return std::string(GENERATED_TEST_DATA_PREFIX "cli_options_subtest");
#endif
}

BOOST_AUTO_TEST_CASE(Configuration_CLI_Test_Defaults_Unchanged) {
	DeferrableStructCollection loaded_deferrables;
	std::string config_destination;

	config_destination = GENERATED_TEST_DATA_PREFIX "TestData/configuration/cli_test_defaults.json";
	std::string command = GetCLI_optionsSubtestExecutablePath() + " --config_output=" + config_destination;
	std::thread thread(ExecuteExternalProcess, command);
	thread.join();
	configuration::LoadConfigurationFromJSONFile(config_destination);
	loaded_deferrables = DeferrableStructCollection();

	configuration::Configuration default_configuration;
	DeferrableStructCollection default_deferrables(default_configuration);

	BOOST_REQUIRE_EQUAL(default_configuration, configuration::Get());
	RequireEqualDeferrables(default_deferrables, loaded_deferrables);
}

BOOST_AUTO_TEST_CASE(ConfigurationTestLong_CLI_Only) {
	DeferrableStructCollection loaded_deferrables;
	std::string config_destination;

	config_destination = GENERATED_TEST_DATA_PREFIX "TestData/configuration/cli_test_long.json";
	std::string command = GetCLI_optionsSubtestExecutablePath() + " --config_output=" + config_destination +
	                      " --focus_coordinates=20 23 0"
	                      " --record_reconstruction_video=true"
	                      " --record_inputs_in_reconstruction_video=true"

	                      " --general_voxel_volume_parameters.voxel_size=0.005"
	                      " --general_voxel_volume_parameters.near_clipping_distance=0.12"
	                      " --general_voxel_volume_parameters.far_clipping_distance=4.12"
	                      " --general_voxel_volume_parameters.truncation_distance=0.05"
	                      " --general_voxel_volume_parameters.max_integration_weight=200"
	                      " --general_voxel_volume_parameters.stop_integration_at_max_weight=true"
	                      " --general_voxel_volume_parameters.block_allocation_band_factor=1.2"

	                      " --general_surfel_volume_parameters.delta_radius=0.4"
	                      " --general_surfel_volume_parameters.gaussian_confidence_sigma=0.5"
	                      " --general_surfel_volume_parameters.max_merge_angle=0.383972436"
	                      " --general_surfel_volume_parameters.max_merge_dist=0.008"
	                      " --general_surfel_volume_parameters.max_surfel_radius=0.0003"
	                      " --general_surfel_volume_parameters.min_radius_overlap_factor=3.4"
	                      " --general_surfel_volume_parameters.stable_surfel_confidence=26"
	                      " --general_surfel_volume_parameters.supersampling_factor=5"
	                      " --general_surfel_volume_parameters.tracking_surfel_max_depth=1.1"
	                      " --general_surfel_volume_parameters.tracking_surfel_min_confidence=4.5"
	                      " --general_surfel_volume_parameters.unstable_surfel_period=21"
	                      " --general_surfel_volume_parameters.unstable_surfel_z_offset=300"
	                      " --general_surfel_volume_parameters.use_gaussian_sample_confidence=false"
	                      " --general_surfel_volume_parameters.use_surfel_merging=false"

	                      " --specific_volume_parameters.array.canonical.size=512 512 512"

	                      " --specific_volume_parameters.hash.canonical.voxel_block_count=262144"
	                      " --specific_volume_parameters.hash.canonical.excess_list_size=131072"
	                      " --specific_volume_parameters.hash.live.voxel_block_count=131072"
	                      " --specific_volume_parameters.hash.live.excess_list_size=131072"
	                      " --specific_volume_parameters.hash.warp.voxel_block_count=131072"
	                      " --specific_volume_parameters.hash.warp.excess_list_size=131072"

	                      " --level_set_evolution.execution_mode=diagnostic"

	                      " --level_set_evolution.weights.learning_rate=0.11"
	                      " --level_set_evolution.weights.Killing_dampening_factor=0.09"
	                      " --level_set_evolution.weights.weight_data_term=2"
	                      " --level_set_evolution.weights.weight_smoothing_term=0.3"
	                      " --level_set_evolution.weights.weight_level_set_term=0.1"
	                      " --level_set_evolution.weights.epsilon=1e-06"

	                      " --level_set_evolution.switches.enable_data_term=false"
	                      " --level_set_evolution.switches.enable_level_set_term=true"
	                      " --level_set_evolution.switches.enable_smoothing_term=false"
	                      " --level_set_evolution.switches.enable_Killing_field=true"
	                      " --level_set_evolution.switches.enable_Sobolev_gradient_smoothing=false"

					      " --level_set_evolution.termination.warp_length_termination_threshold_type=average"
	                      " --level_set_evolution.termination.max_iteration_count=300"
					      " --level_set_evolution.termination.min_iteration_count=5"
	                      " --level_set_evolution.termination.update_length_threshold=0.0002"

	                      " --logging_settings.verbosity_level=warning"
	                      " --logging_settings.log_to_disk=true"
	                      " --logging_settings.log_to_stdout=false"
	                      " --logging_settings.log_benchmarks=true"
	                      " --logging_settings.log_volume_statistics=true"
	                      " --logging_settings.log_trajectory_quaternions=true"
	                      " --logging_settings.log_iteration_number=true"
	                      " --logging_settings.log_surface_tracking_procedure_names=true"
	                      " --logging_settings.log_gradient_length_statistic=true"
	                      " --logging_settings.log_surface_tracking_optimization_energies=true"
	                      " --logging_settings.log_additional_surface_tracking_stats=true"
	                      " --logging_settings.log_warp_update_length_histograms=true"
	                      " --logging_settings.log_voxel_hash_block_usage=true"

	                      " --paths.output_path=" GENERATED_TEST_DATA_PREFIX "TestData/output"
	                      " --paths.calibration_file_path=" STATIC_TEST_DATA_PREFIX "TestData/calibration/snoopy_calib.txt"
	                      " --paths.rgb_image_path_mask=" STATIC_TEST_DATA_PREFIX "TestData/frames/frame_color_%%06i.png"
	                      " --paths.depth_image_path_mask=" STATIC_TEST_DATA_PREFIX "TestData/frames/frame_depth_%%06i.png"
	                      " --paths.mask_image_path_mask=" STATIC_TEST_DATA_PREFIX "TestData/frames/frame_mask_%%06i.png"

	                      " --create_meshing_engine=true"
	                      " --device_type=cpu"
	                      " --use_approximate_raycast=true"
	                      " --use_threshold_filter=true"
	                      " --use_bilateral_filter=true"
	                      " --behavior_on_failure=relocalize"
	                      " --swapping_mode=enabled"
	                      " --tracker_configuration=\"type=rgb,levels=rrbb\""

	                      " --main_engine_settings.draw_frame_index_labels=true"
	                      " --main_engine_settings.library_mode=basic"
	                      " --main_engine_settings.indexing_method=array"
	                      " --main_engine_settings.enable_rigid_alignment=false"

	                      " --telemetry_settings.record_volume_memory_usage=true"
	                      " --telemetry_settings.record_surface_tracking_optimization_energies=true"
	                      " --telemetry_settings.record_surface_tracking_additional_statistics=true"
	                      " --telemetry_settings.record_frame_meshes=true"
	                      " --telemetry_settings.record_warp_update_length_histograms=true"
	                      " --telemetry_settings.use_warp_update_length_histogram_manual_max=true"
	                      " --telemetry_settings.warp_update_length_histogram_max=0.0001"
					      " --telemetry_settings.warp_update_length_histogram_bin_count=32"
	                      " --telemetry_settings.use_CPU_for_mesh_recording=true"
	                      " --telemetry_settings.record_camera_matrices=true"

	                      " --indexing_settings.execution_mode=diagnostic"

	                      " --rendering_settings.skip_points=true"

	                      " --automatic_run_settings.index_of_frame_to_end_before=50"
	                      " --automatic_run_settings.index_of_frame_to_start_at=16"
	                      " --automatic_run_settings.load_volume_and_camera_matrix_before_processing=true"
	                      " --automatic_run_settings.save_volumes_and_camera_matrix_after_processing=true"
	                      " --automatic_run_settings.save_meshes_after_processing=true"
	                      " --automatic_run_settings.exit_after_automatic_processing=true"

					      " --volume_fusion_settings.use_surface_thickness_cutoff=false"
	                      " --volume_fusion_settings.surface_thickness=0.008"

	                      " --depth_fusion_settings.use_surface_thickness_cutoff=true"
	                      " --depth_fusion_settings.surface_thickness=0.008"
	                      ;

	std::cout << "Executing command: " << std::endl;
	std::cout << command << std::endl;
	std::thread thread(ExecuteExternalProcess, command);
	thread.join();
	configuration::LoadConfigurationFromJSONFile(config_destination);
	loaded_deferrables = DeferrableStructCollection();

	configuration::Configuration configuration1 = GenerateChangedUpConfiguration();
	DeferrableStructCollection deferrables1(configuration1);

	BOOST_REQUIRE_EQUAL(configuration1.general_voxel_volume_parameters, configuration::Get().general_voxel_volume_parameters);
	BOOST_REQUIRE_EQUAL(configuration1.general_surfel_volume_parameters, configuration::Get().general_surfel_volume_parameters);
	BOOST_REQUIRE_EQUAL(configuration1.specific_volume_parameters, configuration::Get().specific_volume_parameters);
	BOOST_REQUIRE_EQUAL(configuration1.logging_settings, configuration::Get().logging_settings);
	BOOST_REQUIRE_EQUAL(configuration1.paths, configuration::Get().paths);
	BOOST_REQUIRE_EQUAL(configuration1, configuration::Get());
	RequireEqualDeferrables(deferrables1, loaded_deferrables);
}

//TODO: Restore support for short options when Boost program_options is replaced by CLI11
//BOOST_AUTO_TEST_CASE(ConfigurationTestShort_CLI_Only) {
//	DeferrableStructCollection loaded_deferrables;
//	std::string config_destination;
//
//	config_destination = GENERATED_TEST_DATA_PREFIX "TestData/configuration/cli_test_short.json";
//	std::string command = GetCLI_optionsSubtestExecutablePath() + " --config_output=" + config_destination +
//	                      " -fc=20 23 0"
//	                      " -rrv=true"
//	                      " -riirv=true"
//
//	                      " -gvvp.vs=0.005"
//	                      " -gvvp.ncd=0.12"
//	                      " -gvvp.fcd=4.12"
//	                      " -gvvp.td=0.05"
//	                      " -gvvp.miw=200"
//	                      " -gvvp.siamw=true"
//	                      " -gvvp.babf=1.2"
//
//	                      " -gsvp.dr=0.4"
//	                      " -gsvp.gcs=0.5"
//	                      " -gsvp.mma=0.383972436"
//	                      " -gsvp.mmd=0.008"
//	                      " -gsvp.msd=0.0003"
//	                      " -gsvp.mrof=3.4"
//	                      " -gsvp.ssc=26"
//	                      " -gsvp.sf=5"
//	                      " -gsvp.tsmd=1.1"
//	                      " -gsvp.tsmc=4.5"
//	                      " -gsvp.usp=21"
//	                      " -gsvp.uszo=300"
//	                      " -gsvp.ugsc=false"
//	                      " -gsvp.usm=false"
//
//	                      " -svp.a.c.s=512 512 512"
//
//	                      " -svp.h.c.vbc=262144"
//	                      " -svp.h.c.els=131072"
//	                      " -svp.h.l.vbc=131072"
//	                      " -svp.h.l.els=131072"
//	                      " -svp.h.w.vbc=131072"
//	                      " -svp.h.w.els=131072"
//

// TODO: here the new level_set_evolution.* flags

//
//	                      " -ls.vl=warning"
//	                      " -ls.ltd=true"
//	                      " -ls.lts=false"
//	                      " -ls.lb=true"
//	                      " -ls.lvs=true"
//	                      " -ls.ltq=true"
//	                      " -ls.lstoe=true"
//
//	                      " -p.op=" GENERATED_TEST_DATA_PREFIX "TestData/output"
//	                      " -p.cfp=" STATIC_TEST_DATA_PREFIX "TestData/calibration/snoopy_calib.txt"
//	                      " -p.ripm=" STATIC_TEST_DATA_PREFIX "TestData/frames/frame_color_%%06i.png"
//	                      " -p.dipm=" STATIC_TEST_DATA_PREFIX "TestData/frames/frame_depth_%%06i.png"
//	                      " -p.mipm=" STATIC_TEST_DATA_PREFIX "TestData/frames/frame_mask_%%06i.png"
//
//
//	                      " -cme=true"
//	                      " -dt=cpu"
//	                      " -uar=true"
//	                      " -utf=true"
//	                      " -ubf=true"
//	                      " -bof=relocalize"
//	                      " -sm=enabled"
//	                      " -tc=\"type=rgb,levels=rrbb\""
//
//	                      " -mes.dfil=true"
//	                      " -mes.lm=basic"
//	                      " -mes.im=array"
//	                      " -mes.era=false"
//
//	                      " -ts.rvmu=true"
//	                      " -ts.rlva2s=true"
//	                      " -ts.rcva2s=true"
//	                      " -ts.rlfpTg=true"
//	                      " -ts.rlflTh=true"
//	                      " -ts.Thp=plane_xy"
//	                      " -ts.rfnlts=true"
//	                      " -ts.fns=4"
//	                      " -ts.rfnws=true"
//	                      " -ts.rstoe=true"
//	                      " -ts.rfm=true"
//	                      " -ts.uCfmr=true"
//	                      " -ts.rcm=true"
//
//	                      " -is.em=diagnostic"
//
//	                      " -rs.sp=true"
//
//	                      " -ars.iofteb=50"
//	                      " -ars.ioftsa=16"
//	                      " -ars.lvacmbp=true"
//	                      " -ars.svacmap=true"
//	                      " -ars.smap=true"
//	                      " -ars.eaap=true"
//	;
//
//	std::cout << "Executing command: " << std::endl;
//	std::cout << command << std::endl;
//	std::thread thread(ExecuteExternalProcess, command);
//	thread.join();
//	configuration::LoadConfigurationFromJSONFile(config_destination);
//	loaded_deferrables = DeferrableStructCollection();
//
//	configuration::Configuration configuration1 = GenerateChangedUpConfiguration();
//	DeferrableStructCollection deferrables1(configuration1);
//
//	BOOST_REQUIRE_EQUAL(configuration1.general_voxel_volume_parameters, configuration::Get().general_voxel_volume_parameters);
//	BOOST_REQUIRE_EQUAL(configuration1.general_surfel_volume_parameters, configuration::Get().general_surfel_volume_parameters);
//	BOOST_REQUIRE_EQUAL(configuration1.specific_volume_parameters, configuration::Get().specific_volume_parameters);
//	BOOST_REQUIRE_EQUAL(configuration1.logging_settings, configuration::Get().logging_settings);
//	BOOST_REQUIRE_EQUAL(configuration1.paths, configuration::Get().paths);
//	BOOST_REQUIRE_EQUAL(configuration1, configuration::Get());
//	RequireEqualDeferrables(deferrables1, loaded_deferrables);
//}

BOOST_AUTO_TEST_CASE(ConfigurationFileAnd_CLI_Test) {
	auto configuration1 = GenerateChangedUpConfiguration();
	DeferrableStructCollection deferrables1(configuration1);

	std::string input_config_path = GENERATED_TEST_DATA_PREFIX "TestData/configuration/file_and_cli_test_input.json";
	configuration::SaveConfigurationToJSONFile(input_config_path, configuration1);

	DeferrableStructCollection loaded_deferrables;
	std::string config_destination;

	config_destination = GENERATED_TEST_DATA_PREFIX "TestData/configuration/file_and_cli_test_output.json";
	std::string command = GetCLI_optionsSubtestExecutablePath() + " --config_output=" + config_destination +
	                      " --config=" + input_config_path +
	                      " --general_voxel_volume_parameters.voxel_size=0.008"
	                      " --general_voxel_volume_parameters.near_clipping_distance=0.16"
	                      " --general_voxel_volume_parameters.far_clipping_distance=4.14"
	                      " --automatic_run_settings.index_of_frame_to_end_before=54"
	                      " --automatic_run_settings.index_of_frame_to_start_at=12"
	                      " --automatic_run_settings.load_volume_and_camera_matrix_before_processing=false"
	                      " --device_type=cuda"
	                      " --use_approximate_raycast=false";
	std::cout << "Executing command: " << std::endl;
	std::cout << command << std::endl;
	std::thread thread(ExecuteExternalProcess, command);
	thread.join();

	configuration::LoadConfigurationFromJSONFile(config_destination);
	loaded_deferrables = DeferrableStructCollection();

	configuration1.general_voxel_volume_parameters.voxel_size = 0.008f;
	configuration1.general_voxel_volume_parameters.near_clipping_distance = 0.16f;
	configuration1.general_voxel_volume_parameters.far_clipping_distance = 4.14f;
	deferrables1.automatic_run_settings.index_of_frame_to_end_before = 54;
	deferrables1.automatic_run_settings.index_of_frame_to_start_at = 12;
	deferrables1.automatic_run_settings.load_volume_and_camera_matrix_before_processing = false;
	configuration1.device_type = MEMORYDEVICE_CUDA;
	configuration1.use_approximate_raycast = false;

	BOOST_REQUIRE_EQUAL(configuration1.general_voxel_volume_parameters, configuration::Get().general_voxel_volume_parameters);
	BOOST_REQUIRE_EQUAL(configuration1.general_surfel_volume_parameters, configuration::Get().general_surfel_volume_parameters);
	BOOST_REQUIRE_EQUAL(configuration1.specific_volume_parameters, configuration::Get().specific_volume_parameters);
	BOOST_REQUIRE_EQUAL(configuration1.logging_settings, configuration::Get().logging_settings);
	BOOST_REQUIRE_EQUAL(configuration1.paths, configuration::Get().paths);
	BOOST_REQUIRE_EQUAL(configuration1, configuration::Get());
	RequireEqualDeferrables(deferrables1, loaded_deferrables);
}