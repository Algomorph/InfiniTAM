//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 4/20/20.
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
//stdlib
#include <filesystem>
#include <map>

//log4cplus
#include <log4cplus/loggingmacros.h>
#include <log4cplus/consoleappender.h>

//test_utilities
#include "TestUtilities/TestUtilities.h"
#include "TestUtilities/SnoopyTestUtilities.h"
#include "TestUtilities/WarpAdvancedTestingUtilities.h"
#include "TestUtilities/CameraPoseAndRenderingEngineFixture.h"

//ORUtils
#include "../ORUtils/FileUtils.h"

//ITMLib
//(CPU)
#include "../ITMLib/Engines/EditAndCopy/CPU/EditAndCopyEngine_CPU.h"
#include "../ITMLib/Engines/Indexing/VBH/CPU/IndexingEngine_VoxelBlockHash_CPU.h"
//(CUDA)
#ifndef COMPILE_WITHOUT_CUDA

#include "../ITMLib/Engines/EditAndCopy/CUDA/EditAndCopyEngine_CUDA.h"
#include "../ITMLib/Engines/Indexing/VBH/CUDA/IndexingEngine_VoxelBlockHash_CUDA.h"

#endif
//(misc)

#include "../ITMLib/Engines/VolumeFusion/VolumeFusionEngine.h"
#include "../ITMLib/Engines/Indexing/IndexingEngineFactory.h"
#include "../ITMLib/Engines/VolumeFusion/VolumeFusionEngineFactory.h"
#include "../ITMLib/Engines/Warping/WarpingEngineFactory.h"
#include "../ITMLib/Engines/Meshing/MeshingEngineFactory.h"
#include "../ITMLib/Engines/EditAndCopy/EditAndCopyEngineFactory.h"
#include "../ITMLib/Engines/ViewBuilder/ViewBuilderFactory.h"
#include "../ITMLib/Engines/DepthFusion/DepthFusionEngineFactory.h"
#include "../ITMLib/Engines/Telemetry/TelemetrySettings.h"
#include "../ITMLib/Utils/Configuration/AutomaticRunSettings.h"
#include "../ITMLib/Engines/Main/MainEngineSettings.h"
#include "../ITMLib/Engines/Analytics/AnalyticsEngine.h"


using namespace ITMLib;
using namespace test_utilities;
namespace snoopy = snoopy_test_utilities;

namespace fs = std::filesystem;

void ConstructSnoopyUnmaskedVolumes00() {
	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(), "Constructing snoopy unmasked full volumes at frame 0...");

	VoxelVolume<TSDFVoxel, PlainVoxelArray>* volume_PVA_00;
	VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume_VBH_00;

	BuildSdfVolumeFromImage_NearSurfaceAllocation(&volume_PVA_00,
	                                              snoopy::Frame00DepthPath(),
	                                              snoopy::Frame00ColorPath(),
	                                              snoopy::SnoopyCalibrationPath(),
	                                              MEMORYDEVICE_CPU,
	                                              snoopy::InitializationParameters_Fr00<PlainVoxelArray>());
	BuildSdfVolumeFromImage_NearSurfaceAllocation(&volume_VBH_00,
	                                              snoopy::Frame00DepthPath(),
	                                              snoopy::Frame00ColorPath(),
	                                              snoopy::SnoopyCalibrationPath(),
	                                              MEMORYDEVICE_CPU,
	                                              snoopy::InitializationParameters_Fr00<VoxelBlockHash>());
	test_utilities::ConstructGeneratedVolumeSubdirectoriesIfMissing();
	volume_PVA_00->SaveToDisk(snoopy::PartialVolume00Path<PlainVoxelArray>());
	volume_VBH_00->SaveToDisk(snoopy::PartialVolume00Path<VoxelBlockHash>());

	delete volume_PVA_00;
	delete volume_VBH_00;
}

void ConstructSnoopyMaskedVolumes16and17() {
	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(), "Constructing snoopy masked partial volumes 16 & 17...");

	VoxelVolume<TSDFVoxel, PlainVoxelArray>* volume_PVA_16;
	VoxelVolume<TSDFVoxel, PlainVoxelArray>* volume_PVA_17;
	BuildSdfVolumeFromImage_SurfaceSpanAllocation(&volume_PVA_16,
	                                              &volume_PVA_17,
	                                              snoopy::Frame16DepthPath(),
	                                              snoopy::Frame16ColorPath(),
	                                              snoopy::Frame16MaskPath(),
	                                              snoopy::Frame17DepthPath(),
	                                              snoopy::Frame17ColorPath(),
	                                              snoopy::Frame17MaskPath(),
	                                              snoopy::SnoopyCalibrationPath(),
	                                              MEMORYDEVICE_CPU,
	                                              snoopy::InitializationParameters_Fr16andFr17<PlainVoxelArray>());

	VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume_VBH_16;
	VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume_VBH_17;
	BuildSdfVolumeFromImage_SurfaceSpanAllocation(&volume_VBH_16,
	                                              &volume_VBH_17,
	                                              snoopy::Frame16DepthPath(),
	                                              snoopy::Frame16ColorPath(),
	                                              snoopy::Frame16MaskPath(),
	                                              snoopy::Frame17DepthPath(),
	                                              snoopy::Frame17ColorPath(),
	                                              snoopy::Frame17MaskPath(),
	                                              snoopy::SnoopyCalibrationPath(),
	                                              MEMORYDEVICE_CPU,
	                                              snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());

	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(), "Saving snoopy masked partial volumes 16 & 17...");

	test_utilities::ConstructGeneratedVolumeSubdirectoriesIfMissing();

	volume_PVA_16->SaveToDisk(snoopy::PartialVolume16Path<PlainVoxelArray>());
	volume_VBH_16->SaveToDisk(snoopy::PartialVolume16Path<VoxelBlockHash>());
	volume_PVA_17->SaveToDisk(snoopy::PartialVolume17Path<PlainVoxelArray>());
	volume_VBH_17->SaveToDisk(snoopy::PartialVolume17Path<VoxelBlockHash>());

	volume_PVA_16->Reset();
	volume_VBH_16->Reset();
	volume_PVA_17->Reset();
	volume_VBH_17->Reset();

	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(), "Constructing snoopy masked full volumes 16 & 17 PVA...");

	BuildSdfVolumeFromImage_SurfaceSpanAllocation(&volume_PVA_16,
	                                              &volume_PVA_17,
	                                              snoopy::Frame16DepthPath(),
	                                              snoopy::Frame16ColorPath(),
	                                              snoopy::Frame16MaskPath(),
	                                              snoopy::Frame17DepthPath(),
	                                              snoopy::Frame17ColorPath(),
	                                              snoopy::Frame17MaskPath(),
	                                              snoopy::SnoopyCalibrationPath(),
	                                              MEMORYDEVICE_CPU);

	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(), "Constructing snoopy masked full volumes 16 & 17 VBH...");

	BuildSdfVolumeFromImage_SurfaceSpanAllocation(&volume_VBH_16,
	                                              &volume_VBH_17,
	                                              snoopy::Frame16DepthPath(),
	                                              snoopy::Frame16ColorPath(),
	                                              snoopy::Frame16MaskPath(),
	                                              snoopy::Frame17DepthPath(),
	                                              snoopy::Frame17ColorPath(),
	                                              snoopy::Frame17MaskPath(),
	                                              snoopy::SnoopyCalibrationPath(),
	                                              MEMORYDEVICE_CPU);

	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(), "Saving snoopy masked full volume 16 PVA...");
	volume_PVA_16->SaveToDisk(snoopy::FullVolume16Path<PlainVoxelArray>());
	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(), "Saving snoopy masked full volume 16 VBH...");
	volume_VBH_16->SaveToDisk(snoopy::FullVolume16Path<VoxelBlockHash>());
	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(), "Saving snoopy masked full volume 17 PVA...");
	volume_PVA_17->SaveToDisk(snoopy::FullVolume17Path<PlainVoxelArray>());
	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(), "Saving snoopy masked full volume 17 VBH...");
	volume_VBH_17->SaveToDisk(snoopy::FullVolume17Path<VoxelBlockHash>());

	delete volume_PVA_16;
	delete volume_VBH_16;
	delete volume_PVA_17;
	delete volume_VBH_17;
}

void ConstructStripesTestVolumes() {
	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(), "Constructing stripe test volumes...");
	// region ================================= CONSTRUCT VIEW =========================================================

	RGBD_CalibrationInformation calibration_data;
	readRGBDCalib(snoopy::SnoopyCalibrationPath().c_str(), calibration_data);

	auto view_builder = ViewBuilderFactory::Build(calibration_data, MEMORYDEVICE_CPU);
	Vector2i image_size(640, 480);
	View* view = nullptr;

	UChar4Image rgb(true, false);
	ShortImage depth(true, false);
	ReadImageFromFile(rgb, STATIC_TEST_DATA_PREFIX "TestData/frames/stripes_color.png");
	ReadImageFromFile(depth, STATIC_TEST_DATA_PREFIX "TestData/frames/stripes_depth.png");

	view_builder->UpdateView(&view, &rgb, &depth, false, false, false, true);

	// endregion =======================================================================================================

	VoxelVolume<TSDFVoxel, VoxelBlockHash> volume4(MEMORYDEVICE_CPU, {0x800, 0x20000});
	volume4.Reset();
	IndexingEngineInterface<TSDFVoxel, VoxelBlockHash>& indexer_VBH
			= IndexingEngineFactory::GetDefault<TSDFVoxel, VoxelBlockHash>(MEMORYDEVICE_CPU);
	CameraTrackingState tracking_state(image_size, MEMORYDEVICE_CPU);
	indexer_VBH.AllocateNearAndBetweenTwoSurfaces(&volume4, view, &tracking_state);
	auto depth_fusion_engine_VBH = DepthFusionEngineFactory::Build<TSDFVoxel, WarpVoxel, VoxelBlockHash>(
			MEMORYDEVICE_CPU);
	depth_fusion_engine_VBH->IntegrateDepthImageIntoTsdfVolume(&volume4, view, &tracking_state);
	std::string path = STATIC_TEST_DATA_PREFIX "TestData/volumes/VBH/stripes.dat";

	test_utilities::ConstructGeneratedVolumeSubdirectoriesIfMissing();
	volume4.SaveToDisk(path);

	delete depth_fusion_engine_VBH;
	delete view;
	delete view_builder;
}

template<typename TIndex, MemoryDeviceType TMemoryDeviceType>
void GenerateWarpGradientTestData() {
	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(),
	               "Generating warp field data from snoopy masked partial volumes 16 & 17 "
			               << IndexString<TIndex>() << "...");
	test_utilities::ConstructGeneratedVolumeSubdirectoriesIfMissing();
	std::string volume_output_directory = std::string(test_utilities::GeneratedVolumeDirectory) + IndexString<TIndex>() + "/";
	test_utilities::ConstructGeneratedArraysDirectoryIfMissing();

	ORUtils::OStreamWrapper warp_stats_file(std::string(test_utilities::GeneratedArraysDirectory) + "warp_gradient_stats_" + IndexString<TIndex>() + ".dat", false);

	VoxelVolume<TSDFVoxel, TIndex>* canonical_volume;
	VoxelVolume<TSDFVoxel, TIndex>* live_volume;
	LoadVolume(&live_volume, volume_output_directory + "snoopy_partial_frame_17.dat", TMemoryDeviceType,
	           snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	LoadVolume(&canonical_volume, volume_output_directory + "snoopy_partial_frame_16.dat", TMemoryDeviceType,
	           snoopy::InitializationParameters_Fr16andFr17<TIndex>());

	VoxelVolume<TSDFVoxel, TIndex>* live_volumes[] = {live_volume, nullptr};

	// *** set up level set switches for both iterations

	std::array<LevelSetAlignmentSwitches, 3> switches_iteration_0 = {
			LevelSetAlignmentSwitches(true, false, false, false, false),
			LevelSetAlignmentSwitches(true, false, false, false, true),
			LevelSetAlignmentSwitches(true, false, true, false, true)
	};

	std::array<LevelSetAlignmentSwitches, 4> switches_iteration_1 = {
			LevelSetAlignmentSwitches(false, false, true, false, false),
			LevelSetAlignmentSwitches(true, false, true, false, false),
			LevelSetAlignmentSwitches(true, false, true, true, false),
			LevelSetAlignmentSwitches(true, true, false, false, false)
	};
	// ========================================================================

	VoxelVolume<WarpVoxel, TIndex> warp_field(TMemoryDeviceType, snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	auto reset_warp_field = [&]() {
		warp_field.Reset();
		AllocateUsingOtherVolume(&warp_field, live_volume, MEMORYDEVICE_CPU);
	};

	AllocateUsingOtherVolume(canonical_volume, live_volume, MEMORYDEVICE_CPU);

	std::string warp_field_iteration_0_prefix = "warp_field_0_";
	std::string warp_field_iteration_1_prefix = "warp_field_1_";
	std::string warp_field_file_extension = ".dat";

	// iteration 0
	for (auto& switches : switches_iteration_0) {
		reset_warp_field();
		LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, TIndex, TMemoryDeviceType, DIAGNOSTIC> data_only_motion_tracker(
				switches, SingleIterationTerminationConditions());

		data_only_motion_tracker.Align(&warp_field, live_volumes, canonical_volume);

		warp_field.SaveToDisk(volume_output_directory + warp_field_iteration_0_prefix + SwitchesToPrefix(switches) + warp_field_file_extension);

		unsigned int altered_warp_update_count =
				AnalyticsEngine<WarpVoxel, TIndex, TMemoryDeviceType>::Instance().CountAlteredWarpUpdates(&warp_field);
		warp_stats_file.OStream().write(reinterpret_cast<const char*>(&altered_warp_update_count), sizeof(unsigned int));
		float average_warp_update_length = AnalyticsEngine<WarpVoxel, TIndex, TMemoryDeviceType>::Instance().ComputeWarpUpdateMean(&warp_field);
		warp_stats_file.OStream().write(reinterpret_cast<const char*>(&average_warp_update_length), sizeof(float));
	}

	reset_warp_field();
	std::string warp_0_data_and_tikhonov_sobolev_smoothed_filename =
			warp_field_iteration_0_prefix + SwitchesToPrefix(switches_iteration_0[switches_iteration_0.size() - 1]) + warp_field_file_extension;

	// iteration 1
	for (auto& switches : switches_iteration_1) {
		warp_field.Reset();
		warp_field.LoadFromDisk(volume_output_directory + warp_0_data_and_tikhonov_sobolev_smoothed_filename);
		LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, TIndex, TMemoryDeviceType, DIAGNOSTIC> tracker(switches, SingleIterationTerminationConditions());
		tracker.Align(&warp_field, live_volumes, canonical_volume);
		warp_field.SaveToDisk(volume_output_directory + warp_field_iteration_1_prefix + SwitchesToPrefix(switches) + warp_field_file_extension);
		unsigned int altered_gradient_count = AnalyticsEngine<WarpVoxel, TIndex, TMemoryDeviceType>::Instance().CountAlteredGradients(&warp_field);
		warp_stats_file.OStream().write(reinterpret_cast<const char*>(&altered_gradient_count), sizeof(unsigned int));
		float average_warp_update_length = AnalyticsEngine<WarpVoxel, TIndex, TMemoryDeviceType>::Instance().ComputeWarpUpdateMean(&warp_field);
		warp_stats_file.OStream().write(reinterpret_cast<const char*>(&average_warp_update_length), sizeof(float));
	}

	delete canonical_volume;
	delete live_volume;
}

void GenerateWarpGradient_PVA_to_VBH_TestData() {
	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(),
	               "Generating multi-iteration warp field data from snoopy masked partial volumes 16 & 17 (PVA & VBH)... ");
	LevelSetAlignmentSwitches switches_data_only(true, false, false, false, false);
	GenericWarpTest<MEMORYDEVICE_CPU>(switches_data_only, 10, SAVE_SUCCESSIVE_ITERATIONS);
	LevelSetAlignmentSwitches switches_data_and_tikhonov(true, false, true, false, false);
	GenericWarpTest<MEMORYDEVICE_CPU>(switches_data_and_tikhonov, 5, SAVE_SUCCESSIVE_ITERATIONS);
	LevelSetAlignmentSwitches switches_data_and_tikhonov_and_sobolev_smoothing(true, false, true, false, true);
	GenericWarpTest<MEMORYDEVICE_CPU>(switches_data_and_tikhonov_and_sobolev_smoothing, 5, SAVE_SUCCESSIVE_ITERATIONS);
	GenericWarpTest<MEMORYDEVICE_CPU>(LevelSetAlignmentSwitches(true, false, true, false, true),
	                                  5, GenericWarpTestMode::SAVE_FINAL_ITERATION_AND_FUSION);
}

template<typename TIndex>
void GenerateFusedVolumeTestData() {
	const int iteration = 4;
	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(),
	               "Generating fused volume data from snoopy masked partial volume 16 & 17 warps ("
			               << IndexString<TIndex>() << ") ... ");
	VoxelVolume<TSDFVoxel, TIndex>* warped_live_volume;
	LevelSetAlignmentSwitches data_tikhonov_sobolev_switches(true, false, true, false, true);
	LoadVolume(&warped_live_volume,
	           GetWarpedLivePath<TIndex>(SwitchesToPrefix(data_tikhonov_sobolev_switches), iteration),
	           MEMORYDEVICE_CPU, snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	VoxelVolume<TSDFVoxel, TIndex>* canonical_volume;
	LoadVolume(&canonical_volume, snoopy::PartialVolume16Path<TIndex>(),
	           MEMORYDEVICE_CPU, snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	AllocateUsingOtherVolume(canonical_volume, warped_live_volume, MEMORYDEVICE_CPU);


	VolumeFusionEngineInterface<TSDFVoxel, TIndex>* volume_fusion_engine =
			VolumeFusionEngineFactory::Build<TSDFVoxel, TIndex>(MEMORYDEVICE_CPU);
	volume_fusion_engine->FuseOneTsdfVolumeIntoAnother(canonical_volume, warped_live_volume, 0);

	test_utilities::ConstructGeneratedVolumeSubdirectoriesIfMissing();
	canonical_volume->SaveToDisk(GENERATED_TEST_DATA_PREFIX
	                             "TestData/volumes/" + IndexString<TIndex>() + "/fused.dat");

	delete volume_fusion_engine;
	delete warped_live_volume;
	delete canonical_volume;
}

template<typename TIndex>
void GenerateWarpedVolumeTestData() {
	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(),
	               "Generating warped volume data from snoopy masked partial volume 16 & 17 warps ("
			               << IndexString<TIndex>() << ") ... ");
	VoxelVolume<WarpVoxel, TIndex>* warps;
	LoadVolume(&warps,
	           GENERATED_TEST_DATA_PREFIX "TestData/volumes/" + IndexString<TIndex>() + "/warp_field_0_complete.dat",
	           MEMORYDEVICE_CPU, snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	VoxelVolume<TSDFVoxel, TIndex>* live_volume;
	LoadVolume(&live_volume, snoopy::PartialVolume17Path<TIndex>(),
	           MEMORYDEVICE_CPU, snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	auto warped_live_volume = new VoxelVolume<TSDFVoxel, TIndex>(MEMORYDEVICE_CPU,
	                                                             snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	warped_live_volume->Reset();

	auto warping_engine =
			WarpingEngineFactory::Build<TSDFVoxel, WarpVoxel, TIndex>(MEMORYDEVICE_CPU);

	AllocateUsingOtherVolume(warped_live_volume, live_volume, MEMORYDEVICE_CPU);
	warping_engine->WarpVolume_WarpUpdates(warps, live_volume, warped_live_volume);

	test_utilities::ConstructGeneratedVolumeSubdirectoriesIfMissing();
	warped_live_volume->SaveToDisk(
			GENERATED_TEST_DATA_PREFIX "TestData/volumes/" + IndexString<TIndex>() + "/warped_live.dat");

	delete warping_engine;
	delete warps;
	delete live_volume;
	delete warped_live_volume;
}


configuration::Configuration GenerateDefaultSnoopyConfiguration() {
	using namespace configuration;
	configuration::Configuration
			default_snoopy_configuration(
			Vector3i(0, 0, 0),
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
	MainEngineSettings default_snoopy_main_engine_settings(true, LIBMODE_DYNAMIC, INDEX_HASH, true);
	TelemetrySettings default_snoopy_telemetry_settings;
	IndexingSettings default_snoopy_indexing_settings;
	RenderingSettings default_snoopy_rendering_settings;
	AutomaticRunSettings default_snoopy_automatic_run_settings(716, 16, false, false, false, false);
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
			LevelSetAlignmentTerminationConditions(300, 10, 1e-06)
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
	test_utilities::ConstructGeneratedConfigurationDirectoryIfMissing();
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

template<typename TIndex, MemoryDeviceType TMemoryDeviceType>
void GenerateMeshingTestData() {
	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(),
	               "Generating meshing test data (" << IndexString<TIndex>() << ") ...");
	VoxelVolume<TSDFVoxel, TIndex>* canonical_volume;
	LoadVolume(&canonical_volume, snoopy::PartialVolume16Path<TIndex>(),
	           TMemoryDeviceType, snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	MeshingEngine<TSDFVoxel, TIndex>* meshing_engine =
			MeshingEngineFactory::Build<TSDFVoxel, TIndex>(TMemoryDeviceType);
	Mesh mesh = meshing_engine->MeshVolume(canonical_volume);
	ConstructGeneratedMeshDirectoryIfMissing();
	mesh.WriteOBJ(GENERATED_TEST_DATA_PREFIX "TestData/meshes/mesh_partial_16.obj");

	delete canonical_volume;
	delete meshing_engine;
}


template<MemoryDeviceType TMemoryDeviceType>
void GenerateRenderingTestData_VoxelBlockHash() {
	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(),
	               "Generating VBH rendering test data ... ");
	test_utilities::ConstructGeneratedArraysDirectoryIfMissing();

	CameraPoseAndRenderingEngineFixture<TMemoryDeviceType> fixture;

	ORUtils::OStreamWrapper visible_blocks_file(GENERATED_TEST_DATA_PREFIX "TestData/arrays/visible_blocks.dat", true);
	ORUtils::OStreamWrapper range_images_file(GENERATED_TEST_DATA_PREFIX "TestData/arrays/range_images.dat", true);
	ORUtils::OStreamWrapper raycast_images_file(GENERATED_TEST_DATA_PREFIX "TestData/arrays/raycast_images.dat", true);
	ORUtils::OStreamWrapper point_cloud_images_file(GENERATED_TEST_DATA_PREFIX "TestData/arrays/point_cloud_images.dat", true);
	ORUtils::OStreamWrapper ICP_images_file(GENERATED_TEST_DATA_PREFIX "TestData/arrays/ICP_images.dat", true);
	ORUtils::OStreamWrapper forward_render_images_file(GENERATED_TEST_DATA_PREFIX "TestData/arrays/forward_render_images.dat", true);
	ORUtils::OStreamWrapper rendered_images_file(GENERATED_TEST_DATA_PREFIX "TestData/arrays/rendered_images.dat", true);

	std::vector<int> pose_range_visible_block_counts1, pose_range_visible_block_counts2, pose_range_visible_block_counts3;

	std::vector<int> block_address_range_bounds =
			{1250, 1500,
			 1450, 1800,
			 1750, 2000};
	size_t range_bound_count = block_address_range_bounds.size();
	SaveRawDataToFile<int>(visible_blocks_file, block_address_range_bounds.data(), range_bound_count, MEMORYDEVICE_CPU);

	int i_pose = 0;
	const int pose_count = static_cast<int>(fixture.tracking_states.size());
	for (auto& tracking_state : fixture.tracking_states) {
		LOG4CPLUS_INFO(log4cplus::Logger::getRoot(), "Generating rendering test data for camera pose " << i_pose + 1 << "/" << pose_count);
		i_pose++;

		VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume;
		LoadVolume(&volume, snoopy::PartialVolume17Path<VoxelBlockHash>(), TMemoryDeviceType,
		           snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());

		ORUtils::SE3Pose& pose = *tracking_state->pose_d;
		LOG4CPLUS_INFO(log4cplus::Logger::getRoot(), "Generating raycast/TSDF-surface-finding test data ... ");
		// find visible blocks, count visible blocks
		fixture.rendering_engine->FindVisibleBlocks(volume, &pose, &fixture.calibration_data.intrinsics_d, fixture.render_state);
		int range_visible_block_count1 = fixture.rendering_engine->CountVisibleBlocks(volume, fixture.render_state, block_address_range_bounds[0],
		                                                                              block_address_range_bounds[1]);
		int range_visible_block_count2 = fixture.rendering_engine->CountVisibleBlocks(volume, fixture.render_state, block_address_range_bounds[2],
		                                                                              block_address_range_bounds[3]);
		int range_visible_block_count3 = fixture.rendering_engine->CountVisibleBlocks(volume, fixture.render_state, block_address_range_bounds[4],
		                                                                              block_address_range_bounds[5]);

		pose_range_visible_block_counts1.push_back(range_visible_block_count1);
		pose_range_visible_block_counts2.push_back(range_visible_block_count2);
		pose_range_visible_block_counts3.push_back(range_visible_block_count3);

		const int visible_block_count = volume->index.GetVisibleBlockCount();

		if (visible_block_count == 0) continue; // skip rest for shots without results
		int* visible_codes_device = volume->index.GetVisibleBlockHashCodes();
		SaveRawDataToFile<int>(visible_blocks_file, visible_codes_device, visible_block_count, TMemoryDeviceType);

		LOG4CPLUS_INFO(log4cplus::Logger::getRoot(), "Generating expected depth estimation test data ... ");
		// create expected depths (in legacy InfiniTAM, fills "renderingRangeImage" of the "render state")
		{
			std::shared_ptr<RenderState> render_state_expected_depths = fixture.MakeRenderState();
			fixture.rendering_engine->CreateExpectedDepths(volume, &pose, &fixture.calibration_data.intrinsics_d,
			                                               render_state_expected_depths.get());
			ORUtils::MemoryBlockPersistence::SaveImage(range_images_file, *render_state_expected_depths->renderingRangeImage);
		}

		LOG4CPLUS_INFO(log4cplus::Logger::getRoot(), "Generating raycast/TSDF-surface-finding test data ... ");
		// find surface (in legacy InfiniTAM, fills "raycastResult" of the render state)
		{
			std::shared_ptr<RenderState> render_state_find_surface = fixture.MakeRenderState();
			fixture.rendering_engine->CreateExpectedDepths(volume, &pose, &fixture.calibration_data.intrinsics_d,
			                                               render_state_find_surface.get());
			fixture.rendering_engine->FindSurface(volume, &pose, &fixture.calibration_data.intrinsics_d, render_state_find_surface.get());
			ORUtils::MemoryBlockPersistence::SaveImage(raycast_images_file, *render_state_find_surface->raycastResult);
		}
		LOG4CPLUS_INFO(log4cplus::Logger::getRoot(), "Generating raycast-to-point-cloud conversion test data ... ");
		// create point cloud (in legacy InfiniTAM, fills "raycastResult" of "render state", locations & colors of the point cloud in the "tracking state")
		{

			std::shared_ptr<RenderState> render_state_create_point_cloud = fixture.MakeRenderState();
			fixture.rendering_engine->CreateExpectedDepths(volume, &pose, &fixture.calibration_data.intrinsics_d,
			                                               render_state_create_point_cloud.get());
			fixture.rendering_engine->CreatePointCloud(volume, fixture.view_17, tracking_state.get(), render_state_create_point_cloud.get());

			point_cloud_images_file.OStream().write(reinterpret_cast<const char*>(&(tracking_state->pointCloud->noTotalPoints)),
			                                        sizeof(unsigned int));
			ORUtils::MemoryBlockPersistence::SaveImage(point_cloud_images_file, *tracking_state->pointCloud->locations);
			ORUtils::MemoryBlockPersistence::SaveImage(point_cloud_images_file, *tracking_state->pointCloud->colours);
		}

		LOG4CPLUS_INFO(log4cplus::Logger::getRoot(), "Generating raycast-to-ICP-map (projected point cloud) conversion test data ... ");
		// create icp maps (in legacy InfiniTAM, fills "raycastResult" of "render state", locations and "normals" of pointCloud)
		{
			// colors -- interpreted as normals -- honestly, WTF, Oxford? Yeah, I'm blaming you, Oxford, you heard me! -- of the point cloud in the "tracking state")
			std::shared_ptr<RenderState> render_state_create_ICP_maps = fixture.MakeRenderState();
			fixture.rendering_engine->CreateExpectedDepths(volume, &pose, &fixture.calibration_data.intrinsics_d,
			                                               render_state_create_ICP_maps.get());
			fixture.rendering_engine->CreateICPMaps(volume, fixture.view_17, tracking_state.get(), render_state_create_ICP_maps.get());
			ORUtils::MemoryBlockPersistence::SaveImage(ICP_images_file, *tracking_state->pointCloud->locations);
			ORUtils::MemoryBlockPersistence::SaveImage(ICP_images_file, *tracking_state->pointCloud->colours);
		}

		LOG4CPLUS_INFO(log4cplus::Logger::getRoot(),
		               "Generating 'forward-rendering' test data ('forward-rendering' is rendering a new point cloud from a different camera vantage point while reusing previous data as much as possible) ... ");
		//forward-render
		{
			std::shared_ptr<RenderState> render_state_forward_render = fixture.MakeRenderState();

			std::shared_ptr<CameraTrackingState> tracking_state_forward_render = fixture.MakeCameraTrackingState();
			ORUtils::SE3Pose& adjusted_pose = *tracking_state_forward_render->pose_d;
			adjusted_pose.SetFrom(&pose);
			// nudge the pose a bit to imitate tracking result
			std::vector<float> adjustment_values = {0.02f, 0.02f, -0.02f, 1.f * PI / 180.f, 3.f * PI / 180.f, 0.0f};
			SaveRawDataToFile<float>(forward_render_images_file, adjustment_values.data(), adjustment_values.size(), MEMORYDEVICE_CPU);
			ORUtils::SE3Pose adjustment(adjustment_values[0], adjustment_values[1], adjustment_values[2], adjustment_values[3], adjustment_values[4],
			                            adjustment_values[5]);
			adjusted_pose.MultiplyWith(&adjustment);

			fixture.rendering_engine->CreateExpectedDepths(volume, &adjusted_pose, &fixture.calibration_data.intrinsics_d,
			                                               render_state_forward_render.get());
			fixture.rendering_engine->FindSurface(volume, &adjusted_pose, &fixture.calibration_data.intrinsics_d,
			                                      render_state_forward_render.get());
			fixture.rendering_engine->ForwardRender(volume, fixture.view_17, tracking_state_forward_render.get(),
			                                        render_state_forward_render.get());
			forward_render_images_file.OStream().write(reinterpret_cast<const char*>(&(render_state_forward_render->noFwdProjMissingPoints)),
			                                           sizeof(int));
			ORUtils::MemoryBlockPersistence::SaveImage(forward_render_images_file, *render_state_forward_render->fwdProjMissingPoints);
			ORUtils::MemoryBlockPersistence::SaveImage(forward_render_images_file, *render_state_forward_render->forwardProjection);
		}

		LOG4CPLUS_INFO(log4cplus::Logger::getRoot(),
		               "Generating test data for render actual images based on raycasting in the TSDF (with various settings)...");
		// render image
		{
			std::shared_ptr<RenderState> render_state_render_image = fixture.MakeRenderState();
			fixture.rendering_engine->CreateExpectedDepths(volume, &pose, &fixture.calibration_data.intrinsics_d,
			                                               render_state_render_image.get());
			fixture.rendering_engine->FindSurface(volume, &pose, &fixture.calibration_data.intrinsics_d, render_state_render_image.get());


			UChar4Image output_image(snoopy::frame_image_size, TMemoryDeviceType);
			fixture.rendering_engine->RenderImage(volume, &pose, &fixture.calibration_data.intrinsics_d, render_state_render_image.get(),
			                                      &output_image, IRenderingEngine::RenderImageType::RENDER_COLOUR_FROM_VOLUME,
			                                      IRenderingEngine::RenderRaycastSelection::RENDER_FROM_OLD_RAYCAST);
			ORUtils::MemoryBlockPersistence::SaveImage(rendered_images_file, output_image);
			fixture.rendering_engine->RenderImage(volume, &pose, &fixture.calibration_data.intrinsics_d, render_state_render_image.get(),
			                                      &output_image, IRenderingEngine::RenderImageType::RENDER_COLOUR_FROM_NORMAL,
			                                      IRenderingEngine::RenderRaycastSelection::RENDER_FROM_OLD_RAYCAST);
			ORUtils::MemoryBlockPersistence::SaveImage(rendered_images_file, output_image);
			fixture.rendering_engine->RenderImage(volume, &pose, &fixture.calibration_data.intrinsics_d, render_state_render_image.get(),
			                                      &output_image, IRenderingEngine::RenderImageType::RENDER_SHADED_GREYSCALE_IMAGENORMALS,
			                                      IRenderingEngine::RenderRaycastSelection::RENDER_FROM_OLD_RAYCAST);
			ORUtils::MemoryBlockPersistence::SaveImage(rendered_images_file, output_image);
			fixture.rendering_engine->RenderImage(volume, &pose, &fixture.calibration_data.intrinsics_d, render_state_render_image.get(),
			                                      &output_image, IRenderingEngine::RenderImageType::RENDER_SHADED_GREEN,
			                                      IRenderingEngine::RenderRaycastSelection::RENDER_FROM_OLD_RAYCAST);
			ORUtils::MemoryBlockPersistence::SaveImage(rendered_images_file, output_image);
			fixture.rendering_engine->RenderImage(volume, &pose, &fixture.calibration_data.intrinsics_d, render_state_render_image.get(),
			                                      &output_image, IRenderingEngine::RenderImageType::RENDER_SHADED_OVERLAY,
			                                      IRenderingEngine::RenderRaycastSelection::RENDER_FROM_OLD_RAYCAST);
			ORUtils::MemoryBlockPersistence::SaveImage(rendered_images_file, output_image);
		}

		delete volume;
	}

	SaveRawDataToFile<int>(visible_blocks_file, pose_range_visible_block_counts1.data(), pose_count, MEMORYDEVICE_CPU);
	SaveRawDataToFile<int>(visible_blocks_file, pose_range_visible_block_counts2.data(), pose_count, MEMORYDEVICE_CPU);
	SaveRawDataToFile<int>(visible_blocks_file, pose_range_visible_block_counts3.data(), pose_count, MEMORYDEVICE_CPU);
}


#define GENERATED_TEST_DATA_TYPE_ENUM_DESCRIPTION GeneratedTestDataType, \
    (SNOOPY_UNMASKED_VOLUMES,    "SNOOPY_UNMASKED_VOLUMES", "snoopy_unmasked_volumes", "unmasked_volumes", "unmasked", "u", "su", "suv"), \
    (MASKED_VOLUMES,             "SNOOPY_MASKED_VOLUMES", "snoopy_masked_volumes", "masked_volumes", "masked", "sm", "smv", "mv"), \
    (PVA_WARP_GRADIENTS,         "PVA_WARP_GRADIENTS", "pva_warp_gradients", "pva_warps", "pw", "pva_w", "pva_wg"), \
    (VBH_WARP_GRADIENTS,         "VBH_WARP_GRADIENTS", "vbh_warp_gradients", "vbh_warps", "vw", "vbh_w", "vbh_wg"), \
    (COMPARATIVE_WARP_GRADIENTS, "COMPARATIVE_WARP_GRADIENTS", "comparative_warp_gradients", "warps", "comparative_warps", "w", "cw"), \
    (PVA_WARPED_VOLUMES,         "PVA_WARPED_VOLUMES", "pva_warped_volumes", "pva_wv"), \
    (VBH_WARPED_VOLUMES,         "VBH_WARPED_VOLUMES", "vbh_warped_volumes", "vbh_wv"), \
    (PVA_FUSED_VOLUMES,          "PVA_FUSED_VOLUMES", "pva_fused_volumes", "pva_fv"), \
    (VBH_FUSED_VOLUMES,          "VBH_FUSED_VOLUMES", "vbh_fused_volumes", "vbh_fv"), \
    (CONFUGRATIONS,              "CONFIGURATIONS", "configurations", "config", "c"), \
    (MESHES,                     "MESHES", "meshes", "m"), \
    (RENDERING,                  "RENDERING", "rendering", "r")

GENERATE_SERIALIZABLE_ENUM(GENERATED_TEST_DATA_TYPE_ENUM_DESCRIPTION);

int main(int argc, char* argv[]) {
	log4cplus::initialize();
	log4cplus::SharedAppenderPtr console_appender(new log4cplus::ConsoleAppender(false, true));
	log4cplus::Logger::getRoot().addAppender(console_appender);
	log4cplus::Logger::getRoot().setLogLevel(log4cplus::DEBUG_LOG_LEVEL);

	std::map<GeneratedTestDataType, std::function<void()>> generator_by_string(
			{
					{SNOOPY_UNMASKED_VOLUMES,    ConstructSnoopyUnmaskedVolumes00},
					{MASKED_VOLUMES,             ConstructSnoopyMaskedVolumes16and17},
					{PVA_WARP_GRADIENTS,         GenerateWarpGradientTestData<PlainVoxelArray, MEMORYDEVICE_CPU>},
					{VBH_WARP_GRADIENTS,         GenerateWarpGradientTestData<VoxelBlockHash, MEMORYDEVICE_CPU>},
					{COMPARATIVE_WARP_GRADIENTS, GenerateWarpGradient_PVA_to_VBH_TestData},
					{PVA_WARPED_VOLUMES,         GenerateWarpedVolumeTestData<PlainVoxelArray>},
					{VBH_WARPED_VOLUMES,         GenerateWarpedVolumeTestData<VoxelBlockHash>},
					{PVA_FUSED_VOLUMES,          GenerateFusedVolumeTestData<PlainVoxelArray>},
					{VBH_FUSED_VOLUMES,          GenerateFusedVolumeTestData<VoxelBlockHash>},
					{CONFUGRATIONS,              GenerateConfigurationTestData},
					{MESHES,                     GenerateMeshingTestData<VoxelBlockHash, MEMORYDEVICE_CPU>},
					{RENDERING,                  GenerateRenderingTestData_VoxelBlockHash<MEMORYDEVICE_CPU>}
			});
	if (argc < 2) {
		// calls every generator iteratively
		for (const auto& iter : generator_by_string) {
			(iter.second)();
		}
	} else {
		std::string generated_data_type_argument = argv[1];
		bool wrong_second_argument = argc >= 3 && strcmp(argv[2], "-c") != 0;
		if (generated_data_type_argument == "h" || generated_data_type_argument == "help" || generated_data_type_argument == "-h" ||
		    generated_data_type_argument == "-help" || generated_data_type_argument == "--help" || wrong_second_argument) {
			std::cout << "Generates derived data used for testing the library. " << std::endl;
			std::cout << "Usage:" << std::endl << "generate_derived_test_data " << std::endl << "(runs all modes)  -- OR -- "
			          << std::endl << "generate_derived_test_data <mode> [-c]" << std::endl <<
			          ", where <mode> can be one of: " << std::endl;
			int i_pair = 0;
			for (auto& pair : generator_by_string) {
				if (i_pair < generator_by_string.size() - 1) {
					std::cout << enumerator_to_string(pair.first) << ", " << std::endl;
				} else {
					std::cout << "or " << enumerator_to_string(pair.first) << ".";
				}
				i_pair++;
			}
			std::cout << std::endl;
			std::cout << "For any of these, shorthands can be used, which are typically acronyms with some words omitted"
			             ", e.g. \"suv\" can be used instead of \"SNOOPY_UNMASKED_VOLUMES\" and \"pva_wv\" instead of \"PVA_WARPED_VOLUMES\". "
			             "Don't be afraid to experiment." << std::endl;
			std::cout << "If -c (\"continue\") flag is passed, all generators including and after the specified one in the sequence are called in order." << std::endl;
		} else if(argc < 3){
			GeneratedTestDataType chosen = string_to_enumerator<GeneratedTestDataType>(generated_data_type_argument);
			std::cout << "current path: " << std::filesystem::current_path() << std::endl;
			std::cout << "Generating data using the " << enumerator_to_string(chosen) << " generator." << std::endl;
			generator_by_string[chosen]();
		} else {
			bool hit_start_generator = false;
			GeneratedTestDataType chosen = string_to_enumerator<GeneratedTestDataType>(generated_data_type_argument);
			for (const auto& iter : generator_by_string) {
				if(iter.first == chosen || hit_start_generator){
					(iter.second)();
					hit_start_generator = true;
				}
			}
		}
	}
	return 0;
}