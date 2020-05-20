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


//ITMLib
#include "../ITMLib/SurfaceTrackers/Interface/SurfaceTracker.h"
#include "../ITMLib/Engines/VolumeFusion/VolumeFusionEngine.h"
#include "../ITMLib/Engines/Indexing/IndexingEngineFactory.h"
#include "../ITMLib/Engines/VolumeFusion/VolumeFusionEngineFactory.h"
#include "../ITMLib/Engines/Warping/WarpingEngineFactory.h"
#include "../ITMLib/Engines/Meshing/MeshingEngineFactory.h"
#include "../ITMLib/Engines/EditAndCopy/EditAndCopyEngineFactory.h"
#include "../ITMLib/Utils/Configuration/Configuration.h"
#include "../ITMLib/Utils/Metacoding/SerializableEnum.h"
#include "../ITMLib/Objects/Camera/CalibIO.h"
#include "../ITMLib/Utils/Quaternions/Quaternion.h"

//(CPU)
#include "../ITMLib/Engines/EditAndCopy/CPU/EditAndCopyEngine_CPU.h"
#include "../ITMLib/Engines/Indexing/VBH/CPU/IndexingEngine_VoxelBlockHash_CPU.h"
//(CUDA)
#ifndef COMPILE_WITHOUT_CUDA

#include "../ITMLib/Engines/EditAndCopy/CUDA/EditAndCopyEngine_CUDA.h"
#include "../ITMLib/Engines/Indexing/VBH/CUDA/IndexingEngine_VoxelBlockHash_CUDA.h"

#endif

//test_utilities
#include "TestUtilities/TestUtilities.h"
#include "TestUtilities/SnoopyTestUtilities.h"
#include "TestUtilities/WarpAdvancedTestingUtilities.h"
#include "../ORUtils/FileUtils.h"
#include "../ITMLib/Engines/ViewBuilding/ViewBuilderFactory.h"
#include "../ITMLib/Engines/DepthFusion/DepthFusionEngineFactory.h"
#include "../ITMLib/Engines/Meshing/MeshingEngineFactory.h"
#include "../ITMLib/Objects/Meshing/Mesh.h"
#include "../ITMLib/Utils/Configuration/TelemetrySettings.h"
#include "../ITMLib/Engines/Visualization/VisualizationEngineFactory.h"

//local
#include <log4cplus/loggingmacros.h>
#include <log4cplus/consoleappender.h>

using namespace ITMLib;
using namespace test_utilities;
namespace snoopy = snoopy_test_utilities;

namespace fs = std::filesystem;

void ConstructSnoopyUnmaskedVolumes00() {
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
	volume_PVA_00->SaveToDisk(GENERATED_TEST_DATA_PREFIX + snoopy::PartialVolume00Path<PlainVoxelArray>());
	volume_VBH_00->SaveToDisk(GENERATED_TEST_DATA_PREFIX + snoopy::PartialVolume00Path<VoxelBlockHash>());

	delete volume_PVA_00;
	delete volume_VBH_00;
}

void ConstructSnoopyMaskedVolumes16and17() {
	LOG4CPLUS_DEBUG(log4cplus::Logger::getRoot(), "Constructing snoopy masked partial volumes 16 & 17...");

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

	LOG4CPLUS_DEBUG(log4cplus::Logger::getRoot(), "Saving snoopy masked partial volumes 16 & 17...");

	volume_PVA_16->SaveToDisk(GENERATED_TEST_DATA_PREFIX + snoopy::PartialVolume16Path<PlainVoxelArray>());
	volume_VBH_16->SaveToDisk(GENERATED_TEST_DATA_PREFIX + snoopy::PartialVolume16Path<VoxelBlockHash>());
	volume_PVA_17->SaveToDisk(GENERATED_TEST_DATA_PREFIX + snoopy::PartialVolume17Path<PlainVoxelArray>());
	volume_VBH_17->SaveToDisk(GENERATED_TEST_DATA_PREFIX + snoopy::PartialVolume17Path<VoxelBlockHash>());

	volume_PVA_16->Reset();
	volume_VBH_16->Reset();
	volume_PVA_17->Reset();
	volume_VBH_17->Reset();

	LOG4CPLUS_DEBUG(log4cplus::Logger::getRoot(), "Constructing snoopy masked full volumes 16 & 17 PVA...");

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

	LOG4CPLUS_DEBUG(log4cplus::Logger::getRoot(), "Constructing snoopy masked full volumes 16 & 17 VBH...");

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

	LOG4CPLUS_DEBUG(log4cplus::Logger::getRoot(), "Saving snoopy masked full volume 16 PVA...");
	volume_PVA_16->SaveToDisk(GENERATED_TEST_DATA_PREFIX + snoopy::FullVolume16Path<PlainVoxelArray>());
	LOG4CPLUS_DEBUG(log4cplus::Logger::getRoot(), "Saving snoopy masked full volume 16 VBH...");
	volume_VBH_16->SaveToDisk(GENERATED_TEST_DATA_PREFIX + snoopy::FullVolume16Path<VoxelBlockHash>());
	LOG4CPLUS_DEBUG(log4cplus::Logger::getRoot(), "Saving snoopy masked full volume 17 PVA...");
	volume_PVA_17->SaveToDisk(GENERATED_TEST_DATA_PREFIX + snoopy::FullVolume17Path<PlainVoxelArray>());
	LOG4CPLUS_DEBUG(log4cplus::Logger::getRoot(), "Saving snoopy masked full volume 17 VBH...");
	volume_VBH_17->SaveToDisk(GENERATED_TEST_DATA_PREFIX + snoopy::FullVolume17Path<VoxelBlockHash>());

	delete volume_PVA_16;
	delete volume_VBH_16;
	delete volume_PVA_17;
	delete volume_VBH_17;
}

void ConstructStripesTestVolumes() {
	// region ================================= CONSTRUCT VIEW =========================================================

	RGBDCalib calibration_data;
	readRGBDCalib(snoopy::SnoopyCalibrationPath().c_str(), calibration_data);

	auto view_builder = ViewBuilderFactory::Build(calibration_data, MEMORYDEVICE_CPU);
	Vector2i image_size(640, 480);
	View* view = nullptr;

	UChar4Image rgb(true, false);
	ShortImage depth(true, false);
	ReadImageFromFile(rgb, "TestData/frames/stripes_color.png");
	ReadImageFromFile(depth, "TestData/frames/stripes_depth.png");

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
	std::string path = "TestData/volumes/VBH/stripes.dat";
	volume4.SaveToDisk(path);

	delete depth_fusion_engine_VBH;
	delete view;
	delete view_builder;
}

template<typename TIndex, MemoryDeviceType TMemoryDeviceType>
void GenerateWarpGradientTestData() {
	LOG4CPLUS_DEBUG(log4cplus::Logger::getRoot(),
	                "Generating warp field data from snoopy masked partial volumes 16 & 17 "
			                << IndexString<TIndex>() << "...");

	std::string output_directory = GENERATED_TEST_DATA_PREFIX "TestData/volumes/" + IndexString<TIndex>() + "/";

	VoxelVolume<TSDFVoxel, TIndex>* canonical_volume;
	VoxelVolume<TSDFVoxel, TIndex>* live_volume;
	LoadVolume(&live_volume, output_directory + "snoopy_partial_frame_17.dat", TMemoryDeviceType,
	           snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	LoadVolume(&canonical_volume, output_directory + "snoopy_partial_frame_16.dat", TMemoryDeviceType,
	           snoopy::InitializationParameters_Fr16andFr17<TIndex>());

	SlavchevaSurfaceTracker::Switches data_only_switches(true, false, false, false, false);
	std::string data_only_filename = "warp_field_0_data.dat";
	SlavchevaSurfaceTracker::Switches data_smoothed_switches(false, false, false, false, true);
	std::string data_smoothed_filename = "warp_field_0_smoothed.dat";
	std::string framewise_warps_filename = "warp_field_0_data_framewise_warps.dat";
	SlavchevaSurfaceTracker::Switches warp_complete_switches(true, false, true, false, true);
	std::string warp_complete_filename = "warp_field_0_complete.dat";

	std::vector<std::tuple<std::string, SlavchevaSurfaceTracker::Switches>> configuration_pairs = {
			std::make_tuple(std::string("warp_field_1_tikhonov.dat"),
			                SlavchevaSurfaceTracker::Switches(false, false, true, false, false)),
			std::make_tuple(std::string("warp_field_1_data_and_tikhonov.dat"),
			                SlavchevaSurfaceTracker::Switches(true, false, true, false, false)),
			std::make_tuple(std::string("warp_field_1_data_and_killing.dat"),
			                SlavchevaSurfaceTracker::Switches(true, false, true, true, false)),
			std::make_tuple(std::string("warp_field_1_data_and_level_set.dat"),
			                SlavchevaSurfaceTracker::Switches(true, true, false, false, false))
	};

	VoxelVolume<WarpVoxel, TIndex> warp_field(TMemoryDeviceType,
	                                          snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	warp_field.Reset();
	AllocateUsingOtherVolume(&warp_field, live_volume, MEMORYDEVICE_CPU);
	AllocateUsingOtherVolume(canonical_volume, live_volume, MEMORYDEVICE_CPU);

	SurfaceTracker<TSDFVoxel, WarpVoxel, TIndex, TMemoryDeviceType, TRACKER_SLAVCHEVA_DIAGNOSTIC> dataOnlyMotionTracker(
			data_only_switches);

	dataOnlyMotionTracker.CalculateWarpGradient(&warp_field, canonical_volume, live_volume);
	warp_field.SaveToDisk(output_directory + data_only_filename);

	SurfaceTracker<TSDFVoxel, WarpVoxel, TIndex, TMemoryDeviceType, TRACKER_SLAVCHEVA_DIAGNOSTIC> dataSmoothedMotionTracker(
			data_smoothed_switches);
	dataSmoothedMotionTracker.SmoothWarpGradient(&warp_field, canonical_volume, live_volume);
	warp_field.SaveToDisk(output_directory + data_smoothed_filename);

	warp_field.Reset();
	AllocateUsingOtherVolume(&warp_field, live_volume, TMemoryDeviceType);
	SurfaceTracker<TSDFVoxel, WarpVoxel, TIndex, TMemoryDeviceType, TRACKER_SLAVCHEVA_DIAGNOSTIC> completeMotionTracker(
			warp_complete_switches);
	completeMotionTracker.CalculateWarpGradient(&warp_field, canonical_volume, live_volume);
	completeMotionTracker.SmoothWarpGradient(&warp_field, canonical_volume, live_volume);
	completeMotionTracker.UpdateWarps(&warp_field, canonical_volume, live_volume);
	warp_field.SaveToDisk(output_directory + warp_complete_filename);

	warp_field.Reset();
	AllocateUsingOtherVolume(&warp_field, live_volume, TMemoryDeviceType);
	warp_field.LoadFromDisk(output_directory + data_only_filename);

	dataOnlyMotionTracker.UpdateWarps(&warp_field, canonical_volume, live_volume);
	warp_field.SaveToDisk(output_directory + framewise_warps_filename);

	for (auto& pair : configuration_pairs) {
		EditAndCopyEngineFactory::Instance<WarpVoxel, TIndex, TMemoryDeviceType>().ResetVolume(&warp_field);
		warp_field.LoadFromDisk(output_directory + framewise_warps_filename);
		std::string filename = std::get<0>(pair);
		SurfaceTracker<TSDFVoxel, WarpVoxel, TIndex, TMemoryDeviceType, TRACKER_SLAVCHEVA_DIAGNOSTIC> tracker(
				std::get<1>(pair));
		tracker.CalculateWarpGradient(&warp_field, canonical_volume, live_volume);
		warp_field.SaveToDisk(output_directory + filename);
	}

	delete canonical_volume;
	delete live_volume;
}

void GenerateWarpGradient_PVA_to_VBH_TestData() {
	LOG4CPLUS_DEBUG(log4cplus::Logger::getRoot(),
	                "Generating multi-iteration warp field data from snoopy masked partial volumes 16 & 17 (PVA & VBH)... ");
	SlavchevaSurfaceTracker::Switches switches_data_only(true, false, false, false, false);
	GenericWarpTest<MEMORYDEVICE_CPU>(switches_data_only, 10, SAVE_SUCCESSIVE_ITERATIONS);
	SlavchevaSurfaceTracker::Switches switches_data_and_tikhonov(true, false, true, false, false);
	GenericWarpTest<MEMORYDEVICE_CPU>(switches_data_and_tikhonov, 5, SAVE_SUCCESSIVE_ITERATIONS);
	SlavchevaSurfaceTracker::Switches switches_data_and_tikhonov_and_sobolev_smoothing(true, false, true, false, true);
	GenericWarpTest<MEMORYDEVICE_CPU>(switches_data_and_tikhonov_and_sobolev_smoothing, 5, SAVE_SUCCESSIVE_ITERATIONS);
	GenericWarpTest<MEMORYDEVICE_CPU>(SlavchevaSurfaceTracker::Switches(true, false, true, false, true),
	                                  5, GenericWarpTestMode::SAVE_FINAL_ITERATION_AND_FUSION);
}

template<typename TIndex>
void GenerateFusedVolumeTestData() {
	const int iteration = 4;
	LOG4CPLUS_DEBUG(log4cplus::Logger::getRoot(),
	                "Generating fused volume data from snoopy masked partial volume 16 & 17 warps ("
			                << IndexString<TIndex>() << ") ... ");
	VoxelVolume<TSDFVoxel, TIndex>* warped_live_volume;
	SlavchevaSurfaceTracker::Switches data_tikhonov_sobolev_switches(true, false, true, false, true);
	LoadVolume(&warped_live_volume,
	           GENERATED_TEST_DATA_PREFIX +
	           GetWarpedLivePath<TIndex>(SwitchesToPrefix(data_tikhonov_sobolev_switches), iteration),
	           MEMORYDEVICE_CPU, snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	VoxelVolume<TSDFVoxel, TIndex>* canonical_volume;
	LoadVolume(&canonical_volume, GENERATED_TEST_DATA_PREFIX + snoopy::PartialVolume16Path<TIndex>(),
	           MEMORYDEVICE_CPU, snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	AllocateUsingOtherVolume(canonical_volume, warped_live_volume, MEMORYDEVICE_CPU);


	VolumeFusionEngineInterface<TSDFVoxel, TIndex>* volume_fusion_engine =
			VolumeFusionEngineFactory::Build<TSDFVoxel, TIndex>(MEMORYDEVICE_CPU);
	volume_fusion_engine->FuseOneTsdfVolumeIntoAnother(canonical_volume, warped_live_volume, 0);

	canonical_volume->SaveToDisk(GENERATED_TEST_DATA_PREFIX
	                             "TestData/volumes/" + IndexString<TIndex>() + "/fused.dat");

	delete volume_fusion_engine;
	delete warped_live_volume;
	delete canonical_volume;
}

template<typename TIndex>
void GenerateWarpedVolumeTestData() {
	LOG4CPLUS_DEBUG(log4cplus::Logger::getRoot(),
	                "Generating warped volume data from snoopy masked partial volume 16 & 17 warps ("
			                << IndexString<TIndex>() << ") ... ");
	VoxelVolume<WarpVoxel, TIndex>* warps;
	LoadVolume(&warps,
	           GENERATED_TEST_DATA_PREFIX "TestData/volumes/" + IndexString<TIndex>() + "/warp_field_0_complete.dat",
	           MEMORYDEVICE_CPU, snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	VoxelVolume<TSDFVoxel, TIndex>* live_volume;
	LoadVolume(&live_volume, GENERATED_TEST_DATA_PREFIX + snoopy::PartialVolume17Path<TIndex>(),
	           MEMORYDEVICE_CPU, snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	auto warped_live_volume = new VoxelVolume<TSDFVoxel, TIndex>(MEMORYDEVICE_CPU,
	                                                             snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	warped_live_volume->Reset();

	auto warping_engine =
			WarpingEngineFactory::Build<TSDFVoxel, WarpVoxel, TIndex>(MEMORYDEVICE_CPU);

	AllocateUsingOtherVolume(warped_live_volume, live_volume, MEMORYDEVICE_CPU);
	warping_engine->WarpVolume_WarpUpdates(warps, live_volume, warped_live_volume);

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
			VoxelVolumeParameters(0.004, 0.2, 3.0, 0.04, 100, false, 1.0f),
			SurfelVolumeParameters(),
			SpecificVolumeParameters(
					ArrayVolumeParameters(),
					HashVolumeParameters(
							VoxelBlockHash::VoxelBlockHashParameters(0x40000, 0x20000),
							VoxelBlockHash::VoxelBlockHashParameters(0x20000, 0x20000),
							VoxelBlockHash::VoxelBlockHashParameters(0x20000, 0x20000)
					)
			),
			SlavchevaSurfaceTracker::Parameters(
					0.2f,
					0.1f,
					2.0f,
					0.2f,
					0.2f,
					1e-5f),
			SlavchevaSurfaceTracker::Switches(true, false, true, false, true),
			LoggingSettings(VERBOSITY_WARNING,
			                true,
			                true,
			                true,
			                false,
			                false,
			                false),
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
			configuration::TrackerConfigurationStringPresets::default_intensity_depth_extended_tracker_configuration
	);
	default_snoopy_configuration.source_tree = default_snoopy_configuration.ToPTree();
	TelemetrySettings default_snoopy_telemetry_settings;
	IndexingSettings default_snoopy_indexing_settings;
	AddDeferrableToSourceTree(default_snoopy_configuration, default_snoopy_telemetry_settings);
	AddDeferrableToSourceTree(default_snoopy_configuration, default_snoopy_indexing_settings);
	return default_snoopy_configuration;
}

void GenerateConfigurationTestData() {
	using namespace configuration;
	configuration::Configuration changed_up_configuration = GenerateChangedUpConfiguration();
	LOG4CPLUS_DEBUG(log4cplus::Logger::getRoot(),
	                "Generating configuration test data ... ");
	configuration::Configuration default_snoopy_configuration = GenerateDefaultSnoopyConfiguration();
	configuration::save_configuration_to_json_file(GENERATED_TEST_DATA_PREFIX
	                                               "../Files/infinitam_snoopy_config.json",
	                                               default_snoopy_configuration);
	configuration::Configuration default_configuration;
	default_configuration.device_type = MEMORYDEVICE_CPU;
	configuration::save_configuration_to_json_file(GENERATED_TEST_DATA_PREFIX
	                                               "TestData/configuration/default_config_cpu.json",
	                                               default_configuration);
	default_configuration.device_type = MEMORYDEVICE_CUDA;
	configuration::save_configuration_to_json_file(GENERATED_TEST_DATA_PREFIX
	                                               "TestData/configuration/default_config_cuda.json",
	                                               default_configuration);
	configuration::save_configuration_to_json_file(GENERATED_TEST_DATA_PREFIX
	                                               "TestData/configuration/config1.json", changed_up_configuration);
}

template<typename TIndex, MemoryDeviceType TMemoryDeviceType>
void GenerateMeshingTestData() {
	VoxelVolume<TSDFVoxel, TIndex>* canonical_volume;
	LoadVolume(&canonical_volume, snoopy::PartialVolume16Path<TIndex>(),
	           TMemoryDeviceType, snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	MeshingEngine<TSDFVoxel, TIndex>* meshing_engine =
			MeshingEngineFactory::Build<TSDFVoxel, TIndex>(TMemoryDeviceType);
	Mesh mesh = meshing_engine->MeshVolume(canonical_volume);
	fs::create_directories(GENERATED_TEST_DATA_PREFIX "TestData/meshes");
	mesh.WriteOBJ(GENERATED_TEST_DATA_PREFIX "TestData/meshes/mesh_partial_16.obj");

	delete canonical_volume;
	delete meshing_engine;
}


template<MemoryDeviceType TMemoryDeviceType>
void GenerateRaytracingTestData_VoxelBlockHash() {
	using namespace quaternion;
	VisualizationEngine<TSDFVoxel, VoxelBlockHash>* visualization_engine = VisualizationEngineFactory::MakeVisualizationEngine<TSDFVoxel, VoxelBlockHash>(
			TMemoryDeviceType);
	RGBDCalib calibration_data;
	readRGBDCalib(snoopy::SnoopyCalibrationPath().c_str(), calibration_data);
	RenderState* render_state = new RenderState(Vector2i(640, 480),
	                                            configuration::get().general_voxel_volume_parameters.near_clipping_distance,
	                                            configuration::get().general_voxel_volume_parameters.far_clipping_distance,
	                                            TMemoryDeviceType);
	VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume;
	LoadVolume(&volume, snoopy::PartialVolume17Path<VoxelBlockHash>(), TMemoryDeviceType,
	           snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());

	Vector3f target(-0.09150545, 0.07265271, 0.7908916);
	Vector3f x_axis_unit(1.0f, 0.0f, 0.0f);
	Vector3f rotation_axis = ORUtils::normalize(ORUtils::cross(target, x_axis_unit));

	Vector3f original_viewpoint(0.f);

	Vector3f target_to_viewpoint = original_viewpoint - target;
	Quaternion<float> target_to_viewpoint_quaternion(0.f, target_to_viewpoint.x, target_to_viewpoint.y,
	                                                 target_to_viewpoint.z);
	float angle_radians = 0.0;
	for (int i_sample = 0; i_sample < (360 / 45 /*degrees*/); i_sample++) {
		VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume;
		LoadVolume(&volume, snoopy::PartialVolume17Path<VoxelBlockHash>(), TMemoryDeviceType,
		           snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());

		float half_angle = angle_radians / 2.0f; // for code clarity, math not baked in
		float cos_theta = cos(half_angle);
		float sin_theta = sin(half_angle);
		Quaternion<float> half_rotation_a(cos_theta, rotation_axis.x * sin_theta, rotation_axis.y * sin_theta,
		                                  rotation_axis.z * sin_theta);
		Quaternion<float> half_rotation_b(cos_theta, -rotation_axis.x * sin_theta, -rotation_axis.y * sin_theta,
		                                  -rotation_axis.z * sin_theta);
		Quaternion<float> rotation_result = (half_rotation_a *= target_to_viewpoint_quaternion) *= half_rotation_b;
		Vector3f rotated_target_to_viewpoint(rotation_result.b(), rotation_result.c(), rotation_result.d());
		Vector3f viewpoint = target + rotated_target_to_viewpoint;
		angle_radians += (PI / 4.0f);
		Vector3f viewpoint_to_target_normalized = ORUtils::normalize(-rotated_target_to_viewpoint);
		float angle_x = atan2(viewpoint_to_target_normalized.z, viewpoint_to_target_normalized.y);
//		float angle_x = 0 * PI;
		float angle_y = atan2(viewpoint_to_target_normalized.z, viewpoint_to_target_normalized.x);
//		float angle_z = atan2(viewpoint_to_target_normalized.y, viewpoint_to_target_normalized.x);
		float angle_z = 0 * PI;
		std::cout << "rx: " << angle_x * 180 / PI << " ry: " << angle_y * 180 / PI<< " rz: " << angle_z * 180 / PI<< std::endl;
		ORUtils::SE3Pose pose(viewpoint.x, viewpoint.y, viewpoint.z, angle_x, angle_y, angle_z);
		//ORUtils::SE3Pose pose(viewpoint.x, viewpoint.y, viewpoint.z, 0, 0, 0);
		visualization_engine->FindVisibleBlocks(volume, &pose, &calibration_data.intrinsics_d, render_state);
		int* visible_codes_device = volume->index.GetVisibleBlockHashCodes();
		const int visible_block_count = volume->index.GetVisibleBlockCount();
		ORUtils::OStreamWrapper file(GENERATED_TEST_DATA_PREFIX "TestData/data_blocks/visible_blocks.dat");
		SaveRawDataToFile<int>(file, visible_codes_device, visible_block_count, TMemoryDeviceType);
		std::cout << visible_block_count << std::endl;
		delete volume;
	}

	delete render_state;
	delete visualization_engine;
}


#define GENERATED_TEST_DATA_TYPE_ENUM_DESCRIPTION GeneratedTestDataType, \
    (SNOOPY_UNMASKED_VOLUMES,    "SNOOPY_UNMASKED_VOLUMES", "snoopy_unmasked_volumes", "unmasked_volumes", "unmasked", "u", "su", "suv"), \
    (MASKED_VOLUMES,             "SNOOPY_MASKED_VOLUMES", "snoopy_masked_volumes", "masked_volumes", "masked", "m", "sm", "smv"), \
    (PVA_WARP_GRADIENTS,         "PVA_WARP_GRADIENTS", "pva_warp_gradients", "pva_warps", "pw", "pva_w"), \
    (VBH_WARP_GRADIENTS,         "VBH_WARP_GRADIENTS", "vbh_warp_gradients", "vbh_warps", "vw", "vbh_w"), \
    (COMPARATIVE_WARP_GRADIENTS, "COMPARATIVE_WARP_GRADIENTS", "comparative_warp_gradients", "warps", "comparative_warps", "w"), \
    (PVA_WARPED_VOLUMES,         "PVA_WARPED_VOLUMES", "pva_warped_volumes", "pva_wv"), \
    (VBH_WARPED_VOLUMES,         "VBH_WARPED_VOLUMES", "vbh_warped_volumes", "vbh_wv"), \
    (PVA_FUSED_VOLUMES,          "PVA_FUSED_VOLUMES", "pva_fused_volumes", "pva_fv"), \
    (VBH_FUSED_VOLUMES,          "VBH_FUSED_VOLUMES", "vbh_fused_volumes", "vbh_fv"), \
    (CONFUGRATIONS,              "CONFIGURATIONS", "configurations", "config", "c"), \
    (MESHES,                     "MESHES", "meshes", "m"), \
    (RAYTRACING_CPU,             "RAYTRACING_CPU", "raytracing_cpu", "r_cpu"), \
    (RAYTRACING_CUDA,            "RAYTRACING_CUDA", "raytracing_cuda", "r_cuda")

GENERATE_SERIALIZABLE_ENUM(GENERATED_TEST_DATA_TYPE_ENUM_DESCRIPTION);

int main(int argc, char* argv[]) {
	log4cplus::initialize();
	log4cplus::SharedAppenderPtr console_appender(new log4cplus::ConsoleAppender(false, true));
	log4cplus::Logger::getRoot().addAppender(console_appender);
	log4cplus::Logger::getRoot().setLogLevel(log4cplus::DEBUG_LOG_LEVEL);

	std::unordered_map<GeneratedTestDataType, std::function<void()>> generator_by_string(
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
					{RAYTRACING_CPU,             GenerateRaytracingTestData_VoxelBlockHash<MEMORYDEVICE_CPU>},
					{RAYTRACING_CUDA,            GenerateRaytracingTestData_VoxelBlockHash<MEMORYDEVICE_CUDA>},
			});
	if (argc < 2) {
		// calls every generator iteratively
		for (auto iter : generator_by_string) {
			(iter.second)();
		}
	} else {
		std::string generated_data_type_argument = argv[1];
		GeneratedTestDataType chosen = string_to_enumerator<GeneratedTestDataType>(generated_data_type_argument);
		generator_by_string[chosen]();

	}
	return 0;
}